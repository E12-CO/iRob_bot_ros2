// iRob ROS2 ethernet hardware interface code for iRob ETH controller board (BTS7960 shield)
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h> 
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


// ROS2 library
#include <rclcpp/rclcpp.hpp>

// Sensor messages
// JointState for motor
#include "sensor_msgs/msg/joint_state.hpp"

#include <irob_interface/msg_type.h>

// Define Feedback Loop time 
#define LOOP_TIME_MIL   10 // 10 millisec -> 100Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

// Communication related
#define IROB_TCP_PORT			6767

#define ROS_COMM_TIMEOUT		5 	// consider communication timeout when 5 cycle elapsed (100ms)
#define VEL_CMD_TIMEOUT			5 	// cmd vel timeout when 5 cycle elapsed (100ms)

#define MAX_MOTOR_ID_CNT		8	// Maximum of motor ID grouping for iRob is 8 motors 

// Conversion factor
#define ENCODER_MAX_COUNT		65535.0f
#define ENCODER_HALF_COUNT		(ENCODER_MAX_COUNT / 2.0f)
#define ENCODER_HALF_ANGLE		3.141593f
#define ENCODER_COUNT_FACTOR	(ENCODER_HALF_ANGLE / ENCODER_HALF_ANGLE)

#define RPM_TO_RAD_S  	0.104778 	// 1 rpm == 0.1047 rad/s 
#define RAD_S_TO_RPM	9.549297	// 1 rad/s ~= 0.54 RPM
#define SINE_120 		0.866025 	// Sine(120 degree) in rad unit
#define F2_SQRT3      	1.1547	 	// 2/sqrt(3)


class irob_eth_if : public rclcpp::Node{
	
	public:
	
	// Communication related
	int irob_runner_fsm = 0;
	unsigned long ros_comm_timeout_counter = 0;
	unsigned long cmd_vel_timeout_counter = 0;
	
	enum IROB_RUNNER_FSM_ENUM{
		eIROB_STATE_INIT 		= 0,// FSM Initialize state and TCP connection estabilishment
		eIROB_STATE_W8CONNECT	= 1,// TCP connection wait state
		eIROB_STATE_QUERY		= 2,// iRob ETH parameter query state
		eIROB_STATE_CONFIGURE	= 3,// iRob ETH parameter configuration state
		eIROB_STATE_W8CONFIGURE = 4,// Wait iRob ETH parameter to configured
		
		
		eIROB_STATE_RUN			= 127,// Active connection run state
	};
	
	// Structure to store specific parameter for each motor
	struct sMotorParameters{
		std::string 	sTopicName;
		std::string 	sIpAddress;
		std::string		sLoopRate;
		float			f32SpeedKp;
		float 			f32SpeedKi;
		float			f32SpeedKd;
		float			f32SpeedKff;
		float			f32SpeedMin;
		float			f32SpeedMax;
		uint32_t		u32EncoderCpr;
		float			f32GearRatio;
		int32_t			i32WheelId;
	};

	// Create a vector to store the parameter for each motor
	std::vector<sMotorParameters> vectMotorParams;
	
	// Shared memory between ROS wall timer and TCP thead
	std::array<std::atomic<float>, MAX_MOTOR_ID_CNT> velocity_cmd;
	std::array<std::atomic<float>, MAX_MOTOR_ID_CNT> velocity_feedback;
	int32_t i32MaxScannedWheelId;
	
	// Dedicated Joint state topics for iRob_controller
	// Sensor publishers
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr		pubIrobJointState;
	sensor_msgs::msg::JointState	irobJointFeedback;
	// Sensor subscriber
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr	subIrobJointCommand;
	
	// Used in wall timer callback for publishing joint state feedback for irob controller
	rclcpp::TimerBase::SharedPtr timerJointFb_;
	
	
	
	irob_eth_if() : Node("iRob_ETH_Interface"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club Engineering KMITL : Starting iRob ETH interface..."
			);
		
		bool bIsMoreMotor = false;
		uint32_t u32MotorIndex = 0;
		
		i32MaxScannedWheelId = -1;
		
		do{
			// Declare and get the parameter of the motor
			// in the format of "motor_N.motor_parameter"
			if(u32MotorIndex > 31){
				RCLCPP_ERROR(
					this->get_logger(),
					"Too many motors!"
				);
				std::raise(SIGTERM);
				return;
			}
				
			
			std::stringstream ssMotorInstant;
			ssMotorInstant << "motor_" << u32MotorIndex;
			std::string sMotorInstant = ssMotorInstant.str();
			std::string sParamParent = sMotorInstant + ".";
			
			sMotorParameters	sParam;
			
			this->declare_parameter(sParamParent + "topic_name", rclcpp::PARAMETER_STRING);
			if(this->get_parameter(sParamParent + "topic_name", sParam.sTopicName)){
				
				bIsMoreMotor = true;
			}else{
				// No or no more motor instant found, stop the search
				bIsMoreMotor = false;
				goto exitMotorSearch;
			}
			
			this->declare_parameter(sParamParent + "ip_address", "192.168.1.16");
			this->get_parameter(sParamParent + "ip_address", sParam.sIpAddress);
			
			this->declare_parameter(sParamParent + "loop_rate", "50HZ");
			this->get_parameter(sParamParent + "loop_rate", sParam.sLoopRate);
			
			this->declare_parameter(sParamParent + "speed_kp", 0.0);
			this->get_parameter(sParamParent + "speed_kp", sParam.f32SpeedKp);
			
			this->declare_parameter(sParamParent + "speed_ki", 0.0);
			this->get_parameter(sParamParent + "speed_ki", sParam.f32SpeedKi);
			
			this->declare_parameter(sParamParent + "speed_kd", 0.0);
			this->get_parameter(sParamParent + "speed_kd", sParam.f32SpeedKd);
			
			this->declare_parameter(sParamParent + "speed_kff", 0.0);
			this->get_parameter(sParamParent + "speed_kff", sParam.f32SpeedKff);
			
			this->declare_parameter(sParamParent + "speed_min", 0.0);
			this->get_parameter(sParamParent + "speed_min", sParam.f32SpeedMin);
			
			this->declare_parameter(sParamParent + "speed_max", 10000.0);
			this->get_parameter(sParamParent + "speed_max", sParam.f32SpeedMax);
			
			this->declare_parameter(sParamParent + "encoder_cpr", 1);
			this->get_parameter(sParamParent + "encoder_cpr", sParam.u32EncoderCpr);
			
			this->declare_parameter(sParamParent + "gear_ratio", 1.0);
			this->get_parameter(sParamParent + "gear_ratio", sParam.f32GearRatio);			

			// Register wheel ID to be used for chassis controller
			// Valid ID : 0, 1, 2, 3...
			this->declare_parameter(sParamParent + "wheel_id", -1);
			this->get_parameter(sParamParent + "wheel_id", sParam.i32WheelId);
			
			if(sParam.i32WheelId != -1){
				RCLCPP_INFO(
					this->get_logger(),
					"Found motor_%d with iRob ID %d",
					u32MotorIndex, sParam.i32WheelId
				);
			}else{
				RCLCPP_INFO(
					this->get_logger(),
					"Found motor_%d",
					u32MotorIndex
				);
			}
			
			if(sParam.i32WheelId > i32MaxScannedWheelId)
				i32MaxScannedWheelId = sParam.i32WheelId;

			// Push param into the array
			vectMotorParams.push_back(sParam);
			
			// Look for duplicates in all array except self
			for(int i=0; i < vectMotorParams.size() - 1; i++){
				if(
					(sParam.i32WheelId != -1) &&
					(sParam.i32WheelId == vectMotorParams[i].i32WheelId)
				){
					RCLCPP_ERROR(
						this->get_logger(),
						"Error : found duplicate iRob motor ID %d at motor_%d!",
						sParam.i32WheelId, u32MotorIndex
					);
					
					std::raise(SIGTERM);
					return;
					
				}
			}
			
			// Increment to find next motor instant 	
			u32MotorIndex++;			
		}while(bIsMoreMotor);
		
exitMotorSearch:		
		if(u32MotorIndex < 1){
			RCLCPP_ERROR(
				this->get_logger(),
				"No motor(s) found in the YAML parameter file!"
			);
			std::raise(SIGTERM);
			return;
		}else{
			RCLCPP_INFO(
				this->get_logger(),
				"Found total of %d motors in YAML parameter file",
				u32MotorIndex
			);
		}
		
		if(
			(i32MaxScannedWheelId > 2) && 
			(i32MaxScannedWheelId < 9)
		){
			
			RCLCPP_INFO(
				this->get_logger(),
				"Found %d wheel IDs! Creating joint state for iRob controller!",
				i32MaxScannedWheelId + 1
			);
			
			pubIrobJointState = create_publisher<sensor_msgs::msg::JointState>(
				"irob_motor_feedback", 10
			);
			
			irobJointFeedback.header.frame_id	= "quad_motor";
			irobJointFeedback.velocity.resize(i32MaxScannedWheelId + 1);
			
			subIrobJointCommand = 
				create_subscription<sensor_msgs::msg::JointState>(
					"irob_motor_cmd",
					10,
					std::bind(
						&irob_eth_if::irob_motor_cmd_callback,
						this,
						std::placeholders::_1)
				);
			
			timerJointFb_ = this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&irob_eth_if::irob_feedback_runner,
					this
					)
			);
		}
		
		// Create thread
		RCLCPP_INFO(
			this->get_logger(),
			"Swaping TCP thread..."
		);
		
		for(uint32_t u32Tasks = 0; u32Tasks < u32MotorIndex; u32Tasks++){
			// spawn TCP thread
			std::thread tRunner(
				&irob_eth_if::irob_tcpHandlerTask, 
				this,
				vectMotorParams[u32Tasks]
			);
			tRunner.detach();
			
		}
		
		RCLCPP_INFO(this->get_logger(), "iRob ETH interface started!");
	}
	
	// Motor cmd callback from the topic "irob_motor_cmd" sent from iRob_controller node
	void irob_motor_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr motor_cmd){
		if(
			(motor_cmd->velocity.size() < 3) ||
			(motor_cmd->velocity.size() > 4)
		){
			RCLCPP_ERROR(
				this->get_logger(),
				"Command error, the joint number %ld is incorrect!",
				motor_cmd->velocity.size()
			);
			return;
		}
		
		if(motor_cmd->header.frame_id != "quad_motor"){
			RCLCPP_ERROR(
				this->get_logger(),
				"The joint command is not in the frame \"quad_motor\"!"
			);
			return;
		}
		
		if(motor_cmd->velocity.size() > MAX_MOTOR_ID_CNT){
			RCLCPP_ERROR(
				this->get_logger(),
				"The joint command size %ld exceed the motor count, command rejected!",
				motor_cmd->velocity.size()
			);
			return;
		}
		
		// Convert rad/s to RPM and send it to the controller 
		for(size_t i=0; i <  motor_cmd->velocity.size(); i++)
			velocity_cmd[0].store(motor_cmd->velocity[i] * RAD_S_TO_RPM);
		
				
		cmd_vel_timeout_counter = 0;
	}
	
	// Runner to send encoder velocity feedback to iRob_controller node via "irob_motor_feedback"
	void irob_feedback_runner(void){
		irobJointFeedback.header.stamp = this->get_clock()->now();
		
		for(size_t i = 0; i < i32MaxScannedWheelId; i++){
		irobJointFeedback.velocity[i] = 
			velocity_feedback[i].load(std::memory_order_relaxed);
		}
		
		pubIrobJointState->publish(irobJointFeedback);
	}
	
	// Return true if connected to iRob ETH server 
	bool irob_isConnectedToIrob(int i32ConnectionFd){
		struct sockaddr_in	sIRobConnectAddr;
		socklen_t length = sizeof(sIRobConnectAddr);
		memset(&sIRobConnectAddr, 0, sizeof(sIRobConnectAddr));
		return (
			getpeername(
				i32ConnectionFd, 
				(struct sockaddr *)&sIRobConnectAddr, 
				&length) == 0
		);
		
	}
	
	// iRob_ETH protocol protocol packet helper function
	void irob_protocolSetup(
		tRosClientCommand *tCommandPacket,
		uint8_t u8CmdType, 
		uint8_t u8CmdIdx, 
		uint8_t u8CmdRw,
		uint8_t u8CmdLen
		){
		
		tCommandPacket->regBit.bType 	= u8CmdType;
		tCommandPacket->regBit.bIndex	= u8CmdIdx;
		tCommandPacket->regBit.bRW		= u8CmdRw;
		tCommandPacket->u8DataLength	= u8CmdLen;
	}
	
	// Check the validity of the returned packet from the iRob_ETH
	bool irob_protocolReturnValidate(
		tRosClientCommand *tCommandPacket,
		tRosClientCommand *tReturnPacket
		){
			
		if(
			(tCommandPacket->u8Cmd == tReturnPacket->u8Cmd)
		)
			return true;
		else
			return false;
	}
	
	// Write the protocol packet to socket
	int irob_protocolWrite(int i32ConnectionFd, tRosClientCommand *tCommandPacket){
		uint8_t u8Length;
		
		u8Length = 4;// Default read packet will send 4 bytes (header + command + read length)
		// In the case of write packet, the length will include the data
		if(tCommandPacket->regBit.bRW == eRW_WRITE)
			u8Length = 4 + tCommandPacket->u8DataLength;
		
		return write(i32ConnectionFd, (unsigned char*)&tCommandPacket->u8RbcHeader[0], u8Length);
	}
	
	// Thread that handling iRob_ETH connection, initialization, configuration and safety.
	// Spawn by the number of motor configured in the YAML parameter 
	void irob_tcpHandlerTask(sMotorParameters sParameterData){
		// socket file descriptor
		int i32iRobSocketFd;
		
		// received byte in socket buffer
		int i32RxBytes = -1;
		bool bCanProcessRead = false;
		
		// error return
		int ret;
		
		// Sensor publishers
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr		pubMotorJointState;
		sensor_msgs::msg::JointState	motorJointFeedback;
		// Sensor subscriber
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr	subMotorJointCommand;
		
		// Communication buffer
		tRosClientCommand tRbcTxPacket;// Buffer for sending to the iRob ETH
		tRosClientCommand tRbcRxPacket;// Buffer for receiving packet from iRob ETH
		
		rclcpp::Duration 	dControlSendDuration(0, 10000000U);// 0.01 sec -> 100Hz
		rclcpp::Time   		tControlTick(0, 0);
		// Holds the Joint command (motor speed)
		float 	f32MotorJointCommand;
		
		// FSM 
		unsigned int u32iRobEthFsm = eIROB_STATE_INIT;
		unsigned int u32iRobEthFsmRejmp = eIROB_STATE_INIT;
		unsigned int u32iRobConfigFsm = ePARAM_CTRL_KPIDFF;
		
		
		RCLCPP_INFO(
			this->get_logger(),
			"[%s]Spawning thread with IP %s",
			sParameterData.sTopicName.c_str(),
			sParameterData.sIpAddress.c_str()
		);
		
		// Initialize TCP Socket 
		i32iRobSocketFd = socket(
			AF_INET, 
			SOCK_STREAM | SOCK_NONBLOCK,
			0
		);
		
		if(i32iRobSocketFd < 0){
			RCLCPP_ERROR(
				this->get_logger(),
				"[%s]Can't create socket!",
				sParameterData.sTopicName.c_str()
			);
			return;
		}
		
		struct sockaddr_in	sIRobSocketAddr;
		sIRobSocketAddr.sin_family	= AF_INET;
		sIRobSocketAddr.sin_port	= htons(IROB_TCP_PORT);
		sIRobSocketAddr.sin_addr.s_addr =
			inet_addr(sParameterData.sIpAddress.c_str());
			
		
		// Create the Joint state publisher for motor feedback
		pubMotorJointState = 
			this->create_publisher<sensor_msgs::msg::JointState>(
				sParameterData.sTopicName + "/feedback",
				10
			);
		
		motorJointFeedback.header.frame_id 	= sParameterData.sTopicName;
		motorJointFeedback.name.push_back(sParameterData.sTopicName);
		motorJointFeedback.position.resize(1);
		motorJointFeedback.velocity.resize(1);
		
		// Create lambda callback function to subsribe to the joint command
		auto lmbdJointCommandCallback =
			[this, &f32MotorJointCommand](sensor_msgs::msg::JointState::UniquePtr jointMsg) -> void{
				f32MotorJointCommand = jointMsg->velocity[0];
			};
		
		subMotorJointCommand = 
			this->create_subscription<sensor_msgs::msg::JointState>(
				sParameterData.sTopicName + "/command",
				10,
				lmbdJointCommandCallback
			);	
			
		tControlTick = this->get_clock()->now();	
		
		while(1){
			
			ioctl(
				i32iRobSocketFd,
				FIONREAD,
				&i32RxBytes
			);
			
			if(i32RxBytes >= 4){
				RCLCPP_DEBUG(
					this->get_logger(),
					"Received %d bytes!",
					i32RxBytes
				);
				
				ret = read(
					i32iRobSocketFd,
					(unsigned char*)&tRbcRxPacket.u8RbcHeader[0],
					i32RxBytes
				);
				
				// Check the return header before allow to set the flag
				if(
					(tRbcRxPacket.u8RbcHeader[0] != 'J') ||
					(tRbcRxPacket.u8RbcHeader[1] != 'B')
				){
					bCanProcessRead = false;
				}else{
					
					// TODO : Intercept the feedback data
					if(
						(tRbcRxPacket.regBit.bType 	== eCOMMAND_CONTROL) &&
						(tRbcRxPacket.regBit.bIndex == eCONTROL_CTRL_FEEDBACK) &&
						(tRbcRxPacket.regBit.bRW 	== eRW_READ)
					){
						// Joint state in
						// Position -> rad
						// Velocity -> rad/s
						motorJointFeedback.position[0] = 
							*(float *)&tRbcRxPacket.u8InDataPtr[0] * ENCODER_COUNT_FACTOR;
						
						motorJointFeedback.velocity[0] = 
							*(float *)&tRbcRxPacket.u8InDataPtr[4] * RPM_TO_RAD_S;
							
						motorJointFeedback.header.stamp = this->get_clock()->now();
						
						if(sParameterData.i32WheelId != -1){
							velocity_feedback[sParameterData.i32WheelId].store(
								motorJointFeedback.velocity[0],
								std::memory_order_relaxed
							);
						}
					
						pubMotorJointState->publish(motorJointFeedback);
						
						bCanProcessRead = false;
					}else{
						if(bCanProcessRead == false)
							bCanProcessRead = true;
					}
				}
				
				//i32RxBytes = 0;
			}
			
			switch(u32iRobEthFsm){
				case eIROB_STATE_INIT: // Initialize state
				{
					// Connect to the iRob ETH
					connect(
						i32iRobSocketFd,
						(struct sockaddr *)&sIRobSocketAddr,
						sizeof(sIRobSocketAddr)
					);
					
					// Setup the header
					tRbcTxPacket.u8RbcHeader[0] = 'R';
					tRbcTxPacket.u8RbcHeader[1] = 'B';
					
					// Reset the can process flag
					bCanProcessRead = false;
					
					// Reset the configuration FSM to the first command
					u32iRobConfigFsm = ePARAM_CTRL_KPIDFF;
					
					u32iRobEthFsm = eIROB_STATE_W8CONNECT;
					u32iRobEthFsmRejmp = eIROB_STATE_W8CONNECT;
				}				
				break;	
					
				case eIROB_STATE_W8CONNECT: // Wait for TCP connection to the iRob ETH
				{
					if(!irob_isConnectedToIrob(i32iRobSocketFd)){
						u32iRobEthFsm = eIROB_STATE_INIT;
						break;
					}
					
					// Read the ePARAM_CTRL_STAT to chech if the iRob ETH is configured
					irob_protocolSetup(
						&tRbcTxPacket,
						eCOMMAND_PARAM,
						ePARAM_CTRL_STAT,
						eRW_READ,
						sizeof(uint8_t)
					);
					
					// Prepare to query state
					u32iRobEthFsm = eIROB_STATE_QUERY;
					
					// Try to send the command to check the connection status
					if(
						ret = irob_protocolWrite(i32iRobSocketFd, &tRbcTxPacket),
						(ret < 0)
					){
						if(errno != EAGAIN){
							RCLCPP_ERROR(
								this->get_logger(),
								"[%s]Error protocol tx for ePARAM_CTRL_STAT with code %d",
								sParameterData.sTopicName.c_str(),
								errno
							);
							
							// TODO : Find a better way to handle this error 
							// Try to reconnect 
							u32iRobEthFsm = eIROB_STATE_INIT;
						}
					}
					
					// Timestamping the tick
					tControlTick = this->get_clock()->now();
					
				}
				break;
				
				case eIROB_STATE_QUERY: // Query the configuration status of the control parameters (KPIDFF, MinMax, Encoder CPR and Gear ratio) 
				{
					if(
						(this->get_clock()->now() - tControlTick)
						> dControlSendDuration 
					){
						u32iRobEthFsm = eIROB_STATE_W8CONNECT;
							break;
					}else{
						if(bCanProcessRead == false)
							break;
					}
					
					bCanProcessRead = false;// Clear the can read flag
					
					// Check if the iRob ETH is configured
					// 1. Check if the return packet is valid
					if(irob_protocolReturnValidate(&tRbcTxPacket, &tRbcRxPacket)){
						RCLCPP_INFO(
							this->get_logger(),
							"[%s]ParamStatus 0x%X",
							sParameterData.sTopicName.c_str(),
							((tParameterStatus *)&tRbcRxPacket.u8InDataPtr[0])->u8ParamStat
						);
						// 2. Parse the data bit to check the parameter is not configured
						if(((tParameterStatus *)&tRbcRxPacket.u8InDataPtr[0])->u8ParamStat != 0xFF){
							RCLCPP_INFO(
								this->get_logger(),
								"[%s]iRob ETH of %s is not configured, configure it now...",
								sParameterData.sTopicName.c_str(),
								sParameterData.sTopicName.c_str()
							);
							

						}else{
							RCLCPP_INFO(
								this->get_logger(),
								"[%s]iRob ETH of %s is configured, reconfigure it anyway",
								sParameterData.sTopicName.c_str(),
								sParameterData.sTopicName.c_str()
							);
							
						}

						//jump to the configure state
						u32iRobEthFsm = eIROB_STATE_CONFIGURE;
						
					}else{
						RCLCPP_ERROR(
							this->get_logger(),
							"[%s]Error return packet cmd mismatch the command packet!",
							sParameterData.sTopicName.c_str()
						);
						
						// Send it back to retry 
						u32iRobEthFsm = eIROB_STATE_W8CONNECT;
					}
					
				}
				break;
				
				case eIROB_STATE_CONFIGURE: // Configure all of the control parameters
				{
					
					// Configure the following parameters. Load it from the YAML to the iRob ETH
					// - Ki, Ki, Kd, Kff
					// - Control min, Control max
					// - Encoder count per revolution (CPR)
					// - Gear ratio
					// - Control loop rate
					// - Control loop enable
					
					switch(u32iRobConfigFsm){
						case ePARAM_CTRL_KPIDFF:// Setup PID and feedforward gain
						{	
							// Copy data from ROS parameter struct to the TX packet 
							
							*(float *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
								sParameterData.f32SpeedKp;
							*(float *)(tRbcTxPacket.u8InDataPtr + 0x04) = 
								sParameterData.f32SpeedKi;
							*(float *)(tRbcTxPacket.u8InDataPtr + 0x08) = 
								sParameterData.f32SpeedKd;
							*(float *)(tRbcTxPacket.u8InDataPtr + 0x0C) = 
								sParameterData.f32SpeedKff;
							
							irob_protocolSetup(
								&tRbcTxPacket,
								eCOMMAND_PARAM,
								ePARAM_CTRL_KPIDFF,
								eRW_WRITE,
								PARAM_KPIDFF_SIZE
							);
							
							// Expected to configure the control min max on the next cycle
							u32iRobConfigFsm = ePARAM_CTRL_MINMAX;
							
						}
						break;
						
						case ePARAM_CTRL_MINMAX:// Setup control input min/max
						{
							// Copy data from ROS parameter struct to the TX packet 
							
							*(float *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
								sParameterData.f32SpeedMin;
							*(float *)(tRbcTxPacket.u8InDataPtr + 0x04) = 
								sParameterData.f32SpeedMax;
							
							irob_protocolSetup(
								&tRbcTxPacket,
								eCOMMAND_PARAM,
								ePARAM_CTRL_MINMAX,
								eRW_WRITE,
								PARAM_CTRL_MINMAX
							);
							
							// Expected to configure the encoder cpr on the next cycle
							u32iRobConfigFsm = ePARAM_CTRL_ENC_CPR;
						}
						break;
						
						case ePARAM_CTRL_ENC_CPR:// Setup encoder CPR
						{
							// Copy data from ROS parameter struct to the TX packet 
							
							*(uint32_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
								sParameterData.u32EncoderCpr;
							
							irob_protocolSetup(
								&tRbcTxPacket,
								eCOMMAND_PARAM,
								ePARAM_CTRL_ENC_CPR,
								eRW_WRITE,
								PARAM_ENCCPR_SIZE
							);
							
							// Expected to configure the gear ratio on the next cycle
							u32iRobConfigFsm = ePARAM_CTRL_GEAR_RATIO;
						}
						break;
						
						case ePARAM_CTRL_GEAR_RATIO:// Setup gear ration
						{
							// Copy data from ROS parameter struct to the TX packet 
							
							*(float *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
								sParameterData.f32GearRatio;
							
							irob_protocolSetup(
								&tRbcTxPacket,
								eCOMMAND_PARAM,
								ePARAM_CTRL_GEAR_RATIO,
								eRW_WRITE,
								PARAM_GRRATIO_SIZE
							);
							
							// Configuration finished, commit the parameter
							u32iRobConfigFsm = eCONTROL_CTRL_LOOP_RATE;
						}
						break;
						
						case eCONTROL_CTRL_LOOP_RATE:// Setup control loop rate
						{	
							// Copy data from ROS parameter struct to the TX packet 
							if(sParameterData.sLoopRate == "50HZ"){
								*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
									eLOOP_RATE_50HZ;
							}else if(sParameterData.sLoopRate == "100HZ"){
								*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
									eLOOP_RATE_100HZ;
							}else if(sParameterData.sLoopRate == "250HZ"){
								*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
									eLOOP_RATE_250HZ;
							}else if(sParameterData.sLoopRate == "500HZ"){
								*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
									eLOOP_RATE_500HZ;
							}else if(sParameterData.sLoopRate == "1KHZ"){
								*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
									eLOOP_RATE_1KHZ;
							}else if(sParameterData.sLoopRate == "2KHZ"){
								*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
									eLOOP_RATE_2KHZ;
							}else{
								RCLCPP_WARN(
									this->get_logger(),
									"[%s]Warning : Unknown loop rate : %s ! Default to 50Hz",
									sParameterData.sTopicName.c_str(),
									sParameterData.sLoopRate.c_str()
								);
								
								*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 
									eLOOP_RATE_50HZ;
							}

							irob_protocolSetup(
								&tRbcTxPacket,
								eCOMMAND_CONTROL,
								eCONTROL_CTRL_LOOP_RATE,
								eRW_WRITE,
								sizeof(uint8_t)
							);
							
							// Configuration finished, commit the parameter
							u32iRobConfigFsm = eCONTROL_CTRL_ON_OFF;
						}
						break;
						
						case eCONTROL_CTRL_ON_OFF:// Enable control loop
						{
							*(uint8_t *)(tRbcTxPacket.u8InDataPtr + 0x00) = 1;
							
							irob_protocolSetup(
								&tRbcTxPacket,
								eCOMMAND_CONTROL,
								eCONTROL_CTRL_ON_OFF,
								eRW_WRITE,
								sizeof(uint8_t)
							);
							
							u32iRobConfigFsm = 127;
						}
						break;
					}
					
					// We will wait for the protocol reply on the next state
					u32iRobEthFsm = eIROB_STATE_W8CONFIGURE;
					
					// Send out the configuration command
					if(
						ret = irob_protocolWrite(i32iRobSocketFd, &tRbcTxPacket),
						(ret < 0)
					){
						if(errno != EAGAIN){
							RCLCPP_ERROR(
								this->get_logger(),
								"[%s]Error protocol tx for ePARAM_CTRL_STAT with code %d",
								sParameterData.sTopicName.c_str(),
								errno
							);
							
							// TODO : Find a better way to handle this error 
							// Try to reconnect 
							u32iRobEthFsm = eIROB_STATE_INIT;
						}
					}					
				}
				break;
				
				case eIROB_STATE_W8CONFIGURE: // Wait for the conrol parameter to transfer to the iRob ETH and we get acknowledge back
				{
					if(bCanProcessRead == false)
						break;
					
					bCanProcessRead = false;// Clear the can read flag
					
					// Catch the invalid acknowledgement 
					if(!irob_protocolReturnValidate(&tRbcTxPacket, &tRbcRxPacket)){
						// If not valid, retry it again
						u32iRobConfigFsm = ePARAM_CTRL_KPIDFF;
						u32iRobEthFsm = eIROB_STATE_CONFIGURE;
						break;
					}
					
					// Check if we need to continue sending the control parameters to the iRob ETH
					if(u32iRobConfigFsm != 127){
						u32iRobEthFsm = eIROB_STATE_CONFIGURE;
						
					}else{
						// Reset the config fsm back to start just in case for the next re-config
						u32iRobConfigFsm = ePARAM_CTRL_KPIDFF;
						
						// Jump to run state
						u32iRobEthFsm = eIROB_STATE_RUN;
						
						RCLCPP_INFO(
							this->get_logger(),
							"[%s]Configuration process completed! Rechecking the param status",
							sParameterData.sTopicName.c_str()
						);
					}
					
				}
				break;
				
				case eIROB_STATE_RUN:// Actively send the setpoint
				{
					if(
						(this->get_clock()->now() - tControlTick)
						> dControlSendDuration 
					){
						// Timestamping the tick
						tControlTick = this->get_clock()->now();
						
						if(!irob_isConnectedToIrob(i32iRobSocketFd)){
							RCLCPP_ERROR(
								this->get_logger(),
								"[%s]iRob ETH disconnected! Reconnecting...",
								sParameterData.sTopicName.c_str()
							);
							u32iRobEthFsm = eIROB_STATE_INIT;
							break;
						}
						
						if(sParameterData.i32WheelId != -1)
							f32MotorJointCommand = 
								velocity_cmd[sParameterData.i32WheelId].load(std::memory_order_relaxed);
						
						// Copy the received joint command from topic 
						*(float *)(&tRbcTxPacket.u8InDataPtr + 0x00) = f32MotorJointCommand;
							
						irob_protocolSetup(
							&tRbcTxPacket,
							eCOMMAND_CONTROL,
							eCONTROL_CTRL_SETPOINT,
							eRW_WRITE,
							sizeof(float)
						);
						
						// Send out the setpoint command
						if(
							ret = irob_protocolWrite(i32iRobSocketFd, &tRbcTxPacket),
							(ret < 0)
						){
							if(errno != EAGAIN){
								RCLCPP_ERROR(
									this->get_logger(),
									"[%s]Error protocol tx for eCONTROL_CTRL_SETPOINT with code %d",
									sParameterData.sTopicName.c_str(),
									errno
								);
								
								// TODO : Find a better way to handle this error 
							}
						}
						
					}

				}
				break;
				
				
			}
			
		}
		
		RCLCPP_WARN(
			this->get_logger(),
			"[%s]Terminating TCP thread of %s",
			sParameterData.sTopicName.c_str(),
			sParameterData.sTopicName.c_str()
		);
	}
	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto irobIF {std::make_shared<irob_eth_if>()};
	rclcpp::spin(irobIF);
	//close(irobIF->i32iRobSocketFd);
	rclcpp::shutdown();
}
