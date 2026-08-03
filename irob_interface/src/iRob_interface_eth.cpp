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
#define LOOP_TIME_MIL   20 // 20 millisec -> 50Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

// Communication related
#define IROB_TCP_PORT			6767

#define ROS_COMM_TIMEOUT		5 	// consider communication timeout when 5 cycle elapsed (100ms)

#define VEL_CMD_TIMEOUT			5 	// cmd vel timeout when 5 cycle elapsed (100ms)


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
		eIROB_STATE_RUN			= 5,// Active connection run state
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
	};

	// Create a vector to store the parameter for each motor
	std::vector<sMotorParameters> vectMotorParams;
	
	irob_eth_if() : Node("iRob_ETH_Interface"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club Engineering KMITL : Starting iRob ETH interface..."
			);

		bool bIsMoreMotor = false;
		uint32_t u32MotorIndex = 0;
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
				RCLCPP_INFO(
					this->get_logger(),
					"Found motor_%d",
					u32MotorIndex
				);

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

			// Push param into the array
			vectMotorParams.push_back(sParam);
			
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
	
	int irob_protocolWrite(int i32ConnectionFd, tRosClientCommand *tCommandPacket){
		uint8_t u8Length;
		
		u8Length = 4;// Default read packet will send 4 bytes (header + command + read length)
		// In the case of write packet, the length will include the data
		if(tCommandPacket->regBit.bRW == eRW_WRITE)
			u8Length = 4 + tCommandPacket->u8DataLength;
		
		return write(i32ConnectionFd, (unsigned char*)&tCommandPacket->u8RbcHeader[0], u8Length);
	}
	
	
	void irob_tcpHandlerTask(sMotorParameters sParameterData){
		// socket file descriptor
		int i32iRobSocketFd;
		
		// received byte in socket buffer
		int i32RxBytes;
		bool bCanProcessRead;
		
		// error return
		int ret;
		
		// Sensor publishers
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr		pubMotorJointState;
		// Sensor subscriber
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr	subMotorJointCommand;
		
		// Communication buffer
		tRosClientCommand tRbcTxPacket;// Buffer for sending to the iRob ETH
		tRosClientCommand tRbcRxPacket;// Buffer for receiving packet from iRob ETH
		
		// Holds the Joint command (motor speed)
		float 	f32MotorJointCommand;
		
		// FSM 
		unsigned int u32iRobEthFsm = eIROB_STATE_INIT;
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
		
		while(1){
			
			ioctl(
				i32iRobSocketFd,
				FIONREAD,
				&i32RxBytes
			);
			
			if(i32RxBytes >= 4){
				RCLCPP_INFO(
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
					if(bCanProcessRead == false)
						bCanProcessRead = true;
					
					// TODO : Intercept the feedback data
					
					
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
					
				}
				break;
				
				case eIROB_STATE_QUERY: // Query the configuration status of the control parameters (KPIDFF, MinMax, Encoder CPR and Gear ratio) 
				{
					if(bCanProcessRead == false)
						break;
					
					bCanProcessRead = false;// Clear the can read flag
					
					// Check if the iRob ETH is configured
					// 1. Check if the return packet is valid
					if(irob_protocolReturnValidate(&tRbcTxPacket, &tRbcRxPacket)){
						// 2. Parse the data bit to check the parameter is not configured
						if(((tParameterStatus *)&tRbcRxPacket.u8InDataPtr[0])->u8ParamStat != 0xFF){
							RCLCPP_INFO(
								this->get_logger(),
								"[%s]iRob ETH of %s is not configured, configuring it now...",
								sParameterData.sTopicName.c_str(),
								sParameterData.sTopicName.c_str()
							);
							
							// Need configuration, jump to the configure state
							u32iRobEthFsm = eIROB_STATE_CONFIGURE;
						}else{
							RCLCPP_INFO(
								this->get_logger(),
								"[%s]iRob ETH of %s is configured, skipping configuration process...",
								sParameterData.sTopicName.c_str(),
								sParameterData.sTopicName.c_str()
							);
							
							// No need for configuration, just jump to run state
							u32iRobEthFsm = eIROB_STATE_RUN;
						}
						
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
					
					switch(u32iRobConfigFsm){
						case ePARAM_CTRL_KPIDFF:
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
						
						case ePARAM_CTRL_MINMAX:
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
						
						case ePARAM_CTRL_ENC_CPR:
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
						
						case ePARAM_CTRL_GEAR_RATIO:
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
							u32iRobConfigFsm = ePARAM_COMMIT;
						}
						break;
						
						case ePARAM_COMMIT:
						{
							
							
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
					if(u32iRobConfigFsm != ePARAM_COMMIT){
						u32iRobEthFsm = eIROB_STATE_CONFIGURE;
						
					}else{
						// Reset the config fsm back to start just in case for the next re-config
						u32iRobConfigFsm = ePARAM_CTRL_KPIDFF;
						u32iRobEthFsm = eIROB_STATE_RUN;
					}
					
				}
				break;
				
				case eIROB_STATE_RUN:
				{
					
					
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
