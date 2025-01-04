// iRob ROS2 hardware interface code
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> 

struct termios tty;

#include <rclcpp/rclcpp.hpp>

// Odometry
#include <nav_msgs/msg/odometry.hpp>

// Geometry lib
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>


// Sensor messages
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Motor messages
#include "irob_msgs/msg/irob_motor_msg.hpp"

// Define Feedback Loop time 
#define LOOP_TIME_MIL   20 // 20 millisec -> 50Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

//#define DPS_TO_RADS         (0.017453293f)

// Communication related
#define ROS_COMM_RX_SIZE		32
#define ROS_COMM_TX_SIZE		13

#define ROS_COMM_TIMEOUT		5 	// consider communication timeout when 5 cycle elapsed (100ms)

#define VEL_CMD_TIMEOUT			5 	// cmd vel timeout when 5 cycle elapsed (100ms)

typedef struct __attribute__((packed)){
	// Send by PC
	uint8_t rbcHeader[2];// 'R' and 'B'
	
	union{
		uint8_t reg;
		struct{
			uint8_t address		:5;
			uint8_t type		:2;
			uint8_t rw			:1;
		}regBit;		
	};
	
	uint8_t CTK;
	
	struct{
		int16_t motor1_ctrl;
		int16_t motor2_ctrl;
		int16_t motor3_ctrl;
		int16_t motor4_ctrl;
	}motorControl;
	
	uint8_t cmdDataPC;
	
	// Receive from MCU
	uint8_t ajbHeader[2];// 'J' and 'B'
	
	uint8_t cmdDataMCU;

	struct{
		int16_t motor1_fb;
		int16_t motor2_fb;
		int16_t motor3_fb;
		int16_t motor4_fb;
	}motorFeedBack;
	
	struct{
		int8_t mouse_x_vel;
		int8_t mouse_y_vel;
	}mouseVel;
	
	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;
	
	int16_t mag_x_raw;
	int16_t mag_y_raw;
	int16_t mag_z_raw;
	
	int16_t acc_x_raw;
	int16_t acc_y_raw;
	int16_t acc_z_raw;
	
	uint8_t cks;
}ros_rbc_ioPacket_t;


class irob_rbc_if : public rclcpp::Node{
	
	public:
	
	// Communication related
	int serial_port = 0;
	int irob_runner_fsm = 0;
	unsigned long ros_comm_timeout_counter = 0;
	unsigned long cmd_vel_timeout_counter = 0;
	
	enum IROB_RUNNER_FSM_ENUM{
		IROB_RUN_STATE_INIT = 0,
		IROB_RUN_STATE_RUN
	};

	ros_rbc_ioPacket_t rbc_Packet_t;
	
	// Odometry publisher
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOpticalOdom;

	// Sensor publishers
	rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr 	pubMagSensor;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr				pubIMUSensor;
	std::string magTopicName;
	std::string imuTopicName;

	// Motor publisher -> Send Encoder count to iRob_controller
	rclcpp::Publisher<irob_msgs::msg::IrobMotorMsg>::SharedPtr			pubMotorRPM;
	// Motor subscriber -> Receive Motor command from iRob_controller
	rclcpp::Subscription<irob_msgs::msg::IrobMotorMsg>::SharedPtr		subMotorCmd;

	// Used in wall timer callback
	rclcpp::TimerBase::SharedPtr timer_;
	
	// ROS frame of reference names
	std::string robot_frame_id;
	std::string odom_optical_frame_id;
	std::string imu_frame_id;

	// Serial port name '/dev/blablabla'
	std::string serial_port_;
	
	// Parameters for data conversion
	float mouse_x_const;
	float mouse_y_const;
	
	float gyro_to_radPerSec;
	
	float mag_to_tesla;
	
	float accel_to_meterPerSecSqr;

	
	irob_rbc_if() : Node("iRob_Interface"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club Engineering KMITL : Starting iRob hardware interface..."
			);
		
		declare_parameter("serial_port", "/dev/ttyUSB0");
		get_parameter("serial_port", serial_port_);
		
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		
		declare_parameter("mouse_odom_frame_id", "mouse_odom");
		get_parameter("mouse_odom_frame_id", odom_optical_frame_id);

		declare_parameter("imu_frame_id", "imu_link");
		get_parameter("imu_frame_id", imu_frame_id);

		declare_parameter("mag_sensor_topic", "imu/mag");
		get_parameter("mag_sensor_topic", magTopicName);
		
		declare_parameter("imu_sensor_topic", "imu/data_raw");
		get_parameter("imu_sensor_topic", imuTopicName);
		
		declare_parameter("mouse_x_constant", 0.0);
		get_parameter("mouse_x_constant", mouse_x_const);
		
		declare_parameter("mouse_y_constant", 0.0);
		get_parameter("mouse_y_constant", mouse_y_const);
		
		declare_parameter("gyro_constant", 0.0);
		get_parameter("gyro_constant", gyro_to_radPerSec);
		
		declare_parameter("mag_constant", 0.0);
		get_parameter("mag_constant", mag_to_tesla);
		
		declare_parameter("accel_constant", 0.0);
		get_parameter("accel_constant", accel_to_meterPerSecSqr);
		
		
		char *serial_port_file = new char[serial_port_.length() + 1];
		strcpy(serial_port_file, serial_port_.c_str());
		serial_port = open(serial_port_file, O_RDWR);
		// Can't open serial port
		if(serial_port < -1){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error openning Serial %s", 
				serial_port_.c_str()
				);
			std::raise(SIGTERM);
			return;
		}
		
		if(tcgetattr(serial_port, &tty) != 0){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error %i from tcgetattr: %s\n", 
				errno, 
				strerror(errno)
				);
			std::raise(SIGTERM);
			return;			
		}
		
		
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
		tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
		tty.c_cflag |= CS8; // 8 bits per byte (most common)
		tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		tty.c_lflag &= ~ICANON;
		tty.c_lflag &= ~ECHO; // Disable echo
		tty.c_lflag &= ~ECHOE; // Disable erasure
		tty.c_lflag &= ~ECHONL; // Disable new-line echo
		tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
		tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

		tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
		// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
		// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

		tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 0;

		// Set in/out baud rate to be 230400
		cfsetispeed(&tty, B230400);
		cfsetospeed(&tty, B230400);
		
		if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Error %i from tcsetattr: %s\n", 
				errno, 
				strerror(errno));
			std::raise(SIGTERM);
			return;
		}
	
		// Odometry publisher
	
		pubOpticalOdom = 
			create_publisher<nav_msgs::msg::Odometry>("/mouse_odom", 10);
			
		// Sensor publishers
		
		pubMagSensor = 
			create_publisher<sensor_msgs::msg::MagneticField>(magTopicName, 10);
		
		pubIMUSensor =
			create_publisher<sensor_msgs::msg::Imu>(imuTopicName, 10);
		
		// Motor feedback publisher
		pubMotorRPM =
			create_publisher<irob_msgs::msg::IrobMotorMsg>("/irob_motor_feedback", 10);
			
		// Motor cmd subscriber
		subMotorCmd = 
			create_subscription<irob_msgs::msg::IrobMotorMsg>(
				"/irob_motor_cmd",
				10,
				std::bind(
					&irob_rbc_if::irob_motorcmd_callback,
					this,
					std::placeholders::_1)
			);
		
		// Timer callback
		timer_ = 
			this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&irob_rbc_if::irob_interface_runner, 
					this)
			);
			
		tcflush(serial_port,TCIOFLUSH);// Flush serial buffer before start
		
		RCLCPP_INFO(this->get_logger(), "iRob hardware interface started!");
		
	}
	
	void irob_motorcmd_callback(const irob_msgs::msg::IrobMotorMsg::SharedPtr motor_cmd){
		rbc_Packet_t.motorControl.motor1_ctrl = motor_cmd->motor1;
		rbc_Packet_t.motorControl.motor2_ctrl = motor_cmd->motor2;
		rbc_Packet_t.motorControl.motor3_ctrl = motor_cmd->motor3;
		rbc_Packet_t.motorControl.motor4_ctrl = motor_cmd->motor4;
		cmd_vel_timeout_counter = 0;
	}

	void irob_flushSerial(){
		tcflush(
			serial_port,
			TCIOFLUSH
		);
		tcflush(
			serial_port,
			TCIOFLUSH
		);
	}

	int rx_bytes = 0;
	int idx = 0;
	
	void irob_interface_runner(){
		ros_comm_timeout_counter++;
		cmd_vel_timeout_counter++;
		
		if(ros_comm_timeout_counter > ROS_COMM_TIMEOUT){
			RCLCPP_ERROR(
				this->get_logger(),
				"iRob connection timed out - retrying..."
			);
			ros_comm_timeout_counter = 0;
			irob_runner_fsm = IROB_RUN_STATE_INIT;
		}
		
		if(cmd_vel_timeout_counter > VEL_CMD_TIMEOUT){
			RCLCPP_DEBUG(
				this->get_logger(),
				"iRob velocity command timed out. Robot stopped for safety"
			);
			
			rbc_Packet_t.motorControl.motor1_ctrl = 0.0;
			rbc_Packet_t.motorControl.motor2_ctrl = 0.0;
			rbc_Packet_t.motorControl.motor3_ctrl = 0.0;
			rbc_Packet_t.motorControl.motor4_ctrl = 0.0;
			
			cmd_vel_timeout_counter = 0;
		}
		
		switch(irob_runner_fsm){
			case IROB_RUN_STATE_INIT:
			{
				irob_command_tx();
				irob_runner_fsm = IROB_RUN_STATE_RUN;
			}
			break;
			
			case IROB_RUN_STATE_RUN:
			{
				ioctl(
					serial_port,
					FIONREAD,
					&rx_bytes
					);
					
				if(rx_bytes < ROS_COMM_RX_SIZE)
					break;
				
				if(rx_bytes > ROS_COMM_RX_SIZE)
					rx_bytes = ROS_COMM_RX_SIZE;
				
				ros_comm_timeout_counter = 0;
				
				read(
					serial_port,
					((unsigned char *)&rbc_Packet_t) + ROS_COMM_TX_SIZE,
					rx_bytes
					);
				
				if(
					(rbc_Packet_t.ajbHeader[0] == 'J') &&
					(rbc_Packet_t.ajbHeader[1] == 'B')
				){
					irob_decode_rx();
				}else{
					irob_flushSerial();
				}

				// Send next command to decode on next control cycle
				irob_command_tx();
			}
			break;
			
		}
		
	}
	
	void irob_command_tx(){
		
		rbc_Packet_t.rbcHeader[0] = 'R';
		rbc_Packet_t.rbcHeader[1] = 'B';
		
		rbc_Packet_t.reg 	= 0x00;
		rbc_Packet_t.CTK	= 0x00;
		
		rbc_Packet_t.cmdDataPC	= 0x00;
		
		irob_flushSerial();
		
		write(
			serial_port,
			(unsigned char *)&rbc_Packet_t,
			ROS_COMM_TX_SIZE	// Master write reg to iRob MCU
			);
	}
	
	
	void irob_decode_rx(){
		auto magMsg = sensor_msgs::msg::MagneticField();
		auto imuMsg = sensor_msgs::msg::Imu();
		
		auto motorFBMsg = irob_msgs::msg::IrobMotorMsg();
		
		nav_msgs::msg::Odometry mouseOdomMsg;
		
		// Mouse sensor odometry
		mouseOdomMsg.header.frame_id 		= odom_optical_frame_id;
		mouseOdomMsg.child_frame_id			= robot_frame_id;
		mouseOdomMsg.header.stamp			= this->get_clock()->now();
		mouseOdomMsg.twist.twist.linear.x	= 
			rbc_Packet_t.mouseVel.mouse_x_vel * mouse_x_const;
		mouseOdomMsg.twist.twist.linear.y	= 
			rbc_Packet_t.mouseVel.mouse_y_vel * mouse_y_const;
		
		// Motor RPM feedback
		motorFBMsg.motor1 					= rbc_Packet_t.motorFeedBack.motor1_fb;
		motorFBMsg.motor2					= rbc_Packet_t.motorFeedBack.motor2_fb;
		motorFBMsg.motor3					= rbc_Packet_t.motorFeedBack.motor3_fb;
		motorFBMsg.motor4 					= rbc_Packet_t.motorFeedBack.motor4_fb;
		
		// magnetometer sensor
		magMsg.header.frame_id				= imu_frame_id;
		magMsg.header.stamp 				= mouseOdomMsg.header.stamp;
		magMsg.magnetic_field.x				= 
			rbc_Packet_t.mag_x_raw * mag_to_tesla;
		magMsg.magnetic_field.y				= 
			rbc_Packet_t.mag_y_raw * mag_to_tesla;
		magMsg.magnetic_field.z				= 
			rbc_Packet_t.mag_z_raw * mag_to_tesla;
		
		// Accel + Gyro sensor
		imuMsg.header.frame_id				= imu_frame_id;
		imuMsg.header.stamp					= mouseOdomMsg.header.stamp;
		// Accel
		imuMsg.linear_acceleration_covariance = {0};
		imuMsg.linear_acceleration.x 		= 
			rbc_Packet_t.acc_x_raw * accel_to_meterPerSecSqr;
		imuMsg.linear_acceleration.y		= 
			rbc_Packet_t.acc_y_raw * accel_to_meterPerSecSqr;
		imuMsg.linear_acceleration.z		= 
			rbc_Packet_t.acc_z_raw * accel_to_meterPerSecSqr;
		// Gyro
		imuMsg.angular_velocity_covariance[0] = {0};
		imuMsg.angular_velocity.x 			= 
			rbc_Packet_t.gyro_x_raw * gyro_to_radPerSec;
		imuMsg.angular_velocity.y			= 
			rbc_Packet_t.gyro_y_raw * gyro_to_radPerSec;
		imuMsg.angular_velocity.z			= 
			rbc_Packet_t.gyro_z_raw * gyro_to_radPerSec;
		
		
		// Publish all message
		pubOpticalOdom->publish(mouseOdomMsg);
		pubMagSensor->publish(magMsg);
		pubIMUSensor->publish(imuMsg);
		pubMotorRPM->publish(motorFBMsg);
	
	}
	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto irobIF {std::make_shared<irob_rbc_if>()};
	rclcpp::spin(irobIF);
	close(irobIF->serial_port);
	rclcpp::shutdown();
}
