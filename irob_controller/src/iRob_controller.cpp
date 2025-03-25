// iRob kinematic controller node
// This part bridge between iRob_interface and iRob_holonomic plug-ins
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

// tf2 library
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

// Odometry
#include <nav_msgs/msg/odometry.hpp>

// Geometry lib
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Motor messages
#include "irob_msgs/msg/irob_motor_msg.hpp"

// iRob holonimic plug-in
#include <pluginlib/class_loader.hpp>
#include "iRob_controller/iRob_holonomic.hpp"

#define LOOP_TIME_MIL   20 // 20 millisec - 50Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

class irob_rbc_ctrl : public rclcpp::Node{
	public:
	
	// Odometry publisher
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubWheelOdom;
	
	// twist message subscriber
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subMotion;
	
	// Motor publisher -> Send to iRob_interface
	rclcpp::Publisher<irob_msgs::msg::IrobMotorMsg>::SharedPtr			pubMotorCmd;
	// Motor command message
	irob_msgs::msg::IrobMotorMsg motorCmdMsg;
	
	
	// Motor subscriber -> Receive Encoder count from iRob_interface
	rclcpp::Subscription<irob_msgs::msg::IrobMotorMsg>::SharedPtr		subMotorRPM;
	
	
	// Used in wall timer callback
	rclcpp::TimerBase::SharedPtr timer_;
	
	std::unique_ptr<tf2_ros::TransformBroadcaster> br;
	
	// iRob holonomic
	irob_robot_param_t	robot_irob_param_t;
	cmd_vel_t 			cmdvel_t;
	cmd_vel_t			fbvel_t;
	wheel_vel_t			wheel_cmdvel_t;
	wheel_vel_t			wheel_fbvel_t;
	
	// Local Parameters
	std::string robot_frame_id;
    std::string odom_frame_id;
	
	float gear_ratio = 0.0f;
	
	float cur_x_pos, cur_y_pos, cur_az_ang = 0.0f;
	
	bool pub_tf;
	
	pluginlib::ClassLoader<irob_holonomic_base::irob_holonomic> irob_holoLoader;			
	std::shared_ptr<irob_holonomic_base::irob_holonomic> irob_holo_plugin;

	
	irob_rbc_ctrl() : Node("iRob_controller"),
		irob_holoLoader(
			"irob_controller",
			"irob_holonomic_base::irob_holonomic"
	){
		RCLCPP_INFO(
			this->get_logger(),
			"Robot Club Engineering KMITL : Starting iRob controller..."
		);	
		
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		
		declare_parameter("odom_frame_id", "odom");
		get_parameter("odom_frame_id", odom_frame_id);
					
		declare_parameter("irob_holo_type", (const uint8_t)IROB_HOLO_OMNI3);
		get_parameter("irob_holo_type", robot_irob_param_t.irob_robot_type);
		
		declare_parameter("robot_length", 0.01f);
		get_parameter("robot_length", robot_irob_param_t.robot_length);
		
		declare_parameter("robot_width", 0.01f);
		get_parameter("robot_length", robot_irob_param_t.robot_width);
		
		declare_parameter("wheel_radius", 0.01f);
		get_parameter("robot_length", robot_irob_param_t.robot_wheel_radius);
		
		declare_parameter("gear_ratio", 1.0f);
		get_parameter("gear_ratio", robot_irob_param_t.robot_gear_ratio);
		
		declare_parameter("publish_tf", true);
		get_parameter("publish_tf", pub_tf);
		
		switch(robot_irob_param_t.irob_robot_type){
			case IROB_HOLO_OMNI3:
				irob_holo_plugin = irob_holoLoader.createSharedInstance(
					"irob_holonomic_plugins::Omni3"
					);
				
				RCLCPP_INFO(
					this->get_logger(),
					"iRob 3 wheels omni"
				);
				
				break;
			
			case IROB_HOLO_OMNI4:
				irob_holo_plugin = irob_holoLoader.createSharedInstance(
					"irob_holonomic_plugins::Omni4"
					);
					
				RCLCPP_INFO(
					this->get_logger(),
					"iRob 4 wheels omni"
				);	
					
				break;
			
			case IROB_HOLO_MECA4:
				irob_holo_plugin = irob_holoLoader.createSharedInstance(
					"irob_holonomic_plugins::Meca4"
					);
					
				RCLCPP_INFO(
					this->get_logger(),
					"iRob 4 wheels mecanum"
				);		
					
				break;
			
			default:
				RCLCPP_ERROR(
					this->get_logger(),
					"Invalid iRob Holonomic type! : %d\n",
					robot_irob_param_t.irob_robot_type
				);
				return;
		}
		
		if(irob_holo_plugin->init_iRob(&robot_irob_param_t) < 0){
			RCLCPP_ERROR(
				this->get_logger(),
				"iRob Holonomic initializ error!"
			);
			return;
		}
		
		// Odometry publisher
		pubWheelOdom =
			create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
		
		// Motor cmd publisher
		pubMotorCmd = 
			create_publisher<irob_msgs::msg::IrobMotorMsg>("irob_motor_cmd", 1);
		
		// Motor feedback subscriber
		subMotorRPM = 
			create_subscription<irob_msgs::msg::IrobMotorMsg>(
				"irob_motor_feedback", 
				10,
				std::bind(
					&irob_rbc_ctrl::irob_motorfeedback_callback,
					this,
					std::placeholders::_1)
			);
			
		// cmd_vel twist message subscriber	
		subMotion = 
			create_subscription<geometry_msgs::msg::Twist>(
				"cmd_vel",
				10,
				std::bind(
					&irob_rbc_ctrl::motion_callback, 
					this, 
					std::placeholders::_1) 
			);
			
		// Timer callback
		timer_ =
			this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&irob_rbc_ctrl::irob_controller_runner,
					this)
			);
		
		RCLCPP_INFO(
			this->get_logger(),
			"iRob Controller started!"
		);
	
	}
	
	void irob_motorfeedback_callback(const irob_msgs::msg::IrobMotorMsg::SharedPtr motor_feedback){
		
		wheel_fbvel_t.v1 = motor_feedback->motor1;
		wheel_fbvel_t.v2 = motor_feedback->motor2;
		wheel_fbvel_t.v3 = motor_feedback->motor3;
		wheel_fbvel_t.v4 = motor_feedback->motor4;
		
		// Forward kinematic
		// Wheel velocity to XY and AZ
		
		irob_holo_plugin->forward_kinematic(
			&fbvel_t,
			&wheel_fbvel_t
		);
		
		cur_az_ang 	+= fbvel_t.vel_az * LOOP_TIME_SEC; 
		cur_x_pos 	+= ((fbvel_t.vel_x * cos(cur_az_ang)) - (fbvel_t.vel_y * sin(cur_az_ang))) * LOOP_TIME_SEC;
		cur_y_pos 	+= ((fbvel_t.vel_x * sin(cur_az_ang)) + (fbvel_t.vel_y * cos(cur_az_ang))) * LOOP_TIME_SEC;
			
		
		nav_msgs::msg::Odometry robotOdom;
		br = std::make_unique<tf2_ros::TransformBroadcaster>(this);
		geometry_msgs::msg::TransformStamped transform;

		tf2::Quaternion xyz_angular;
		xyz_angular.setRPY(0, 0, cur_az_ang);
		//xyz_angular = xyz_angular.normalize();
		
		// Publish message on Odom topic 
		robotOdom.header.frame_id 			= odom_frame_id;
		robotOdom.child_frame_id			= robot_frame_id;
		robotOdom.header.stamp 				= this->get_clock()->now();
		robotOdom.twist.twist.linear.x 		= fbvel_t.vel_x;
		robotOdom.twist.twist.linear.y		= fbvel_t.vel_y;
		robotOdom.twist.twist.angular.z 	= fbvel_t.vel_az;
		robotOdom.pose.pose.orientation.x 	= xyz_angular.getX();
		robotOdom.pose.pose.orientation.y 	= xyz_angular.getY();
		robotOdom.pose.pose.orientation.z 	= xyz_angular.getZ();
		robotOdom.pose.pose.orientation.w 	= xyz_angular.getW();
		robotOdom.pose.pose.position.x 		= cur_x_pos;
		robotOdom.pose.pose.position.y 		= cur_y_pos;
		pubWheelOdom->publish(robotOdom);

		if(pub_tf){
			// Do the Odom transform
			transform.header.stamp 				= robotOdom.header.stamp;
			transform.header.frame_id 			= odom_frame_id;
			transform.child_frame_id 			= robot_frame_id;
			transform.transform.translation.x 	= cur_x_pos;
			transform.transform.translation.y 	= cur_y_pos;
			transform.transform.rotation.x 		= round(xyz_angular.getX() * 100) / 100;
			transform.transform.rotation.y 		= round(xyz_angular.getY() * 100) / 100;
			transform.transform.rotation.z 		= round(xyz_angular.getZ() * 100) / 100;
			transform.transform.rotation.w 		= round(xyz_angular.getW() * 100) / 100;
			br->sendTransform(transform);
		}
	}
	
	void motion_callback(const geometry_msgs::msg::Twist::SharedPtr twist_data){		
		cmdvel_t.vel_x	= twist_data->linear.x;
		cmdvel_t.vel_y	= twist_data->linear.y;
		cmdvel_t.vel_az	= twist_data->angular.z;
		
		// Inverse Kinematic 
		// XY and AZ to wheel velocity
		
		irob_holo_plugin->inverse_kinematic(
			&cmdvel_t,
			&wheel_cmdvel_t
		);
		
		motorCmdMsg.motor1 = round(wheel_cmdvel_t.v1); 
		motorCmdMsg.motor2 = round(wheel_cmdvel_t.v2);
		motorCmdMsg.motor3 = round(wheel_cmdvel_t.v3);
		motorCmdMsg.motor4 = round(wheel_cmdvel_t.v4);
		
		pubMotorCmd->publish(motorCmdMsg);
	}
	
	void irob_controller_runner(){
		
		
	}
	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto irob_ctrlr {std::make_shared<irob_rbc_ctrl>()};
	rclcpp::spin(irob_ctrlr);
	rclcpp::shutdown();
}