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
#include "sensor_msgs/msg/joint_state.hpp"

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
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr			pubMotorCmd;
	// Motor command message
	sensor_msgs::msg::JointState motorCmdMsg;
	
	// Motor subscriber -> Receive Encoder count from iRob_interface
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr		subMotorRPM;

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
					
		declare_parameter("irob_holo_type", "irob_holonomic_plugins::Omni3");
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
		
		irob_holo_plugin = 
			irob_holoLoader.createSharedInstance(
				robot_irob_param_t.irob_robot_type
			);
		
		RCLCPP_INFO(
			this->get_logger(),
			"Using plugin : %s",
			robot_irob_param_t.irob_robot_type.c_str()
		);
		
		if(irob_holo_plugin->init_iRob(&robot_irob_param_t) < 0){
			RCLCPP_ERROR(
				this->get_logger(),
				"iRob Holonomic initialize error!"
			);
			return;
		}
		
		// Odometry publisher
		pubWheelOdom =
			create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
		
		// Motor cmd publisher
		pubMotorCmd = 
			create_publisher<sensor_msgs::msg::JointState>("irob_motor_cmd", 1);
		
		// the iRob_interaface is using the frame id to identify the motor command.
		motorCmdMsg.header.frame_id = "quad_motor";
		// resize the velocity vector to 4
		motorCmdMsg.velocity.resize(4);
		
		// Motor feedback subscriber
		subMotorRPM = 
			create_subscription<sensor_msgs::msg::JointState>(
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
	
	void irob_motorfeedback_callback(const sensor_msgs::msg::JointState::SharedPtr motor_feedback){
		
		if(motor_feedback->header.frame_id != "quad_motor"){
			RCLCPP_ERROR(
				this->get_logger(),
				"Received joint feedback with wrong frame name of \"%s\"",
				motor_feedback->header.frame_id
			);
			
			return;
		}
		
		wheel_fbvel_t.v1 = motor_feedback->velocity[0];
		wheel_fbvel_t.v2 = motor_feedback->velocity[1];
		wheel_fbvel_t.v3 = motor_feedback->velocity[2];
		wheel_fbvel_t.v4 = motor_feedback->velocity[3];
		
		// Forward kinematic
		// Wheel velocity to XY and AZ
		
		irob_holo_plugin->forward_kinematic(
			&fbvel_t,
			&wheel_fbvel_t
		);
		
		// Forward euler integration (1st order integration)
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
		
		motorCmdMsg.header.stamp 	= this->get_clock()->now();
		motorCmdMsg.velocity[0] = round(wheel_cmdvel_t.v1); 
		motorCmdMsg.velocity[1] = round(wheel_cmdvel_t.v2);
		motorCmdMsg.velocity[2] = round(wheel_cmdvel_t.v3);
		motorCmdMsg.velocity[3] = round(wheel_cmdvel_t.v4);
		
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