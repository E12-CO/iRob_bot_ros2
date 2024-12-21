// iRob ROS2 maneuv3r position controller
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

// ROS2 RCLCPP
#include <rclcpp/rclcpp.hpp>

// Odometry
#include <nav_msgs/msg/odometry.hpp>

// Geometry lib
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

// tf2 lib
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include "tf2/utils.h"

// Sensor messages
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"

// iRob command message
#include "irob_msgs/msg/irob_cmd_msg.hpp"

// Define Feedback Loop time 
#define LOOP_TIME_MIL   20 // 20 millisec -> 50Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

class irob_rbc_maneuv3r : public rclcpp::Node{
	
	public:
	
	// Pub
	// cmd_vel twist publisher
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 		pubMotion;
	// buffer to publish twist message
	geometry_msgs::msg::Twist twist;
	
	// iRob status message
	rclcpp::Publisher<irob_msgs::msg::IrobCmdMsg>::SharedPtr		pubiRobStat;
	
	// Sub
	// Pose stamped command
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr	subPoseStamped;
	// buffer for Setpoint message (PoseStamped stripped down to Stamped)0
	geometry_msgs::msg::Pose poseSetpoint;
	
	// iRob mode message
	rclcpp::Subscription<irob_msgs::msg::IrobCmdMsg>::SharedPtr		subiRobCmd;
	// Buffer for receive messages
	std::string irob_cmd;
	
	
	// Wall timer for PID control loop 50Hz
	rclcpp::TimerBase::SharedPtr timer_;
	
	// tf2 related
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	
	// ROS fram of reference names
	std::string map_frame_id;
	std::string robot_frame_id;
	std::string odom_frame_id;
	
	// Use odom transform instead of map transform
	bool use_odom_tf_only = false;
	
	// Goal tolerance 
	double walk_goal_tolerance;
	double rotate_goal_tolerance;
	
	uint8_t loop_fsm = 0;
	
	
	double cVel, cHeading, cVelAz;
	
	// buffer for transform stamped to be used as position feedback
	geometry_msgs::msg::TransformStamped poseFeedback;
	
	double spYaw, fYaw;
	
	double diff_x, diff_y;
	
	// PID for walk
	double eDist;
	
	double walkKp;
	double walkKi;
	double walkKd;
	double walkMax;
	
	double walkIntg;
	double walkDiff;

	double prevDist;
	
	// PID for rotate
	double eOrient;
	
	double rotateKp;
	double rotateKi;
	double rotateKd;
	double rotateMax;
	
	double rotateIntg;
	double rotateDiff;
	
	double prevOrient;
	
	irob_rbc_maneuv3r() : Node("iRob_maneuv3r"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club Engineering KMITL : Starting iRob maneuv3r..."
			);
		
		declare_parameter("map_frame_id", "map");
		get_parameter("map_frame_id", map_frame_id);
		
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		
		declare_parameter("odom_frame_id", "odom");
		get_parameter("odom_frame_id", odom_frame_id);
		
		declare_parameter("use_odom_tf_only", false);
		get_parameter("use_odom_tf_only", use_odom_tf_only);
		
		declare_parameter("walk_kp", 1.0);
		get_parameter("walk_kp", walkKp);
		declare_parameter("walk_ki", 0.0);
		get_parameter("walk_ki", walkKi);
		declare_parameter("walk_kd", 0.0);
		get_parameter("walk_kd", walkKd);
		declare_parameter("walk_max_vel", 1.0);
		get_parameter("walk_max_vel", walkMax);
		declare_parameter("walk_goal_tolerance", 0.01);
		get_parameter("walk_goal_tolerance", walk_goal_tolerance);
		
		declare_parameter("rotate_kp", 1.0);
		get_parameter("rotate_kp", rotateKp);
		declare_parameter("rotate_ki", 0.0);
		get_parameter("rotate_ki", rotateKi);
		declare_parameter("rotate_kd", 0.0);
		get_parameter("rotate_kd", rotateKd);
		declare_parameter("rotate_max_vel", 3.1415);
		get_parameter("rotate_max_vel", rotateMax);
		declare_parameter("rotate_goal_tolerance", 0.174533);// Default 10 degree
		get_parameter("rotate_goal_tolerance", rotate_goal_tolerance);
		
		if(use_odom_tf_only == true)
			map_frame_id = odom_frame_id;
		
		
		
		tf_buffer_ =
			std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ =
			std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		
		pubMotion = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_irob_auto", 10);
		
		pubiRobStat = create_publisher<irob_msgs::msg::IrobCmdMsg>("/irob_maneuv3r_status", 10);
		
		subPoseStamped = 
			create_subscription<geometry_msgs::msg::PoseStamped>(
				"/irob_pose_cmd",
				10,
				std::bind(
					&irob_rbc_maneuv3r::irob_pose_callback,
					this,
					std::placeholders::_1)
			);
		
		subiRobCmd =
			create_subscription<irob_msgs::msg::IrobCmdMsg>(
				"/irob_cmd",
				10,
				std::bind(
					&irob_rbc_maneuv3r::irob_cmd_callback,
					this,
					std::placeholders::_1)
			);
			
		timer_ = 
			this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&irob_rbc_maneuv3r::irob_loop_runner, 
					this)
			);
	
		RCLCPP_INFO(this->get_logger(), "iRob maneuv3r started!");
	
	}
	
	void irob_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_command){
		poseSetpoint.position.x 	= pose_command->pose.position.x;
		poseSetpoint.position.y 	= pose_command->pose.position.y;
		poseSetpoint.orientation.x 	= pose_command->pose.orientation.x;
		poseSetpoint.orientation.y 	= pose_command->pose.orientation.y;
		poseSetpoint.orientation.z 	= pose_command->pose.orientation.z;
		poseSetpoint.orientation.w 	= pose_command->pose.orientation.w;
	}
	
	void irob_cmd_callback(const irob_msgs::msg::IrobCmdMsg::SharedPtr irob_command){
		irob_cmd = irob_command->irobcmd;
		if(irob_cmd == "stop"){
			loop_fsm = 0;
		}
	}
	
	void irob_loop_runner(){
		
		switch(loop_fsm){
			case 0:// Idle case, wait for start command from /irob_cmd
			{
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
				twist.angular.z = 0.0;
				walkIntg = 0.0;
				rotateIntg = 0.0;
				if(irob_cmd == "run")
					loop_fsm = 1;
			}
			break;
			
			case 1:// PID loop inside here
			{
				// Get current robot position by looking up transform
				try {
						
					poseFeedback = 
						tf_buffer_->lookupTransform(
							robot_frame_id, map_frame_id,
							tf2::TimePointZero
						);
				} catch (const tf2::TransformException & ex) {
				  RCLCPP_INFO(
					this->get_logger(), 
					"Could not transform %s to %s: %s",
					robot_frame_id.c_str(), 
					map_frame_id.c_str(), 
					ex.what()
					);
				  return;
				}
				
				// Convert Quaternion to RPY to get Yaw (robot orientation)
				tf2::Quaternion quat_tf;
				//geometry_msgs::msg::Quaternion quat_msg = poseFeedback.transform.rotation;
				//tf2::fromMsg(quat_msg, quat_tf);
				tf2::fromMsg(poseFeedback.transform.rotation, quat_tf);
				//tf2::Matrix3x3 m(quat_tf);
				fYaw = tf2::getYaw(quat_tf);
				/*
				m.getRPY(
					fRoll,
					fPitch,
					fYaw	// <---- We only use this
				);*/
				
				tf2::fromMsg(poseSetpoint.orientation, quat_tf);
				spYaw = tf2::getYaw(quat_tf);
				
				// Calculate Euclidean distance
				// Dx and Dy
				diff_x = poseSetpoint.position.x - poseFeedback.transform.translation.x;
				diff_y = poseSetpoint.position.y - poseFeedback.transform.translation.y;
				
				// Calculate Heading
				cHeading = 
					atan2pi(
						diff_y,
						diff_x
					);
						
				// Dx and Dy squared		
				diff_x = diff_x * diff_x;
				diff_y = diff_y * diff_y;
				// Finally calculate the Euclidean distance
				eDist = sqrt(diff_x + diff_y);
				
				// Walk PID controller 
				
				walkIntg += eDist * walkKi;
				
				walkDiff = (eDist - prevDist) * walkKd;
				prevDist = eDist;
				
				cVel = 
					(eDist * walkKp) 	+
					walkIntg			+
					walkDiff			;
				
				// Linear velocity envelope
				if(cVel > walkMax)
					cVel = walkMax;
				if(cVel < -walkMax)
					cVel = -walkMax;
				
				// Rotate PID controller
				eOrient = 0 - fYaw;			// TODO : GET SETPOINT IN RPY 
				
				rotateIntg += eOrient * rotateKi;
				
				rotateDiff = (eOrient - prevOrient) * rotateKd;
				prevOrient = eOrient;
				
				cVelAz = 
					(eOrient * rotateKp)+
					rotateIntg			+
					rotateDiff			;
				
				
				// Angular velocity envelope
				if(cVelAz > rotateMax)
					cVelAz = rotateMax;
				if(cVelAz < -rotateMax)
					cVelAz = -rotateMax;
				
				// Goal checker 
				if(abs(cVel) <= walk_goal_tolerance){
					cVel = 0.0;
					walkIntg = 0.0;
				}
				
				if(abs(cVelAz) <= rotate_goal_tolerance){
					cVelAz = 0.0;
					rotateIntg = 0.0;
				}
				
				if(
					(abs(cVel) <= walk_goal_tolerance) &&
					(abs(cVelAz) <= rotate_goal_tolerance)
				){
					loop_fsm = 0;
				}
				
				maneuv3r_update_Cmdvel(
					cVel,
					cHeading,
					cVelAz
					);
			}
			break;
		}
		
		
		pubMotion->publish(twist);
	}
	
	private:
	
	double atan2pi(double y, double x) {
	  double at = atan2(y, x);
	  if (at < 0.0)
		at += 6.28319;
	  return at;
	}
	
	void maneuv3r_update_Cmdvel(
		double vel, 
		double heading, 
		double az) {
			
	  // Convert the R and Theta (polar coordinates) into X and Y velocity component (Cartesian coordinates).
	  twist.linear.x 	= vel * cos(heading);
	  twist.linear.y 	= vel * sin(heading);

	  // Commading angular velocity.
	  twist.angular.z 	= az;
	}

	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto irobPOSE {std::make_shared<irob_rbc_maneuv3r>()};
	rclcpp::spin(irobPOSE);
	rclcpp::shutdown();
}