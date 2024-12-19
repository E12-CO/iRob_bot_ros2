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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

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
	
	// ROS fram of reference names
	std::string map_frame_id;
	std::string robot_frame_id;
	
	// Pub
	// cmd_vel twist publisher
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 		pubMotion;
	// buffer to publish twist message
	geometry_msgs::msg::Twist twist;
	
	// iRob status message
	rclcpp::Publisher<irob_msgs::msg::IrobCmdMsg>::SharedPtr		pubiRobStat;
	
	// Sub
	// Pose stamped command
	rclcpp::Subscription<geometry::msgs::PoseStamped>::SharedPtr	subPoseStamped;
	
	// iRob mode message
	rclcpp::Subscription<irob_msgs::msg::IrobCmdMsg>::SharedPtr		subiRobCmd;
	// Buffer for receive messages
	std::string irob_cmd;
	
	
	// Wall timer for PID control loop 50Hz
	rclcpp::TimerBase::SharedPtr timer_;
	
	// tf2 related
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	
	
	uint8_t loop_fsm = 0;
	
	// buffer for Setpoint message (PoseStamped stripped down to Stamped)0
	geometry::msgs::Pose poseSetpoint;
	
	double cVel, cHeading, cVelAz;
	
	// buffer fot transform stamped to be used as position feedback
	geometry_msgs::msg::TransformStamped poseFeedback;
	
	double rRoll, rPitch, rYaw;
	
	double diff_x, diff_y;
	
	double eDist;
	
	// PID for walk
	double walkKp;
	double walkKi;
	double walkKd;
	double walkMax;
	
	double walkIntg;
	double walkDiff;

	double preveDist;
	
	// PID for rotate
	double rotateKp;
	double rotateKi;
	double rotateKd;
	double rotateMax;
	
	double rotateIntg;
	
	irob_rbc_if() : Node("iRob_Interface"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club Engineering KMITL : Starting iRob maneuv3r..."
			);
		
		declare_parameter("map_frame_id", "map");
		get_parameter("map_frame_id", map_frame_id);
		
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		
		declare_parameter();
		get_parameter();
		
		
		tf_buffer_ =
			std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ =
			std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		
		pubMotion = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_irob_auto", 10);
		
		pubiRobStat = create_publisher<irob_msgs::msg::IrobCmdMsg>("/irob_maneuv3r_status", 10);
		
		subPoseStamped = 
			create_subscription<geometry::msgs::PoseStamped>(
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
	
	void irob_pose_callback(const geometry::msgs::PoseStamped::SharedPtr pose_command){
		poseSetpoint.position.x 	= pose_command->position.x;
		poseSetpoint.position.y 	= pose_command->position.y;
		poseSetpoint.orientation.x 	= pose_command->orientation.x;
		poseSetpoint.orientation.y 	= pose_command->orientation.y;
		poseSetpoint.orientation.z 	= pose_command->orientation.z;
		poseSetpoint.orientation.w 	= pose_command->orientation.w;
	}
	
	void irob_cmd_callback(const irob_msgs::msg::IrobCMdMsg::SharedPtr irob_command){
		irob_cmd = irob_command->irob_cmd;
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
				geometry_msgs::msg::Quaternion quat_msg = poseFeedback.transform.rotation;
				tf2::fromMsg(quat_msg, quat_tf);
				tf2::Matrix3x3 m(quat_tf);
				m.getRPY(
					rRoll,
					rPitch,
					rYaw	// <---- We only use this
				);
				
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
				
				walkIntg += eDist * walkKi;
				
				walkDiff = (eDist - preveDist) * walkKd;
				preveDist = eDist;
				
				cVel = 
					(eDist * walkKp) 	+
					walkIntg			+
					walkDiff			;
					
					
				
				maneuv3r_update_Cmdvel(
					cVel,
					cHeading,
					cVelAz
					);
			}
			break;
		}
		
		
		twistout->publish(twist);
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