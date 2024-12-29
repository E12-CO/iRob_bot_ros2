// iRob trajectory logging server with CSV saving function (soon).
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

// ROS2 RCLCPP
#include <rclcpp/rclcpp.hpp>

// Odometry
#include <nav_msgs/msg/path.hpp>

// Geometry lib
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

// tf2 lib
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include "tf2/utils.h"

// Define Feedback Loop time 
#define LOOP_TIME_MIL   200 // 200 millisec -> 5Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

class irob_traj_server : public rclcpp::Node{
	
	public:
	
	// Path publisher 
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr			pubPath;
	nav_msgs::msg::Path	pathMsg;
	
	// Wall timer for pose sampling
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
	
	// Update threshold
	double dist_update_threshold;
	double ang_update_threshold;
	
	// buffer for transform stamped to be used as position feedback
	geometry_msgs::msg::TransformStamped poseFeedback;
	
	// buffer to copy from transform stamped to PoseStamped
	geometry_msgs::msg::PoseStamped 	 poseCurrent;
	
	tf2::Quaternion quat_tf;
	
	double fYaw;
	double prevYaw;
	double diffOrient;
	
	double prevX, prevY, diffX, diffY;
	
	double diffDist;
	
	irob_traj_server() : Node("iRob_trajectory_server"){
		RCLCPP_INFO(
			this->get_logger(),
			"Robot Club Engineering KMITL : Starting iRob trajectory server..."
			);
		
		declare_parameter("map_frame_id", "map");
		get_parameter("map_frame_id", map_frame_id);
		
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		
		declare_parameter("odom_frame_id", "odom");
		get_parameter("odom_frame_id", odom_frame_id);
		
		declare_parameter("use_odom_tf_only", false);
		get_parameter("use_odom_tf_only", use_odom_tf_only);
		
		declare_parameter("distance_update_threshold", 0.05);// Update every 0.05 meter
		get_parameter("distance_update_threshold", dist_update_threshold);
		declare_parameter("angle_update_threshold", 0.174533);// Update every 10 degree
		get_parameter("angle_update_threshold", ang_update_threshold);
		
		if(use_odom_tf_only == true)
				map_frame_id = odom_frame_id;
			
		// Set up transform buffer
		tf_buffer_ =
			std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ =
			std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		
		// Set up path publisher
		pubPath = 
			create_publisher<nav_msgs::msg::Path>(
				"/irob_trajectory",
				10
			);
		
		// Set the header frane_id of the Path message
		pathMsg.header.frame_id = map_frame_id;
		
		// Set up wall timer
		timer_ = 
			this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&irob_traj_server::irob_trajectoryUpdate,
					this)
			);	
		
		RCLCPP_INFO(this->get_logger(), "iRob trajectory server started!");
	}
	
	void irob_trajectoryUpdate(){
		try{	
			poseFeedback = 
				tf_buffer_->lookupTransform(
					map_frame_id, robot_frame_id,
					tf2::TimePointZero,
					tf2::Duration(1)				// tf lookup timeout 1 sec
				);
			
		}catch(const tf2::TransformException & ex){
			RCLCPP_WARN(
				this->get_logger(), 
				"Could not transform %s to %s: %s Path won't be updated naja!",
				robot_frame_id.c_str(), 
				map_frame_id.c_str(), 
				ex.what()
			);
			return ;
		}
		
		// Get Yaw orientation from Pose feedback
		tf2::fromMsg(
			poseFeedback.transform.rotation, 
			quat_tf
			);
		fYaw = tf2::getYaw(quat_tf);

		// Calculate delta position
		diffX = poseFeedback.transform.translation.x - prevX;
		diffY = poseFeedback.transform.translation.y - prevY;
		
		diffX = diffX * diffX;
		diffY = diffY * diffY;
		
		diffDist = sqrt(diffX + diffY);
		
		// Calculate delta orientation
		diffOrient = fYaw - prevYaw;
		prevYaw = fYaw;
		
		// Update Postition and/or Orientation
		if(
		(diffDist > dist_update_threshold)	||
		(diffOrient > ang_update_threshold)
		){
			// Copy TransformStamped to PoseStamped
			poseCurrent.header 	=  
				poseFeedback.header;
				
			poseCurrent.pose.position.x =
				poseFeedback.transform.translation.x;
			poseCurrent.pose.position.y =
				poseFeedback.transform.translation.y;
			poseCurrent.pose.position.z =
				poseFeedback.transform.translation.z;
			poseCurrent.pose.orientation.x =
				poseFeedback.transform.rotation.x;
			poseCurrent.pose.orientation.y =
				poseFeedback.transform.rotation.y;
			poseCurrent.pose.orientation.z =
				poseFeedback.transform.rotation.z;
			poseCurrent.pose.orientation.w =
				poseFeedback.transform.rotation.w;
			
			pathMsg.poses.push_back(poseCurrent);
			pubPath->publish(pathMsg);
		}
	}
	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto irobTRAJ {std::make_shared<irob_traj_server>()};
	rclcpp::spin(irobTRAJ);
	rclcpp::shutdown();
}