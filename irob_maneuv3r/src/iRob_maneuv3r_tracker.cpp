// iRob ROS2 maneuv3r path tracking controller
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

// ROS2 RCLCPP
#include <rclcpp/rclcpp.hpp>

// Nav lib
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

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
#define LOOP_TIME_MIL   50 // 50 millisec -> 20Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

class irob_rbc_maneuv3r_tracker : public rclcpp::Node{
	
	public:
	
	// Pub
	// cmd_vel twist publisher
	std::string twist_topic_name;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 		pubMotion;
	// buffer to publish twist message
	geometry_msgs::msg::Twist twist;
	
	// iRob status message
	std::string irob_status_topic_name;
	rclcpp::Publisher<irob_msgs::msg::IrobCmdMsg>::SharedPtr		pubiRobStat;
	
	// Sub
	// Path command (trajectory)
	std::string path_topic_name;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr	subPath;
	// buffer for Path message
	nav_msgs::msg::Path	pathMsg;
	size_t path_Length;
	
	geometry_msgs::msg::Pose poseSetpoint;
	
	// iRob mode message
	std::string irob_cmd_topic_name;
	rclcpp::Subscription<irob_msgs::msg::IrobCmdMsg>::SharedPtr		subiRobCmd;
	// Buffer for receive messages
	std::string irob_cmd;
	
	// Wall timer for PID control loop 50Hz
	rclcpp::TimerBase::SharedPtr timer_;
	
	// tf2 related
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	
	/* PARAMETERS */
	
	// ROS fram of reference names
	std::string map_frame_id;
	std::string robot_frame_id;
	std::string odom_frame_id;
	
	// Use odom transform instead of map transform
	bool use_odom_tf_only = false;
	
	// Lookahead distance
	double lookahead_dist;
	
	// Goal tolerance 
	double walk_goal_tolerance;
	double rotate_goal_tolerance;
	
	// Max linear velocity
	double max_vel;
	
	// Velocity filter
	double walk_vel_filter;

	double heading_velocity_braking_ratio;
	int lookahead_points = 0;
	
	double walkKp;
	double walkKi;
	double walkKd;
	double walkMax;
	
	double rotateKp;
	double rotateKi;
	double rotateKd;
	double rotateMax;
	
	
	irob_rbc_maneuv3r_tracker() : Node("iRob_maneuv3r_tracker"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club Engineering KMITL : Starting iRob maneuv3r tracker..."
			);
		
		declare_parameter("map_frame_id", "map");
		get_parameter("map_frame_id", map_frame_id);
		
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		
		declare_parameter("odom_frame_id", "odom");
		get_parameter("odom_frame_id", odom_frame_id);
		
		declare_parameter("use_odom_tf_only", false);
		get_parameter("use_odom_tf_only", use_odom_tf_only);
		
		declare_parameter("lookahead_distance", 1.0);
		get_parameter("lookahead_distance", lookahead_dist);
		
		declare_parameter("brake_ratio", 1.0);
		get_parameter("brake_ratio", heading_velocity_braking_ratio);
		
		declare_parameter("walk_kp", 1.0);
		get_parameter("walk_kp", walkKp);
		declare_parameter("walk_ki", 0.0);
		get_parameter("walk_ki", walkKi);
		declare_parameter("walk_kd", 0.0);
		get_parameter("walk_kd", walkKd);
		declare_parameter("walk_max_vel", 1.0);
		get_parameter("walk_max_vel", walkMax);
		declare_parameter("walk_vel_filter", 0.4);
		get_parameter("walk_vel_filter", walk_vel_filter);
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
		
		declare_parameter("twist_topic_name", "cmd_vel_irob_auto");
		get_parameter("twist_topic_name", twist_topic_name);
		
		declare_parameter("path_topic_name", "smooth_path");
		get_parameter("path_topic_name", path_topic_name);
		
		declare_parameter("irob_status_topic_name", "irob_maneuv3r_status");
		get_parameter("irob_status_topic_name", irob_status_topic_name);
		
		declare_parameter("irob_cmd_topic_name", "irob_cmd");
		get_parameter("irob_cmd_topic_name", irob_cmd_topic_name);
		
		if(use_odom_tf_only == true)
			map_frame_id = odom_frame_id;
		
		tf_buffer_ =
			std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ =
			std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		
		pubMotion = create_publisher<geometry_msgs::msg::Twist>(twist_topic_name, 10);
		
		pubiRobStat = create_publisher<irob_msgs::msg::IrobCmdMsg>(irob_status_topic_name, 10);
		
		subPath = 
			create_subscription<nav_msgs::msg::Path>(
				path_topic_name,
				10,
				std::bind(
					&irob_rbc_maneuv3r_tracker::irob_path_callback,
					this,
					std::placeholders::_1)
			);
		
		subiRobCmd =
			create_subscription<irob_msgs::msg::IrobCmdMsg>(
				irob_cmd_topic_name,
				10,
				std::bind(
					&irob_rbc_maneuv3r_tracker::irob_cmd_callback,
					this,
					std::placeholders::_1)
			);
			
		timer_ = 
			this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&irob_rbc_maneuv3r_tracker::irob_loop_runner, 
					this)
			);
	
		RCLCPP_INFO(this->get_logger(), "iRob maneuv3r started!");
	
	}
	
	// Callback to get Path message
	void irob_path_callback(const nav_msgs::msg::Path path_command){
		pathMsg = path_command;
		path_Length = pathMsg.poses.size();
		
		if(path_Length < 1){
			RCLCPP_ERROR(
				this->get_logger(),
				"Received empty path !"
			);
			
			return;
		}
		
		RCLCPP_INFO(
			this->get_logger(), 
				"Received trajectory! Point count %ld",
				path_Length
				);
		
		current_pose = 0;
		// Add first point from Path to setpoint
		poseSetpoint = pathMsg.poses[0].pose;
		
		irob_cmd = "run";
	}
	
	// Callback to get iRob state command
	void irob_cmd_callback(const irob_msgs::msg::IrobCmdMsg::SharedPtr irob_command){
		irob_cmd = irob_command->irobcmd;
		if(irob_cmd == "stop"){
			loop_fsm = 0;
		}
	}
	
	// Lookup transform to get current pose
	int irob_getPose(){
		try {
						
			poseFeedback = 
				tf_buffer_->lookupTransform(
					map_frame_id, robot_frame_id, 
					tf2::TimePointZero,
					tf2::Duration(1)
				);
				
		} catch (const tf2::TransformException & ex) {
			RCLCPP_INFO(
				this->get_logger(), 
				"Could not transform %s to %s: %s",
				robot_frame_id.c_str(), 
				map_frame_id.c_str(), 
				ex.what()
			);
			
			loop_fsm = 0;
			irob_cmd = "";
			
		  return -1;
		}
		
		// Convert Quaternion to RPY to get Yaw (robot orientation)
		
		// Orientation from Pose feedback 
		tf2::fromMsg(poseFeedback.transform.rotation, quat_tf);
		fYaw = tf2::getYaw(quat_tf);			
		
		// RCLCPP_INFO(
			// this->get_logger(),
			// "Current Orientation %f",
			// fYaw
		// );
		
		return 0;
	}
	

	double look_distance;
	size_t next_pose = 0;
	void irob_updateCarrot(){
		// Only look for next pose when the current pose is not the last pose
		if(current_pose < path_Length){
			// Look forward for the closest point to the lookahead distance from current setpoint
			for(size_t c=current_pose; c < path_Length; c++){
				look_distance = 
					irob_euclideanDistance(
						pathMsg.poses[c].pose.position.y - poseFeedback.transform.translation.y,
						pathMsg.poses[c].pose.position.x - poseFeedback.transform.translation.x
					);
					
				// If found the next suitable Pose
				// Update setpoint to that Pose
				if(look_distance >= lookahead_dist){					
					next_pose = c;

					RCLCPP_INFO(
						this->get_logger(),
						"Next setpoint pose %ld out of %ld",
						next_pose, path_Length - 1	
					);
					
					return;
				}
					
			}
			
			// If not found, use next Pose
			next_pose = current_pose + 1;
		}
		// else{
			// current_pose = 0;
			// loop_fsm = 0;
			// irob_cmd = "stop";
		// }
	}
	
	// Current Pose
	size_t current_pose = 0;
	
	// Main FSM
	uint8_t loop_fsm = 0;
	
	// Velocity control
	double cVel, cHeading, cVelAz;
	double fVel;
	
	// buffer for transform stamped to be used as position feedback
	geometry_msgs::msg::TransformStamped poseFeedback;
	
	tf2::Quaternion quat_tf;
	double spYaw, fYaw;
	
	double diff_x, diff_y;
	
	// Velocity limiter
	double next_heading;
	double del_heading;
	
	// PID for walk
	double eDist;
	double walkIntg;
	double walkDiff;
	double prevDist;
	
	// PID for rotate
	double eOrient;
	double rotateIntg;
	double rotateDiff;
	double prevOrient;
	
	void irob_loop_runner(){
		
		switch(loop_fsm){
			case 0:// Idle case, wait for start command from /irob_cmd
			{
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
				twist.angular.z = 0.0;
				walkIntg = 0.0;
				rotateIntg = 0.0;
				if(irob_cmd == "run"){
					loop_fsm = 1;
				}
			}
			break;
			
			case 1:// PID loop inside here
			{
				// 1. Get current robot position by looking up transform
				if(irob_getPose() < 0)
					break;
				
				// Calculate Euclidean distance
				// Dx and Dy
				diff_x = poseSetpoint.position.x - poseFeedback.transform.translation.x;
				diff_y = poseSetpoint.position.y - poseFeedback.transform.translation.y;
				
				// Calculate Heading
				cHeading = 
					atan2(
						diff_y,
						diff_x
					)  - fYaw;

				// Finally calculate the Euclidean distance
				eDist = irob_euclideanDistance(diff_x, diff_y);
				
				// Calculate Yaw setpoint
				tf2::fromMsg(poseSetpoint.orientation, quat_tf);
				spYaw = tf2::getYaw(quat_tf);
				
				// If the goal Pose is not the last one, use velocity scaling
				if(next_pose < (path_Length - 1)){
					
					// 2. Look for next Setpoint
					irob_updateCarrot();
					 
					// 3. Calculate Heading from current Pose to next Pose
					next_heading = atan2(
							pathMsg.poses[next_pose].pose.position.y -
							poseSetpoint.position.x,
							pathMsg.poses[next_pose].pose.position.x -
							poseSetpoint.position.x
						);
					
					// 4. Scale velocity based on heading difference Pose heading and measured heading
					del_heading = abs(abs(next_heading) - abs(cHeading));
					if(del_heading > 3.141593){
						del_heading = 6.283185 - del_heading;	
					}
					
					del_heading = del_heading/3.141593; // Scaling between 0 to 1
					del_heading = del_heading * del_heading;
					RCLCPP_INFO(
						this->get_logger(),
						"Velocity brake scaling %f",
						del_heading
					);
					
					
					cVel = abs(walkMax - (del_heading * heading_velocity_braking_ratio));	
				}else{
					// Else if the goal Pose is the last one, use PID controller to approach the goal
					walkIntg += eDist * walkKi;
				
					walkDiff = (eDist - prevDist) * walkKd;
					prevDist = eDist;
					
					cVel =
						(eDist * walkKp) 	+
						walkIntg			+
						walkDiff			; 
						
					RCLCPP_INFO(
						this->get_logger(),
						"PID mode"
					);	
				}
				
				
				if(cVel > walkMax)
					cVel = walkMax;
				
				if(cVel < -walkMax)
					cVel = -walkMax;
				
				fVel = ((1 - walk_vel_filter) * fVel) + (walk_vel_filter * cVel);
				
				// 5. Update next setpoint
				if(next_pose != current_pose){
					current_pose = next_pose;
					poseSetpoint = pathMsg.poses[current_pose].pose;
				}
				
				// Rotate PID controller
				eOrient = spYaw - fYaw;			
				if(abs(eOrient) > 3.141593){
					if(eOrient < 0.0)
						eOrient += 6.283185;
					else
						eOrient -= 6.283185;
				}
				
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
				
				// 
				RCLCPP_INFO(
					this->get_logger(),
					"Vel %f | Heading %f ",
					fVel, cHeading
				);
				
				// Goal checker 
				if((abs(eDist) <= walk_goal_tolerance)){						
					fVel = 0.0;
					walkIntg = 0.0;
					
					if(next_pose >= (path_Length - 1)){
						RCLCPP_INFO(
							this->get_logger(),
							"Goal reached!"
						);
						
						next_pose = 0;
						current_pose = 0;
						cVelAz = 0.0;
						loop_fsm = 0;
						irob_cmd = "stop";
					}
				}
				
				if(abs(eOrient) <= rotate_goal_tolerance){
					cVelAz = 0.0;
					rotateIntg = 0.0;
				}
				
			}
			break;
		}
		
		maneuv3r_update_Cmdvel(
			fVel,
			cHeading,
			cVelAz
			);
		
		pubMotion->publish(twist);
	}
	
	private:
	
	double atan2pi(double y, double x) {
	  double at = atan2(y, x);
	   if (at < 0.0)
		 at += 6.28319;
	  return at;
	}
	
	double irob_euclideanDistance(double dx, double dy){
		dx = dx * dx;
		dy = dy * dy;
		
		return sqrt(dx + dy);
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
	auto irobPOSE {std::make_shared<irob_rbc_maneuv3r_tracker>()};
	rclcpp::spin(irobPOSE);
	rclcpp::shutdown();
}
