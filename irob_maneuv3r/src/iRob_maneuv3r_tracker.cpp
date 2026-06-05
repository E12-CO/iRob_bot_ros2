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

// Visualization message
#include <visualization_msgs/msg/marker.hpp>

// Sensor messages
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"

// iRob command message
#include "irob_msgs/msg/irob_cmd_msg.hpp"

// Define Feedback Loop time 
#define LOOP_TIME_MIL   50 // 50 millisec -> 20Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

#define CHOOSE_MIN(x,y)	((x < y) ? x : y)
#define CAP_HIGH(x,c)	((x > c) ? c : x)
#define CAP_LOW(x,c)	((x < c) ? c : x)
 
class irob_rbc_maneuv3r_tracker : public rclcpp::Node{
	
	public:
	
	// Pub
	// cmd_vel twist publisher
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 		pubMotion;
	// buffer to publish twist message
	geometry_msgs::msg::Twist twist;
	
	// iRob status message
	rclcpp::Publisher<irob_msgs::msg::IrobCmdMsg>::SharedPtr		pubiRobStat;
	
	// Path publisher 
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr			pubPath;
	nav_msgs::msg::Path	pathTargetMsg;
	
	// Sub
	// Path command (trajectory)
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr	subPath;
	// buffer for Path message
	nav_msgs::msg::Path	pathMsg;
	size_t path_Length;
	
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
	
	// Visualization markers
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr	pubCarrotMark;
	visualization_msgs::msg::Marker	carrotMark;
	
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
	uint32_t lookahead_ff_points = 0;
	
	double walkKp;
	double walkKi;
	double walkKd;
	double walkMax;
	double walkMin;
	
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
		
		declare_parameter("lookahead_ff_points", 10);
		get_parameter("lookahead_ff_points", lookahead_ff_points);
		
		declare_parameter("walk_kp", 1.0);
		get_parameter("walk_kp", walkKp);
		declare_parameter("walk_ki", 0.0);
		get_parameter("walk_ki", walkKi);
		declare_parameter("walk_kd", 0.0);
		get_parameter("walk_kd", walkKd);
		
		declare_parameter("walk_max_vel", 1.0);
		get_parameter("walk_max_vel", walkMax);
		declare_parameter("walk_min_vel", 0.1);
		get_parameter("walk_min_vel", walkMin);


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
		
		pubMotion = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		
		pubiRobStat = create_publisher<irob_msgs::msg::IrobCmdMsg>("irob_stat", 10);
		
		// Set up target path publisher
		pubPath = 
			create_publisher<nav_msgs::msg::Path>(
				"irob_target_vect",
				10
			);
			
		// Set the header frane_id of the Path message
		pathTargetMsg.header.frame_id = map_frame_id;	
		pathTargetMsg.poses.resize(2);
				
		// Visualization
		pubCarrotMark = create_publisher<visualization_msgs::msg::Marker>("carrot", 10);		
		
		subPath = 
			create_subscription<nav_msgs::msg::Path>(
				"path",
				10,
				std::bind(
					&irob_rbc_maneuv3r_tracker::irob_path_callback,
					this,
					std::placeholders::_1)
			);
		
		subiRobCmd =
			create_subscription<irob_msgs::msg::IrobCmdMsg>(
				"irob_cmd",
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
			RCLCPP_ERROR(
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

					RCLCPP_DEBUG(
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

	}
	
	void irob_drawCarrot(){
		carrotMark.header.frame_id = "map";
		carrotMark.header.stamp = this->get_clock()->now();
		carrotMark.id = 0;
		carrotMark.type = carrotMark.SPHERE;
		carrotMark.action = carrotMark.ADD;
		carrotMark.scale.x = 0.5;
		carrotMark.scale.y = 0.5;
		carrotMark.color.a = 1.0;
        carrotMark.color.r = 1.0;
        carrotMark.color.r = 0.5;
        carrotMark.pose.position.x = pathMsg.poses[current_pose].pose.position.x;
        carrotMark.pose.position.y = pathMsg.poses[current_pose].pose.position.y;
        carrotMark.pose.position.z = 0.5;
        pubCarrotMark->publish(carrotMark);
	}
	
	uint32_t feedforward_points = 0;
	uint32_t remain_poses = 0;
	float angleCost;
	float finalCost;
	
	float calculated_cmd_vel; 
	
	void irob_lookaheadFeedforward(){
		remain_poses = path_Length - current_pose;
		
		// determine how many ff points we can use
		// If the lookahead feedforward points is more than the remaining poses
		// chose the least one instaed.
		feedforward_points = CHOOSE_MIN(lookahead_ff_points, remain_poses);
		
		// Accumulate the angle cost
		// from the angle of 0->1 & 1->2, 1->2 & 2->3, 2->3 & 3->4 and so on
		// lookahead 3 points produce 1 angle
		// lookahead 4 points produce 2 angles
		// lookahead 5 points produce 3 angles
		// lookahead n points produce n-2 angles
		angleCost = 0.0f;
		for(uint32_t i=0; i < (feedforward_points - 2); i++){
			angleCost += abs(
				atan2(
					pathMsg.poses[current_pose + i + 1].pose.position.y - pathMsg.poses[current_pose + i].pose.position.y ,
					pathMsg.poses[current_pose + i + 1].pose.position.x - pathMsg.poses[current_pose + i].pose.position.x
				) - 
				atan2(
					pathMsg.poses[current_pose + i + 2].pose.position.y - pathMsg.poses[current_pose + i + 1].pose.position.y ,
					pathMsg.poses[current_pose + i + 2].pose.position.x - pathMsg.poses[current_pose + i + 1].pose.position.x
				)
			);
		}
		
		// Also add the angle cost from the current position to the feed forward
		// angleCost += abs(
			// atan2(
				// pathMsg.poses[current_pose].pose.position.y - poseFeedback.transform.translation.y,
				// pathMsg.poses[current_pose].pose.position.x - poseFeedback.transform.translation.x
			// ) - 
			// atan2(
				// pathMsg.poses[current_pose + 1].pose.position.y - pathMsg.poses[current_pose].pose.position.y ,
				// pathMsg.poses[current_pose + 1].pose.position.x - pathMsg.poses[current_pose].pose.position.x
			// )
		// );
	
		RCLCPP_DEBUG(
			this->get_logger(),
			"Angle Cost %.02f", angleCost
		);
	
		// Convert the cost to exponential dekay
		finalCost = 1 / exp(angleCost);
		finalCost = round(finalCost * 100.0) / 100.0;
		
		calculated_cmd_vel = walkMax * finalCost;// Scale the max speed by the exponential decay
		calculated_cmd_vel = CAP_LOW(calculated_cmd_vel, walkMin);// Prevent the velocity for being too small
			
		RCLCPP_DEBUG(
			this->get_logger(),
			"Command Vel %.02f", calculated_cmd_vel
		);
			
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
				
				pathTargetMsg.poses[0].pose.position.x = 
					poseFeedback.transform.translation.x;
				pathTargetMsg.poses[0].pose.position.y = 
					poseFeedback.transform.translation.y;
				
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
					 
					irob_lookaheadFeedforward(); 
					 
					cVel = calculated_cmd_vel;
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
				RCLCPP_DEBUG(
					this->get_logger(),
					"Vel %f | Heading %f ",
					cVel, cHeading
				);
				
				irob_drawCarrot();
				
				// Goal checker 
				if((abs(eDist) <= walk_goal_tolerance)){						
					cVel = 0.0;
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
				
				pubPath->publish(pathTargetMsg);
			}
			break;
		}
		
		maneuv3r_update_Cmdvel(
			cVel,
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
