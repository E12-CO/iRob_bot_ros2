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
#define LOOP_TIME_SEC	(LOOP_TIME_MIL/1000.0f) // Loop time in second

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
	// Carrot
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr	pubCarrotMark;
	visualization_msgs::msg::Marker	carrotMark;
	// Velocity vector
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr	pubVelVector;
	visualization_msgs::msg::Marker velVectorMark;
	
	/* PARAMETERS */
	
	// ROS fram of reference names
	std::string map_frame_id;
	std::string robot_frame_id;
	std::string odom_frame_id;
	
	// Use odom transform instead of map transform
	bool use_odom_tf_only = false;
	
	typedef struct{
		// Global velocity and acceleration limit
		float f32WalkAccelMax;
		float f32WalkVelMax;
		float f32WalkVelCurveMin; // <- minimum velocity when appraching the curve
		
		// For pure pursuit traker
		float f32LookaheadDistance;
		uint32_t u32LookaheadFeedforwardPoints;
		
		// For goal appraching PID controller 
		float f32WalkKp;
		float f32WalkKi;
		float f32WalkKd;
	}tWalkParameters;
	
	tWalkParameters tWalkControlParameters;
	
	typedef struct{
		// For orientation PID controller
		float f32RotateVelMax;
		
		float f32RotateKp;
		float f32RotateKi;
		float f32RotateKd;
	}tRotateParameters;
	
	tRotateParameters tRotateControlParameters;
	
	typedef struct{
		float f32WalkGoalTolerance;
		float f32RotateGoalTolerance;
	}tGoalPoseTolerance;
	
	tGoalPoseTolerance tGoalToleranceParameters;
	
	irob_rbc_maneuv3r_tracker() : Node("iRob_maneuv3r_tracker"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club Engineering KMITL : Starting iRob maneuv3r tracker..."
			);
		
		// ROS tf related 
		declare_parameter("map_frame_id", "map");
		get_parameter("map_frame_id", map_frame_id);
		
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		
		declare_parameter("odom_frame_id", "odom");
		get_parameter("odom_frame_id", odom_frame_id);
		
		declare_parameter("use_odom_tf_only", false);
		get_parameter("use_odom_tf_only", use_odom_tf_only);
		
		
		// Walk Min-Max parameters
		declare_parameter("walk_max_accel", 1.0f);
		get_parameter("walk_max_accel", tWalkControlParameters.f32WalkAccelMax);
		declare_parameter("walk_max_vel", 1.0f);
		get_parameter("walk_max_vel", tWalkControlParameters.f32WalkVelMax);
		declare_parameter("walk_min_vel", 0.1f);
		get_parameter("walk_min_vel", tWalkControlParameters.f32WalkVelCurveMin);
		
		// walk pure pursuit
		declare_parameter("lookahead_distance", 1.0f);
		get_parameter("lookahead_distance", tWalkControlParameters.f32LookaheadDistance);
		declare_parameter("lookahead_ff_points", 10);
		get_parameter("lookahead_ff_points", tWalkControlParameters.u32LookaheadFeedforwardPoints);
		
		// Walk PID
		declare_parameter("walk_kp", 1.0f);
		get_parameter("walk_kp", tWalkControlParameters.f32WalkKp);
		declare_parameter("walk_ki", 0.0f);
		get_parameter("walk_ki", tWalkControlParameters.f32WalkKi);
		declare_parameter("walk_kd", 0.0f);
		get_parameter("walk_kd", tWalkControlParameters.f32WalkKd);
		
		
		// Rotate Min-Max parameters
		declare_parameter("rotate_max_vel", 3.1415f);
		get_parameter("rotate_max_vel", tRotateControlParameters.f32RotateVelMax);
		
		// Rotate PID 
		declare_parameter("rotate_kp", 1.0f);
		get_parameter("rotate_kp", tRotateControlParameters.f32RotateKp);
		declare_parameter("rotate_ki", 0.0f);
		get_parameter("rotate_ki", tRotateControlParameters.f32RotateKi);
		declare_parameter("rotate_kd", 0.0f);
		get_parameter("rotate_kd", tRotateControlParameters.f32RotateKd);
		
		// Goal checker
		declare_parameter("walk_goal_tolerance", 0.01f);
		get_parameter("walk_goal_tolerance", tGoalToleranceParameters.f32WalkGoalTolerance);
		declare_parameter("rotate_goal_tolerance", 0.174533f);// Default 10 degree
		get_parameter("rotate_goal_tolerance", tGoalToleranceParameters.f32RotateGoalTolerance);
		
		// Check if we will use the transform of odom->base_link instead of map->base_link
		if(use_odom_tf_only == true)
			map_frame_id = odom_frame_id;
		
		tf_buffer_ =
			std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ =
			std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		
		pubMotion = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		
		pubiRobStat = create_publisher<irob_msgs::msg::IrobCmdMsg>("irob_stat", 10);
				
		// Visualization
		pubCarrotMark 	= create_publisher<visualization_msgs::msg::Marker>("carrot", 10);		
		pubVelVector 	= create_publisher<visualization_msgs::msg::Marker>("vel_vector", 10);
		
		
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
				if(look_distance >= tWalkControlParameters.f32LookaheadDistance){					
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
		carrotMark.header.frame_id = map_frame_id;
		carrotMark.header.stamp = this->get_clock()->now();
		carrotMark.ns = "carrot";
		carrotMark.id = 0;
		carrotMark.type = carrotMark.SPHERE;
		carrotMark.action = carrotMark.ADD;
		carrotMark.scale.x = 0.5;
		carrotMark.scale.y = 0.5;
		carrotMark.scale.z = 0.5;
		carrotMark.color.a = 1.0;
        carrotMark.color.r = 1.0;
        carrotMark.color.g = 0.5;
        carrotMark.pose.position.x = pathMsg.poses[current_pose].pose.position.x;
        carrotMark.pose.position.y = pathMsg.poses[current_pose].pose.position.y;
        carrotMark.pose.position.z = 0.5;
        pubCarrotMark->publish(carrotMark);
	}
	
	void irob_drawVelVector(){
		velVectorMark.header.frame_id = map_frame_id;
		velVectorMark.header.stamp = this->get_clock()->now();
		velVectorMark.ns = "vel_vector";
		velVectorMark.id = 0;
		velVectorMark.type = velVectorMark.ARROW;
		velVectorMark.action = velVectorMark.ADD;
		velVectorMark.scale.x = cVel;
		velVectorMark.scale.y = 0.1;
		velVectorMark.scale.z = 0.1;
		
		velVectorMark.color.a = 1.0;
        velVectorMark.color.b = 1.0;
        velVectorMark.color.g = 0.5;
		
		// Position of marker in the middle of the base link
		velVectorMark.pose.position.x = poseFeedback.transform.translation.x;
		velVectorMark.pose.position.y = poseFeedback.transform.translation.y;
		velVectorMark.pose.position.z = 1.0;
		// Marker orientation is based on the heading
		tf2::Quaternion vecQ;
		vecQ.setRPY(0.0, 0.0, cHeading);
		velVectorMark.pose.orientation = tf2::toMsg(vecQ);
		
		pubVelVector->publish(velVectorMark);
	}
	
	uint32_t u32FeedforwardPoints = 0;
	uint32_t u32RemainPoses = 0;
	float f32AngleCost;
	float f32FinalCost;
	
	float f32CalculatedFeedForwardVel; 
	float f32PrevVelCmd = 0.0f;
	float f32CurrentAccel = 0.0f;
	
	void irob_lookaheadFeedforward(){
		u32RemainPoses = path_Length - current_pose;
		
		// determine how many ff points we can use
		// If the lookahead feedforward points is more than the remaining poses
		// chose the least one instaed.
		u32FeedforwardPoints = 
			CHOOSE_MIN(
				tWalkControlParameters.u32LookaheadFeedforwardPoints, 
				u32RemainPoses
			);
		
		// Accumulate the angle cost
		// from the angle of 0->1 & 1->2, 1->2 & 2->3, 2->3 & 3->4 and so on
		// lookahead 3 points produce 1 angle
		// lookahead 4 points produce 2 angles
		// lookahead 5 points produce 3 angles
		// lookahead n points produce n-2 angles
		f32AngleCost = 0.0f;
		for(uint32_t i=0; i < (u32FeedforwardPoints - 2); i++){
			f32AngleCost += abs(
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
	
		RCLCPP_DEBUG(
			this->get_logger(),
			"Angle Cost %.02f", f32AngleCost
		);
	
		// Convert the cost to exponential decay
		f32FinalCost = 1 / exp(f32AngleCost);
		f32FinalCost = round(f32FinalCost * 100.0) / 100.0;
		
		f32CalculatedFeedForwardVel = tWalkControlParameters.f32WalkVelMax * f32FinalCost;// Scale the max speed by the exponential decay
		f32CalculatedFeedForwardVel = 
			CAP_LOW(
				f32CalculatedFeedForwardVel, 
				tWalkControlParameters.f32WalkVelCurveMin
			);// Prevent the velocity for being too small
			
		// Acceleration limit
		f32CurrentAccel = (f32CalculatedFeedForwardVel - f32PrevVelCmd) / LOOP_TIME_SEC;// dV/dt
		f32CurrentAccel = 
			CAP_HIGH(
				CAP_LOW(f32CurrentAccel, -tWalkControlParameters.f32WalkAccelMax),
				tWalkControlParameters.f32WalkAccelMax
			);// Cap the min and max acceleration
		f32CalculatedFeedForwardVel = f32PrevVelCmd + (f32CurrentAccel * LOOP_TIME_SEC);// Integrate the acceleration back to velocity
		f32PrevVelCmd = f32CalculatedFeedForwardVel;// Next control cycle will remember the previous velocity command	
			
		RCLCPP_DEBUG(
			this->get_logger(),
			"Command Vel %.02f", f32CalculatedFeedForwardVel
		);
			
	}
	
	// Current Pose
	size_t current_pose = 0;
	
	// Main FSM
	uint8_t loop_fsm = 0;
	
	// Velocity control
	double cVel, cHeading, cVelAz;
	
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
					
					// 3. Calculate velocity with feed forward control
					irob_lookaheadFeedforward(); 
					 
					cVel = f32CalculatedFeedForwardVel;
				}else{
					// 4. Else if the goal Pose is the last one, use PID controller to approach the goal
					walkIntg += eDist * tWalkControlParameters.f32WalkKi;
				
					walkDiff = (eDist - prevDist) * tWalkControlParameters.f32WalkKd;
					prevDist = eDist;
					
					cVel =
						(eDist * tWalkControlParameters.f32WalkKp) 	+
						walkIntg			+
						walkDiff			; 
						
					RCLCPP_INFO(
						this->get_logger(),
						"PID mode"
					);	
				}
				
				// Velocity envelope
				if(cVel > tWalkControlParameters.f32WalkVelMax)
					cVel = tWalkControlParameters.f32WalkVelMax;
				
				if(cVel < -tWalkControlParameters.f32WalkVelMax)
					cVel = -tWalkControlParameters.f32WalkVelMax;

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
				
				rotateIntg += eOrient * tRotateControlParameters.f32RotateKi;
				
				rotateDiff = (eOrient - prevOrient) * tRotateControlParameters.f32RotateKd;
				prevOrient = eOrient;
				
				cVelAz = 
					(eOrient * tRotateControlParameters.f32RotateKp)+
					rotateIntg			+
					rotateDiff			;
				
				
				// Angular velocity envelope
				if(cVelAz > tRotateControlParameters.f32RotateVelMax)
					cVelAz = tRotateControlParameters.f32RotateVelMax;
				if(cVelAz < -tRotateControlParameters.f32RotateVelMax)
					cVelAz = -tRotateControlParameters.f32RotateVelMax;
				
				// Velocity and Heading debug
				RCLCPP_DEBUG(
					this->get_logger(),
					"Vel %f | Heading %f ",
					cVel, cHeading
				);
				
				// Visualization update
				irob_drawCarrot();
				irob_drawVelVector();
				
				// Goal checkers
				if((abs(eDist) <= tGoalToleranceParameters.f32WalkGoalTolerance)){						
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
				
				if(abs(eOrient) <= tGoalToleranceParameters.f32RotateGoalTolerance){
					cVelAz = 0.0;
					rotateIntg = 0.0;
				}

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
		float vel, 
		float heading, 
		float az) {
			
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
