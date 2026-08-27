# iRob_bot_ros2

iRob_bot_ros2 is a ROS2 support package for [iRob-bot](https://github.com/E12-CO/iRob-bot) project, this includes :
- ```irob_msgs``` for motor message and control message
- ```irob_interface``` for hardware interface
- ```irob_controller``` for various types of holonomic robot (3 wheels Omni, 4  wheels Omni and 4 wheels Mecanum)
- ```irob_maneuv3r``` for A to B position controller 
- ```irob_maneuv3r_tracker``` for pure-pursuit path tracking controller
- ```irob_trajectory_maker``` for manual path planning via RViz
- ```irob_trajectory_server``` for publishing trajectory using PoseStamped message. Save to CSV file soon (work in progress)
- ```irob_launcher``` for launching the above packages

# Dependencies
- ROS2 Humble (rclcpp & rclpy)
- tf2
- [CCMA](https://github.com/UniBwTAS/ccma) for irob_trajectory_maker

# Installation

0. create ROS workspace if you don't already have it
1. navigate to your workspace's source folder. For example ```ros2_ws/src```
2. git clone this project
3. navigate back to the root of your source folder. For example ```ros2_ws```
4. runs the following command
```colcon build --symlink-install --packages-select irob_msgs irob_interface irob_controller irob_maneuv3r irob_trajectory_maker```

# TODO
- implement the ```irob_interface_wireless``` to be used with the original iRob-bot
- built-in EKF to support other odometry method (based on optical mouse sensor, LiDAR and/or dead wheels)
- save trajectory from ```irob_trajectory_server``` into CSV format
