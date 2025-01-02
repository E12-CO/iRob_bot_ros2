# iRob_bot_ros2

iRob_bot_ros2 is a ROS2 support package for [iRob-bot](https://github.com/E12-CO/iRob-bot) project, this includes :
- ```irob_msgs``` for motor message and control message
- ```irob_interface``` for hardware interface
- ```irob_controller``` for various types of holonomic robot (3 wheels Omni, 4  wheels Omni and 4 wheels Mecanum)
- ```irob_maneuv3r``` for A to B position controller (Work in progress)
- ```irob_trajectory_server``` for publishing trajectory using PoseStamped message. Save to CSV file soon 
- ```irob_launcher``` for launching the above packages

# TODO
- implement the ```irob_interface_wireless``` to be used with the original iRob-bot
- implement the pure pursuit controller 
- built-in EKF to support other odometry method (based on optical mouse sensor, LiDAR and/or dead wheels)
- save trajectory from ```irob_trajectory_server``` into CSV format
- implement path smoothing and tracking algorithm to make use of the ```irob_trajectory_server```
