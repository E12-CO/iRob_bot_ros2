# iRob_controller

```iRob_controller``` is a ROS node responsible of forward and inverse kinematic calculations of the drive train system. The supported drive train systems are
- 3 wheels Omni-directional drive train
- 4 wheels Omni-directional drive train
- 4 wheels Mecanum drive train

The node supported namespace. 

# ```iRob_controller``` node
## Related ROS topics
### Subscribed topics
- ```cmd_vel``` to receive the message type ```geometry_msgs::msg::Twist``` to be used in inverse kinematic calculation (control robot) and control the robot. This message is published by ```teleop_twist``` or ```iRob_maneuv3r``` node.
- ```irob_motor_feedback``` to receive the motor joint state message with the type ```sensor_msgs::msg::JointState``` to be used in forward kinematic calculation (estimate robot odometry). This message is published by ```iRob_interface``` node.

### Published topics
- ```irob_motor_cmd``` to publish the motor joint (speed) command with the message type of ```sensor_msgs::msg::JointState```. This topic is subscribed by the ```iRob_interface``` node to command the motor speed.
- ```/tf2``` to ```sendTransform()``` between robot frame to odometry frame.

## Wheel conventions (Look from top view)
The motor/wheel order is counting couter-clockwise. +x axis is the front side of the robot, +y is the left side of the robot
### 3 Wheels Omni
```
    Top View        +x      m0 -> motor 0
        m0           ^      m1 -> motor 1
       /  \          |      m2 -> motor 2
      /  C \  +y <---o      C -> robot center
    m1 ---- m2
	
```
### 4 Wheels Omni
```
    Top View        +x      m0 -> motor 0 (motor_lf, left front)
   /m0 ----- m3\     ^      m1 -> motor 1 (motor_lb, left back)
    | \    / |       |      m2 -> motor 2 (motor_rb, right back)
    |   Cen  | +y <--o      m3 -> motor 3 (motor_rf, right front)
    | /    \ |              Cen -> robot center
   \m1 ----- m2/
```
### 4 Wheels Mecanum. Top veiw should see the wheels making the X symbol on real robot
```
    Top View        +x      m0 -> motor 0 (motor_lf, left front)
   \m0 ----- m3/     ^      m1 -> motor 1 (motor_lb, left back)
    | \    / |       |      m2 -> motor 2 (motor_rb, right back)
    |   Cen  | +y <--o      m3 -> motor 3 (motor_rf, right front)
    | /    \ |              Cen -> robot center
   /m1 ----- m2\
```
## Parameter .yaml configurations
- ```robot_frame_id``` is the frame of the robot where the wheels are attached to. Default is ```base_link``` but can be changed accroding to the robot's URDF.
- ```odome_frame_id``` is the odometry frame name. Default is ```odom``` but can be changed in case of multi-robot on same ROS domain.
- ```irob_holo_type``` is the drive train system type. The available types are
	- ```irob_holonomic_plugins::Omni3``` for 3 wheels Omni robot.
	- ```irob_holonomic_plugins::Omni4``` for 4 wheels Omni robot.
	- ```irob_holonomic_plugins::Meca4``` for 4 wheels Mecanum robot.
- ```robot_length``` and ```robot_width``` depends on the type of the drive train. **Using meter (m) unit*.
| **Drive Train type**          | ```robot_length```                           | ```robot_width```                            |
|-------------------------------|----------------------------------------------|----------------------------------------------|
| irob_holonomic_plugins::Omni3 | Wheel center to Robot center distance        | unused                                       |
| irob_holonomic_plugins::Omni4 | Wheel center to Robot center distance        | unused                                       |
| irob_holonomic_plugins::Meca4 | Wheel center to Robot X axis center distance | Wheel center to Robot Y axis center distance |
- ```wheel_radius``` is the **radius** of the wheel, **measured in meter (m) unit**.
- ```gear_ratio``` is the ratio of the motor shaft rotation per **one** gearbox output shaft rotation. For example of 1:70 ratio, One output rotation per 70 motor rotations. The ```gear_ratio``` will be ```70.0```
- ```publish_tf``` is to set whether to publish ```odom->base_link``` transform. This usually set to ```true``` if you are using something like SLAM Toolbox. But please remember that this odometry is degraded as the robot move faster. In real competition, other odometry source such as dead wheel (tracking wheel) is greatly recommended 