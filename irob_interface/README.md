# iRob_interface

```iRob_interface``` is a collection of ROS nodes used to interface with the low level hardware. Which comprise of 
- ```iRob_interface``` node is the legacy iRob interface node developed during ABU Robocon 2025. Which is capable of interfacing the ESP32 or STM32F3 controller for upto four closed loop motors and an IMU sensor. 
- ```iRob_interface_eth``` node is the new iRob interface node that specifically developed for interfacing upto 32 networked **iRob_ETH** closed loop motor controllers over TCP/IP socket. 

Both node supported namespace.  

*For the low level hardware/firmware of the legacy iRob system and the iRob_ETH. Pleas refer to the __[iRob-bot](https://github.com/E12-CO/iRob-bot)__ repository*.

# ```iRob_interface``` node
## Related ROS topics
### Subscribed topics
- ```irob_motor_cmd``` to receive the motor joint (speed) command with the message type of ```sensor_msgs::msg::JointState``` from the ```iRob_controller``` node.

### Published topics
- ```irob_motor_feedback``` to publish the motor joint state message with the type ```sensor_msgs::msg::JointState``` to be used in forward kinematic calculation (estimate robot odometry). This message is used by the ```iRob_controller``` node.
- 6 DoF IMU with the message type of ```sensor_msgs::msg::Imu```. The topic name is configured via parameter. Default is ```imu/data_raw```.
- 3 Dof Magnetometer with the message type of ```sensor_msgs::msg::MagneticField```. The topic name is configured via parameter. Default is ```imu/mag```.

## Parameter .yaml configurations
- ```serial_port``` is to select the usb serial file for example ```/dev/ttyUSB0``` and ```/dev/ttyACM0```. To make sure that the serial port is stay consistent. Use **udev rule** to set up the fixed device name, for example ```/dev/ESP32```.
- ```robot_frame_id``` is the robot frame name according to the URDF. By default is ```base_link```.
- ```mouse_odom_frame_id``` is unsed, originally left there for the optical mouse sensor.
- ```imu_frame_id``` is the frame name of IMU according to the URDF. The subscriber such as Cartographer need this frame name in order to lookup for the coordiate transform of where the IMU sensor is mounted on the robot.
- ```mag_sensor_topic``` is the topic name of the 3 DoF manetometer sensor. Default is ```imu/mag```. By default we don't use magnetometer due to it sensitivity to hard and soft iron interference. It provide next to no benefit with paring with IMU+Lidar or IMU+dead wheel.
- ```imu_sensor_topic``` is the topic name of the 6 DoF IMU sensor. Default is ```imu/data_raw```.
- ```mouse_x_constant``` and ```mouse_y_constant``` is unused, originally used to convert the optical mouse sensor reading to m/s velocity measurement.
- ```gyro_constant``` is a conversion factor from the raw gyro LSB reading to rad/s unit.
- ```mag_constant``` is a conversion factor from the raw magneto LSB reading to tesla.
- ```accel_constant``` is a conversion factor from the raw accelerometer LSB reading to m/s^2

# ```iRob_interface_eth``` node
## Related ROS topics
### Subscribed topics
- ```<motor_name>/command``` to receive the motor joint speed with the message type of ```sensor_msgs::msg::JointState```. The main topic name ```<motor_name>``` is depends on the parameter configuration ```topic_name```.
- ```irob_motor_cmd``` to receive the motor joint (speed) command with the message type of ```sensor_msgs::msg::JointState``` from the ```iRob_controller``` node. This single topic only available when there are motors with ```wheel_id``` between 3 to 8 ids.

### Published topics
- ```<motor_name>/feedback``` to publish the motor joint speed and position with the message type of ```sensor_msgs::msg::JointState```. The main topic name ```<motor_name>``` is depends on the parameter configuration ```topic_name```.
- ```irob_motor_feedback``` to publish the motor joint state message with the type ```sensor_msgs::msg::JointState``` to be used in forward kinematic calculation (estimate robot odometry). This message is used by the ```iRob_controller``` node. This single topic only available when there are motors with ```wheel_id``` between 3 to 8 ids.

## Parameter .yaml configurations

### parameter structure
Due to the flexibility design of ```iRob_interface_eth``` node. The available publisher/subscription topics are dynamically allocated, which depends on the YAML parameter configuration. It was designed in this way in order to support up to 32 individual networked closed loop iRob_ETH controllers. And each motor has it own ```command``` and ```feedback``` topic. Additionally, the motor can be assigned with ```wheel_id``` to group them into a pair of iRob motor control/feedback message that will be used in conjunction with ```iRob_controller``` node for the drive train system.

The YAML structure is the following
```
iRob_ETH_Interface: # Node name, can be renamed with launch file or when under namespace
    ros__parameters:
        # motor_X where 'X' is the motor instant number
        # Valid from 0 to 31
        motor_0: # Motor instant 0
            topic_name:     "motor_lf" # Topic name that translate to motor_lf/command and motor_lf/feedback
            ip_address:     "192.168.1.16" # iRob_ETH IP address, right now the network address is fixed at 192.168.1.X and the iRob_ETH can be assigned from .16 to .48. Maximum of 32 instantces
            wheel_id:       0 # Assign the wheel_id to the motor to be used with iRob_controller. Left out or set it -1 to exclude it from the group
            loop_rate:      "100Hz" # Set the speed control loop rate
            speed_kp:       1.0 # Set the Kp of the controller 
            speed_ki:       0.01 # Set the Ki of the controller
            speed_kd:       0.001 # Set the Kd of the controller
            speed_kff:      5.0 # Set the Kff (feed forward) of the controller
            speed_min:      -5000.0 # Set the minimum control 
            speed_max:      5000.0 # Set the maximum control
            encoder_cpr:    28  # Set the encoder CPR
            gear_ratio:     1.0 # Set the gear ratio (UNUSED FOR NOW)

        motor_1: # Motor instant 1
            ...
            ...
```
Each individual motor has its own instant in the format of ```motor_x``` where ```x``` is the instant number. The number ```x``` must always start from 0. Every new motor instant must follow the unskipped numerical order from the previous instant. For example ```0, 1, 2, 3``` is a correct sequence. ```0, 1, 3, 4``` is **incorrect** sequence.

### parameter information
- ```topic_name``` is a topic name set to be unique to each motor. For example if the ```topic_name``` is set to ```motor67```, there will be two topics named ```motor67/command``` and ```motor67/feedback```.
- ```ip_address``` is to set the IP address of the iRob_ETH to the specific motor instant. On iRob_ETH board, this can be set with rotary switch and sliding switch.
- ```wheel_id``` is to allocate this specific motor instant to be used with ```iRob_controller``` via ```irob_motor_cmd``` and ```irob_motor_feedback``` topics. The ID starts from 0 all the way to 7. Leave as -1 in order to excluded it from the group.
- ```speed_kp```, ```speed_ki```, ```speed_kd``` and ```spedd_kff``` are the controller gain for PID and feedforward controller. Tune the ```speed_kff``` first to make sure that the open loop motor speed is closest to the setpoint. Then tune ```speed_kp```, ```speed_ki``` and ```speed_kd``` to handle the reactive transient response.
- ```speed_min``` and ```speed_max``` are just to help limit the unintended speed command. These are in the **rpm** unit.
- ```encoder_cpr``` is the Count Per Revolution (CPR) obtained from mutiplying the encoder Pulse Per Revolution (PPR) with 4. 
- ```gear_ratio``` is UNUSED for now. As for the drive train system. Sees ```iRob_controller``` README.