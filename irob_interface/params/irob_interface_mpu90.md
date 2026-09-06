# iRob Interface MPU90 Parameters

This guide explains how to use `irob_interface_mpu90.yaml` with the `iRob_interface` ROS2 node.

## Stable ESP32 Serial Path

`irob_interface_mpu90.yaml` uses a fixed ESP32 serial path:

```yaml
serial_port: "/dev/ESP32"
```

Install the included udev rule to create the stable `/dev/ESP32` path for the ESP32 CP210x USB serial adapter:

```bash
cd ~/ROS_iROB_V0
sudo cp src/iRob_bot_ros2/irob_controller/esp32.rules /etc/udev/rules.d/99-esp32.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and reconnect the ESP32. If using WSL, detach and attach the USB device again with `usbipd`.

Verify the fixed device name:

```bash
ls -l /dev/ESP32
```

Expected result:

```text
/dev/ESP32 -> ttyUSB0
```

## Run The Interface

Build and source the workspace:

```bash
cd ~/ROS_iROB_V0
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to irob_interface
source install/setup.bash
```

Run `iRob_interface` with the MPU90 parameter file:

```bash
ros2 run irob_interface iRob_interface --ros-args \
  --params-file ~/ROS_iROB_V0/src/iRob_bot_ros2/irob_interface/params/irob_interface_mpu90.yaml
```

You can also override the serial port manually:

```bash
ros2 run irob_interface iRob_interface --ros-args \
  --params-file ~/ROS_iROB_V0/src/iRob_bot_ros2/irob_interface/params/irob_interface_mpu90.yaml \
  -p serial_port:=/dev/ESP32
```

## Check Communication

In another terminal:

```bash
ros2 node list
ros2 param get /iRob_Interface serial_port
ros2 topic list
```

The controller and interface communicate through these motor topics:

```text
/irob_motor_cmd
/irob_motor_feedback
```

If the node prints `iRob connection timed out - retrying...`, check that:

```text
1. /dev/ESP32 exists and points to the ESP32 serial device.
2. No other program is using the serial port.
3. The ESP32 firmware is uploaded and running.
4. The ESP32 firmware baud rate matches the ROS interface baud rate.
```