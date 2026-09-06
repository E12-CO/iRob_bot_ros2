# `/dev/ESP32` For `irob_interface_mpu90.yaml`

`irob_interface_mpu90.yaml` uses this serial port:

```yaml
serial_port: "/dev/ESP32"
```

`/dev/ESP32` is not the real Linux device. It is a stable udev symlink to the real USB serial device, such as `/dev/ttyUSB0`.

The real device name can change after reconnecting USB:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

The fixed name stays the same:

```text
/dev/ESP32
```

## Install Udev Rule

From the ROS workspace root:

```bash
cd ~/ROS_iROB_V0
sudo cp src/iRob_bot_ros2/irob_controller/esp32.rules /etc/udev/rules.d/99-esp32.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and reconnect the ESP32. If using WSL, attach the USB device to WSL again with `usbipd`.

## Verify

```bash
ls -l /dev/ESP32
```

Expected result:

```text
/dev/ESP32 -> ttyUSB0
```

If `/dev/ESP32` exists, `irob_interface_mpu90.yaml` can use it directly.