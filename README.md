# Rokae ROS2 Control

## 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
* state:
    * joint effort
    * joint position
    * joint velocity

## 2. Build

Tested environment:

* Ubuntu 24.04 ROS2 Jazzy

```bash
cd ~/ros2_ws
colcon build --packages-up-to rokae_ros2_control --symlink-install
```

## 3. Rokae end-tool RS485 gripper

The hardware plugin can control a gripper joint through the robot end-tool
RS485 Modbus RTU bridge. Add a joint whose name contains `gripper` or `hand`,
then select the protocol with hardware parameters.

AG2F90D example:

```xml
<hardware>
  <plugin>rokae_ros2_control/RokaeHardware</plugin>
  <param name="arm_ip">192.168.1.161</param>
  <param name="local_ip">192.168.1.233</param>
  <param name="gripper_type">ag2f90d</param>
  <param name="gripper_slave_id">1</param>
  <param name="gripper_close_pos">9000</param>
  <param name="gripper_max_opening_distance">0.09</param>
  <param name="gripper_configure_xpanel">false</param>
  <param name="gripper_read_interval_ms">500</param>
</hardware>
```

`gripper_configure_xpanel=false` keeps the current controller/RobotAssist
end-tool 24V and RS485 settings. Set it to `true` only when the robot model and
controller firmware support configuring xPanel RS485 from the SDK.

The gripper command and state use meters: `0.0` is closed, and
`gripper_max_opening_distance` is fully open.
