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

AG2F90C/AG2F90D example:

```xml
<hardware>
  <plugin>rokae_ros2_control/RokaeHardware</plugin>
  <param name="arm_ip">192.168.0.160</param>
  <param name="local_ip">192.168.0.233</param>
  <param name="gripper_type">ag2f90_c</param>
  <param name="gripper_slave_id">1</param>
  <param name="gripper_close_pos">9000</param>
  <param name="gripper_max_opening_distance">0.09</param>
  <param name="gripper_configure_xpanel">false</param>
  <param name="gripper_read_interval_ms">500</param>
  <param name="right_tool_torque">1.0</param>
  <param name="right_tool_velocity">1.0</param>
</hardware>
```

`gripper_configure_xpanel=false` keeps the current controller/RobotAssist
end-tool 24V and RS485 settings. Set it to `true` only when the robot model and
controller firmware support configuring xPanel RS485 from the SDK.

Supported ChangingTek aliases are `ag2f90_c`, `ag2f90c`, `ag2f90`,
`changingtek90c`, `ag2f90_d`, `ag2f90d`, and `changingtek90d`. The driver uses
the matching feedback register for the C/D variant; `gripper_feedback_reg` can
override it for unusual firmware. Tool dynamics follow the Tianji convention:
`left_tool_torque`, `left_tool_velocity`, `right_tool_torque`, and
`right_tool_velocity` are normalized `0.0` to `1.0` values and can be updated at
runtime with `ros2 param set`. AG2F90 maps velocity and torque to registers
`0x0104` and `0x0105`; these, plus acceleration/deceleration registers, can be
overridden with `gripper_velocity_reg`, `gripper_torque_reg`,
`gripper_acceleration_reg`, and `gripper_deceleration_reg`. The AG2F90 command
and state are normalized: `0.0` is closed, and `1.0` is fully open. If a
configured gripper is not connected, the exported state stays at `0.0` instead
of an invalid sentinel value.

## 4. Load Configuration

The plugin applies an xCoreSDK `setLoad()` setting when it initializes the
real-time motion controller. The default load is zero for every gripper type
and for configurations without a gripper. Configure identified load data
explicitly in hardware parameters. For an AG2F90 with the currently identified
values:

```xml
<param name="load_mass">1.2874</param>
<param name="load_cog">[-0.030145, -0.010147, 0.14785]</param>
<param name="load_inertia">[0.0, 0.0, 0.0]</param>
```

The three parameters may also be provided partially; unspecified components
remain zero. This keeps newly identified grippers configurable without a
driver code change.

Jodell RG75 example:

```xml
<param name="gripper_type">jodell</param>
<param name="left_tool_torque">1.0</param>
<param name="left_tool_velocity">1.0</param>
```

For RG75, position remains the physical opening distance in meters
(`0.0` to `0.038372`) for the RViz prismatic joint. The preferred dynamics
parameters are the normalized Tianji-style `*_tool_torque` and
`*_tool_velocity`; they are converted to the RG75 force and velocity bytes
(`0` to `255`). Reported `velocity` and `effort` state interfaces expose the
raw velocity and force bytes read from status register `0x07D2`.
