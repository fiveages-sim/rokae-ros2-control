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
  <param name="gripper_type">ag2f90_d</param>
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

The AG2F90D gripper command and state are normalized: `0.0` is closed, and
`1.0` is fully open. If a configured gripper is not connected, the exported
state stays at `0.0` instead of an invalid sentinel value.

## 4. Load Configuration

The plugin applies an xCoreSDK `setLoad()` setting when it initializes the
real-time motion controller. Without a gripper the default load is zero.
`ag2f90_d` (including the accepted `ag2f90d`, `ag2f90`, and
`changingtek90d` aliases) selects this identified load automatically:

```xml
<param name="load_mass">1.2874</param>
<param name="load_cog">[-0.030145, -0.010147, 0.14785]</param>
<param name="load_inertia">[0.0, 0.0, 0.0]</param>
```

The values shown above are the built-in AG2F90 preset, so they do not need to
be repeated in the hardware XML. These three parameters optionally override
all or part of the selected preset, which supports newly identified grippers
without a driver code change. Until an RG75/Jodell load is identified, its
built-in default remains zero and can be overridden using the same parameters.

Jodell RG75 example:

```xml
<param name="gripper_type">jodell</param>
<param name="gripper_torque_scale">1.0</param>
<param name="gripper_velocity">100</param>
```

For RG75, position remains the physical opening distance in meters
(`0.0` to `0.038372`) for the RViz prismatic joint. `gripper_torque_scale`
is clamped to `0.0` to `1.0` and converted to the command force byte
(`0` to `255`); `gripper_velocity` is the command velocity byte (`0` to
`255`). Reported `velocity` and `effort` state interfaces expose the raw
velocity and force bytes read from status register `0x07D2`.
