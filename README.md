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
  <param name="tool_torque">1.0</param>
  <param name="tool_velocity">1.0</param>
</hardware>
```

`gripper_configure_xpanel=false` keeps the current controller/RobotAssist
end-tool 24V and RS485 settings. Set it to `true` only when the robot model and
controller firmware support configuring xPanel RS485 from the SDK.

Supported ChangingTek aliases are `ag2f90_c`, `ag2f90c`, `ag2f90`,
`changingtek90c`, `ag2f90_d`, `ag2f90d`, and `changingtek90d`. The driver uses
the matching feedback register for the C/D variant; `gripper_feedback_reg` can
override it for unusual firmware. Each Rokae hardware instance exposes `tool_torque` and `tool_velocity`.
These are normalized `0.0` to `1.0` values and can be updated at
runtime with `ros2 param set`. AG2F90 maps velocity and torque to registers
`0x0104` and `0x0105`; these, plus acceleration/deceleration registers, can be
overridden with `gripper_velocity_reg`, `gripper_torque_reg`,
`gripper_acceleration_reg`, and `gripper_deceleration_reg`. The AG2F90 command
and state are normalized: `0.0` is closed, and `1.0` is fully open. If a
configured gripper is not connected, the exported state stays at `0.0` instead
of an invalid sentinel value.

### LinkerHand O6 / O7 / L6

`gripper_type` 支持 `linkerhand_o6`、`linkerhand_o7`、`linkerhand_l6`，
也支持短名 `o6`、`o7`、`l6`。通过机械臂末端 RS485 的 Modbus RTU
桥接通信，左右手默认地址分别为 `0x28`、`0x27`。

| 型号 | 主控关节数 | 位置寄存器 | 力矩寄存器 | 速度寄存器 |
| --- | --- | --- | --- | --- |
| O6 / L6 | 6 | 0–5 | 6–11 | 12–17 |
| O7 | 7 | 0–6 | 7–13 | 14–20 |

`FC04` 读取位置，`FC16` 写入位置、力矩和速度。Rokae SDK 单次最多操作
3 个 `int16` 寄存器；O7 每个寄存器区按 `3 + 3 + 1` 分帧，避免越界写入下一区。
O7 的寄存器顺序为 `thumb_joint3`、`thumb_joint2`、`index_joint`、
`middle_joint`、`ring_joint`、`pinky_joint`、`thumb_joint1`，与现有 USB-Modbus
驱动一致。命令和反馈使用 URDF 关节弧度范围，按型号转换到原始值 0–255
（0 为弯曲端，255 为伸直端）；O6 拇指横摆范围区分左右手。

可选参数为 `hand_slave_id`、`hand_configure_xpanel`（默认 `false`）和
`hand_read_interval_ms`（默认 100）。`tool_torque`、`tool_velocity`
使用 0–1 的归一化值。保持 `hand_configure_xpanel=false` 时，需要提前在
RobotAssist 中配置末端供电和 RS485 通信参数。

WCE3 示例（右手 O7，O6 时将两处 `o7` 改为 `o6`）：

```bash
ros2 launch ocs2_arm_controller full_body.launch.py robot:=fiveages_wce3 \
  right_type:=linkerhand_o7 hardware:=real xacro_right_ee_drive_type:=rokae_rs485
```

控制器为 `right_hand_controller`（左手为 `left_hand_controller`），不是
`adaptive_gripper_controller`。RViz 的 `GripperControlPanel` 是共用的开合面板，
也会向灵巧手控制器发送命令。

配置需要包含 7 个机械臂关节，随后是该型号完整的主控手指关节；驱动按名称映射
手指关节，允许手指关节在末尾任意排序。缺失、重复、混合左右手的关节及未知末端型号
会使初始化失败，避免把手指关节计入机械臂自由度。

## 4. Runtime faults and retries

首次启动或主动切换控制模式时，`startMove()` 最多尝试 5 次，失败间隔为 100 ms。
尝试耗尽、实时循环异常/意外停止、实时线程启动失败或状态更新异常后，驱动锁定故障，
尝试停止 SDK 实时运动。之后 `read()` / `write()` 返回 `ERROR`，不再重建控制线程，
也不接受控制参数更新或再次激活。正常停用不清除该故障。

排除故障后需要重启运控进程，重新初始化硬件；驱动不会在故障消失后自动恢复运动。
主动修改 `ctrl_mode` / 阻抗参数产生的正常停止和重启仍然保留。

## 5. Load Configuration

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
<param name="tool_torque">1.0</param>
<param name="tool_velocity">1.0</param>
```

For RG75, position remains the physical opening distance in meters
(`0.0` to `0.038372`) for the RViz prismatic joint. The preferred dynamics
parameters are the normalized instance-local `tool_torque` and
`tool_velocity`; they are converted to the RG75 force and velocity bytes
(`0` to `255`). Reported `velocity` and `effort` state interfaces expose the
raw velocity and force bytes read from status register `0x07D2`.

## ERG32-150 rotating gripper (absolute position)

`gripper_type=erg32` (`ERG32` also accepted) controls two joints through Rokae
end-tool RS485. The seven arm joints must come first, followed by the matching
`left_hand_rotate_joint` / `left_hand_gripper_joint` pair (or `right_hand_`, or
unprefixed names); the last two joints may appear in either order.

Rotation uses radians in `[-pi, pi]`, encoded as signed integer degrees. Grip
uses **single-finger travel** in meters, `0.0` closed to `0.016` open, corresponding
to 32 mm total finger travel. Only absolute rotation is sent; turn count and the
unused relative-position register are zero. Multi-turn motion is not enabled.
Use the existing `*_hand_controller` and ERG32 controller template; ordinary grip
targets retain the rotation target.

```xml
<param name="gripper_type">erg32</param>
<param name="gripper_slave_id">9</param>
<param name="gripper_configure_xpanel">false</param>
<param name="gripper_read_interval_ms">100</param>
<param name="gripper_feedback_timeout_ms">1000</param>
<param name="gripper_activation_timeout_ms">10000</param>
<param name="tool_torque">0.2</param>
<param name="tool_velocity">0.2</param>
```

Set dynamics on the hardware node that owns the tool. Torque and velocity apply to both
axes, use normalized 0–1 device scales, and can be changed at runtime. **Zero is
the device's minimum force/speed, not a stop command.** Set up compatible RS485
settings and end-tool power in RobotAssist when `gripper_configure_xpanel=false`.
Read interval must be at least 20 ms; feedback timeout must exceed read interval.
The default interval is a starting point, not a guaranteed communication rate.

Activation first reads status. An already ready device is preserved; otherwise
both axes are reset and activated, and the driver waits for both `gSTA` fields.
Activation can physically move the gripper. A device reporting faults must be
serviced/reset explicitly; the driver does not automatically clear faults.
Targets start from measured positions. Current-position parameters are staged
with both triggers cleared before releasing stop; no startup motion is triggered. An initial
angle outside the supported range or a nonzero turn-count feedback is rejected. Deactivation stops and joins the
worker and writes `rSTOP` on both axes while retaining `rACT`.

FC16 writes are split into grip parameters/trigger (2 registers), rotation
parameters (3), then absolute trigger (1); only changed axes are triggered.
FC03 status reads use three blocks of 3 registers, so feedback is not an atomic
snapshot. The communication worker publishes private snapshots; ROS `read()`
copies them into the exported interfaces. Position-derived velocity uses the
steady-clock sampling interval. Effort is unavailable (`NaN`), since raw current
bytes are not calibrated physical force/torque. Object-state/drop transitions and
device fault bytes are logged; raw speed/current are not exposed as SI values.

Initialization failure prevents hardware activation. Motion-transfer failures,
device faults/loss of activation, out-of-range rotation feedback, and stale
feedback stop both axes on a best-effort basis and propagate an error to the
hardware's latched fault path. Restart the control process after resolving a
runtime fault. A disconnected device cannot acknowledge the stop request.


Each `RokaeHardware` node owns one arm and uses `tool_torque` / `tool_velocity`
without side prefixes. For example:

```bash
ros2 param set /ar5_right_system tool_velocity 0.2
ros2 param set /ar5_right_system tool_torque 0.3
```

The `left_tool_*` / `right_tool_*` names are removed for Rokae. Update custom
hardware XML and scripts that explicitly set those names. Tianji's dual-arm
`MarvinHardware` and separate USB/CAN end-effector drivers retain their existing
parameter names. Controller topics, actions, joint names, and position commands
are unchanged. The inspected `fa-py-libraries/ros2_robot_interface` checkout does
not hard-code the removed parameters; its generic parameter-setting API still
requires callers to supply the new names when addressing a Rokae hardware node.
