# rover_arm_bringup

Single-joint ODrive CAN bring-up bridge for a University Mars Rover 6-DOF
arm, built for ROS 2 Humble. This package proves that **one real arm joint**
can be represented correctly in ROS 2 as a revolute joint and driven through
`/joint_states`, even though the only position feedback available is an AMT
encoder mounted on the motor shaft (no output-side encoder).

---

## 1. What this package does

`rover_arm_bringup` provides a single node, `single_joint_odrive_bridge`,
that:

1. Subscribes to ODrive CAN feedback (`controller_status`) for one axis.
2. Converts the motor-shaft encoder reading (in motor turns) into the real
   gearbox-output joint angle (in radians), using a fixed gear ratio.
3. Publishes that calculated angle to `/joint_states`, so `robot_state_publisher`,
   RViz, and (later) MoveIt 2 see a correct revolute joint.
4. Accepts joint-space commands (radians) from a topic, a `JointTrajectory`
   message, or an optional `FollowJointTrajectory` action.
5. Converts joint-space commands back into motor turns and sends them to
   ODrive as a position-control `ControlMessage`.
6. Provides a `set_zero` service to define the current physical pose as
   joint = 0 rad.
7. Clamps/refuses unsafe commands (no feedback yet, out-of-range targets,
   oversized steps).

This is a **bring-up bridge**, intentionally simple, so one joint can be
validated against real hardware before the full 6-DOF `ros2_control` +
MoveIt 2 stack is built.

## 2. Why we calculate joint position manually

The arm uses a Harmonic gearbox (currently assumed 80:1) between the motor
and the joint output, and there is **no encoder after the gearbox**. The
only position sensor is the AMT encoder on the motor shaft, read out through
the ODrive S1 as "motor position in turns."

That means ROS never receives the joint angle directly — it must be
calculated:

```
joint_position_rad = direction * (motor_position_turns - zero_offset_turns) * 2*pi / gear_ratio
```

And the reverse, to convert a desired joint command into a motor target:

```
motor_target_turns = zero_offset_turns + direction * joint_target_rad * gear_ratio / (2*pi)
```

Because there is no absolute output-side reference, **the zero point is
relative and must be set manually** (or, in the future, via homing/limit
switches). See [Homing / zeroing procedure](#9-how-to-set-zero) below.

## 3. Hardware assumptions

- ODrive S1 motor controller, one axis per joint.
- ODrive D6374 (or similar) BLDC motor.
- AMT encoder mounted directly on the motor shaft (not the gearbox output).
- Harmonic gearbox, assumed **80:1** for `base_joint` (configurable per joint).
- No output-side / joint-side encoder.
- CAN bus wired to the host computer, `can0`, 1 Mbit/s, with proper termination.
- ODrive already configured (via odrivetool / GUI) for CAN protocol, the
  correct node ID, motor calibration, and encoder calibration.

## 4. Software assumptions

- Ubuntu 22.04 + ROS 2 Humble.
- `odrive_can` ROS 2 package installed and built in the same workspace
  (provides `odrive_can/msg/ControllerStatus`, `odrive_can/msg/ControlMessage`,
  and the `odrive_can/srv/AxisState` service used to request closed-loop control).
- `robot_state_publisher` and `xacro` installed (`ros-humble-robot-state-publisher`,
  `ros-humble-xacro`).
- This package, `rover_arm_bringup`, placed in the same workspace `src/`.

**Important:** the exact field names of `odrive_can/msg/ControllerStatus`
and `odrive_can/msg/ControlMessage` depend on the version of `odrive_can`
you have installed. This package assumes:

- `ControllerStatus.pos_estimate` (motor turns)
- `ControllerStatus.vel_estimate` (motor turns/sec)
- `ControlMessage.control_mode`, `.input_mode`, `.input_pos`, `.input_vel`, `.input_torque`

Before first run, verify these match your installed interface:

```bash
ros2 interface show odrive_can/msg/ControllerStatus
ros2 interface show odrive_can/msg/ControlMessage
```

If your installed version differs, update the field names in
`rover_arm_bringup/single_joint_odrive_bridge.py` (`_controller_status_callback`
and `send_joint_position_command`) accordingly.

## 5. Workspace layout

This package expects to live inside a normal `colcon` workspace, e.g.:

```
~/Desktop/Drive System/ros2_ws/
├── src/
│   ├── rover_arm_bringup/      <- this package
│   └── odrive_can/             <- ODrive ROS 2 CAN driver
├── build/
├── install/
└── log/
```

If you are moving this package to the Jetson, copy the whole
`rover_arm_bringup/` folder into `~/<your_ws>/src/` on the Jetson and build
there — there is nothing in this package that is laptop-specific (no
hardcoded absolute paths).

## 6. Build instructions

```bash
cd ~/Desktop/Drive\ System/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_arm_bringup
source install/setup.bash
```

## 7. Bring-up: CAN interface

```bash
cd ~/Desktop/Drive\ System/ros2_ws

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up

ip -details link show can0
```

Expected output should include:

```
state ERROR-ACTIVE
bitrate 1000000
```

If you instead see `state BUS-OFF` or no `can0` device, check wiring,
termination resistors, and that the CAN-USB/CAN-FD adapter driver is loaded.

## 8. Bring-up: ODrive CAN node

Source the workspace:

```bash
cd ~/Desktop/Drive\ System/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the ODrive CAN node:

```bash
ros2 launch odrive_can example_launch.yaml
```

Check the ODrive topics exist:

```bash
ros2 topic list | grep odrive
```

Check controller status is publishing:

```bash
ros2 topic echo /odrive_axis0/controller_status --once
```

If nothing prints, see [Troubleshooting](#11-troubleshooting) below.

Inspect the actual message definition if needed:

```bash
ros2 interface show odrive_can/msg/ControllerStatus
```

Request closed-loop control on the axis (state 8 = `AXIS_STATE_CLOSED_LOOP_CONTROL`):

```bash
ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"
```

## 9. Running the bridge

Via launch file (loads `config/single_joint.yaml`):

```bash
ros2 launch rover_arm_bringup single_joint_bridge.launch.py
```

Or run the node directly with parameter overrides:

```bash
ros2 run rover_arm_bringup single_joint_odrive_bridge --ros-args \
  -p joint_name:=base_joint \
  -p odrive_ns:=/odrive_axis0 \
  -p gear_ratio:=80.0 \
  -p direction:=1.0 \
  -p zero_offset_turns:=0.0 \
  -p max_joint_step_rad:=0.05
```

The node will log that it is waiting for ODrive feedback, and **will not
send any command automatically on startup.**

### How to set zero

Because there is no output-side encoder, ROS does not know the true joint
zero after a reboot — only motor turns relative to wherever the motor
happened to be. The procedure is:

1. Physically place the joint at a known, safe "zero" pose.
2. Start the ODrive CAN node (Section 8) and confirm it is in closed-loop control.
3. Start the bridge (this section).
4. Confirm feedback is arriving: `ros2 topic echo /joint_states`.
5. Call the zero service:

   ```bash
   ros2 service call /base_joint/set_zero std_srvs/srv/Trigger "{}"
   ```

6. Only now send motion commands. `set_zero` sets `zero_offset_turns` to the
   current motor position, so the current physical pose becomes `0.0 rad`.

This manual procedure is expected and acceptable for bring-up. Later this
should be replaced by automatic homing (limit switch, hard stop, or an
output-side encoder) — see Section 13.

### Command a small movement

```bash
ros2 topic pub --once /base_joint/target_position_rad std_msgs/msg/Float64 "{data: 0.03}"
```

Return to zero:

```bash
ros2 topic pub --once /base_joint/target_position_rad std_msgs/msg/Float64 "{data: 0.0}"
```

Watch `/joint_states` update:

```bash
ros2 topic echo /joint_states
```

### Trajectory test (final-point only, bring-up behavior)

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{ joint_names: ['base_joint'], points: [ { positions: [0.04], velocities: [0.0], time_from_start: {sec: 2, nanosec: 0} } ] }"
```

### Reverse direction

If the joint physically moves the wrong way relative to a positive command,
flip the sign of `direction` in `config/single_joint.yaml`:

```yaml
direction: -1.0
```

or override at launch:

```bash
ros2 run rover_arm_bringup single_joint_odrive_bridge --ros-args -p direction:=-1.0
```

## 10. Visualizing in RViz

```bash
ros2 launch rover_arm_bringup one_joint_rviz.launch.py
```

This starts `robot_state_publisher` with the one-joint URDF and (by
default) the bridge node. In a separate terminal:

```bash
rviz2
```

In RViz:

1. Set **Fixed Frame** to `base_link`.
2. Click **Add** → **RobotModel**.
3. Send a target via `/base_joint/target_position_rad` (Section 9) and
   confirm the link rotates about Z in RViz, matching the commanded radians.

If you only want RViz support without sending real motor commands, disable
the bridge node:

```bash
ros2 launch rover_arm_bringup one_joint_rviz.launch.py start_bridge:=false
```

## 11. Troubleshooting

**`/odrive_axis0/controller_status` does not publish:**
- ODrive CAN node not running.
- CAN bitrate mismatch between host and ODrive (must both be 1,000,000).
- ODrive node ID mismatch (this package assumes node ID `0`).
- CAN wiring issue (CAN_H/CAN_L swapped or open).
- Missing 120 Ω termination resistors on the bus.
- ODrive not powered, or DC bus undervoltage.
- ODrive not actually configured for CAN protocol (`odrv0.can.config.protocol`).
- Wrong interface name — confirm it's `can0`, not `can1`/`vcan0`/etc.

**Bridge logs "no ODrive feedback":**
- Confirm `/odrive_axis0/controller_status` is actually publishing (Section 8).
- Confirm the `odrive_ns` parameter matches the real ODrive namespace.
- Confirm the ODrive ROS node is launched and connected before the bridge.

**Joint moves the wrong way:**
- Flip `direction` from `1.0` to `-1.0` (Section 9).

**Joint moves too far / too fast:**
- Reduce `max_joint_step_rad`.
- Double check `gear_ratio` matches the real gearbox.
- Double check you are sending radians, not degrees.
- Confirm `input_pos` on your ODrive firmware really expects motor turns
  (not counts) — check ODrive docs/GUI for your firmware version.

**RViz model does not move:**
- Check `/joint_states` is actually publishing (`ros2 topic echo /joint_states`).
- Check the joint name in `/joint_states` matches `base_joint` in the URDF exactly.
- Check `robot_state_publisher` is running.
- Check RViz's Fixed Frame is `base_link`.

**ODrive receives CAN commands but the motor does not move:**
- Confirm the axis is actually in `AXIS_STATE_CLOSED_LOOP_CONTROL` (state 8).
- Clear ODrive errors (`odrv0.axis0.error`, or via ODrive GUI/`odrivetool`).
- Check current limits aren't preventing motion.
- Confirm `control_mode` is really position control on the ODrive side.
- Confirm encoder calibration completed successfully.
- Confirm motor calibration completed successfully.
- Check DC bus voltage is within range.

**Joint position "jumps" after a reboot:**
- Expected — see Section 9. There is no output-side encoder, so ROS cannot
  know the true joint zero after a power cycle. Always re-home (place at a
  known pose and call `set_zero`) before trusting commands after a restart.

## 12. Acceptance checklist

- [ ] `colcon build --packages-select rover_arm_bringup` succeeds.
- [ ] Node launches without crashing.
- [ ] Node subscribes to `/odrive_axis0/controller_status`.
- [ ] Node publishes `/joint_states`, containing `base_joint`.
- [ ] Published position is in radians, calculated from motor turns / gear ratio.
- [ ] `/base_joint/set_zero` works and reports the new zero offset.
- [ ] A message to `/base_joint/target_position_rad` results in a converted
      motor-turn command sent to ODrive.
- [ ] Oversized commands are clamped (`max_joint_step_rad`), and out-of-range
      commands are clamped to `[joint_min_rad, joint_max_rad]`.
- [ ] `direction` and `gear_ratio` are both changeable via parameters.
- [ ] The one-joint RViz model moves in response to `/joint_states`.

## 13. Next step: expanding to six joints

The node and config are intentionally per-joint. To expand to the full arm
(`base_joint`, `shoulder_joint`, `elbow_joint`, `wrist_pitch_joint`,
`wrist_roll_joint`, `wrist_yaw_joint`):

1. Give each joint its own `odrive_ns` (e.g. `/odrive_axis0` ... `/odrive_axis5`),
   `gear_ratio`, `direction`, `zero_offset_turns`, `joint_min_rad`/`joint_max_rad`,
   and velocity limit.
2. Either:
   - Run six instances of `single_joint_odrive_bridge`, each remapped/parameterized
     for one joint (simplest, no new code), or
   - Build `multi_joint_odrive_bridge.py`, a single node that loops over a
     list of per-joint configs and publishes one combined `/joint_states`
     message with all six joint names/positions/velocities. The conversion
     math (`motor_turns_to_joint_rad`, etc.) is already isolated per-joint in
     this package, so that refactor is mostly "wrap this class in a list."
3. Add real homing (limit switches or hard-stop homing) per joint instead of
   manual `set_zero`, since six joints made of manual zeroing is error-prone.

## 14. Final target architecture: ros2_control + MoveIt 2

This bridge node is a stand-in. The intended final architecture is:

```
MoveIt 2
   |
JointTrajectoryController (ros2_control)
   |
ros2_control hardware interface (custom, CAN-based)
   |
ODrive S1 CAN
   |
AMT motor shaft encoder feedback
   |
calculated joint state (same math as this bridge)
```

In that architecture, the conversion math in this package
(`motor_turns_to_joint_rad`, `joint_rad_to_motor_turns`,
`motor_turns_per_sec_to_joint_rad_s`) moves into a `ros2_control`
`SystemInterface::read()`/`write()` implementation instead of a bridge node,
and `JointTrajectoryController` + MoveIt 2 replace the topic/action
interfaces this node exposes for testing. Validating the math and safety
behavior here, on one joint, is what makes that later integration
low-risk.
