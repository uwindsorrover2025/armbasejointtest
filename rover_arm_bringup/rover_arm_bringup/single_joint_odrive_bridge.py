#!/usr/bin/env python3
"""Single-joint ODrive CAN bring-up bridge.

Bridges one rover-arm joint, driven by an ODrive S1 + AMT motor-shaft
encoder through an 80:1 (configurable) harmonic gearbox, into ROS 2.

There is no output-side encoder. The only position feedback is the motor
shaft encoder read through ODrive. This node therefore calculates the real
gearbox-output joint angle in software:

    joint_position_rad = direction * (motor_turns - zero_offset_turns)
                          * 2*pi / gear_ratio

and the reverse, for outgoing commands:

    motor_target_turns = zero_offset_turns
                          + direction * joint_target_rad * gear_ratio / (2*pi)

This is a bring-up bridge, not the final architecture. The eventual system
will replace this node with a ros2_control hardware interface driven by
MoveIt 2 + JointTrajectoryController. This node exists so that one joint can
be proven end-to-end (encoder -> ODrive -> ROS -> /joint_states -> RViz)
before that larger integration is attempted.
"""

import asyncio
import math
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from control_msgs.action import FollowJointTrajectory
from odrive_can.msg import ControlMessage, ControllerStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

TWO_PI = 2.0 * math.pi

# ODrive control_mode / input_mode values used by the outgoing ControlMessage.
# These match the odrive_can ROS 2 package's ODrive enum values. Verify on
# your installed version with:
#   ros2 interface show odrive_can/msg/ControlMessage
ODRIVE_CONTROL_MODE_POSITION_CONTROL = 3  # ODrive CONTROL_MODE_POSITION_CONTROL
ODRIVE_INPUT_MODE_PASSTHROUGH = 1         # ODrive INPUT_MODE_PASSTHROUGH (no internal shaping)


class SingleJointODriveBridge(Node):
    """Bridges one ODrive-driven gearbox joint into /joint_states and back."""

    def __init__(self) -> None:
        super().__init__('single_joint_odrive_bridge')

        self._declare_parameters()
        self._load_parameters()

        # Feedback state. We never publish or command motion based on
        # default/zero values before real feedback has arrived.
        self._have_feedback: bool = False
        self._last_motor_pos_turns: float = 0.0
        self._last_motor_vel_turns_s: float = 0.0
        self._last_commanded_joint_rad: Optional[float] = None

        # --- Publishers ---
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._control_pub = self.create_publisher(
            ControlMessage, f'{self._odrive_ns}/control_message', 10)

        # --- Subscriptions ---
        self._status_sub = self.create_subscription(
            ControllerStatus,
            f'{self._odrive_ns}/controller_status',
            self._controller_status_callback,
            10,
        )
        self._target_sub = self.create_subscription(
            Float64,
            f'/{self._joint_name}/target_position_rad',
            self._target_position_callback,
            10,
        )
        self._trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._joint_trajectory_callback,
            10,
        )

        # --- Service ---
        self._set_zero_srv = self.create_service(
            Trigger, f'/{self._joint_name}/set_zero', self._set_zero_callback)

        # --- Optional action server for simple MoveIt-style testing ---
        self._cancel_requested = False
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory_action,
            goal_callback=self._trajectory_goal_callback,
            cancel_callback=self._trajectory_cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # --- Periodic /joint_states publishing ---
        publish_period_s = 1.0 / self._publish_rate_hz
        self._publish_timer = self.create_timer(publish_period_s, self.publish_joint_state)

        self.get_logger().info(
            f"single_joint_odrive_bridge started for joint '{self._joint_name}' "
            f"(odrive_ns={self._odrive_ns}, gear_ratio={self._gear_ratio}, "
            f"direction={self._direction}, zero_offset_turns={self._zero_offset_turns}). "
            f"No command will be sent automatically. Waiting for ODrive feedback."
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('joint_name', 'base_joint')
        self.declare_parameter('odrive_ns', '/odrive_axis0')
        self.declare_parameter('gear_ratio', 80.0)
        self.declare_parameter('direction', 1.0)
        self.declare_parameter('zero_offset_turns', 0.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_joint_step_rad', 0.05)
        self.declare_parameter('joint_min_rad', -1.57)
        self.declare_parameter('joint_max_rad', 1.57)
        self.declare_parameter('allow_large_commands', False)
        self.declare_parameter('require_feedback_before_command', True)

    def _load_parameters(self) -> None:
        self._joint_name: str = self.get_parameter('joint_name').get_parameter_value().string_value
        self._odrive_ns: str = self.get_parameter('odrive_ns').get_parameter_value().string_value
        self._gear_ratio: float = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self._direction: float = self.get_parameter('direction').get_parameter_value().double_value
        self._zero_offset_turns: float = self.get_parameter(
            'zero_offset_turns').get_parameter_value().double_value
        self._publish_rate_hz: float = self.get_parameter(
            'publish_rate_hz').get_parameter_value().double_value
        self._max_joint_step_rad: float = self.get_parameter(
            'max_joint_step_rad').get_parameter_value().double_value
        self._joint_min_rad: float = self.get_parameter('joint_min_rad').get_parameter_value().double_value
        self._joint_max_rad: float = self.get_parameter('joint_max_rad').get_parameter_value().double_value
        self._allow_large_commands: bool = self.get_parameter(
            'allow_large_commands').get_parameter_value().bool_value
        self._require_feedback_before_command: bool = self.get_parameter(
            'require_feedback_before_command').get_parameter_value().bool_value

        if self._gear_ratio == 0.0:
            raise ValueError('gear_ratio parameter must not be zero')
        if self._publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz parameter must be positive')

    # ------------------------------------------------------------------
    # Conversion helpers
    # ------------------------------------------------------------------
    def motor_turns_to_joint_rad(self, motor_turns: float) -> float:
        """Convert ODrive motor-shaft turns into gearbox output joint radians."""
        return self._direction * (motor_turns - self._zero_offset_turns) * TWO_PI / self._gear_ratio

    def motor_turns_per_sec_to_joint_rad_s(self, motor_vel_turns_per_sec: float) -> float:
        """Convert ODrive motor-shaft turns/sec into joint radians/sec."""
        return self._direction * motor_vel_turns_per_sec * TWO_PI / self._gear_ratio

    def joint_rad_to_motor_turns(self, joint_rad: float) -> float:
        """Convert a desired joint angle (radians) into ODrive motor turns."""
        return self._zero_offset_turns + self._direction * joint_rad * self._gear_ratio / TWO_PI

    def clamp_joint_target(self, target_joint_rad: float) -> float:
        """Clamp a requested joint target to joint limits and the per-command step limit."""
        clamped = target_joint_rad

        if clamped < self._joint_min_rad or clamped > self._joint_max_rad:
            self.get_logger().warn(
                f'Target {target_joint_rad:.4f} rad is outside joint limits '
                f'[{self._joint_min_rad:.4f}, {self._joint_max_rad:.4f}] rad - clamping to limits.'
            )
            clamped = max(self._joint_min_rad, min(self._joint_max_rad, clamped))

        current_joint_rad = self.motor_turns_to_joint_rad(self._last_motor_pos_turns)
        step = clamped - current_joint_rad

        if not self._allow_large_commands and abs(step) > self._max_joint_step_rad:
            step_sign = 1.0 if step > 0.0 else -1.0
            limited = current_joint_rad + step_sign * self._max_joint_step_rad
            self.get_logger().warn(
                f'Requested step {step:.4f} rad exceeds max_joint_step_rad='
                f'{self._max_joint_step_rad:.4f} rad - clamping target from '
                f'{clamped:.4f} rad to {limited:.4f} rad. Set allow_large_commands:=true '
                f'to override (not recommended on real hardware).'
            )
            clamped = limited

        return clamped

    # ------------------------------------------------------------------
    # Feedback handling
    # ------------------------------------------------------------------
    def _controller_status_callback(self, msg: ControllerStatus) -> None:
        self._last_motor_pos_turns = msg.pos_estimate
        self._last_motor_vel_turns_s = msg.vel_estimate

        if not self._have_feedback:
            self.get_logger().info('Received first ODrive feedback. Commands are now permitted.')
        self._have_feedback = True

        joint_rad = self.motor_turns_to_joint_rad(self._last_motor_pos_turns)
        self.get_logger().debug(
            f'Received motor position: {self._last_motor_pos_turns:.4f} turns | '
            f'Calculated joint position: {joint_rad:.4f} rad'
        )

    # ------------------------------------------------------------------
    # /joint_states publishing
    # ------------------------------------------------------------------
    def publish_joint_state(self) -> None:
        """Publish the calculated joint position/velocity in radians.

        Nothing is published until real ODrive feedback has been received,
        so downstream consumers never see a fabricated zero position.
        """
        if not self._have_feedback:
            return

        joint_rad = self.motor_turns_to_joint_rad(self._last_motor_pos_turns)
        joint_vel_rad_s = self.motor_turns_per_sec_to_joint_rad_s(self._last_motor_vel_turns_s)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self._joint_name]
        msg.position = [joint_rad]
        msg.velocity = [joint_vel_rad_s]
        msg.effort = []  # not measured in this bring-up version

        self._joint_state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Command handling
    # ------------------------------------------------------------------
    def send_joint_position_command(self, target_joint_rad: float) -> None:
        """Validate, clamp, convert, and send a joint-space position command to ODrive."""
        if self._require_feedback_before_command and not self._have_feedback:
            self.get_logger().warn(
                'No ODrive feedback received yet - refusing to send command '
                f'(requested {target_joint_rad:.4f} rad).'
            )
            return

        clamped_joint_rad = self.clamp_joint_target(target_joint_rad)
        motor_target_turns = self.joint_rad_to_motor_turns(clamped_joint_rad)

        self.get_logger().info(
            f'Command requested: {target_joint_rad:.4f} rad | '
            f'Command clamped: {clamped_joint_rad:.4f} rad | '
            f'Sending ODrive motor target: {motor_target_turns:.4f} turns'
        )

        control_msg = ControlMessage()
        control_msg.control_mode = ODRIVE_CONTROL_MODE_POSITION_CONTROL  # position control
        control_msg.input_mode = ODRIVE_INPUT_MODE_PASSTHROUGH           # passthrough, no shaping
        control_msg.input_pos = float(motor_target_turns)
        control_msg.input_vel = 0.0
        control_msg.input_torque = 0.0

        self._control_pub.publish(control_msg)
        self._last_commanded_joint_rad = clamped_joint_rad

    def _target_position_callback(self, msg: Float64) -> None:
        self.get_logger().info(f'Received target_position_rad: {msg.data:.4f} rad')
        self.send_joint_position_command(msg.data)

    def _joint_trajectory_callback(self, msg: JointTrajectory) -> None:
        """Bring-up only: command the final trajectory point, no interpolation."""
        if self._joint_name not in msg.joint_names:
            self.get_logger().warn(
                f"Received JointTrajectory without configured joint '{self._joint_name}' "
                f'in joint_names={list(msg.joint_names)} - ignoring.'
            )
            return
        if not msg.points:
            self.get_logger().warn('Received empty JointTrajectory - ignoring.')
            return

        index = msg.joint_names.index(self._joint_name)
        final_point = msg.points[-1]
        if index >= len(final_point.positions):
            self.get_logger().warn(
                'Final trajectory point is missing a position for the configured joint - ignoring.'
            )
            return

        target_rad = final_point.positions[index]
        self.get_logger().info(
            f'JointTrajectory received with {len(msg.points)} point(s); '
            f'commanding final point only (bring-up mode, no interpolation): {target_rad:.4f} rad'
        )
        self.send_joint_position_command(target_rad)

    # ------------------------------------------------------------------
    # set_zero service
    # ------------------------------------------------------------------
    def _set_zero_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._have_feedback:
            response.success = False
            response.message = 'Cannot zero: no ODrive feedback has been received yet.'
            self.get_logger().warn(response.message)
            return response

        self._zero_offset_turns = self._last_motor_pos_turns
        # Keep the ROS parameter in sync so `ros2 param get` reflects reality.
        self.set_parameters(
            [Parameter('zero_offset_turns', Parameter.Type.DOUBLE, self._zero_offset_turns)]
        )

        response.success = True
        response.message = (
            f'Zero set. New zero_offset_turns = {self._zero_offset_turns:.6f} turns. '
            f'Current physical joint position is now 0.0 rad.'
        )
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Optional FollowJointTrajectory action server (bring-up / MoveIt testing)
    # ------------------------------------------------------------------
    def _trajectory_goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        trajectory = goal_request.trajectory
        if self._joint_name not in trajectory.joint_names:
            self.get_logger().warn(
                f"FollowJointTrajectory goal rejected: joint '{self._joint_name}' not present "
                f'in goal joint_names={list(trajectory.joint_names)}.'
            )
            return GoalResponse.REJECT
        if not trajectory.points:
            self.get_logger().warn('FollowJointTrajectory goal rejected: trajectory has no points.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _trajectory_cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('FollowJointTrajectory cancel requested.')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def _execute_trajectory_action(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        index = trajectory.joint_names.index(self._joint_name)
        self._cancel_requested = False

        result = FollowJointTrajectory.Result()
        start_time = self.get_clock().now()

        for point_num, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested or self._cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = 'Trajectory canceled by request.'
                self.get_logger().info('FollowJointTrajectory goal canceled.')
                return result

            if index >= len(point.positions):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Trajectory point missing position for configured joint.'
                self.get_logger().warn(result.error_string)
                return result

            target_rad = point.positions[index]
            self.get_logger().info(
                f'FollowJointTrajectory: commanding point {point_num + 1}/{len(trajectory.points)} '
                f'-> {target_rad:.4f} rad'
            )
            self.send_joint_position_command(target_rad)

            target_time = start_time + Duration(
                seconds=point.time_from_start.sec,
                nanoseconds=point.time_from_start.nanosec,
            )
            while self.get_clock().now() < target_time:
                if goal_handle.is_cancel_requested or self._cancel_requested:
                    break
                await asyncio.sleep(0.05)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Trajectory execution complete.'
        self.get_logger().info('FollowJointTrajectory goal succeeded.')
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SingleJointODriveBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
