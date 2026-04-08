#!/usr/bin/env python3
"""
Trajectory bridge for Panda: implements FollowJointTrajectory action server
and publishes interpolated joint commands to /joint_command for Isaac Sim.
After each trajectory, idle publishing repeats the last commanded arm positions (setpoint hold),
not /joint_states feedback, so the sim keeps applying corrective torque instead of drifting.
"""

import bisect
import signal
import threading
import time
from typing import List, Tuple

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .constants import (
    ALL_JOINTS,
    GRIPPER_CMD_TOPIC,
    GRIPPER_OPEN,
    PANDA_ARM_JOINTS,
    PANDA_GRIPPER_JOINTS,
)

INTERP_HZ = 125
INTERP_DT = 1.0 / INTERP_HZ
IDLE_HZ = 20
NANOSEC_TO_SEC = 1e-9
ARM_ACTION = "panda_arm_controller/follow_joint_trajectory"


def duration_to_sec(d) -> float:
    """Convert duration-like (sec, nanosec) to seconds."""
    return d.sec + d.nanosec * NANOSEC_TO_SEC


class TrajectoryBridgeNode(Node):
    """Bridges FollowJointTrajectory actions to /joint_command (JointState) for Isaac Sim."""

    def __init__(self) -> None:
        super().__init__("trajectory_bridge")
        self._cancel_event = threading.Event()
        self._trajectory_canceled = False
        try:
            self._old_sigint = signal.signal(signal.SIGINT, self._on_sigint)
        except (ValueError, AttributeError):
            self._old_sigint = None
        self._pub = self.create_publisher(JointState, "/joint_command", 10)
        self._state_lock = threading.Lock()
        self._hold_lock = threading.Lock()
        self._hold_arm_positions: list[float] | None = None
        self._latest_position: dict = {}
        self._latest_velocity: dict = {}
        self._gripper_target: float = GRIPPER_OPEN
        self._executing: bool = False
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 10)
        self.create_subscription(Float64, GRIPPER_CMD_TOPIC, self._gripper_cmd_cb, 10)
        self._idle_timer = self.create_timer(1.0 / IDLE_HZ, self._idle_cb)
        self._arm_server = ActionServer(
            self, FollowJointTrajectory, ARM_ACTION, self._execute_callback
        )
        self.get_logger().info(
            f"Trajectory bridge: {ARM_ACTION} + {GRIPPER_CMD_TOPIC} -> /joint_command (single publisher)"
        )

    def _on_sigint(self, signum, frame) -> None:
        self._cancel_event.set()
        if callable(self._old_sigint):
            self._old_sigint(signum, frame)

    def _joint_states_cb(self, msg: JointState) -> None:
        with self._state_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._latest_position[name] = msg.position[i]
                if msg.velocity and i < len(msg.velocity):
                    self._latest_velocity[name] = msg.velocity[i]

    def _gripper_cmd_cb(self, msg: Float64) -> None:
        self._gripper_target = float(msg.data)

    def _get_arm_state(self) -> Tuple[List[float], List[float]]:
        """Return (positions, velocities) for arm joints from latest /joint_states."""
        with self._state_lock:
            pos = [self._latest_position.get(j, 0.0) for j in PANDA_ARM_JOINTS]
            vel = [self._latest_velocity.get(j, 0.0) for j in PANDA_ARM_JOINTS]
        return pos, vel

    def _build_joint_state_msg(
        self,
        arm_positions: List[float],
        arm_velocities: List[float],
        gripper_positions: List[float],
        gripper_velocities: List[float],
    ) -> JointState:
        """Build a JointState for /joint_command."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = arm_positions + gripper_positions
        msg.velocity = arm_velocities + gripper_velocities
        msg.effort = []
        return msg

    def _waypoints_from_trajectory(self, trajectory) -> List[Tuple[float, dict, dict]]:
        """Convert trajectory to list of (time_sec, position_dict, velocity_dict)."""
        with self._state_lock:
            base_pos = dict(self._latest_position)
            base_vel = dict(self._latest_velocity)
        waypoints = []
        joint_names = trajectory.joint_names
        for point in trajectory.points:
            t_sec = duration_to_sec(point.time_from_start)
            pos = dict(base_pos)
            vel = dict(base_vel)
            if point.positions and len(point.positions) == len(joint_names):
                for name, p in zip(joint_names, point.positions):
                    pos[name] = p
            if point.velocities and len(point.velocities) == len(joint_names):
                for name, v in zip(joint_names, point.velocities):
                    vel[name] = v
            waypoints.append((t_sec, pos, vel))
        return waypoints

    def _interpolate(
        self, waypoints: List[Tuple[float, dict, dict]], t: float
    ) -> Tuple[dict, dict]:
        """Linear interpolation at time t. Returns (position_dict, velocity_dict) for arm joints."""
        times = [wp[0] for wp in waypoints]
        idx = bisect.bisect_right(times, t) - 1
        idx = max(0, min(idx, len(waypoints) - 1))
        t0, pos0, vel0 = waypoints[idx]
        if idx + 1 < len(waypoints):
            t1, pos1, vel1 = waypoints[idx + 1]
            dt = t1 - t0
            alpha = (t - t0) / dt if dt > 0 else 1.0
            pos = {
                n: (1 - alpha) * pos0.get(n, 0.0) + alpha * pos1.get(n, 0.0)
                for n in PANDA_ARM_JOINTS
            }
            vel = {
                n: (1 - alpha) * vel0.get(n, 0.0) + alpha * vel1.get(n, 0.0)
                for n in PANDA_ARM_JOINTS
            }
        else:
            pos = {n: pos0.get(n, 0.0) for n in PANDA_ARM_JOINTS}
            vel = {n: vel0.get(n, 0.0) for n in PANDA_ARM_JOINTS}
        return pos, vel

    def _run_trajectory(self, trajectory) -> None:
        """Interpolate trajectory at INTERP_HZ and publish to /joint_command."""
        waypoints = self._waypoints_from_trajectory(trajectory)
        if not waypoints:
            return
        t_end = waypoints[-1][0]
        start_wall = time.perf_counter()
        while rclpy.ok() and not self._cancel_event.is_set():
            t_elapsed = time.perf_counter() - start_wall
            if t_elapsed >= t_end:
                break
            next_tick = start_wall + (int(t_elapsed / INTERP_DT) + 1) * INTERP_DT
            wait_sec = min(INTERP_DT, next_tick - time.perf_counter())
            if wait_sec > 0 and self._cancel_event.wait(timeout=wait_sec):
                self._trajectory_canceled = True
                return
            t = min(time.perf_counter() - start_wall, t_end)
            pos, vel = self._interpolate(waypoints, t)
            g = self._gripper_target
            msg = self._build_joint_state_msg(
                [pos[n] for n in PANDA_ARM_JOINTS],
                [vel[n] for n in PANDA_ARM_JOINTS],
                [g, g],
                [0.0, 0.0],
            )
            self._pub.publish(msg)
        # Final state
        _, pos_final, vel_final = waypoints[-1]
        g = self._gripper_target
        msg = self._build_joint_state_msg(
            [pos_final.get(n, 0.0) for n in PANDA_ARM_JOINTS],
            [0.0] * len(PANDA_ARM_JOINTS),
            [g, g],
            [0.0, 0.0],
        )
        self._pub.publish(msg)
        with self._hold_lock:
            self._hold_arm_positions = [pos_final.get(n, 0.0) for n in PANDA_ARM_JOINTS]

    def _idle_cb(self) -> None:
        """Hold last commanded arm setpoint (not measured state) so Isaac keeps applying PD."""
        if self._executing:
            return
        g = self._gripper_target
        with self._hold_lock:
            hold = list(self._hold_arm_positions) if self._hold_arm_positions is not None else None
        if hold is not None:
            msg = self._build_joint_state_msg(
                hold,
                [0.0] * len(PANDA_ARM_JOINTS),
                [g, g],
                [0.0, 0.0],
            )
            self._pub.publish(msg)
            return
        with self._state_lock:
            if not all(j in self._latest_position for j in PANDA_ARM_JOINTS):
                return
        arm_pos, arm_vel = self._get_arm_state()
        msg = self._build_joint_state_msg(arm_pos, arm_vel, [g, g], [0.0, 0.0])
        self._pub.publish(msg)

    def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()
        self._cancel_event.clear()
        self._trajectory_canceled = False
        self._executing = True
        try:
            thread = threading.Thread(target=self._run_trajectory, args=(trajectory,), daemon=True)
            thread.start()
            thread.join()
        finally:
            self._executing = False
        result = FollowJointTrajectory.Result()
        if self._trajectory_canceled:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            goal_handle.canceled()
        else:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
