#!/usr/bin/env python3
"""
Trajectory bridge for Panda: implements FollowJointTrajectory action server
and publishes interpolated joint commands to /joint_command for Isaac Sim.

MoveIt sends trajectories via the action; this node interpolates at a fixed
rate and publishes sensor_msgs/JointState so Isaac's articulation controller
can follow. Gripper state is passed through from /joint_states (no trajectory).
"""

import signal
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

INTERP_HZ = 125
INTERP_DT = 1.0 / INTERP_HZ

PANDA_ARM_JOINTS = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]

PANDA_GRIPPER_JOINTS = [
    "panda_finger_joint1",
    "panda_finger_joint2",
]

ALL_JOINTS = PANDA_ARM_JOINTS + PANDA_GRIPPER_JOINTS

# Must match MoveIt controller config (gripper_moveit_controllers.yaml)
ARM_ACTION = "panda_arm_controller/follow_joint_trajectory"


class TrajectoryBridgeNode(Node):
    def __init__(self):
        super().__init__("trajectory_bridge")
        self._cancel_event = threading.Event()
        self._trajectory_canceled = False
        self._old_sigint = signal.signal(
            signal.SIGINT,
            lambda s, f: self._on_sigint(s, f),
        )
        self._pub = self.create_publisher(JointState, "/joint_command", 10)
        self._state_lock = threading.Lock()
        self._latest_position = {}
        self._latest_velocity = {}
        self._sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_cb,
            10,
        )
        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            ARM_ACTION,
            self._execute_callback,
        )
        self.get_logger().info(
            f"Trajectory bridge: {ARM_ACTION} -> /joint_command (arm + gripper)"
        )

    def _on_sigint(self, signum, frame):
        self._cancel_event.set()
        if callable(self._old_sigint):
            self._old_sigint(signum, frame)

    def _joint_states_cb(self, msg):
        with self._state_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._latest_position[name] = msg.position[i]
                if msg.velocity and i < len(msg.velocity):
                    self._latest_velocity[name] = msg.velocity[i]

    def _run_trajectory(self, trajectory):
        traj_names = list(trajectory.joint_names)
        waypoints = []
        with self._state_lock:
            base_pos = dict(self._latest_position)
            base_vel = dict(self._latest_velocity)
        for point in trajectory.points:
            t_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            pos = dict(base_pos)
            vel = dict(base_vel)
            if point.positions and len(point.positions) == len(trajectory.joint_names):
                for name, p in zip(trajectory.joint_names, point.positions):
                    pos[name] = p
            if point.velocities and len(point.velocities) == len(trajectory.joint_names):
                for name, v in zip(trajectory.joint_names, point.velocities):
                    vel[name] = v
            waypoints.append((t_sec, pos, vel))
        if not waypoints:
            return
        t_end = waypoints[-1][0]
        start_wall = time.perf_counter()
        t_elapsed = 0.0
        while rclpy.ok() and not self._cancel_event.is_set() and t_elapsed < t_end:
            next_tick = start_wall + (int(t_elapsed / INTERP_DT) + 1) * INTERP_DT
            now = time.perf_counter()
            wait_sec = min(INTERP_DT, next_tick - now)
            if wait_sec > 0 and self._cancel_event.wait(timeout=wait_sec):
                self._trajectory_canceled = True
                return
            t_elapsed = time.perf_counter() - start_wall
            if t_elapsed >= t_end or self._cancel_event.is_set():
                if self._cancel_event.is_set():
                    self._trajectory_canceled = True
                break
            t = min(t_elapsed, t_end)
            i = 0
            while i + 1 < len(waypoints) and waypoints[i + 1][0] <= t:
                i += 1
            t0, pos0, vel0 = waypoints[i]
            if i + 1 < len(waypoints):
                t1, pos1, vel1 = waypoints[i + 1]
                dt_seg = t1 - t0
                alpha = (t - t0) / dt_seg if dt_seg > 0 else 1.0
                pos = {
                    name: (1 - alpha) * pos0.get(name, 0.0) + alpha * pos1.get(name, 0.0)
                    for name in PANDA_ARM_JOINTS
                }
                vel = {
                    name: (1 - alpha) * vel0.get(name, 0.0) + alpha * vel1.get(name, 0.0)
                    for name in PANDA_ARM_JOINTS
                }
            else:
                pos = {name: pos0.get(name, 0.0) for name in PANDA_ARM_JOINTS}
                vel = {name: vel0.get(name, 0.0) for name in PANDA_ARM_JOINTS}
            with self._state_lock:
                finger1 = self._latest_position.get("panda_finger_joint1", 0.04)
                finger2 = self._latest_position.get("panda_finger_joint2", 0.04)
                v1 = self._latest_velocity.get("panda_finger_joint1", 0.0)
                v2 = self._latest_velocity.get("panda_finger_joint2", 0.0)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(ALL_JOINTS)
            msg.position = [pos[name] for name in PANDA_ARM_JOINTS] + [finger1, finger2]
            msg.velocity = [vel[name] for name in PANDA_ARM_JOINTS] + [v1, v2]
            msg.effort = []
            self._pub.publish(msg)
        t_final, pos_final, vel_final = waypoints[-1]
        with self._state_lock:
            finger1 = self._latest_position.get("panda_finger_joint1", 0.04)
            finger2 = self._latest_position.get("panda_finger_joint2", 0.04)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [pos_final.get(n, 0.0) for n in PANDA_ARM_JOINTS] + [finger1, finger2]
        msg.velocity = [0.0] * len(ALL_JOINTS)
        msg.effort = []
        self._pub.publish(msg)

    def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()
        self._cancel_event.clear()
        self._trajectory_canceled = False
        thread = threading.Thread(target=self._run_trajectory, args=(trajectory,), daemon=True)
        thread.start()
        thread.join()
        result = FollowJointTrajectory.Result()
        if self._trajectory_canceled:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            goal_handle.canceled()
            return result
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
