"""
Send one fixed ExecutePose goal to verify panda_link0 matches Isaac.

Default: position (0.4, 0, 0.3) m with tip +Z aligned to -world Z (common "point down"
check). Must match launch ``cartesian_tip_link`` (Isaac launch defaults to panda_hand):
the quaternion is the desired **tip link** orientation in ``panda_link0``.

Prereq: executor + trajectory bridge + Isaac (or sim) running.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation

from moveitpy_execute_node_msgs.action import ExecutePose

from .constants import FRAME_ID
from .utils import sleep_until_ok

ACTION_NAME = "execute_pose"


def quat_tip_z_along_minus_world_z() -> tuple[float, float, float, float]:
    """
    Tip frame basis in panda_link0: X = world +X, Y = world -Y, Z = world -Z.
    Assumes tip +Z is the "pointing" axis; adjust with --rpy-deg if your URDF differs.
    """
    r_mat = (
        (1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
    )
    q = Rotation.from_matrix(r_mat).as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def pose_from_cli(
    x: float,
    y: float,
    z: float,
    *,
    rpy_deg: tuple[float, float, float] | None,
) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = FRAME_ID
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    if rpy_deg is not None:
        roll, pitch, yaw = rpy_deg
        q = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_quat()
    else:
        q = quat_tip_z_along_minus_world_z()
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg


def send_goal(
    node: rclpy.node.Node,
    pose: PoseStamped,
    *,
    plan_only: bool,
    timeout_result: float,
) -> tuple[bool, str]:
    client = ActionClient(node, ExecutePose, ACTION_NAME)
    node.get_logger().info(f"Waiting for action server '{ACTION_NAME}'...")
    if not client.wait_for_server(timeout_sec=15.0):
        return False, "Action server not available (is executor_node running?)"

    pose.header.stamp = node.get_clock().now().to_msg()
    goal = ExecutePose.Goal()
    goal.target_pose = pose
    goal.plan_only = plan_only

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    gh = send_future.result()
    if gh is None or not gh.accepted:
        return False, "Goal rejected"

    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_result)
    fr = result_future.result()
    if fr is None or fr.result is None:
        return False, "No result from action server"
    r = fr.result
    return bool(r.success), r.message or ""


def main(args: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Single ExecutePose to a fixed goal (verify MoveIt vs Isaac frames).",
    )
    p.add_argument("--x", type=float, default=0.4, help="Goal position x in panda_link0 (m)")
    p.add_argument("--y", type=float, default=0.0, help="Goal position y in panda_link0 (m)")
    p.add_argument("--z", type=float, default=0.3, help="Goal position z in panda_link0 (m)")
    p.add_argument(
        "--rpy-deg",
        nargs=3,
        type=float,
        default=None,
        metavar=("R", "P", "Y"),
        help="Override orientation: intrinsic Euler XYZ (deg) for the **tip link** in panda_link0. "
        "If omitted, uses tip +Z parallel to -world Z, tip X parallel to +world X.",
    )
    p.add_argument(
        "--plan-only",
        action="store_true",
        help="Only plan, do not execute (faster check that the pose is reachable).",
    )
    p.add_argument("--settle", type=float, default=1.0, help="Sleep (s) before sending goal")
    p.add_argument("--timeout", type=float, default=60.0, help="Action result timeout (s)")
    parsed, ros_args = p.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = rclpy.create_node("demo_fixed_pose")
    logger = node.get_logger()

    rpy: tuple[float, float, float] | None = None
    if parsed.rpy_deg is not None:
        rpy = (parsed.rpy_deg[0], parsed.rpy_deg[1], parsed.rpy_deg[2])

    pose = pose_from_cli(parsed.x, parsed.y, parsed.z, rpy_deg=rpy)
    logger.info(
        f"Goal frame={pose.header.frame_id} pos=({parsed.x}, {parsed.y}, {parsed.z}) "
        f"quat_xyzw=({pose.pose.orientation.x:.4f}, {pose.pose.orientation.y:.4f}, "
        f"{pose.pose.orientation.z:.4f}, {pose.pose.orientation.w:.4f}) "
        f"plan_only={parsed.plan_only}. "
        "Tip orientation is for the launch cartesian_tip_link (default panda_hand)."
    )

    try:
        if parsed.settle > 0:
            sleep_until_ok(parsed.settle)
        ok, msg = send_goal(
            node,
            pose,
            plan_only=parsed.plan_only,
            timeout_result=parsed.timeout,
        )
        if ok:
            logger.info(f"OK: {msg}")
            return 0
        logger.error(f"Failed: {msg}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
