"""
Demo: grasp sequence using the executor action.

Open gripper → move to target pose (from pose source) → close gripper → move to home.
Uses the ExecutePose action and /gripper_command; the executor node must be running.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.logging import get_logger
from moveitpy_execute_node_msgs.action import ExecutePose

from .constants import GRIPPER_CLOSE, GRIPPER_OPEN
from .gripper import send_gripper_command
from .pose_sources import get_default_pose_source
from .utils import sleep_until_ok

# Timing (seconds)
SETTLE_TIME_S = 5.0
GRIPPER_WAIT_S = 2.0
PAUSE_AFTER_MOVE_S = 0.3

ACTION_NAME = "execute_pose"


def run_grasp_sequence(node: rclpy.node.Node) -> None:
    logger = node.get_logger()
    pose_source = get_default_pose_source()

    client = ActionClient(node, ExecutePose, ACTION_NAME)
    logger.info(f"Waiting for action server '{ACTION_NAME}'...")
    if not client.wait_for_server(timeout_sec=10.0):
        logger.error("Action server not available. Is the executor node running?")
        return

    # Give scene and bridge time to be ready
    sleep_until_ok(0.5)

    logger.info("Open gripper")
    send_gripper_command(node, GRIPPER_OPEN)
    sleep_until_ok(GRIPPER_WAIT_S)

    target = pose_source.get_target_pose()
    target.header.stamp = node.get_clock().now().to_msg()
    logger.info("Move to target pose")
    _send_pose_and_wait(node, client, target, logger)
    sleep_until_ok(PAUSE_AFTER_MOVE_S)

    logger.info("Close gripper")
    send_gripper_command(node, GRIPPER_CLOSE)
    sleep_until_ok(GRIPPER_WAIT_S)

    home = pose_source.get_home_pose()
    home.header.stamp = node.get_clock().now().to_msg()
    logger.info("Move to home")
    _send_pose_and_wait(node, client, home, logger)
    logger.info("Grasp sequence done.")


def _send_pose_and_wait(node, client, pose, logger) -> None:
    goal_msg = ExecutePose.Goal()
    goal_msg.target_pose = pose
    send_future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        logger.error("Goal rejected")
        return
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    result = result_future.result().result
    if not result.success:
        logger.warning(f"ExecutePose failed: {result.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("demo_grasp_sequence")
    logger = get_logger("moveitpy_execute_node.demo_grasp_sequence")
    logger.info(f"Waiting {SETTLE_TIME_S}s for scene to settle...")
    sleep_until_ok(SETTLE_TIME_S)
    try:
        run_grasp_sequence(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
