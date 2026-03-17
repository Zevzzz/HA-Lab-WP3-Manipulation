"""
Demo: continuously plan and execute to random end-effector poses.

Uses the ExecutePose action; the executor node must be running.
"""

import random
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveitpy_execute_node_msgs.action import ExecutePose
from scipy.spatial.transform import Rotation

from .constants import FRAME_ID
from .utils import sleep_until_ok

# Workspace bounds (x, y, z) in frame_id
POSITION_X_RANGE = (0.25, 0.55)
POSITION_Y_RANGE = (-0.30, 0.30)
POSITION_Z_RANGE = (0.20, 0.60)

SETTLE_TIME_S = 5.0
INTER_ATTEMPT_S = 3.0
MAX_SAMPLES_PER_ATTEMPT = 5
ACTION_NAME = "execute_pose"


def random_pose_stamped() -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = FRAME_ID
    pose.pose.position.x = random.uniform(*POSITION_X_RANGE)
    pose.pose.position.y = random.uniform(*POSITION_Y_RANGE)
    pose.pose.position.z = random.uniform(*POSITION_Z_RANGE)
    q = Rotation.random().as_quat()  # (x, y, z, w)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def run_random_pose_loop(node: rclpy.node.Node) -> None:
    logger = node.get_logger()
    client = ActionClient(node, ExecutePose, ACTION_NAME)
    logger.info(f"Waiting for action server '{ACTION_NAME}'...")
    if not client.wait_for_server(timeout_sec=10.0):
        logger.error("Action server not available. Is the executor node running?")
        return

    attempt = 0
    while rclpy.ok():
        attempt += 1
        logger.info(f"Attempt {attempt}: sampling random pose goal")
        success = False
        for sample in range(1, MAX_SAMPLES_PER_ATTEMPT + 1):
            pose = random_pose_stamped()
            pose.header.stamp = node.get_clock().now().to_msg()
            p = pose.pose.position
            q = pose.pose.orientation
            logger.info(
                f"  Goal {attempt}.{sample}: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})"
            )
            goal_msg = ExecutePose.Goal()
            goal_msg.target_pose = pose
            send_future = client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                logger.warning(f"  Goal {attempt}.{sample} rejected")
                continue
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
            result = result_future.result().result
            if result.success:
                logger.info(f"  Attempt {attempt}.{sample} succeeded")
                success = True
                break
            logger.warning(f"  Attempt {attempt}.{sample} failed: {result.message}")
        if not success:
            logger.error(f"Attempt {attempt}: all {MAX_SAMPLES_PER_ATTEMPT} samples failed")
        sleep_until_ok(INTER_ATTEMPT_S)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("demo_random_poses")
    logger = get_logger("moveitpy_execute_node.demo_random_poses")
    logger.info(f"Waiting {SETTLE_TIME_S}s for scene to settle...")
    sleep_until_ok(SETTLE_TIME_S)

    worker = threading.Thread(target=run_random_pose_loop, args=(node,), daemon=True)
    worker.start()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
