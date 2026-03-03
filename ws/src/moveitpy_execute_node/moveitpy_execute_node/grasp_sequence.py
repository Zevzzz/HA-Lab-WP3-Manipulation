"""
Simple grasp sequence (no GraspGen): open gripper -> move to pre-grasp -> close gripper -> home.
"""

import time

import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from std_msgs.msg import Float64

from .constants import ARM_CONTROLLER, GRIPPER_CLOSE, GRIPPER_OPEN, PLANNING_GROUP, TIP_LINK
from .grasp_pose_source import get_grasp_pose, HOME_POSE

GRIPPER_CMD_TOPIC = "/gripper_command"
SETTLE_TIME_S = 5.0
GRIPPER_WAIT_S = 2.0
SLEEP_CHECK_INTERVAL_S = 0.1


def sleep_until_ok(duration_sec: float) -> None:
    elapsed = 0.0
    while elapsed < duration_sec and rclpy.ok():
        time.sleep(min(SLEEP_CHECK_INTERVAL_S, duration_sec - elapsed))
        elapsed += SLEEP_CHECK_INTERVAL_S


def send_gripper_command(node, pub, value: float) -> None:
    msg = Float64()
    msg.data = float(value)
    pub.publish(msg)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)


def plan_and_execute(robot: MoveItPy, arm, pose: PoseStamped, logger) -> bool:
    logger.info("Planning...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link=TIP_LINK)
    plan_result = arm.plan()
    if not plan_result:
        logger.error("Planning failed")
        return False
    logger.info("Executing...")
    status = robot.execute(plan_result.trajectory, [ARM_CONTROLLER])
    logger.info(f"Execution status: {status}")
    return True


def run_sequence(node, robot, arm, logger) -> None:
    pub = node.create_publisher(Float64, GRIPPER_CMD_TOPIC, 10)
    # Give gripper_command node and sim time to be ready
    sleep_until_ok(0.5)

    logger.info("Open gripper")
    send_gripper_command(node, pub, GRIPPER_OPEN)
    sleep_until_ok(GRIPPER_WAIT_S)

    grasp_pose = get_grasp_pose()
    grasp_pose.header.stamp = node.get_clock().now().to_msg()
    logger.info("Move to pre-grasp")
    if not plan_and_execute(robot, arm, grasp_pose, logger):
        return
    sleep_until_ok(0.3)

    logger.info("Close gripper")
    send_gripper_command(node, pub, GRIPPER_CLOSE)
    sleep_until_ok(GRIPPER_WAIT_S)

    home = PoseStamped()
    home.header.frame_id = HOME_POSE.header.frame_id
    home.header.stamp = node.get_clock().now().to_msg()
    home.pose = HOME_POSE.pose
    logger.info("Move to home")
    plan_and_execute(robot, arm, home, logger)
    logger.info("Sequence done.")


def main() -> None:
    rclpy.init()
    logger = get_logger("moveitpy_execute_node.grasp_sequence")
    node = rclpy.create_node("grasp_sequence")
    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component(PLANNING_GROUP)
    logger.info(f"MoveItPy ready. Waiting {SETTLE_TIME_S}s for scene to settle...")
    sleep_until_ok(SETTLE_TIME_S)
    run_sequence(node, robot, arm, logger)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
