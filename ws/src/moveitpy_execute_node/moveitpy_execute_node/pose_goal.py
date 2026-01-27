import random
import time

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy


def try_plan_and_execute(robot, planning_component, logger):
    logger.info("Planning trajectory...")
    plan_result = planning_component.plan()

    if plan_result:
        logger.info("Executing plan...")
        robot_trajectory = plan_result.trajectory
        # Provide explicit controller list to satisfy MoveItPy signature.
        status = robot.execute(robot_trajectory, ["panda_arm_controller"])
        logger.info(f"Execution status: {status}")
        return True
    logger.error("Planning failed")
    return False


def main():
    rclpy.init()
    logger = get_logger("moveitpy_execute_node.pose_goal")

    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component("panda_arm")
    logger.info("MoveItPy instance created")

    # Give RViz/time for startup and any TF/joint state settling.
    logger.info("Waiting 5s for RViz/controllers/joint states to settle...")
    time.sleep(5.0)

    # Loop: pick random pose targets, plan/execute, pause, repeat.
    attempt = 0
    while rclpy.ok():
        attempt += 1
        logger.info(f"Attempt {attempt}: sampling random pose goal")

        # Try multiple samples before giving up on this cycle.
        success = False
        for sample_idx in range(1, 6):
            # Start state = current (refresh every sample).
            arm.set_start_state_to_current_state()

            # Random pose goal (in panda base frame), end-effector link panda_link8
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "panda_link0"
            pose_goal.pose.orientation.w = 1.0
            pose_goal.pose.position.x = random.uniform(0.25, 0.55)
            pose_goal.pose.position.y = random.uniform(-0.30, 0.30)
            pose_goal.pose.position.z = random.uniform(0.20, 0.60)

            logger.info(
                f"Attempt {attempt}.{sample_idx} goal: "
                f"x={pose_goal.pose.position.x:.3f} "
                f"y={pose_goal.pose.position.y:.3f} "
                f"z={pose_goal.pose.position.z:.3f}"
            )

            arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
            success = try_plan_and_execute(robot, arm, logger)
            if success:
                logger.info(f"Attempt {attempt}.{sample_idx} succeeded")
                break
            logger.warning(f"Attempt {attempt}.{sample_idx} failed; resampling")

        if not success:
            logger.error(f"Attempt {attempt}: all samples failed")

        time.sleep(3.0)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
