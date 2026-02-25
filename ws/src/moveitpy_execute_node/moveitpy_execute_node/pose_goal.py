"""
Pose goal demo: plans and executes to random end-effector poses (position + orientation).
"""

import random
import threading
import time

import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from scipy.spatial.transform import Rotation

# Workspace bounds (x, y, z) in panda_link0 frame
POSITION_X_RANGE = (0.25, 0.55)
POSITION_Y_RANGE = (-0.30, 0.30)
POSITION_Z_RANGE = (0.20, 0.60)

FRAME_ID = "panda_link0"
TIP_LINK = "panda_link8"
ARM_CONTROLLER = "panda_arm_controller"
PLANNING_GROUP = "panda_arm"

SETTLE_TIME_S = 5.0
INTER_ATTEMPT_S = 3.0
MAX_SAMPLES_PER_ATTEMPT = 5
SLEEP_CHECK_INTERVAL_S = 0.1


def sleep_until_ok(duration_sec: float) -> None:
    """Sleep for duration_sec but return early if rclpy is shutting down."""
    elapsed = 0.0
    while elapsed < duration_sec and rclpy.ok():
        time.sleep(min(SLEEP_CHECK_INTERVAL_S, duration_sec - elapsed))
        elapsed += SLEEP_CHECK_INTERVAL_S


def random_pose_stamped() -> PoseStamped:
    """Return a PoseStamped with random position in workspace and uniform random orientation."""
    pose = PoseStamped()
    pose.header.frame_id = FRAME_ID
    pose.pose.position.x = random.uniform(*POSITION_X_RANGE)
    pose.pose.position.y = random.uniform(*POSITION_Y_RANGE)
    pose.pose.position.z = random.uniform(*POSITION_Z_RANGE)
    quat_xyzw = Rotation.random().as_quat()  # (x, y, z, w)
    pose.pose.orientation.x = quat_xyzw[0]
    pose.pose.orientation.y = quat_xyzw[1]
    pose.pose.orientation.z = quat_xyzw[2]
    pose.pose.orientation.w = quat_xyzw[3]
    return pose


def try_plan_and_execute(robot: MoveItPy, planning_component, logger) -> bool:
    """Plan and execute one trajectory. Returns True on success."""
    logger.info("Planning trajectory...")
    plan_result = planning_component.plan()
    if not plan_result:
        logger.error("Planning failed")
        return False
    logger.info("Executing plan...")
    status = robot.execute(plan_result.trajectory, [ARM_CONTROLLER])
    logger.info(f"Execution status: {status}")
    return True


def run_attempt_loop(robot: MoveItPy, arm, logger) -> None:
    """Run the plan/execute loop. Blocking so main can respond to Ctrl+C via daemon thread."""
    attempt = 0
    while rclpy.ok():
        attempt += 1
        logger.info(f"Attempt {attempt}: sampling random pose goal")
        success = False
        for sample_idx in range(1, MAX_SAMPLES_PER_ATTEMPT + 1):
            arm.set_start_state_to_current_state()
            pose_goal = random_pose_stamped()
            p = pose_goal.pose.position
            q = pose_goal.pose.orientation
            logger.info(
                f"Attempt {attempt}.{sample_idx} goal: "
                f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})"
            )
            arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=TIP_LINK)
            success = try_plan_and_execute(robot, arm, logger)
            if success:
                logger.info(f"Attempt {attempt}.{sample_idx} succeeded")
                break
            logger.warning(f"Attempt {attempt}.{sample_idx} failed; resampling")
        if not success:
            logger.error(f"Attempt {attempt}: all samples failed")
        sleep_until_ok(INTER_ATTEMPT_S)


def main() -> None:
    rclpy.init()
    logger = get_logger("moveitpy_execute_node.pose_goal")
    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component(PLANNING_GROUP)
    logger.info("MoveItPy instance created")
    logger.info(f"Waiting {SETTLE_TIME_S}s for RViz/controllers/joint states to settle...")
    sleep_until_ok(SETTLE_TIME_S)
    worker = threading.Thread(target=run_attempt_loop, args=(robot, arm, logger), daemon=True)
    worker.start()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
