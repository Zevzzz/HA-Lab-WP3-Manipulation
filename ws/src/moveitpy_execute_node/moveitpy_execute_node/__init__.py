"""
moveitpy_execute_node: modular motion planning and execution for Panda.

Core API (for in-process or action-based use):
  - PoseExecutor: plan_to_pose(), execute_trajectory(), plan_and_execute()
  - PoseSource: get_target_pose(), get_home_pose(); implement ConstantPoseSource or GraspGen
  - pose_utils: pose_stamped_from_xyz_hand_rpy(), copy_pose_stamped()
  - gripper: send_gripper_command()
  - utils: sleep_until_ok()

Nodes:
  - executor_node: long-lived ExecutePose action server (run from launch)
  - trajectory_bridge: FollowJointTrajectory -> /joint_command (Isaac Sim)
  - gripper_command: /gripper_command -> /joint_command (alternative to bridge)
  - demo_grasp_sequence, demo_random_poses: demos that call ExecutePose
  - grasp_execution: GraspExecutionConfig, plan/execute multi-step grasp (approach, close, home)
"""

from .constants import (
    ARM_CONTROLLER,
    FRAME_ID,
    GRIPPER_CLOSE,
    GRIPPER_CMD_TOPIC,
    GRIPPER_OPEN,
    PLANNING_GROUP,
    TIP_LINK,
)
from .grasp_execution import (
    GraspExecutionConfig,
    execute_grasp_sequence,
    plan_grasp_with_optional_approach,
)
from .gripper import send_gripper_command
from .motion import PoseExecutor
from .pose_sources import ConstantPoseSource, PoseSource, get_default_home_pose, get_default_pose_source
from .pose_utils import copy_pose_stamped, pose_stamped_from_xyz_hand_rpy
from .utils import sleep_until_ok

__all__ = [
    "ARM_CONTROLLER",
    "ConstantPoseSource",
    "execute_grasp_sequence",
    "FRAME_ID",
    "GraspExecutionConfig",
    "GRIPPER_CLOSE",
    "GRIPPER_CMD_TOPIC",
    "GRIPPER_OPEN",
    "PoseExecutor",
    "PoseSource",
    "PLANNING_GROUP",
    "TIP_LINK",
    "copy_pose_stamped",
    "get_default_home_pose",
    "get_default_pose_source",
    "plan_grasp_with_optional_approach",
    "pose_stamped_from_xyz_hand_rpy",
    "send_gripper_command",
    "sleep_until_ok",
]
