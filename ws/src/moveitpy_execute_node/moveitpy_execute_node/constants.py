"""
Shared constants for Panda arm execution.

All motion planning and execution code uses these; trajectory_bridge and
gripper_command_node use the same topic and joint names for consistency.
"""

# Frame and planning
FRAME_ID = "panda_link0"
TIP_LINK = "panda_link8"
PLANNING_GROUP = "panda_arm"
ARM_CONTROLLER = "panda_arm_controller"

# Joint names (order matches trajectory_bridge and Isaac Sim)
PANDA_ARM_JOINTS = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]
PANDA_GRIPPER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]
ALL_JOINTS = PANDA_ARM_JOINTS + PANDA_GRIPPER_JOINTS

# Gripper positions (single value per finger; symmetric)
GRIPPER_OPEN = 0.04
GRIPPER_CLOSE = 0.0

# Topics (used by executor, demos, trajectory_bridge, gripper_command_node)
GRIPPER_CMD_TOPIC = "/gripper_command"
