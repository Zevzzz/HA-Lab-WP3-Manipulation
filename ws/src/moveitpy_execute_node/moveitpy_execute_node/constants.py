"""
Shared constants for Panda arm execution and grasp sequence.
"""

# Frame and planning
FRAME_ID = "panda_link0"
TIP_LINK = "panda_link8"
PLANNING_GROUP = "panda_arm"
ARM_CONTROLLER = "panda_arm_controller"

# Joint names (order matches trajectory_bridge and Isaac)
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
