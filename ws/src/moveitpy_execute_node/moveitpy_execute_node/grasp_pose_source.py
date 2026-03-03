"""
Grasp pose source: returns pre-grasp and home poses for the simple grasp sequence.

Poses are defined as xyz (m) + hand orientation (roll, pitch, yaw in rad). We command
panda_link8, but the visible gripper is panda_hand, which has a fixed -45° yaw from link8
(URDF panda_hand_joint). So we express desired *hand* RPY and convert to link8 orientation.
This module is the single place to swap in GraspGen later: replace get_grasp_pose()
to return a pose from perception + GraspGen instead of a constant.
"""

import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from .constants import FRAME_ID

# panda_hand is attached to panda_link8 with rpy="0 0 -0.785398163397" (URDF).
# So hand = link8 * R_z(-45°). We want hand at desired RPY => link8 = desired * R_z(+45°).
HAND_YAW_OFFSET_RAD = -0.785398163397  # from panda_hand_joint in panda.urdf


def _pose_from_xyz_hand_rpy(
    frame_id: str,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> PoseStamped:
    """
    Build PoseStamped for panda_link8 from position (m) and desired *hand* orientation
    (roll, pitch, yaw in rad, intrinsic XYZ). Converts to link8 orientation by undoing
    the hand's fixed -45° yaw so the visible gripper matches the given RPY.
    """
    out = PoseStamped()
    out.header.frame_id = frame_id
    out.pose.position.x = x
    out.pose.position.y = y
    out.pose.position.z = z
    r_hand = Rotation.from_euler("xyz", [roll, pitch, yaw])
    r_undo_yaw = Rotation.from_euler("z", [-HAND_YAW_OFFSET_RAD])  # +45° so link8 gives desired hand
    r_link8 = r_hand * r_undo_yaw
    # as_quat() may return (4,) or (1,4) when rotation has shape (1,); flatten to 4 elements
    q = np.asarray(r_link8.as_quat()).reshape(-1)[:4]  # (x, y, z, w)
    out.pose.orientation.x = float(q[0])
    out.pose.orientation.y = float(q[1])
    out.pose.orientation.z = float(q[2])
    out.pose.orientation.w = float(q[3])
    return out


# Pre-grasp: above small cube; desired *hand* orientation = gripper opening down (180° about X).
PRE_GRASP_POSE = _pose_from_xyz_hand_rpy(
    FRAME_ID,
    x=0.4,
    y=0.0,
    z=0.15,
    roll=math.pi,
    pitch=0.0,
    yaw=0.0,
)

# Home: Panda "ready" pose; desired hand orientation 180° about X (from RV Panda Driver).
HOME_POSE = _pose_from_xyz_hand_rpy(
    FRAME_ID,
    x=0.307,
    y=0.0,
    z=0.590,
    roll=math.pi,
    pitch=0.0,
    yaw=0.0,
)


def get_grasp_pose() -> PoseStamped:
    """
    Return the target pose for the arm before closing the gripper (pre-grasp).

    For the simple sequence this is a constant. Later: take scene/object and
    return a pose from GraspGen (or another grasp planner).
    """
    out = PoseStamped()
    out.header.frame_id = PRE_GRASP_POSE.header.frame_id
    out.pose = PRE_GRASP_POSE.pose
    return out
