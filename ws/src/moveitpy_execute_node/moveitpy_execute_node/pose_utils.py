"""
Pose utilities for Panda end-effector.

We command panda_link8 (TIP_LINK), but the visible gripper is panda_hand, which
has a fixed -45° yaw from link8 (URDF panda_hand_joint). So we express desired
*hand* orientation (RPY) and convert to link8 quaternion for planning.
"""

from typing import Optional, Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from .constants import FRAME_ID

# From panda_hand_joint in panda.urdf: rpy="0 0 -0.785398163397"
# hand = link8 * R_z(-45°). So link8 = desired_hand * R_z(+45°).
HAND_YAW_OFFSET_RAD = -0.785398163397


def pose_stamped_from_xyz_hand_rpy(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    *,
    frame_id: str = FRAME_ID,
) -> PoseStamped:
    """
    Build PoseStamped for panda_link8 from position (m) and desired *hand* RPY (rad, intrinsic XYZ).

    Converts to link8 orientation so the visible gripper in sim matches the given RPY.
    """
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    r_hand = Rotation.from_euler("xyz", [roll, pitch, yaw])
    r_undo_yaw = Rotation.from_euler("z", [-HAND_YAW_OFFSET_RAD])
    r_link8 = r_hand * r_undo_yaw
    q = np.asarray(r_link8.as_quat()).reshape(-1)[:4]  # (x, y, z, w)
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


def copy_pose_stamped(source: PoseStamped, frame_id: Optional[str] = None) -> PoseStamped:
    """Return a copy of source, optionally overriding frame_id."""
    out = PoseStamped()
    out.header.frame_id = frame_id if frame_id is not None else source.header.frame_id
    out.header.stamp = source.header.stamp
    out.pose = source.pose
    return out
