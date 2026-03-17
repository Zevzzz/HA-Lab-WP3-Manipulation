"""
Rigid transform utilities (4x4 matrices, position + quaternion).

Uses scipy.spatial.transform.Rotation and numpy. No ROS types.
Used to transform object-frame grasps to world frame in grasp_with_candidates.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation


def pose_to_matrix4(position: Tuple[float, float, float], quat_xyzw: Tuple[float, float, float, float]) -> np.ndarray:
    """Build 4x4 homogeneous matrix from position (x,y,z) and quaternion (x,y,z,w)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = position
    return T


def matrix4_to_pose(T: np.ndarray) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Extract (position (x,y,z), quat (x,y,z,w)) from 4x4 homogeneous matrix."""
    position = tuple(float(x) for x in T[:3, 3])
    quat_xyzw = tuple(float(x) for x in Rotation.from_matrix(T[:3, :3]).as_quat())
    return position, quat_xyzw


def transform_pose(
    T_parent_child: np.ndarray,
    position_child: Tuple[float, float, float],
    quat_child_xyzw: Tuple[float, float, float, float],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """
    Express a pose from child frame in parent frame.

    T_parent_child: 4x4 transform from child to parent.
    position_child, quat_child_xyzw: pose in child frame.
    Returns (position_parent, quat_parent_xyzw).
    """
    T_child_pose = pose_to_matrix4(position_child, quat_child_xyzw)
    T_parent_pose = T_parent_child @ T_child_pose
    return matrix4_to_pose(T_parent_pose)


def translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    """4x4 homogeneous matrix for translation only (identity rotation)."""
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T
