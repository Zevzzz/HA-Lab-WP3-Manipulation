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


def matrix4_compose(T_a_b: np.ndarray, T_b_c: np.ndarray) -> np.ndarray:
    """Compose transforms: T_a_c = T_a_b @ T_b_c."""
    return T_a_b @ T_b_c


def world_object_matrix_from_position_rpy(
    x: float,
    y: float,
    z: float,
    rpy_deg: Tuple[float, float, float],
) -> np.ndarray:
    """
    Pose of object (centroid) frame in world (panda_link0).

    Rotation order: intrinsic XYZ (degrees), translation = object center in world.
    Use (0,0,yaw) for a vertical mug rotated about world +Z only.
    """
    rx, ry, rz = rpy_deg
    r = Rotation.from_euler("xyz", [rx, ry, rz], degrees=True)
    T = np.eye(4)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = (x, y, z)
    return T


def matrix4_from_rpy_xyz(
    rpy_deg: Tuple[float, float, float],
    translation: Tuple[float, float, float],
) -> np.ndarray:
    """Fixed SE(3) from Euler XYZ (degrees) + translation (intrinsic XYZ order)."""
    rx, ry, rz = rpy_deg
    r = Rotation.from_euler("xyz", [rx, ry, rz], degrees=True)
    T = np.eye(4)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = translation
    return T


def franka_T_link8_to_panda_hand() -> np.ndarray:
    """
    URDF panda_hand_joint: child panda_hand in parent panda_link8, rpy = (0, 0, -45°), xyz = 0.

    GraspGen Franka poses align with the hand / tool convention, not the bare flange.
    MoveIt pose_link is panda_link8 → goal: T_world_link8 = T_world_grasp @ inv(this).
    """
    return matrix4_from_rpy_xyz((0.0, 0.0, -45.0), (0.0, 0.0, 0.0))


def grasp_pose_world_to_link8_goal(
    T_world_grasp: np.ndarray,
    T_grasp_to_link8: np.ndarray,
) -> np.ndarray:
    """
    Convert GraspGen/tool TCP pose in world to panda_link8 goal in world.

    T_world_grasp: 4x4, grasp frame in world (from YAML + object pose).
    T_grasp_to_link8: constant 4x4 such that p_w = T_world_grasp @ p_g and
    p_w = T_world_link8 @ p_l with T_world_link8 = T_world_grasp @ T_grasp_to_link8.

    Equivalently: same world point expressed from grasp origin vs link8 origin
    for the rigid offset between GraspGen TCP and MoveIt pose_link (panda_link8).
    """
    return matrix4_compose(T_world_grasp, T_grasp_to_link8)
