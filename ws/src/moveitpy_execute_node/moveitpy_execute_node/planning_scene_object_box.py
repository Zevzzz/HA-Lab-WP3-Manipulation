"""
Planning-scene box for the grasp target: obstacle during approach, removed before final grasp-in.

The box matches the axis-aligned bounds of the **mean-centered** point cloud in the same
**PC centroid frame** used by GraspGen YAML poses. It is placed in ``panda_link0`` with
orientation ``R_world = R_world_object @ R_sim_from_pc`` so box axes align with PC axes in world.

Publishing uses ``moveit_msgs/CollisionObject`` on ``collision_object`` (same as ``ground_plane_scene``).
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Protocol

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

from .transform_utils import matrix4_compose, matrix4_to_pose
from .utils import sleep_until_ok

# Distinct from ground_plane_scene's id to avoid accidental REMOVE collisions.
DEFAULT_TARGET_OBJECT_COLLISION_ID = "grasp_eval_target_object"

_MIN_HALF_EXTENT_M = 1e-4


def load_point_cloud_xyz(path: Path) -> np.ndarray:
    """
    Load (N, 3) float64 point coordinates. Supports ``.npy`` (Nx3) and ``.ply`` (requires trimesh).
    """
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    suf = path.suffix.lower()
    if suf == ".npy":
        arr = np.asarray(np.load(path), dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 3:
            raise ValueError(f".npy must be (N, 3+) array, got {arr.shape}")
        return arr[:, :3].copy()
    if suf == ".ply":
        try:
            import trimesh
        except ImportError as e:
            raise ImportError(
                "Loading .ply requires trimesh (e.g. pip install trimesh in the ROS env)."
            ) from e
        geom = trimesh.load(str(path), process=False)
        pts = np.asarray(geom.vertices, dtype=np.float64)[:, :3]
        if len(pts) < 3:
            raise ValueError("PLY has too few vertices")
        return pts
    raise ValueError(f"Unsupported point cloud extension {suf!r}; use .ply or .npy")


def centered_aabb_half_extents(points_xyz: np.ndarray) -> np.ndarray:
    """Mean-center ``points_xyz`` (N,3); return half-extents (hx, hy, hz) along centered axes."""
    pc = np.asarray(points_xyz, dtype=np.float64)
    if pc.ndim != 2 or pc.shape[1] != 3:
        raise ValueError(f"points_xyz must be (N, 3), got {pc.shape}")
    if len(pc) < 3:
        raise ValueError("Need at least 3 points for AABB")
    centered = pc - np.mean(pc, axis=0)
    mn = np.min(centered, axis=0)
    mx = np.max(centered, axis=0)
    half = (mx - mn) / 2.0
    half = np.maximum(half, _MIN_HALF_EXTENT_M)
    return half.astype(np.float64)


def resolve_pc_path_next_to_grasps_yaml(
    yaml_path: Path,
    *,
    explicit: Optional[Path] = None,
) -> Optional[Path]:
    """
    If ``explicit`` is set, return it if the file exists.
    Else try ``<stem without _grasps>.ply`` then ``.npy`` beside the YAML.
    """
    yaml_path = Path(yaml_path).resolve()
    if explicit is not None:
        p = Path(explicit).resolve()
        return p if p.is_file() else None
    stem = yaml_path.stem.replace("_grasps", "")
    parent = yaml_path.parent
    for ext in (".ply", ".npy"):
        cand = parent / (stem + ext)
        if cand.is_file():
            return cand
    return None


def build_T_world_pc_centroid_frame(
    T_world_object: np.ndarray,
    T_sim_from_pc: np.ndarray,
) -> np.ndarray:
    """
    Full pose of **PC centroid frame** (GraspGen object axes) in world.

    ``T_world_object``: centroid at origin, object RPY in world (from ``world_object_matrix_from_position_rpy``).
    ``T_sim_from_pc``: PC→sim rotation (typically identity); translation zero.
    """
    return matrix4_compose(T_world_object, T_sim_from_pc)


class ApproachObstacleScene(Protocol):
    """Inject into grasp execution: enable obstacle before approach, disable before final grasp."""

    def enable_for_approach(self) -> None: ...

    def disable_after_approach(self) -> None: ...


class PlanningSceneTargetObjectBox:
    """
    Publishes a BOX ``CollisionObject`` for approach planning, then removes it.

    Uses the same QoS pattern as ``ground_plane_scene`` for compatibility with MoveIt's subscriber.
    """

    def __init__(
        self,
        node: Node,
        *,
        reference_frame: str,
        collision_object_id: str,
        half_extents_xyz_m: np.ndarray,
        T_world_box: np.ndarray,
        settle_after_publish_s: float,
        publish_repeat: int = 2,
    ) -> None:
        self._node = node
        self._frame = reference_frame
        self._id = collision_object_id
        self._settle_s = float(settle_after_publish_s)
        self._repeat = max(1, int(publish_repeat))
        half = np.asarray(half_extents_xyz_m, dtype=np.float64).reshape(3)
        dims = (2.0 * half).tolist()
        pos, quat = matrix4_to_pose(T_world_box)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = dims

        self._co_add = CollisionObject()
        self._co_add.header.frame_id = self._frame
        self._co_add.id = self._id
        self._co_add.operation = CollisionObject.ADD
        self._co_add.primitives.append(prim)
        self._co_add.primitive_poses.append(pose)

        self._co_remove = CollisionObject()
        self._co_remove.header.frame_id = self._frame
        self._co_remove.id = self._id
        self._co_remove.operation = CollisionObject.REMOVE

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = node.create_publisher(CollisionObject, "collision_object", qos)

    def _stamp_and_publish(self, co: CollisionObject) -> None:
        co.header.stamp = self._node.get_clock().now().to_msg()
        for _ in range(self._repeat):
            self._pub.publish(co)

    def enable_for_approach(self) -> None:
        self._stamp_and_publish(self._co_add)
        sleep_until_ok(self._settle_s)

    def disable_after_approach(self) -> None:
        self._stamp_and_publish(self._co_remove)
        sleep_until_ok(self._settle_s)


def try_build_planning_scene_target_box(
    node: Node,
    *,
    pc_path: Path,
    T_world_object: np.ndarray,
    T_sim_from_pc: np.ndarray,
    padding_m: float,
    reference_frame: str,
    collision_object_id: str,
    settle_after_publish_s: float,
) -> tuple[Optional[PlanningSceneTargetObjectBox], str]:
    """
    Load PC, compute AABB half-extents (centered), apply padding, build bridge.

    Returns ``(None, reason)`` on failure, else ``(bridge, "")``.
    """
    try:
        pts = load_point_cloud_xyz(pc_path)
        half = centered_aabb_half_extents(pts)
        if padding_m > 0.0:
            half = half + float(padding_m)
        T_world_box = build_T_world_pc_centroid_frame(T_world_object, T_sim_from_pc)
        bridge = PlanningSceneTargetObjectBox(
            node,
            reference_frame=reference_frame,
            collision_object_id=collision_object_id,
            half_extents_xyz_m=half,
            T_world_box=T_world_box,
            settle_after_publish_s=settle_after_publish_s,
        )
        return bridge, ""
    except Exception as e:
        return None, str(e)
