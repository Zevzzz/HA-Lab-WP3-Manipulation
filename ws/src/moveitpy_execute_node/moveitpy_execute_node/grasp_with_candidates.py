"""
GraspGen eval client: YAML grasp (object frame) -> world pose -> ExecutePose.

Transform chain:
  T_world_grasp   = T_world_object @ T_object_grasp
  T_world_command = T_world_grasp @ T_gripper_offset   (default: identity)

MoveIt pose_link: `panda_pose_goal_isaac.launch.py` defaults `cartesian_tip_link:=panda_hand` so goals
are interpreted as **panda_hand** (pair with `--yaml-pose-frame link8`, i.e. no extra offset, for
typical GraspGen YAML). Non-Isaac / flange workflows: launch with `cartesian_tip_link:=panda_link8` and
use `--yaml-pose-frame hand` to convert hand→flange via URDF.

A pure rotation offset does **not** change goal **translation**; wrong XY vs the mug is usually YAML
position in the object frame or prim-vs-centroid frame mismatch (`--pc-centroid-shift-*`).

CRITICAL — point cloud frame vs simulation object frame:
  • GraspGen poses are in the **same orthonormal basis as the centered point cloud** (axes of the
    PLY you exported — align that mesh to Isaac in CloudCompare or similar before sampling).
  • If the PLY frame still does not match the sim prim at runtime, apply one fixed rotation with
    ``--sim-from-pc-frame-rpy-deg R P Y`` (intrinsic XYZ deg). Chain:
    T_world_grasp = T_world_sim_object @ T_sim_from_pc @ T_pc_grasp.

CRITICAL — two different "object origins":
  • Grasp YAML / GraspGen use the **point cloud centroid** frame (mean-centered .ply / .npy).
  • Isaac / USD often place the asset with origin at **base corner, bbox min, etc.** — NOT the centroid.

If you pass the prim's world position as --object-center but grasps are in the **centroid** frame,
every goal is shifted by (prim → centroid) in the horizontal plane → gripper lands **beside** the mug
even when the math is otherwise correct. Fix with --pc-centroid-shift-local-xyz (or measure true
centroid in world and pass that as --object-center).

Expects executor_node (ExecutePose). Appends one row to grasp_execution_results.csv.

Execution uses ``grasp_execution`` (approach, grasp, close gripper, lift +Z or home) — see
``GraspExecutionConfig`` and CLI flags ``--no-approach``, ``--approach-offset-m``, ``--post-grasp-lift-z``, etc.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from moveitpy_execute_node_msgs.action import ExecutePose

from .constants import FRAME_ID as WORLD_FRAME_ID
from .planning_scene_object_box import (
    DEFAULT_TARGET_OBJECT_COLLISION_ID,
    ApproachObstacleScene,
    resolve_pc_path_next_to_grasps_yaml,
    try_build_planning_scene_target_box,
)
from scipy.spatial.transform import Rotation

from .grasp_execution import (
    GraspExecutionConfig,
    execute_grasp_sequence,
    plan_grasp_with_optional_approach,
)
from .transform_utils import (
    T_graspgen_tcp_to_moveit_panda_hand,
    franka_T_link8_to_panda_hand,
    grasp_pose_world_to_link8_goal,
    matrix4_compose,
    matrix4_from_rpy_xyz,
    matrix4_to_pose,
    pose_to_matrix4,
    world_object_matrix_from_position_rpy,
)

ACTION_NAME = "execute_pose"
DEFAULT_LOG_DIR = Path("data/logs")
EXECUTION_LOG_NAME = "grasp_execution_results.csv"

DEFAULT_TABLE_Z_M = 0.05
DEFAULT_OBJECT_X_M = 0.4
DEFAULT_OBJECT_Y_M = 0.0
YAML_KEY_OBJECT_HALF_HEIGHT_M = "object_half_height_m"


@dataclass(frozen=True)
class ObjectPoseConfig:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class GraspsYamlData:
    grasps: list[dict]
    object_half_height_m: Optional[float]


def load_grasps_yaml(path: Path) -> GraspsYamlData:
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    with open(path) as f:
        doc = yaml.safe_load(f)
    if not doc or "grasps" not in doc:
        raise ValueError(f"YAML must contain 'grasps' list: {path}")
    grasps = list(doc["grasps"])
    half_height = doc.get(YAML_KEY_OBJECT_HALF_HEIGHT_M)
    if half_height is not None:
        half_height = float(half_height)
    return GraspsYamlData(
        grasps=grasps,
        object_half_height_m=half_height,
    )


def resolve_object_pose(
    table_z: float,
    object_x: float,
    object_y: float,
    object_half_height_m: Optional[float],
    object_center_override: Optional[tuple[float, float, float]],
) -> ObjectPoseConfig:
    if object_center_override is not None:
        return ObjectPoseConfig(
            x=object_center_override[0],
            y=object_center_override[1],
            z=object_center_override[2],
        )
    if object_half_height_m is None:
        raise ValueError(
            "YAML has no 'object_half_height_m'. Regenerate with graspgen_request.py or pass --object-center x y z."
        )
    return ObjectPoseConfig(
        x=object_x,
        y=object_y,
        z=table_z + object_half_height_m,
    )


def effective_object_centroid_world_m(
    base: ObjectPoseConfig,
    object_rpy_deg: tuple[float, float, float],
    shift_world_xyz: tuple[float, float, float],
    shift_local_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    """
    World position of the **point cloud centroid** (GraspGen frame origin).

    base: user-supplied "object" position (often wrong: Isaac prim / mesh root).
    shift_world_xyz: added in panda_link0 (e.g. hand-tuned nudge).
    shift_local_xyz: vector from that reference to **centroid**, in axes that rotate with object_rpy
    (mug-fixed frame with same Euler XYZ as --object-rpy-deg); rotated into world then added.
    """
    rx, ry, rz = object_rpy_deg
    R = Rotation.from_euler("xyz", [rx, ry, rz], degrees=True).as_matrix()
    local = np.array(shift_local_xyz, dtype=float)
    world_shift = R @ local
    return (
        base.x + shift_world_xyz[0] + float(world_shift[0]),
        base.y + shift_world_xyz[1] + float(world_shift[1]),
        base.z + shift_world_xyz[2] + float(world_shift[2]),
    )


def grasp_to_pose_stamped(
    grasp: dict,
    T_world_object: np.ndarray,
    T_gripper_offset: np.ndarray,
    world_frame_id: str,
    T_sim_from_pc: np.ndarray,
    T_graspgen_tcp_align: np.ndarray,
) -> PoseStamped:
    """T_command = T_world @ T_sim @ T_yaml @ T_graspgen_align @ T_gripper_offset (see transform_utils)."""
    p = grasp.get("position") or [0.0, 0.0, 0.0]
    o = grasp.get("orientation") or [0.0, 0.0, 0.0, 1.0]
    T_obj_grasp = pose_to_matrix4(
        (float(p[0]), float(p[1]), float(p[2])),
        (float(o[0]), float(o[1]), float(o[2]), float(o[3])),
    )
    T_world_grasp = matrix4_compose(
        matrix4_compose(T_world_object, T_sim_from_pc),
        T_obj_grasp,
    )
    T_world_grasp = matrix4_compose(T_world_grasp, T_graspgen_tcp_align)
    T_world_cmd = grasp_pose_world_to_link8_goal(T_world_grasp, T_gripper_offset)
    pos_world, quat_world = matrix4_to_pose(T_world_cmd)
    msg = PoseStamped()
    msg.header.frame_id = world_frame_id
    msg.pose.position.x = pos_world[0]
    msg.pose.position.y = pos_world[1]
    msg.pose.position.z = pos_world[2]
    msg.pose.orientation.x = quat_world[0]
    msg.pose.orientation.y = quat_world[1]
    msg.pose.orientation.z = quat_world[2]
    msg.pose.orientation.w = quat_world[3]
    return msg


def send_pose_and_wait(
    node: Node,
    client: ActionClient,
    pose: PoseStamped,
    plan_only: bool = False,
    timeout_send: float = 5.0,
    timeout_result: float = 60.0,
):
    pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg = ExecutePose.Goal()
    goal_msg.target_pose = pose
    goal_msg.plan_only = plan_only
    send_future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=timeout_send)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return False, "Goal rejected"
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_result)
    future_result = result_future.result()
    if future_result is None:
        return False, "No result (server may have died)"
    result = future_result.result
    if result is None:
        return False, "No result (server may have died)"
    return bool(result.success), (result.message or "")


def append_execution_log(
    log_dir: Path,
    yaml_path: str,
    total_candidates: int,
    candidate_index_used: int,
    num_failed_before: int,
    success: bool,
    message: str = "",
) -> None:
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / EXECUTION_LOG_NAME
    write_header = not log_file.exists()
    with open(log_file, "a", newline="") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow([
                "timestamp_iso", "yaml_path", "total_candidates", "candidate_index_used",
                "num_failed_before", "success", "message",
            ])
        ts = datetime.utcnow().isoformat() + "Z"
        w.writerow([ts, yaml_path, total_candidates, candidate_index_used, num_failed_before, success, message])


def run(
    node: Node,
    yaml_path: Path,
    log_dir: Path,
    table_z: float,
    object_x: float,
    object_y: float,
    object_center_override: Optional[tuple[float, float, float]],
    object_rpy_deg: tuple[float, float, float],
    T_gripper_offset: np.ndarray,
    pc_centroid_shift_world_xyz: tuple[float, float, float],
    pc_centroid_shift_local_xyz: tuple[float, float, float],
    yaml_pose_frame: str,
    sim_from_pc_frame_rpy_deg_cli: Optional[tuple[float, float, float]],
    exec_config: GraspExecutionConfig,
    align_graspgen_franka_fingers: bool,
    *,
    approach_collision_pc: Optional[Path] = None,
    approach_collision_disabled: bool = False,
    approach_collision_padding_m: float = 0.0,
    approach_collision_settle_s: float = 0.15,
) -> int:
    logger = node.get_logger()
    try:
        data = load_grasps_yaml(yaml_path)
    except Exception as e:
        logger.error(f"Failed to load YAML: {e}")
        append_execution_log(log_dir, str(yaml_path), 0, 0, 0, False, message=f"Load error: {e}")
        return 1

    grasps = data.grasps
    total = len(grasps)
    if total == 0:
        logger.error("No grasps in YAML")
        append_execution_log(log_dir, str(yaml_path), 0, 0, 0, False, message="No grasps")
        return 1

    if sim_from_pc_frame_rpy_deg_cli is not None:
        align_rpy = sim_from_pc_frame_rpy_deg_cli
        align_source = "CLI --sim-from-pc-frame-rpy-deg"
    else:
        align_rpy = (0.0, 0.0, 0.0)
        align_source = "identity (no pc→sim rotation)"
    T_sim_from_pc = (
        matrix4_from_rpy_xyz(align_rpy, (0.0, 0.0, 0.0))
        if any(abs(x) > 1e-9 for x in align_rpy)
        else np.eye(4)
    )
    T_graspgen_tcp_align = T_graspgen_tcp_to_moveit_panda_hand() if align_graspgen_franka_fingers else np.eye(4)
    logger.info(
        f"PC→sim object frame: rpy deg (intrinsic xyz) = {align_rpy} ({align_source})"
    )
    logger.info(
        f"GraspGen→panda_hand finger align: {'on (GraspGen +X close → URDF +Y, see transform_utils)' if align_graspgen_franka_fingers else 'off'}"
    )

    try:
        object_pose = resolve_object_pose(
            table_z=table_z,
            object_x=object_x,
            object_y=object_y,
            object_half_height_m=data.object_half_height_m,
            object_center_override=object_center_override,
        )
    except ValueError as e:
        logger.error(str(e))
        append_execution_log(log_dir, str(yaml_path), total, 0, 0, False, message=str(e))
        return 1

    cx, cy, cz = effective_object_centroid_world_m(
        object_pose,
        object_rpy_deg,
        pc_centroid_shift_world_xyz,
        pc_centroid_shift_local_xyz,
    )
    T_world_object = world_object_matrix_from_position_rpy(cx, cy, cz, object_rpy_deg)

    approach_obstacle_scene: ApproachObstacleScene | None = None
    if approach_collision_disabled:
        logger.info(
            "Approach collision box disabled (no MoveIt object AABB; use --no-approach-collision-box "
            "or --no-object-collision-box)."
        )
    elif not exec_config.use_approach:
        logger.info("Approach collision box skipped (approach motion disabled).")
    else:
        pc_resolved = resolve_pc_path_next_to_grasps_yaml(
            yaml_path, explicit=approach_collision_pc
        )
        if pc_resolved is None:
            logger.info(
                "Approach collision box: no .ply/.npy beside YAML; "
                "pass --approach-collision-box-pc or add a matching point cloud file."
            )
        else:
            br, err = try_build_planning_scene_target_box(
                node,
                pc_path=pc_resolved,
                T_world_object=T_world_object,
                T_sim_from_pc=T_sim_from_pc,
                padding_m=approach_collision_padding_m,
                reference_frame=WORLD_FRAME_ID,
                collision_object_id=DEFAULT_TARGET_OBJECT_COLLISION_ID,
                settle_after_publish_s=approach_collision_settle_s,
            )
            if br is not None:
                approach_obstacle_scene = br
                logger.info(
                    f"Approach collision box: ON from {pc_resolved} "
                    f"(MoveIt id {DEFAULT_TARGET_OBJECT_COLLISION_ID!r})."
                )
            else:
                logger.warning(f"Approach collision box skipped: {err}")

    logger.info(
        f"Eval: user reference pos (m) = ({object_pose.x}, {object_pose.y}, {object_pose.z}); "
        f"effective PC centroid (m) = ({cx:.5f}, {cy:.5f}, {cz:.5f}); "
        f"object rpy deg = {object_rpy_deg}; frame = {WORLD_FRAME_ID}; "
        f"yaml_pose_frame = {yaml_pose_frame} (must match launch cartesian_tip_link: "
        f"default Isaac uses panda_hand + yaml-pose-frame link8 = identity TCP offset)"
    )

    if grasps:
        g0 = grasps[0].get("position") or [0.0, 0.0, 0.0]
        logger.info(
            f"Grasp 1 YAML position in **object/centroid** frame (m): "
            f"({float(g0[0]):.4f}, {float(g0[1]):.4f}, {float(g0[2]):.4f}) — "
            f"large X/Y means goal is offset from centroid (rim grasp), not centered over the mug."
        )
        p0 = grasp_to_pose_stamped(
            grasps[0],
            T_world_object,
            T_gripper_offset,
            WORLD_FRAME_ID,
            T_sim_from_pc,
            T_graspgen_tcp_align,
        )
        g = p0.pose
        logger.info(
            f"Grasp 1 command pose ({WORLD_FRAME_ID}): pos=({g.position.x:.5f}, {g.position.y:.5f}, {g.position.z:.5f}) "
            f"quat_xyzw=({g.orientation.x:.4f}, {g.orientation.y:.4f}, {g.orientation.z:.4f}, {g.orientation.w:.4f})"
        )

    client = ActionClient(node, ExecutePose, ACTION_NAME)
    logger.info(f"Waiting for action server '{ACTION_NAME}'...")
    if not client.wait_for_server(timeout_sec=10.0):
        logger.error("Action server not available. Is the executor node running?")
        append_execution_log(log_dir, str(yaml_path), total, 0, 0, False, message="Action server not available")
        return 1

    candidate_index_used = 0
    num_failed_before = 0
    last_message = ""
    winning_pose: Optional[PoseStamped] = None
    winning_approach: Optional[PoseStamped] = None

    for i, g in enumerate(grasps):
        idx_1based = i + 1
        pose = grasp_to_pose_stamped(
            g, T_world_object, T_gripper_offset, WORLD_FRAME_ID, T_sim_from_pc, T_graspgen_tcp_align,
        )
        logger.info(f"Planning grasp {idx_1based}/{total}")
        ok, msg, approach_pose = plan_grasp_with_optional_approach(
            node,
            client,
            pose,
            exec_config,
            send_pose_and_wait,
            plan_timeout_s=15.0,
            approach_obstacle_scene=approach_obstacle_scene,
        )
        if ok:
            candidate_index_used = idx_1based
            num_failed_before = i
            winning_pose = pose
            winning_approach = approach_pose
            last_message = msg or "Plan found."
            logger.info(f"Grasp {idx_1based} planned OK ({msg}). Executing sequence.")
            break
        last_message = msg
        logger.warning(f"Grasp {idx_1based} failed: {msg}")

    if winning_pose is not None:
        exec_ok, msg = execute_grasp_sequence(
            node,
            client,
            winning_pose,
            exec_config,
            send_pose_and_wait,
            approach_pose=winning_approach,
            home_pose=None,
            exec_timeout_s=60.0,
            logger=logger,
            approach_obstacle_scene=approach_obstacle_scene,
        )
        success = exec_ok
        last_message = msg if msg else last_message
        if not exec_ok:
            logger.error(f"Execution failed: {last_message}")
    else:
        success = False

    append_execution_log(
        log_dir, str(yaml_path), total, candidate_index_used, num_failed_before, success, last_message,
    )
    logger.info(
        f"Logged: yaml={yaml_path}, used_grasp={candidate_index_used}, success={success}"
    )
    return 0 if success else 1


def main(args=None) -> int:
    parser = argparse.ArgumentParser(
        description="GraspGen eval: object-frame YAML -> panda_link0 pose -> ExecutePose (YAML order).",
    )
    parser.add_argument("--path", type=Path, required=True, help="Grasps YAML")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="CSV log directory")
    parser.add_argument("--table-z", type=float, default=DEFAULT_TABLE_Z_M, help="Table z if using half-height from YAML")
    parser.add_argument("--object-x", type=float, default=DEFAULT_OBJECT_X_M, help="Object center x (without --object-center)")
    parser.add_argument("--object-y", type=float, default=DEFAULT_OBJECT_Y_M, help="Object center y (without --object-center)")
    parser.add_argument(
        "--object-center",
        nargs="+",
        type=float,
        default=None,
        metavar="Z | X Y Z",
        help="Object centroid in panda_link0 (m). One float: Z only (X,Y from --object-x / --object-y, "
        f"defaults {DEFAULT_OBJECT_X_M}, {DEFAULT_OBJECT_Y_M}). Three floats: X Y Z.",
    )
    parser.add_argument(
        "--object-yaw-deg",
        type=float,
        default=None,
        help="Spin object frame about world +Z at centroid (deg). Same as --object-rpy-deg 0 0 YAW.",
    )
    parser.add_argument(
        "--object-rpy-deg",
        nargs=3,
        type=float,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="Full object orientation Euler XYZ deg (overrides --object-yaw-deg if both set).",
    )
    parser.add_argument(
        "--yaml-pose-frame",
        type=str,
        choices=("hand", "link8"),
        default="link8",
        help="link8 (default): no URDF offset; YAML TCP matches launch cartesian_tip_link (Isaac default: panda_hand). "
        "hand: right-multiply inv(link8→hand) so YAML hand frame becomes panda_link8 goal — use with "
        "cartesian_tip_link:=panda_link8.",
    )
    parser.add_argument(
        "--gripper-offset-rpy-deg",
        nargs=3,
        type=float,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="Extra T_cmd = T_grasp @ offset after yaml-pose-frame (Euler XYZ deg). Rare.",
    )
    parser.add_argument(
        "--gripper-offset-xyz",
        nargs=3,
        type=float,
        default=None,
        metavar=("DX", "DY", "DZ"),
        help="Translation part of gripper offset (m), with --gripper-offset-rpy-deg.",
    )
    parser.add_argument(
        "--pc-centroid-shift-world-xyz",
        nargs=3,
        type=float,
        default=None,
        metavar=("DX", "DY", "DZ"),
        help="Add this delta in panda_link0 (m) after resolving centroid (rare; use local shift first).",
    )
    parser.add_argument(
        "--pc-centroid-shift-local-xyz",
        nargs=3,
        type=float,
        default=None,
        metavar=("LX", "LY", "LZ"),
        help="Vector from your --object-center reference (e.g. Isaac prim) to the **point cloud centroid**, "
        "in mug-fixed axes (same convention as object rpy). Rotated by object rpy then added. "
        "Example: prim at bottom corner → centroid at center: set LY/LX ~ half-width in meters.",
    )
    parser.add_argument(
        "--sim-from-pc-frame-rpy-deg",
        nargs=3,
        type=float,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="Rotation from point-cloud (YAML) frame to Isaac object frame (intrinsic XYZ deg). "
        "Prefer aligning the PLY in CloudCompare first; use this only if a fixed offset remains. "
        "Typical Y-up vs Z-up: try -90 0 0 or 90 0 0.",
    )
    parser.add_argument(
        "--no-approach",
        action="store_true",
        help="Skip linear approach waypoint; go directly to grasp pose (default: approach enabled).",
    )
    parser.add_argument(
        "--approach-offset-m",
        type=float,
        default=0.2,
        help="Retreat distance (m) along TCP axis before linear-in grasp (default: 0.2).",
    )
    parser.add_argument(
        "--approach-axis-local",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 1.0],
        metavar=("LX", "LY", "LZ"),
        help="Unit direction in TCP frame from approach toward final (default: 0 0 1 = +Z).",
    )
    parser.add_argument(
        "--no-open-gripper-first",
        action="store_true",
        help="Do not open gripper before the approach motion.",
    )
    parser.add_argument(
        "--gripper-settle-s",
        type=float,
        default=0.2,
        help="Sleep after close before lift or home (default: 0.2).",
    )
    parser.add_argument(
        "--post-grasp-lift-z",
        type=float,
        default=0.3,
        help="After close, move TCP this many meters along +panda_link0 Z (default: 0.3). Ignored with --move-home-after.",
    )
    parser.add_argument(
        "--no-post-grasp-lift",
        action="store_true",
        help="After close, stay at grasp (no vertical lift).",
    )
    parser.add_argument(
        "--move-home-after",
        action="store_true",
        help="After close, move to default home instead of lifting.",
    )
    parser.add_argument(
        "--inter-segment-settle-s",
        type=float,
        default=0.4,
        help="Sleep after approach (and after close before lift/home) so MoveIt sees updated /joint_states (default: 0.4). Set 0 to disable.",
    )
    parser.add_argument(
        "--no-align-graspgen-franka-fingers",
        action="store_true",
        help="Disable fixed GraspGen→panda_hand rotation (GraspGen +X close vs URDF finger axis on +Y). Default: alignment ON.",
    )
    parser.add_argument(
        "--approach-collision-box-pc",
        type=Path,
        default=None,
        help="Point cloud .ply or .npy for AABB obstacle during approach only. "
        "Default: file beside YAML with same stem as grasps (e.g. Mug_1_grasps.yaml → Mug_1.ply).",
    )
    parser.add_argument(
        "--no-approach-collision-box",
        action="store_true",
        help="Do not publish MoveIt's AABB obstacle from the point cloud (no object-shaped planning-scene box). "
        "Floor collision from launch (add_ground_collision) is unchanged. Does not disable Isaac PhysX colliders.",
    )
    parser.add_argument(
        "--no-object-collision-box",
        action="store_true",
        help="Alias for --no-approach-collision-box (same effect).",
    )
    parser.add_argument(
        "--approach-collision-box-padding-m",
        type=float,
        default=0.0,
        help="Padding added to each AABB half-extent of the approach obstacle (m). Default: 0.",
    )
    parser.add_argument(
        "--approach-collision-scene-settle-s",
        type=float,
        default=0.15,
        help="Sleep after each collision_object ADD/REMOVE so MoveIt applies the update. Default: 0.15.",
    )
    parsed, unknown = parser.parse_known_args(args)

    object_center_override: Optional[tuple[float, float, float]] = None
    if parsed.object_center is not None:
        oc = parsed.object_center
        if len(oc) == 1:
            object_center_override = (
                float(parsed.object_x),
                float(parsed.object_y),
                float(oc[0]),
            )
        elif len(oc) == 3:
            object_center_override = (float(oc[0]), float(oc[1]), float(oc[2]))
        else:
            parser.error("--object-center: pass exactly 1 float (Z only) or 3 floats (X Y Z)")

    if parsed.object_rpy_deg is not None:
        object_rpy_deg = (parsed.object_rpy_deg[0], parsed.object_rpy_deg[1], parsed.object_rpy_deg[2])
    elif parsed.object_yaw_deg is not None:
        object_rpy_deg = (0.0, 0.0, parsed.object_yaw_deg)
    else:
        object_rpy_deg = (0.0, 0.0, 0.0)

    if parsed.gripper_offset_rpy_deg is not None:
        xyz = parsed.gripper_offset_xyz if parsed.gripper_offset_xyz is not None else (0.0, 0.0, 0.0)
        T_user = matrix4_from_rpy_xyz(
            (parsed.gripper_offset_rpy_deg[0], parsed.gripper_offset_rpy_deg[1], parsed.gripper_offset_rpy_deg[2]),
            (xyz[0], xyz[1], xyz[2]),
        )
    else:
        T_user = np.eye(4)

    if parsed.yaml_pose_frame == "link8":
        T_base = np.eye(4)
    else:
        T_base = np.linalg.inv(franka_T_link8_to_panda_hand())
    T_gripper_offset = matrix4_compose(T_base, T_user)

    sw = (
        tuple(parsed.pc_centroid_shift_world_xyz)
        if parsed.pc_centroid_shift_world_xyz is not None
        else (0.0, 0.0, 0.0)
    )
    sl = (
        tuple(parsed.pc_centroid_shift_local_xyz)
        if parsed.pc_centroid_shift_local_xyz is not None
        else (0.0, 0.0, 0.0)
    )

    sim_rpy_cli: Optional[tuple[float, float, float]] = None
    if parsed.sim_from_pc_frame_rpy_deg is not None:
        sim_rpy_cli = (
            parsed.sim_from_pc_frame_rpy_deg[0],
            parsed.sim_from_pc_frame_rpy_deg[1],
            parsed.sim_from_pc_frame_rpy_deg[2],
        )

    ax = parsed.approach_axis_local
    lift_z = (
        0.0
        if parsed.move_home_after or parsed.no_post_grasp_lift
        else float(parsed.post_grasp_lift_z)
    )
    exec_config = GraspExecutionConfig(
        use_approach=not parsed.no_approach,
        approach_offset_m=float(parsed.approach_offset_m),
        approach_direction_local_xyz=(float(ax[0]), float(ax[1]), float(ax[2])),
        open_gripper_before=not parsed.no_open_gripper_first,
        gripper_settle_s=float(parsed.gripper_settle_s),
        inter_segment_settle_s=float(parsed.inter_segment_settle_s),
        move_home_after=parsed.move_home_after,
        post_grasp_lift_world_z_m=lift_z,
    )

    rclpy.init(args=unknown)
    node = rclpy.create_node("grasp_with_candidates")
    try:
        exit_code = run(
            node,
            parsed.path,
            parsed.log_dir,
            table_z=parsed.table_z,
            object_x=parsed.object_x,
            object_y=parsed.object_y,
            object_center_override=object_center_override,
            object_rpy_deg=object_rpy_deg,
            T_gripper_offset=T_gripper_offset,
            pc_centroid_shift_world_xyz=sw,
            pc_centroid_shift_local_xyz=sl,
            yaml_pose_frame=parsed.yaml_pose_frame,
            sim_from_pc_frame_rpy_deg_cli=sim_rpy_cli,
            exec_config=exec_config,
            align_graspgen_franka_fingers=not parsed.no_align_graspgen_franka_fingers,
            approach_collision_pc=parsed.approach_collision_box_pc,
            approach_collision_disabled=bool(
                parsed.no_approach_collision_box or parsed.no_object_collision_box
            ),
            approach_collision_padding_m=float(parsed.approach_collision_box_padding_m),
            approach_collision_settle_s=float(parsed.approach_collision_scene_settle_s),
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
