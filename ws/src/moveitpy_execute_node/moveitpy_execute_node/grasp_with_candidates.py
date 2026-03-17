"""
One-shot client: load grasps YAML (object frame), transform to world frame, try ExecutePose.

Grasps in the YAML are in object frame (origin = object centroid). Object pose in world
is derived from table height + object_half_height_m in YAML, or from --object-center.

Usage:
  ros2 run moveitpy_execute_node grasp_with_candidates --path <path_to_yaml> [options]

Expects the executor node (ExecutePose action server) to be running. Logs one row to
data/logs/grasp_execution_results.csv (or --log-dir).
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml

from moveitpy_execute_node_msgs.action import ExecutePose

from .constants import FRAME_ID as WORLD_FRAME_ID
from .transform_utils import transform_pose, translation_matrix

ACTION_NAME = "execute_pose"
DEFAULT_LOG_DIR = Path("data/logs")
EXECUTION_LOG_NAME = "grasp_execution_results.csv"

# Default eval setup: object centered on table at (x, y); table surface at table_z.
# Object center z = table_z + object_half_height_m (from YAML).
DEFAULT_TABLE_Z_M = 0.05
DEFAULT_OBJECT_X_M = 0.4
DEFAULT_OBJECT_Y_M = 0.0

# YAML key written by graspgen_request (must match scripts/graspgen_request.py)
YAML_KEY_OBJECT_HALF_HEIGHT_M = "object_half_height_m"


@dataclass(frozen=True)
class ObjectPoseConfig:
    """Object center position in world frame (meters). Rotation is identity (upright)."""
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class GraspsYamlData:
    """Parsed grasps YAML: grasps in object frame + optional object_half_height_m."""
    grasps: list[dict]
    object_half_height_m: Optional[float]


def load_grasps_yaml(path: Path) -> GraspsYamlData:
    """Load YAML: grasps list (object frame) and optional object_half_height_m."""
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
    return GraspsYamlData(grasps=grasps, object_half_height_m=half_height)


def resolve_object_pose(
    table_z: float,
    object_x: float,
    object_y: float,
    object_half_height_m: Optional[float],
    object_center_override: Optional[tuple[float, float, float]],
) -> ObjectPoseConfig:
    """
    Resolve object center in world frame.

    If object_center_override (x,y,z) is set, use it. Else require object_half_height_m
    and set center z = table_z + object_half_height_m, x/y = object_x, object_y.
    """
    if object_center_override is not None:
        return ObjectPoseConfig(
            x=object_center_override[0],
            y=object_center_override[1],
            z=object_center_override[2],
        )
    if object_half_height_m is None:
        raise ValueError(
            "YAML has no 'object_half_height_m'. Either regenerate the YAML with graspgen_request.py "
            "(which writes it automatically) or pass --object-center x y z."
        )
    return ObjectPoseConfig(
        x=object_x,
        y=object_y,
        z=table_z + object_half_height_m,
    )


def grasp_object_frame_to_pose_stamped(
    grasp: dict,
    T_world_object: np.ndarray,
    world_frame_id: str,
) -> PoseStamped:
    """Transform one object-frame grasp to world-frame PoseStamped."""
    p = grasp.get("position") or [0.0, 0.0, 0.0]
    o = grasp.get("orientation") or [0.0, 0.0, 0.0, 1.0]
    pos_obj = (float(p[0]), float(p[1]), float(p[2]))
    quat_obj = (float(o[0]), float(o[1]), float(o[2]), float(o[3]))
    pos_world, quat_world = transform_pose(T_world_object, pos_obj, quat_obj)
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
    """Send ExecutePose goal and wait for result. Returns (success, message)."""
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
    """Append one row to grasp_execution_results.csv."""
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
) -> int:
    """Load YAML, resolve object pose, transform grasps to world frame, try candidates."""
    logger = node.get_logger()
    try:
        data = load_grasps_yaml(yaml_path)
    except Exception as e:
        logger.error(f"Failed to load YAML: {e}")
        append_execution_log(
            log_dir, str(yaml_path), 0, 0, 0, False,
            message=f"Load error: {e}",
        )
        return 1

    grasps = data.grasps
    total = len(grasps)
    if total == 0:
        logger.error("No grasps in YAML")
        append_execution_log(log_dir, str(yaml_path), 0, 0, 0, False, message="No grasps")
        return 1

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
        append_execution_log(
            log_dir, str(yaml_path), total, 0, 0, False,
            message=str(e),
        )
        return 1

    T_world_object = translation_matrix(object_pose.x, object_pose.y, object_pose.z)
    logger.info(
        f"Object center in world: ({object_pose.x}, {object_pose.y}, {object_pose.z})"
    )

    client = ActionClient(node, ExecutePose, ACTION_NAME)
    logger.info(f"Waiting for action server '{ACTION_NAME}'...")
    if not client.wait_for_server(timeout_sec=10.0):
        logger.error("Action server not available. Is the executor node running?")
        append_execution_log(
            log_dir, str(yaml_path), total, 0, 0, False,
            message="Action server not available",
        )
        return 1

    candidate_index_used = 0
    num_failed_before = 0
    last_message = ""
    winning_pose: Optional[PoseStamped] = None

    # Phase 1: plan-only until one candidate plans successfully
    for i, g in enumerate(grasps):
        idx_1based = i + 1
        pose = grasp_object_frame_to_pose_stamped(g, T_world_object, WORLD_FRAME_ID)
        logger.info(f"Planning candidate {idx_1based}/{total}")
        success, msg = send_pose_and_wait(node, client, pose, plan_only=True, timeout_result=15.0)
        if success:
            candidate_index_used = idx_1based
            num_failed_before = i
            winning_pose = pose
            last_message = msg or "Plan found."
            logger.info(f"Candidate {idx_1based} planned successfully, executing once.")
            break
        last_message = msg
        logger.warning(f"Candidate {idx_1based} failed: {msg}")

    # Phase 2: execute the winning plan once (if we have one)
    if winning_pose is not None:
        exec_ok, msg = send_pose_and_wait(node, client, winning_pose, plan_only=False)
        if not exec_ok:
            success = False
            last_message = msg or "Execution failed."
            logger.error(f"Execution failed after successful plan: {last_message}")
        else:
            success = True
            last_message = last_message or "OK"
    else:
        success = False
    append_execution_log(
        log_dir, str(yaml_path), total, candidate_index_used, num_failed_before, success, last_message,
    )
    logger.info(
        f"Logged: yaml={yaml_path}, total={total}, used={candidate_index_used}, failed_before={num_failed_before}, success={success}"
    )
    return 0 if success else 1


def main(args=None) -> int:
    parser = argparse.ArgumentParser(
        description="Load grasps YAML (object frame), transform to world, try ExecutePose; log to CSV.",
    )
    parser.add_argument("--path", type=Path, required=True, help="Path to grasps YAML")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="Directory for execution log CSV")
    parser.add_argument(
        "--table-z",
        type=float,
        default=DEFAULT_TABLE_Z_M,
        help=f"Table surface z in world frame (m). Object center z = table_z + object_half_height_m (default: {DEFAULT_TABLE_Z_M})",
    )
    parser.add_argument(
        "--object-x",
        type=float,
        default=DEFAULT_OBJECT_X_M,
        help=f"Object center x in world frame (m) (default: {DEFAULT_OBJECT_X_M})",
    )
    parser.add_argument(
        "--object-y",
        type=float,
        default=DEFAULT_OBJECT_Y_M,
        help=f"Object center y in world frame (m) (default: {DEFAULT_OBJECT_Y_M})",
    )
    parser.add_argument(
        "--object-center",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Override object center in world (m). If set, ignores --table-z/--object-x/--object-y and object_half_height_m",
    )
    parsed, unknown = parser.parse_known_args(args)

    object_center_override: Optional[tuple[float, float, float]] = None
    if parsed.object_center is not None:
        object_center_override = (parsed.object_center[0], parsed.object_center[1], parsed.object_center[2])

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
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
