"""
Modular grasp execution: approach waypoint, final pose, gripper, then post-grasp motion.

Default post-grasp: translate TCP +0.3 m along ``panda_link0`` +Z (lift). Optional: move to
home instead. Swap ``send_pose``, ``home_pose``, or extend ``GraspExecutionConfig`` for other flows.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from .constants import GRIPPER_CLOSE, GRIPPER_OPEN
from .gripper import send_gripper_command
from .pose_sources import get_default_home_pose
from .utils import sleep_until_ok


class ExecutePoseClientFn(Protocol):
    """Pluggable plan/execute step (default: ``grasp_with_candidates.send_pose_and_wait``)."""

    def __call__(
        self,
        node: Node,
        client: ActionClient,
        pose: PoseStamped,
        *,
        plan_only: bool = ...,
        timeout_send: float = ...,
        timeout_result: float = ...,
    ) -> Tuple[bool, str]:
        ...


@dataclass
class GraspExecutionConfig:
    """Tunable grasp pipeline; pass through from CLI or construct in higher-level eval code."""

    use_approach: bool = True
    approach_offset_m: float = 0.05
    # Unit vector in TCP frame: motion from approach waypoint toward final is along this axis in world.
    approach_direction_local_xyz: tuple[float, float, float] = (0.0, 0.0, 1.0)
    open_gripper_before: bool = True
    gripper_open_wait_s: float = 0.5
    gripper_close_value: float = GRIPPER_CLOSE
    gripper_settle_s: float = 1.5
    # Pause after each motion segment so sim + MoveIt ``current_state`` match (avoids immediate replan fail).
    inter_segment_settle_s: float = 0.4
    # After close: either home (if True) or lift along +Z of pose frame (typically panda_link0).
    move_home_after: bool = False
    post_grasp_lift_world_z_m: float = 0.3


def compute_approach_pose(
    final_tcp_pose: PoseStamped,
    offset_m: float,
    direction_local_xyz: tuple[float, float, float],
) -> PoseStamped:
    """
    Retreat waypoint: same orientation as ``final_tcp_pose``, position offset by
    ``-offset_m * (R * direction_local)`` so moving approach -> final is a straight line
    along the TCP axis (default local +Z, e.g. down onto a rim when +Z points into the grasp).
    """
    v = np.array(direction_local_xyz, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        v = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        v = v / n
    q = np.array(
        [
            final_tcp_pose.pose.orientation.x,
            final_tcp_pose.pose.orientation.y,
            final_tcp_pose.pose.orientation.z,
            final_tcp_pose.pose.orientation.w,
        ],
        dtype=np.float64,
    )
    r = Rotation.from_quat(q)
    d_world = r.apply(v)
    out = PoseStamped()
    out.header.frame_id = final_tcp_pose.header.frame_id
    pf = final_tcp_pose.pose.position
    out.pose.orientation = final_tcp_pose.pose.orientation
    out.pose.position.x = float(pf.x - offset_m * d_world[0])
    out.pose.position.y = float(pf.y - offset_m * d_world[1])
    out.pose.position.z = float(pf.z - offset_m * d_world[2])
    return out


def compute_post_grasp_lift_pose(
    grasp_tcp_pose: PoseStamped,
    delta_z_m: float,
) -> PoseStamped:
    """Same orientation as grasp; position shifted by +delta_z along the pose ``frame_id`` +Z (e.g. panda_link0)."""
    out = PoseStamped()
    out.header.frame_id = grasp_tcp_pose.header.frame_id
    out.pose.orientation = grasp_tcp_pose.pose.orientation
    p = grasp_tcp_pose.pose.position
    out.pose.position.x = p.x
    out.pose.position.y = p.y
    out.pose.position.z = float(p.z + delta_z_m)
    return out


def plan_grasp_with_optional_approach(
    node: Node,
    client: ActionClient,
    final_pose: PoseStamped,
    config: GraspExecutionConfig,
    send_pose: ExecutePoseClientFn,
    *,
    plan_timeout_s: float = 15.0,
) -> tuple[bool, str, Optional[PoseStamped]]:
    """
    Plan-only check for the selected candidate. Returns (ok, message, approach_pose or None).
    """
    approach: Optional[PoseStamped] = None
    parts: list[str] = []

    if config.use_approach:
        approach = compute_approach_pose(
            final_pose,
            config.approach_offset_m,
            config.approach_direction_local_xyz,
        )
        ok_a, msg_a = send_pose(
            node,
            client,
            approach,
            plan_only=True,
            timeout_result=plan_timeout_s,
        )
        if not ok_a:
            return False, f"Approach plan failed: {msg_a}", approach
        parts.append(msg_a)

    ok_f, msg_f = send_pose(
        node,
        client,
        final_pose,
        plan_only=True,
        timeout_result=plan_timeout_s,
    )
    if not ok_f:
        prefix = "Final plan failed" if not parts else "Final plan failed (after approach ok)"
        return False, f"{prefix}: {msg_f}", approach
    parts.append(msg_f)

    if config.move_home_after:
        tag = "approach + final + home (home not pre-planned)" if config.use_approach else "final + home (home not pre-planned)"
        return True, f"Planned {tag}. " + " | ".join(parts), approach

    if config.post_grasp_lift_world_z_m > 1e-9:
        lift_pose = compute_post_grasp_lift_pose(final_pose, config.post_grasp_lift_world_z_m)
        ok_l, msg_l = send_pose(
            node,
            client,
            lift_pose,
            plan_only=True,
            timeout_result=plan_timeout_s,
        )
        if not ok_l:
            return False, f"Post-grasp lift plan failed: {msg_l}", approach
        parts.append(msg_l)

    return True, "Planned " + "; ".join(parts), approach


def execute_grasp_sequence(
    node: Node,
    client: ActionClient,
    final_pose: PoseStamped,
    config: GraspExecutionConfig,
    send_pose: ExecutePoseClientFn,
    *,
    approach_pose: Optional[PoseStamped],
    home_pose: Optional[PoseStamped] = None,
    exec_timeout_s: float = 60.0,
    logger=None,
) -> tuple[bool, str]:
    """
    Run motion + gripper + post-grasp lift or home after plans succeeded.
    ``approach_pose`` must be provided when ``config.use_approach`` (from planning step).
    """
    log = logger.info if logger is not None else (lambda *_a, **_k: None)

    if config.open_gripper_before:
        log("Opening gripper before approach.")
        send_gripper_command(node, GRIPPER_OPEN)
        sleep_until_ok(config.gripper_open_wait_s)

    if config.use_approach:
        if approach_pose is None:
            return False, "Internal error: use_approach but approach_pose is None"
        log("Executing approach waypoint.")
        ok, msg = send_pose(
            node,
            client,
            approach_pose,
            plan_only=False,
            timeout_result=exec_timeout_s,
        )
        if not ok:
            return False, f"Approach execution failed: {msg}"
        if config.inter_segment_settle_s > 0.0:
            log(f"Settling {config.inter_segment_settle_s}s after approach (sim / joint_states sync).")
            sleep_until_ok(config.inter_segment_settle_s)

    log("Executing final grasp pose.")
    ok, msg = send_pose(
        node,
        client,
        final_pose,
        plan_only=False,
        timeout_result=exec_timeout_s,
    )
    if not ok:
        return False, f"Final pose execution failed: {msg}"

    log("Closing gripper.")
    send_gripper_command(node, config.gripper_close_value)
    sleep_until_ok(config.gripper_settle_s)

    if config.inter_segment_settle_s > 0.0 and (
        config.move_home_after or config.post_grasp_lift_world_z_m > 1e-9
    ):
        log(f"Settling {config.inter_segment_settle_s}s after gripper close before retreat.")
        sleep_until_ok(config.inter_segment_settle_s)

    if config.move_home_after:
        home = home_pose if home_pose is not None else get_default_home_pose()
        log("Moving to home.")
        ok_h, msg_h = send_pose(
            node,
            client,
            home,
            plan_only=False,
            timeout_result=exec_timeout_s,
        )
        if not ok_h:
            return False, f"Home execution failed: {msg_h}"
        return True, "Grasp sequence complete (home reached)."

    if config.post_grasp_lift_world_z_m > 1e-9:
        lift_pose = compute_post_grasp_lift_pose(final_pose, config.post_grasp_lift_world_z_m)
        log(f"Lifting {config.post_grasp_lift_world_z_m} m along +Z ({lift_pose.header.frame_id}).")
        ok_l, msg_l = send_pose(
            node,
            client,
            lift_pose,
            plan_only=False,
            timeout_result=exec_timeout_s,
        )
        if not ok_l:
            return False, f"Post-grasp lift failed: {msg_l}"
        return True, "Grasp sequence complete (lift)."

    return True, "Grasp sequence complete (no post-grasp motion)."
