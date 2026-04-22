"""
Core motion planning and execution.

PoseExecutor wraps MoveItPy and exposes plan/execute for a single pose or trajectory.
Used by the executor node (action server) and can be used in-process by other code.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Optional

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy

from .constants import ARM_CONTROLLER, PLANNING_GROUP, TIP_LINK

if TYPE_CHECKING:
    from rclpy.node import Node


class PoseExecutor:
    """
    Plan and execute end-effector motions using MoveItPy.

    The node passed to __init__ must already be running and must have been
    started with the MoveIt config parameters (e.g. from launch). MoveItPy
    uses the node name to find that node and its params.
    """

    def __init__(self, node: Node, *, tip_link: str | None = None) -> None:
        self._node = node
        self._tip_link = tip_link or TIP_LINK
        self._robot = MoveItPy(node_name=node.get_name())
        self._arm = self._robot.get_planning_component(PLANNING_GROUP)

    def plan_to_pose(self, pose: PoseStamped) -> Optional[object]:
        """
        Plan to the given pose. Returns the plan result (with .trajectory) or None if planning failed.

        Caller can then call execute_trajectory(plan_result.trajectory).
        """
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=self._tip_link)
        t0 = time.perf_counter()
        plan_result = self._arm.plan()
        elapsed = time.perf_counter() - t0
        ok = bool(plan_result)
        self._node.get_logger().info(
            f"plan_to_pose: planning took {elapsed:.3f}s ({'success' if ok else 'failed'})"
        )
        return plan_result if plan_result else None

    def execute_trajectory(self, trajectory: object) -> bool:
        """
        Execute a planned trajectory (e.g. from plan_to_pose).

        Returns True if execution completed successfully.
        """
        status = self._robot.execute(trajectory, [ARM_CONTROLLER])
        return bool(status)

    def plan_and_execute(self, pose: PoseStamped) -> bool:
        """
        Plan to pose and execute in one step. Returns True on success.
        """
        plan_result = self.plan_to_pose(pose)
        if not plan_result:
            self._node.get_logger().warning(
                "ExecutePose: planning failed (start state may be stale vs sim; try settle between segments)."
            )
            return False
        if not self.execute_trajectory(plan_result.trajectory):
            self._node.get_logger().warning(
                "ExecutePose: trajectory execution failed (controller / Isaac / time parameterization)."
            )
            return False
        return True
