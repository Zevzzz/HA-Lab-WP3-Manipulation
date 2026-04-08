"""
Executor node: exposes ExecutePose action for motion planning and execution.

This node owns the MoveItPy instance and serves as the single entry point for
"move to pose" requests from demos, GraspGen evaluation, or other nodes.
"""

from moveitpy_execute_node_msgs.action import ExecutePose

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from .constants import TIP_LINK
from .motion import PoseExecutor


class ExecutorNode(Node):
    """
    Long-lived node that provides the ExecutePose action.

    Must be launched with the same name as passed to MoveItPy (default "moveit_py")
    and with the MoveIt config parameters so that MoveItPy can connect.
    """

    DEFAULT_NODE_NAME = "moveit_py"
    ACTION_NAME = "execute_pose"

    def __init__(self, *, node_name: str = DEFAULT_NODE_NAME) -> None:
        super().__init__(node_name)
        self.declare_parameter("cartesian_tip_link", TIP_LINK)
        tip = self.get_parameter("cartesian_tip_link").get_parameter_value().string_value
        if not tip:
            tip = TIP_LINK
        self._executor = PoseExecutor(self, tip_link=tip)
        self.get_logger().info(f"ExecutePose Cartesian pose_link={tip}")
        self._action_server = ActionServer(
            self,
            ExecutePose,
            self.ACTION_NAME,
            self._execute_pose_callback,
        )
        self.get_logger().info(
            f"Executor node ready. Action '{self.ACTION_NAME}' available."
        )

    def _execute_pose_callback(self, goal_handle) -> ExecutePose.Result:
        goal = goal_handle.request
        pose = goal.target_pose
        # Ensure stamp is set for TF lookups
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            pose.header.stamp = self.get_clock().now().to_msg()

        result = ExecutePose.Result()
        plan_only = getattr(goal, "plan_only", False)
        try:
            if plan_only:
                plan_result = self._executor.plan_to_pose(pose)
                success = plan_result is not None
                result.message = "Plan found." if success else "Planning failed."
            else:
                success = self._executor.plan_and_execute(pose)
                result.message = "Execution completed." if success else "Planning or execution failed."
            # Must be a strict Python bool for ROS message serialization (PyBool_Check)
            result.success = bool(success)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"ExecutePose failed: {e}")
            result.success = False
            result.message = str(e)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
