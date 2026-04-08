#!/usr/bin/env python3
"""
Bridge Isaac Sim topics <-> topic_based_ros2_control (TopicBasedSystem).

The Panda URDF (ros2_control_hardware_type:=isaac) expects:
  - State in:  /isaac_joint_states
  - Commands: plugin publishes to /isaac_joint_commands

Typical Isaac graphs use /joint_states and /joint_command. This node:
  - Forwards Isaac joint states -> /isaac_joint_states (so ros2_control can track the sim robot)
  - Forwards /isaac_joint_commands -> /joint_command (so Isaac applies ros2_control targets)

No trajectory interpolation: joint_trajectory_controller (stock) sends discrete setpoints;
TopicBasedSystem publishes one JointState per update (industry-standard pattern with Isaac).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacRos2ControlRelay(Node):
    def __init__(self) -> None:
        super().__init__("isaac_ros2_control_relay")
        self.declare_parameter("sim_joint_states_in", "/joint_states")
        self.declare_parameter("ros2_joint_states_out", "/isaac_joint_states")
        self.declare_parameter("ros2_joint_commands_in", "/isaac_joint_commands")
        self.declare_parameter("sim_joint_commands_out", "/joint_command")

        st_in = self.get_parameter("sim_joint_states_in").get_parameter_value().string_value
        st_out = self.get_parameter("ros2_joint_states_out").get_parameter_value().string_value
        cmd_in = self.get_parameter("ros2_joint_commands_in").get_parameter_value().string_value
        cmd_out = self.get_parameter("sim_joint_commands_out").get_parameter_value().string_value

        self._pub_state = self.create_publisher(JointState, st_out, 10)
        self._pub_cmd = self.create_publisher(JointState, cmd_out, 10)
        self.create_subscription(JointState, st_in, self._on_sim_state, 10)
        self.create_subscription(JointState, cmd_in, self._on_ros2_cmd, 10)

        self.get_logger().info(
            f"Isaac <-> ros2_control relay: '{st_in}' -> '{st_out}'; "
            f"'{cmd_in}' -> '{cmd_out}'"
        )

    def _on_sim_state(self, msg: JointState) -> None:
        self._pub_state.publish(msg)

    def _on_ros2_cmd(self, msg: JointState) -> None:
        self._pub_cmd.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacRos2ControlRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
