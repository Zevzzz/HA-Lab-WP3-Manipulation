"""Gripper command helper: publish target to /gripper_command and allow delivery."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .constants import GRIPPER_CMD_TOPIC

# Number of spin_once calls after publish so the message is delivered (e.g. to trajectory_bridge).
GRIPPER_PUBLISH_SPIN_COUNT = 20
GRIPPER_PUBLISH_SPIN_TIMEOUT_S = 0.05


def send_gripper_command(
    node: Node,
    value: float,
    *,
    topic: str = GRIPPER_CMD_TOPIC,
    spin_after: int = GRIPPER_PUBLISH_SPIN_COUNT,
    spin_timeout_s: float = GRIPPER_PUBLISH_SPIN_TIMEOUT_S,
) -> None:
    """
    Publish a single gripper command (position) and spin a few times so it is delivered.

    Use this from executor, demos, or any node that needs to set gripper open/close.
    The trajectory_bridge (or gripper_command_node) subscribes to this topic.
    """
    pub = node.create_publisher(Float64, topic, 10)
    msg = Float64()
    msg.data = float(value)
    pub.publish(msg)
    for _ in range(spin_after):
        rclpy.spin_once(node, timeout_sec=spin_timeout_s)
