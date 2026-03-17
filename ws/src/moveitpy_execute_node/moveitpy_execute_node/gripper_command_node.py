"""
Node that forwards gripper commands to Isaac: on /gripper_command (Float64) it
updates the target and continuously publishes /joint_command so the gripper
keeps applying force (e.g. keeps squeezing when closed) instead of freezing after 1s.
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .constants import (
    ALL_JOINTS,
    GRIPPER_CMD_TOPIC,
    GRIPPER_CLOSE,
    GRIPPER_OPEN,
    PANDA_ARM_JOINTS,
    PANDA_GRIPPER_JOINTS,
)

JOINT_CMD_TOPIC = "/joint_command"
JOINT_STATES_TOPIC = "/joint_states"
CMD_HZ = 20


class GripperCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_command")
        self._lock = threading.Lock()
        self._position: dict[str, float] = {}
        self._velocity: dict[str, float] = {}
        self._gripper_target: float = GRIPPER_OPEN
        self._pub = self.create_publisher(JointState, JOINT_CMD_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_states_cb, 10)
        self.create_subscription(
            Float64, GRIPPER_CMD_TOPIC, self._gripper_cmd_cb, 10
        )
        self._timer = self.create_timer(1.0 / CMD_HZ, self._timer_cb)
        self.get_logger().info(
            f"Gripper command: {GRIPPER_CMD_TOPIC} -> {JOINT_CMD_TOPIC} at {CMD_HZ} Hz (hold target)"
        )

    def _joint_states_cb(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._position[name] = msg.position[i]
                if msg.velocity and i < len(msg.velocity):
                    self._velocity[name] = msg.velocity[i]

    def _build_joint_command(self, gripper_position: float) -> JointState | None:
        with self._lock:
            if not self._position:
                return None
            arm_pos = [self._position.get(j, 0.0) for j in PANDA_ARM_JOINTS]
            arm_vel = [self._velocity.get(j, 0.0) for j in PANDA_ARM_JOINTS]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = arm_pos + [gripper_position, gripper_position]
        msg.velocity = arm_vel + [0.0, 0.0]
        msg.effort = []
        return msg

    def _timer_cb(self) -> None:
        cmd = self._build_joint_command(self._gripper_target)
        if cmd:
            self._pub.publish(cmd)

    def _gripper_cmd_cb(self, msg: Float64) -> None:
        self._gripper_target = float(msg.data)
        self.get_logger().info(f"Gripper target set: {self._gripper_target} (open={GRIPPER_OPEN}, close={GRIPPER_CLOSE})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
