"""
Publish a planning-scene box under z=0 so MoveIt treats the ground as solid.

Frame defaults to panda_link0 (matches static world->panda_link0 in our launches).
A large thin BOX sits with its top at z=0; anything below is in collision.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


def main() -> None:
    rclpy.init()
    node = GroundPlaneSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


class GroundPlaneSceneNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_plane_scene")
        self.declare_parameter("reference_frame", "panda_link0")
        self.declare_parameter("object_id", "ground_plane")
        self.declare_parameter("half_extent_xy_m", 5.0)
        self.declare_parameter("thickness_m", 0.05)
        self.declare_parameter("publish_delay_s", 2.0)
        self.declare_parameter("repeat_publish", 3)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(CollisionObject, "collision_object", qos)
        delay = float(self.get_parameter("publish_delay_s").value)
        self._timer = self.create_timer(delay, self._publish_after_startup)
        self._co: CollisionObject | None = None

    def _build_collision_object(self) -> CollisionObject:
        frame = self.get_parameter("reference_frame").get_parameter_value().string_value
        oid = self.get_parameter("object_id").get_parameter_value().string_value
        he = float(self.get_parameter("half_extent_xy_m").value)
        t = float(self.get_parameter("thickness_m").value)

        co = CollisionObject()
        co.header.frame_id = frame
        co.id = oid
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0 * he, 2.0 * he, t]

        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.z = -t / 2.0

        co.primitives.append(box)
        co.primitive_poses.append(pose)
        return co

    def _publish_after_startup(self) -> None:
        self.destroy_timer(self._timer)
        if self._co is None:
            self._co = self._build_collision_object()
        n = max(1, int(self.get_parameter("repeat_publish").value))
        t = float(self.get_parameter("thickness_m").value)
        he = float(self.get_parameter("half_extent_xy_m").value)
        self.get_logger().info(
            f"Publishing '{self._co.id}' in {self._co.header.frame_id}: "
            f"BOX {2.0 * he:.1f} x {2.0 * he:.1f} x {t:.3f} m, top at z=0 ({n}x)"
        )
        for _ in range(n):
            self._co.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._co)
        self.get_logger().info("Ground plane collision object done.")


if __name__ == "__main__":
    main()
