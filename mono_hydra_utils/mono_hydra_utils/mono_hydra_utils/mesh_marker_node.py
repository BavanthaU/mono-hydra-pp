#!/usr/bin/env python3
from __future__ import annotations

from collections.abc import Sequence

import rclpy
from geometry_msgs.msg import Point
from kimera_pgmo_msgs.msg import Mesh
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class MeshMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("mono_hydra_dsg_mesh_marker")

        self.declare_parameter("input_mesh_topic", "/hydra_dsg_visualizer/dsg_mesh")
        self.declare_parameter("output_marker_topic", "/hydra_dsg_visualizer/dsg_mesh_marker")
        self.declare_parameter("fallback_frame_id", "map")
        self.declare_parameter("mesh_alpha", 0.92)
        self.declare_parameter("default_color", [0.72, 0.74, 0.78, 0.92])
        self.declare_parameter("max_triangles", 0)

        self.input_topic = str(self.get_parameter("input_mesh_topic").value)
        self.output_topic = str(self.get_parameter("output_marker_topic").value)
        self.fallback_frame_id = str(self.get_parameter("fallback_frame_id").value)
        self.mesh_alpha = float(self.get_parameter("mesh_alpha").value)
        self.default_color = self._make_color(self.get_parameter("default_color").value)
        self.max_triangles = int(self.get_parameter("max_triangles").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Marker, self.output_topic, qos)
        self.subscription = self.create_subscription(Mesh, self.input_topic, self._on_mesh, qos)

        self.get_logger().info(
            f"Rendering Kimera-PGMO mesh '{self.input_topic}' as RViz Marker '{self.output_topic}'"
        )

    def _make_color(self, values: object) -> ColorRGBA:
        color = ColorRGBA()
        rgba = list(values) if isinstance(values, Sequence) and not isinstance(values, str) else []
        color.r = float(rgba[0]) if len(rgba) > 0 else 0.62
        color.g = float(rgba[1]) if len(rgba) > 1 else 0.64
        color.b = float(rgba[2]) if len(rgba) > 2 else 0.68
        color.a = self.mesh_alpha
        return color

    def _delete_marker(self, msg: Mesh) -> None:
        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = marker.header.frame_id or self.fallback_frame_id
        marker.ns = "dsg_mesh"
        marker.id = 0
        marker.action = Marker.DELETE
        self.publisher.publish(marker)

    def _vertex_color(self, vertex) -> ColorRGBA:
        color = ColorRGBA()
        if getattr(vertex, "has_color", False):
            color.r = vertex.color.r
            color.g = vertex.color.g
            color.b = vertex.color.b
            color.a = self.mesh_alpha
        else:
            color = self.default_color
        return color

    def _on_mesh(self, msg: Mesh) -> None:
        if not msg.vertices or not msg.triangles:
            self._delete_marker(msg)
            return

        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = marker.header.frame_id or self.fallback_frame_id
        marker.ns = "dsg_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color = self.default_color

        triangles = msg.triangles
        if self.max_triangles > 0:
            triangles = triangles[: self.max_triangles]

        vertices = msg.vertices
        for triangle in triangles:
            for index in triangle.vertex_indices:
                if index >= len(vertices):
                    continue
                vertex = vertices[index]
                point = Point()
                point.x = vertex.pos.x
                point.y = vertex.pos.y
                point.z = vertex.pos.z
                marker.points.append(point)
                marker.colors.append(self._vertex_color(vertex))

        if not marker.points:
            self._delete_marker(msg)
            return

        self.publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = MeshMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
