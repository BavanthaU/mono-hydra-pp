#!/usr/bin/env python3

from __future__ import annotations

import copy

import rclpy
from pose_graph_tools_msgs.msg import PoseGraph, PoseGraphEdge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _gtsam_symbol_index(key: int) -> int:
    return int(key) & ((1 << 56) - 1)


class KimeraPoseGraphBridgeNode(Node):
    """Forwards Kimera VIO pose-graph loop edges to Hydra when requested.

    This node does not detect or score loop closures. The loop edges must come
    from the Kimera VIO LCD output topic, matching the ROS 1 benchmarked stack.
    """

    def __init__(self) -> None:
        super().__init__("kimera_pose_graph_bridge_node")

        self.declare_parameter("pose_graph_topic", "/mono_hydra_vio_ros/pose_graph")
        self.declare_parameter(
            "pose_graph_incremental_topic",
            "/mono_hydra_vio_ros/pose_graph_incremental",
        )
        self.declare_parameter("external_loop_closures_topic", "/hydra/external_loop_closures")
        self.declare_parameter("publish_external_loop_closures", False)
        self.declare_parameter("invert_external_loop_pose", False)

        self.pose_graph_topic = self.get_parameter("pose_graph_topic").value
        self.pose_graph_incremental_topic = self.get_parameter("pose_graph_incremental_topic").value
        self.external_loop_closures_topic = self.get_parameter("external_loop_closures_topic").value
        self.publish_external_loop_closures = bool(
            self.get_parameter("publish_external_loop_closures").value
        )
        self.invert_external_loop_pose = bool(self.get_parameter("invert_external_loop_pose").value)

        self.key_to_stamp_ns: dict[tuple[int, int], int] = {}
        self.forwarded_edges = 0
        self.skipped_edges = 0
        self.external_pub = self.create_publisher(PoseGraph, self.external_loop_closures_topic, 10)
        self.full_sub = self.create_subscription(
            PoseGraph, self.pose_graph_topic, self._on_pose_graph, 20
        )
        self.incremental_sub = self.create_subscription(
            PoseGraph, self.pose_graph_incremental_topic, self._on_incremental_pose_graph, 100
        )

        mode = "enabled" if self.publish_external_loop_closures else "disabled"
        self.get_logger().info(
            "Kimera pose graph bridge listening to '%s' and '%s'; external LC forwarding %s"
            % (self.pose_graph_topic, self.pose_graph_incremental_topic, mode)
        )

    def _on_pose_graph(self, msg: PoseGraph) -> None:
        self._cache_nodes(msg)

    def _on_incremental_pose_graph(self, msg: PoseGraph) -> None:
        self._cache_nodes(msg)
        if not self.publish_external_loop_closures:
            return

        outgoing = PoseGraph()
        outgoing.header = copy.deepcopy(msg.header)
        for edge in msg.edges:
            if edge.type != PoseGraphEdge.LOOPCLOSE:
                continue

            from_stamp = self.key_to_stamp_ns.get((edge.robot_from, edge.key_from))
            to_stamp = self.key_to_stamp_ns.get((edge.robot_to, edge.key_to))
            if from_stamp is None or to_stamp is None:
                self.get_logger().warning(
                    "Skipping Kimera loop edge without cached timestamps: %d -> %d"
                    % (edge.key_from, edge.key_to)
                )
                self.skipped_edges += 1
                continue

            external_edge = copy.deepcopy(edge)
            external_edge.key_from = from_stamp
            external_edge.key_to = to_stamp
            if self.invert_external_loop_pose:
                self.get_logger().warning(
                    "invert_external_loop_pose is reserved for explicit convention tests"
                )
            outgoing.edges.append(external_edge)

        if outgoing.edges:
            self.external_pub.publish(outgoing)
            self.forwarded_edges += len(outgoing.edges)
            self.get_logger().info(
                "Forwarded %d Kimera loop edge(s) to Hydra external LC topic; total=%d"
                % (len(outgoing.edges), self.forwarded_edges),
                throttle_duration_sec=5.0,
            )

    def _cache_nodes(self, msg: PoseGraph) -> None:
        for node in msg.nodes:
            stamp = _stamp_ns(node.header.stamp)
            if stamp == 0:
                continue
            self.key_to_stamp_ns[(node.robot_id, node.key)] = stamp
            self.key_to_stamp_ns[(node.robot_id, _gtsam_symbol_index(node.key))] = stamp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KimeraPoseGraphBridgeNode()
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
