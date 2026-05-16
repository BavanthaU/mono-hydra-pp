#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class RgbToMonoNode(Node):
    def __init__(self) -> None:
        super().__init__("mono_hydra_rgb_to_mono")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("output_topic", "/rvio2_bridge/cam0/image_raw")
        self.declare_parameter("queue_size", 2)

        self.bridge = CvBridge()
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        queue_size = int(self.get_parameter("queue_size").value)
        self.publisher = self.create_publisher(Image, self.output_topic, queue_size)
        self.create_subscription(Image, self.input_topic, self._on_image, queue_size)
        self.get_logger().info(f"RGB-to-mono bridge forwarding {self.input_topic} -> {self.output_topic}")

    def _on_image(self, msg: Image) -> None:
        try:
            mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            out = self.bridge.cv2_to_imgmsg(mono, encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"RGB-to-mono conversion failed: {exc}", throttle_duration_sec=2.0)
            return
        out.header = msg.header
        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbToMonoNode()
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
