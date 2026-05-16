#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class VioFeatureInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("mono_hydra_vio_feature_interface")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter("enable_sparse_depth", False)
        self.declare_parameter("dense_depth_topic", "/camera/depth_cam/image_raw")
        self.declare_parameter("semantic_label_topic", "/camera/seg_cam/labels_argmax")
        self.declare_parameter("semantic_mask_topic", "/mono_hydra_vio/semantic_feature_mask")
        self.declare_parameter("semantic_mask_inflate_px", 8)
        self.declare_parameter("semantic_reject_labels", [0])
        self.declare_parameter("sparse_depth_topic", "/mono_hydra_vio/sparse_depth")
        self.declare_parameter("sparse_depth_stride", 16)
        self.declare_parameter("sparse_depth_min_m", 0.05)
        self.declare_parameter("sparse_depth_max_m", 8.0)
        self.declare_parameter("superpoint_keypoint_mask_topic", "")

        self.bridge = CvBridge()
        self.enable_sparse_depth = bool(self.get_parameter("enable_sparse_depth").value)
        self.inflate_px = max(0, int(self.get_parameter("semantic_mask_inflate_px").value))
        self.reject_labels = {int(v) for v in self.get_parameter("semantic_reject_labels").value}
        self.sparse_stride = max(1, int(self.get_parameter("sparse_depth_stride").value))
        self.depth_min = float(self.get_parameter("sparse_depth_min_m").value)
        self.depth_max = float(self.get_parameter("sparse_depth_max_m").value)
        self.latest_feature_mask: Optional[np.ndarray] = None
        self.latest_keypoint_mask: Optional[np.ndarray] = None

        self.mask_pub = self.create_publisher(Image, str(self.get_parameter("semantic_mask_topic").value), 2)
        self.sparse_depth_pub = self.create_publisher(Image, str(self.get_parameter("sparse_depth_topic").value), 2)
        self.create_subscription(Image, str(self.get_parameter("semantic_label_topic").value), self._on_labels, 2)
        self.create_subscription(Image, str(self.get_parameter("dense_depth_topic").value), self._on_depth, 2)
        keypoint_topic = str(self.get_parameter("superpoint_keypoint_mask_topic").value)
        if keypoint_topic:
            self.create_subscription(Image, keypoint_topic, self._on_keypoints, 2)
        self.get_logger().info(
            f"VIO feature interface ready sparse_depth={self.enable_sparse_depth} "
            f"reject_labels={sorted(self.reject_labels)} inflate_px={self.inflate_px}"
        )

    def _on_keypoints(self, msg: Image) -> None:
        keypoints = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.latest_keypoint_mask = keypoints > 0

    def _on_labels(self, msg: Image) -> None:
        labels = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        labels = np.asarray(labels)
        keep = np.ones(labels.shape[:2], dtype=np.uint8) * 255
        for label in self.reject_labels:
            keep[labels == label] = 0
        if self.inflate_px > 0:
            rejected = (keep == 0).astype(np.uint8)
            kernel = np.ones((2 * self.inflate_px + 1, 2 * self.inflate_px + 1), dtype=np.uint8)
            keep[cv2.dilate(rejected, kernel, iterations=1) > 0] = 0
        self.latest_feature_mask = keep > 0
        mask_msg = self.bridge.cv2_to_imgmsg(keep, encoding="mono8")
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    def _on_depth(self, msg: Image) -> None:
        if not self.enable_sparse_depth:
            return
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1").astype(np.float32)
        valid = np.isfinite(depth) & (depth >= self.depth_min) & (depth <= self.depth_max)
        if self.latest_feature_mask is not None:
            mask = self.latest_feature_mask
            if mask.shape != valid.shape:
                mask = cv2.resize(mask.astype(np.uint8), (valid.shape[1], valid.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
            valid &= mask
        if self.latest_keypoint_mask is not None:
            keypoints = self.latest_keypoint_mask
            if keypoints.shape != valid.shape:
                keypoints = cv2.resize(keypoints.astype(np.uint8), (valid.shape[1], valid.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
            valid &= keypoints
        else:
            grid = np.zeros_like(valid)
            grid[:: self.sparse_stride, :: self.sparse_stride] = True
            valid &= grid

        sparse = np.zeros_like(depth, dtype=np.float32)
        sparse[valid] = depth[valid]
        sparse_msg = self.bridge.cv2_to_imgmsg(sparse, encoding="32FC1")
        sparse_msg.header = msg.header
        self.sparse_depth_pub.publish(sparse_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VioFeatureInterfaceNode()
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
