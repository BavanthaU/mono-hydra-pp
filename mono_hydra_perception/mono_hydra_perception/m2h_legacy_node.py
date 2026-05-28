#!/usr/bin/env python3
from __future__ import annotations

import copy
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.io import loadmat
from sensor_msgs.msg import CameraInfo, Image
from yaml.loader import SafeLoader


def _load_mapping(path: Path) -> dict[int, int]:
    with path.open() as f:
        data = yaml.load(f, Loader=SafeLoader)

    mapping: dict[int, int] = {}
    for key, labels in data.items():
        if "labels" not in key:
            continue
        parts = key.split("/")
        if len(parts) < 2:
            continue
        mapped_id = int(parts[1])
        for label in labels:
            mapping[int(label)] = mapped_id
    return mapping


def _color_encode(labelmap: np.ndarray, colors: np.ndarray) -> np.ndarray:
    labelmap = labelmap.astype(np.int32)
    labelmap_rgb = np.zeros((*labelmap.shape, 3), dtype=np.uint8)
    for label in np.unique(labelmap):
        if label < 0 or label >= colors.shape[0]:
            continue
        labelmap_rgb[labelmap == label] = colors[label]
    return labelmap_rgb


class M2HLegacyRos2Node(Node):
    """ROS 2 wrapper for the original ITC M2H node used by the ROS 1 stack."""

    def __init__(self) -> None:
        super().__init__("mono_hydra_perception_m2h")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        share = Path(get_package_share_directory("mono_hydra_perception"))
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("image_depth_topic", "/camera/depth_cam/image_raw")
        self.declare_parameter("image_semantic_topic", "/camera/seg_cam/image_raw")
        self.declare_parameter("label_ids_topic", "/camera/seg_cam/labels_argmax")
        self.declare_parameter("input_queue_size", 256)
        self.declare_parameter("output_queue_size", 10)
        self.declare_parameter("warn_output_lag_s", 5.0)
        self.declare_parameter("max_output_lag_s", 0.0)
        self.declare_parameter("publish_synced_inputs", False)
        self.declare_parameter("synced_rgb_topic", "/mono_hydra_perception/synced/image_raw")
        self.declare_parameter("synced_camera_info_topic", "/mono_hydra_perception/synced/camera_info")
        self.declare_parameter("publish_label_ids", True)
        self.declare_parameter("publish_color_semantic", False)
        self.declare_parameter("model_path", str(share / "weights" / "m2h_indoor.pt"))
        self.declare_parameter("feed_width", 256)
        self.declare_parameter("feed_height", 256)
        self.declare_parameter("skip_frequency", 5)
        self.declare_parameter("arch_name", "vit_small")
        self.declare_parameter("model_variant", "default")
        self.declare_parameter("num_classes", 41)
        self.declare_parameter("min_depth", 0.001)
        self.declare_parameter("max_depth", 10.0)
        self.declare_parameter("depth_output_scale", 0.967)
        self.declare_parameter("color20_mat_filepath", str(share / "config" / "colors" / "color_config20.mat"))
        self.declare_parameter("objects40_csv_mapping", str(share / "config" / "colors" / "nyud40_config.yaml"))
        self.declare_parameter("log_every_n_frames", 30)

        self.bridge = CvBridge()
        self.use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.depth_topic = str(self.get_parameter("image_depth_topic").value)
        self.semantic_topic = str(self.get_parameter("image_semantic_topic").value)
        self.labels_topic = str(self.get_parameter("label_ids_topic").value)
        self.input_queue_size = max(1, int(self.get_parameter("input_queue_size").value))
        self.output_queue_size = max(1, int(self.get_parameter("output_queue_size").value))
        self.warn_output_lag_s = max(0.0, float(self.get_parameter("warn_output_lag_s").value))
        self.max_output_lag_s = max(0.0, float(self.get_parameter("max_output_lag_s").value))
        self.publish_synced_inputs = bool(self.get_parameter("publish_synced_inputs").value)
        self.synced_rgb_topic = str(self.get_parameter("synced_rgb_topic").value)
        self.synced_camera_info_topic = str(self.get_parameter("synced_camera_info_topic").value)
        self.publish_label_ids = bool(self.get_parameter("publish_label_ids").value)
        self.publish_color_semantic = bool(self.get_parameter("publish_color_semantic").value)
        self.feed_width = max(1, int(self.get_parameter("feed_width").value))
        self.feed_height = max(1, int(self.get_parameter("feed_height").value))
        self.skip_frequency = max(1, int(self.get_parameter("skip_frequency").value))
        self.depth_output_scale = float(self.get_parameter("depth_output_scale").value)
        self.log_every_n_frames = int(self.get_parameter("log_every_n_frames").value)
        self.latest_camera_info: Optional[CameraInfo] = None
        self.frame_count = 0
        self.processed_count = 0
        self.total_inference_ms = 0.0

        try:
            import torch
            import torch.nn.functional as torch_f
            from m2h_core import ModelConfig, build_model
        except Exception as exc:
            raise RuntimeError("Original M2H backend requires torch, timm, mamba-ssm, einops, and scipy.") from exc

        self.torch = torch
        self.torch_f = torch_f
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        torch.backends.cudnn.benchmark = True

        cfg = ModelConfig(
            arch=str(self.get_parameter("arch_name").value),
            flavor=str(self.get_parameter("model_variant").value),
            min_depth=float(self.get_parameter("min_depth").value),
            max_depth=float(self.get_parameter("max_depth").value),
            num_classes=int(self.get_parameter("num_classes").value),
        )
        self.model = build_model(cfg).to(self.device)
        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"Original M2H checkpoint not found: {model_path}")
        checkpoint = torch.load(str(model_path), map_location=self.device)
        self.model.load_state_dict(checkpoint)
        self.model.eval()

        self.mean = torch.tensor([0.4803, 0.4800, 0.4723], dtype=torch.float32, device=self.device).view(1, 3, 1, 1)
        self.std = torch.tensor([0.2594, 0.2573, 0.2641], dtype=torch.float32, device=self.device).view(1, 3, 1, 1)

        color_path = Path(str(self.get_parameter("color20_mat_filepath").value)).expanduser()
        mapping_path = Path(str(self.get_parameter("objects40_csv_mapping").value)).expanduser()
        self.colors = loadmat(str(color_path))["colors"].astype(np.uint8)
        self.mapping = _load_mapping(mapping_path)
        self.mapping_lut = np.zeros(256, dtype=np.uint8)
        for label_id, mapped_id in self.mapping.items():
            if 0 <= label_id < self.mapping_lut.shape[0]:
                self.mapping_lut[label_id] = np.uint8(np.clip(mapped_id, 0, 255))

        self.depth_pub = self.create_publisher(Image, self.depth_topic, self.output_queue_size)
        self.labels_pub = (
            self.create_publisher(Image, self.labels_topic, self.output_queue_size)
            if self.publish_label_ids
            else None
        )
        self.semantic_pub = (
            self.create_publisher(Image, self.semantic_topic, self.output_queue_size)
            if self.publish_color_semantic
            else None
        )
        self.synced_rgb_pub = (
            self.create_publisher(Image, self.synced_rgb_topic, self.output_queue_size)
            if self.publish_synced_inputs
            else None
        )
        self.synced_camera_info_pub = (
            self.create_publisher(CameraInfo, self.synced_camera_info_topic, self.output_queue_size)
            if self.publish_synced_inputs
            else None
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.input_queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        camera_info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        if self.publish_synced_inputs:
            self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, camera_info_qos)
        self.create_subscription(Image, self.image_topic, self._on_image, image_qos)
        self.get_logger().info(
            f"Original M2H loaded model={model_path.name} device={self.device} "
            f"size={self.feed_width}x{self.feed_height} skip={self.skip_frequency} "
            f"queue={self.input_queue_size} output_queue={self.output_queue_size} "
            f"synced_inputs={self.publish_synced_inputs} input_qos=best_effort "
            f"warn_lag={self.warn_output_lag_s:.2f}s drop_lag={self.max_output_lag_s:.2f}s"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _publish_synced_inputs(self, msg: Image) -> None:
        if not self.publish_synced_inputs or self.synced_rgb_pub is None:
            return
        self.synced_rgb_pub.publish(msg)
        if self.synced_camera_info_pub is None or self.latest_camera_info is None:
            return
        info_msg = copy.deepcopy(self.latest_camera_info)
        info_msg.header = msg.header
        self.synced_camera_info_pub.publish(info_msg)

    def _output_lag_s(self, msg: Image) -> Optional[float]:
        if not self.use_sim_time:
            return None
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if now_ns <= 0 or stamp_ns <= 0:
            return None
        lag_s = (now_ns - stamp_ns) * 1.0e-9
        return lag_s if lag_s >= 0.0 else None

    def _should_drop_for_lag(self, msg: Image) -> bool:
        lag_s = self._output_lag_s(msg)
        if lag_s is None:
            return False
        if self.max_output_lag_s > 0.0 and lag_s > self.max_output_lag_s:
            self.get_logger().warn(
                "Dropping stale original M2H frame %.1fs behind /clock." % lag_s,
                throttle_duration_sec=5.0,
            )
            return True
        if self.warn_output_lag_s > 0.0 and lag_s > self.warn_output_lag_s:
            self.get_logger().warn(
                "Original M2H output is %.1fs behind /clock; real-time replay is outrunning perception." % lag_s,
                throttle_duration_sec=5.0,
            )
        return False

    def _image_to_rgb(self, msg: Image) -> np.ndarray:
        encoding = (msg.encoding or "").lower()
        if encoding == "rgb8":
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        if encoding == "bgr8":
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        if image.shape[2] == 4:
            image = image[:, :, :3]
        return image

    def _on_image(self, msg: Image) -> None:
        self.frame_count += 1
        if (self.frame_count - 1) % self.skip_frequency != 0:
            return
        if self._should_drop_for_lag(msg):
            return
        start = time.perf_counter()
        try:
            rgb = self._image_to_rgb(msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f"Original M2H image conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        original_h, original_w = rgb.shape[:2]
        with self.torch.no_grad():
            raw = np.transpose(rgb, (2, 0, 1))
            tensor = self.torch.from_numpy(raw).float().to(self.device)
            tensor = (tensor / 255.0).unsqueeze(0)
            tensor = self.torch_f.interpolate(
                tensor,
                (self.feed_width, self.feed_height),
                mode="bilinear",
                align_corners=False,
            )
            tensor = (tensor - self.mean) / self.std
            depth_out, semantic_out, _, _ = self.model(tensor)
            depth = self.torch_f.interpolate(
                depth_out,
                size=(original_h, original_w),
                mode="bilinear",
                align_corners=False,
            ).squeeze(0)
            depth_np = self.torch.clamp(depth, min=0).squeeze(0).cpu().numpy().astype(np.float32)
            depth_np *= self.depth_output_scale

            semantic_logits = self.torch_f.interpolate(
                semantic_out,
                size=(original_h, original_w),
                mode="bilinear",
                align_corners=False,
            ).squeeze(0)
            semantic_ids = self.torch.argmax(semantic_logits, dim=0).cpu().numpy().astype(np.uint8)

        mapped = self.mapping_lut[semantic_ids]

        depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding="32FC1")
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)

        if self.labels_pub is not None:
            label_msg = self.bridge.cv2_to_imgmsg(mapped, encoding="mono8")
            label_msg.header = msg.header
            self.labels_pub.publish(label_msg)

        if self.semantic_pub is not None:
            color_msg = self.bridge.cv2_to_imgmsg(_color_encode(mapped, self.colors), encoding="rgb8")
            color_msg.header = msg.header
            self.semantic_pub.publish(color_msg)

        self._publish_synced_inputs(msg)
        inference_ms = (time.perf_counter() - start) * 1000.0
        self.processed_count += 1
        self.total_inference_ms += inference_ms
        if self.log_every_n_frames > 0 and self.processed_count % self.log_every_n_frames == 0:
            avg = self.total_inference_ms / float(self.processed_count)
            self.get_logger().info(f"Original M2H average inference {avg:.2f} ms")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2HLegacyRos2Node()
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
