#!/usr/bin/env python3
from __future__ import annotations

import csv
import copy
import time
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def _load_label_mapping(path: str) -> Dict[int, int]:
    if not path:
        return {}
    import yaml
    from yaml.loader import SafeLoader

    with open(path) as handle:
        data = yaml.load(handle, Loader=SafeLoader) or {}

    mapping: Dict[int, int] = {}
    for key, labels in data.items():
        if not key.endswith("/labels") or not isinstance(labels, list):
            continue
        parts = key.split("/")
        if len(parts) < 2:
            continue
        target_id = int(parts[1])
        for label_id in labels:
            mapping[int(label_id)] = target_id
    return mapping


def _load_colors(path: str) -> Optional[np.ndarray]:
    if not path:
        return None
    path_obj = Path(path)
    if path_obj.suffix.lower() == ".csv":
        colors = {}
        with open(path_obj, newline="") as handle:
            for row in csv.DictReader(handle):
                idx = int(float(row.get("id", row.get("class", 0))))
                colors[idx] = (
                    int(float(row.get("red", 0))),
                    int(float(row.get("green", 0))),
                    int(float(row.get("blue", 0))),
                )
        if not colors:
            return None
        palette = np.zeros((max(colors.keys()) + 1, 3), dtype=np.uint8)
        for idx, rgb in colors.items():
            palette[idx] = rgb
        return palette

    try:
        from scipy.io import loadmat
    except Exception:
        return None
    colors = loadmat(path).get("colors")
    if colors is None:
        return None
    colors = np.asarray(colors)
    if colors.max() <= 1.0:
        colors = colors * 255.0
    return np.clip(colors, 0, 255).astype(np.uint8)


def _map_labels(label_ids: np.ndarray, mapping: Dict[int, int]) -> np.ndarray:
    if not mapping:
        return label_ids.astype(np.uint8)
    vectorized = np.vectorize(lambda x: mapping.get(int(x), int(x)), otypes=[np.uint8])
    return vectorized(label_ids)


class M2HHMXLargeRos2Node(Node):
    """ROS 2 wrapper for the real M2H-HMX-Large model."""

    def __init__(self) -> None:
        super().__init__("mono_hydra_perception_torch")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter("dataset", "itc")
        self.declare_parameter("device", "auto")
        self.declare_parameter("half", True)
        self.declare_parameter("skip_frequency", 3)
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
        self.declare_parameter("publish_edges", False)
        self.declare_parameter("publish_normals", False)
        self.declare_parameter("config_path", "")
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("label_mapping_yaml", "")
        self.declare_parameter("color_map_path", "")
        self.declare_parameter("depth_scale", 1.427)
        self.declare_parameter("image_height", 0)
        self.declare_parameter("image_width", 0)

        self.bridge = CvBridge()
        self.use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.dataset = str(self.get_parameter("dataset").value).lower()
        self.skip_frequency = max(1, int(self.get_parameter("skip_frequency").value))
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.depth_topic = self.get_parameter("image_depth_topic").value
        self.semantic_topic = self.get_parameter("image_semantic_topic").value
        self.labels_topic = self.get_parameter("label_ids_topic").value
        self.input_queue_size = max(1, int(self.get_parameter("input_queue_size").value))
        self.output_queue_size = max(1, int(self.get_parameter("output_queue_size").value))
        self.warn_output_lag_s = max(0.0, float(self.get_parameter("warn_output_lag_s").value))
        self.max_output_lag_s = max(0.0, float(self.get_parameter("max_output_lag_s").value))
        self.publish_synced_inputs = bool(self.get_parameter("publish_synced_inputs").value)
        self.synced_rgb_topic = str(self.get_parameter("synced_rgb_topic").value)
        self.synced_camera_info_topic = str(self.get_parameter("synced_camera_info_topic").value)
        self.publish_label_ids = bool(self.get_parameter("publish_label_ids").value)
        self.publish_color_semantic = bool(self.get_parameter("publish_color_semantic").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.frame_count = 0
        self.latest_camera_info: Optional[CameraInfo] = None

        try:
            import torch
            from m2h_hmx_large.m2h_hmx_large_loader import MEAN, STD, M2HHMXV3Loader, _colorize_semseg
        except Exception as exc:
            raise RuntimeError(
                "Real M2H-HMX-Large mode requested, but model dependencies are missing. "
                "Install torch, transformers, timm, einops, and mamba-ssm in this ROS 2 environment."
            ) from exc

        self.torch = torch
        self.mean_values = MEAN
        self.std_values = STD
        self.colorize_semseg = _colorize_semseg

        share = Path(get_package_share_directory("mono_hydra_perception"))
        defaults = {
            "itc": (
                share / "config" / "m2h_hmx_v3_1_large_itc_mt_hr.yml",
                share / "weights" / "itc_large__miou_0.393_rmse_0.523_weights.pt",
            ),
            "scannet": (
                share / "config" / "m2h_hmx_v3_1_large_scannet_ft.yml",
                share / "weights" / "scannet_large__miou_0.761_rmse_0.221_weights.pt",
            ),
            "nyud": (
                share / "config" / "m2h_hmx_v3_nyudv2_large.yml",
                share / "weights" / "nyudv2_large__miou_0.656_rmse_0.380_weights.pt",
            ),
        }
        default_config, default_checkpoint = defaults.get(self.dataset, defaults["itc"])
        config_param = str(self.get_parameter("config_path").value)
        checkpoint_param = str(self.get_parameter("checkpoint_path").value)
        config_path = Path(config_param) if config_param else default_config
        checkpoint_path = Path(checkpoint_param) if checkpoint_param else default_checkpoint

        device_param = str(self.get_parameter("device").value)
        device = "cuda" if device_param == "auto" and torch.cuda.is_available() else device_param
        if device_param == "auto" and not torch.cuda.is_available():
            device = "cpu"
        half = bool(self.get_parameter("half").value)
        tasks = ("semseg", "depth") if self.dataset in ("itc", "scannet") else None
        self.loader = M2HHMXV3Loader(config_path, checkpoint_path, device=device, half=half, tasks=tasks)
        self.device = self.loader.device
        self.dtype = torch.float16 if half and self.device.type == "cuda" else torch.float32
        self.image_size = self.loader.image_size
        override_h = int(self.get_parameter("image_height").value)
        override_w = int(self.get_parameter("image_width").value)
        if override_h > 0 and override_w > 0:
            self.image_size = (override_h, override_w)

        self.mean = torch.as_tensor(self.mean_values, device=self.device, dtype=self.dtype).view(1, 3, 1, 1)
        self.std = torch.as_tensor(self.std_values, device=self.device, dtype=self.dtype).view(1, 3, 1, 1)
        self.min_depth = float(self.loader.min_depth)
        self.max_depth = float(self.loader.max_depth)
        self.num_classes = int(self.loader.num_classes)
        self.scaled_min_depth = self.min_depth * self.depth_scale
        self.scaled_max_depth = self.max_depth * self.depth_scale

        mapping_path = str(self.get_parameter("label_mapping_yaml").value)
        if not mapping_path and self.dataset in ("itc", "nyud"):
            candidate = share / "config" / "colors" / "nyud40_no_unknown_config.yaml"
            mapping_path = str(candidate) if candidate.exists() else ""
        color_path = str(self.get_parameter("color_map_path").value)
        if not color_path:
            candidate = share / "config" / "colors" / ("scannet20_config.csv" if self.dataset == "scannet" else "nyud20_config.csv")
            color_path = str(candidate) if candidate.exists() else ""
        self.label_map = _load_label_mapping(mapping_path) if mapping_path else {}
        self.colors = _load_colors(color_path) if color_path else None
        self.mapped_num_classes = max(self.label_map.values()) + 1 if self.label_map else self.num_classes

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
            f"real M2H-HMX-Large loaded dataset={self.dataset} device={self.device} "
            f"half={half} size={self.image_size[1]}x{self.image_size[0]} skip={self.skip_frequency} "
            f"queue={self.input_queue_size} output_queue={self.output_queue_size} "
            f"synced_inputs={self.publish_synced_inputs} input_qos=best_effort warn_lag={self.warn_output_lag_s:.1f}s "
            f"drop_lag={self.max_output_lag_s:.1f}s"
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

    def _prep_image(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        orig_h, orig_w, _ = cv_img.shape
        resized = cv2.resize(cv_img, (self.image_size[1], self.image_size[0]), interpolation=cv2.INTER_LINEAR)
        arr = resized.astype(np.float32) / 255.0
        tensor = self.torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0).to(device=self.device, dtype=self.dtype)
        return (tensor - self.mean) / self.std, orig_h, orig_w

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
                "Dropping stale M2H-HMX-Large frame %.1fs behind /clock."
                % lag_s,
                throttle_duration_sec=5.0,
            )
            return True
        if self.warn_output_lag_s > 0.0 and lag_s > self.warn_output_lag_s:
            self.get_logger().warn(
                "M2H-HMX-Large output is %.1fs behind /clock; real-time replay is outrunning perception."
                % lag_s,
                throttle_duration_sec=5.0,
            )
        return False

    def _colorize_semantics(self, labels: np.ndarray) -> np.ndarray:
        if self.colors is not None:
            clipped = np.clip(labels, 0, self.colors.shape[0] - 1)
            return self.colors[clipped][:, :, ::-1]
        return self.colorize_semseg(labels, self.mapped_num_classes)[:, :, ::-1]

    def _on_image(self, msg: Image) -> None:
        self.frame_count += 1
        if self.frame_count % self.skip_frequency != 0:
            return
        if self._should_drop_for_lag(msg):
            return
        start = time.time()
        try:
            image_tensor, orig_h, orig_w = self._prep_image(msg)
            outputs = self.loader.predict({"images": image_tensor, "compute_edges": False, "compute_normals": False})
        except Exception as exc:
            self.get_logger().error(f"M2H-HMX-Large inference failed: {exc}")
            return

        semseg_logits = outputs.get("semseg")
        if semseg_logits is not None:
            labels = self.torch.argmax(semseg_logits, dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
            mapped_labels = _map_labels(labels, self.label_map)
            if mapped_labels.shape[0] != orig_h or mapped_labels.shape[1] != orig_w:
                mapped_labels = cv2.resize(mapped_labels, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)
            if self.labels_pub is not None:
                labels_msg = self.bridge.cv2_to_imgmsg(mapped_labels, encoding="mono8")
                labels_msg.header = msg.header
                self.labels_pub.publish(labels_msg)
            if self.semantic_pub is not None:
                sem_msg = self.bridge.cv2_to_imgmsg(self._colorize_semantics(mapped_labels), encoding="bgr8")
                sem_msg.header = msg.header
                self.semantic_pub.publish(sem_msg)

        depth_pred = outputs.get("depth")
        if depth_pred is not None:
            depth_np = depth_pred.squeeze(0).squeeze(0).detach().cpu().numpy().astype(np.float32)
            depth_np = np.clip(depth_np * self.depth_scale, self.scaled_min_depth, self.scaled_max_depth)
            if depth_np.shape[0] != orig_h or depth_np.shape[1] != orig_w:
                depth_np = cv2.resize(depth_np, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_np.astype(np.float32), encoding="32FC1")
            depth_msg.header = msg.header
            self.depth_pub.publish(depth_msg)

        self._publish_synced_inputs(msg)
        self.get_logger().info(
            f"M2H-HMX-Large inference {time.time() - start:.3f}s",
            throttle_duration_sec=5.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2HHMXLargeRos2Node()
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
