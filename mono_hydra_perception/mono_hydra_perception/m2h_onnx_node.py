#!/usr/bin/env python3
from __future__ import annotations

import copy
import time
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def _providers(requested: str) -> List[str]:
    import onnxruntime as ort

    available = ort.get_available_providers()
    mode = (requested or "auto").strip().lower()
    if mode == "cpu":
        return ["CPUExecutionProvider"]
    if mode == "cuda":
        return ["CUDAExecutionProvider", "CPUExecutionProvider"] if "CUDAExecutionProvider" in available else ["CPUExecutionProvider"]
    if "CUDAExecutionProvider" in available:
        return ["CUDAExecutionProvider", "CPUExecutionProvider"]
    return ["CPUExecutionProvider"]


def _color_table(num_classes: int) -> np.ndarray:
    count = max(int(num_classes), 1)
    table = np.zeros((count, 3), dtype=np.uint8)
    for idx in range(count):
        table[idx] = ((37 * idx + 17) % 256, (67 * idx + 29) % 256, (97 * idx + 53) % 256)
    table[0] = (0, 0, 0)
    return table


def _preprocess_image(
    image: np.ndarray,
    source_color_order: str,
    width: int,
    height: int,
    model_expects_rgb: bool,
    scale_to_01: bool,
    mean: Sequence[float],
    std: Sequence[float],
) -> np.ndarray:
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        source_color_order = "rgb"
    elif image.ndim == 3 and image.shape[2] == 4:
        image = image[:, :, :3]
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError(f"unsupported image shape {image.shape}")

    source = (source_color_order or "rgb").strip().lower()
    target = "rgb" if model_expects_rgb else "bgr"
    if source != target:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB if source == "bgr" else cv2.COLOR_RGB2BGR)

    image = cv2.resize(image[:, :, :3], (width, height), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    if scale_to_01:
        image *= 1.0 / 255.0
    image = (image - np.asarray(mean, dtype=np.float32).reshape(1, 1, 3)) / np.asarray(std, dtype=np.float32).reshape(1, 1, 3)
    return np.transpose(image, (2, 0, 1))[np.newaxis, :, :, :].astype(np.float32)


def _parse_depth(array: np.ndarray, scale: float, depth_min: float, depth_max: float) -> np.ndarray:
    depth = np.asarray(array)
    if depth.ndim == 4 and depth.shape[0] == 1 and depth.shape[1] == 1:
        depth = depth[0, 0]
    elif depth.ndim == 3 and depth.shape[0] == 1:
        depth = depth[0]
    elif depth.ndim != 2:
        raise ValueError(f"cannot parse depth output shape {depth.shape}")
    depth = depth.astype(np.float32) * float(scale)
    if depth_max > depth_min:
        depth = np.clip(depth, depth_min, depth_max)
    return depth.astype(np.float32)


def _parse_semantic(array: np.ndarray, num_classes: int) -> np.ndarray:
    semantic = np.asarray(array)
    if semantic.ndim == 4 and semantic.shape[0] == 1:
        if semantic.shape[1] == num_classes or semantic.shape[1] > 1:
            semantic = np.argmax(semantic[0], axis=0)
        elif semantic.shape[3] == num_classes or semantic.shape[3] > 1:
            semantic = np.argmax(semantic[0], axis=2)
        elif semantic.shape[1] == 1:
            semantic = semantic[0, 0]
    elif semantic.ndim == 3 and semantic.shape[0] == 1:
        semantic = semantic[0]
    if semantic.ndim != 2:
        raise ValueError(f"cannot parse semantic output shape {semantic.shape}")
    return np.maximum(np.rint(semantic).astype(np.int32), 0)


class M2HOnnxRos2Node(Node):
    def __init__(self) -> None:
        super().__init__("mono_hydra_perception_onnx")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        share = Path(get_package_share_directory("mono_hydra_perception"))
        self.declare_parameter("model_path", str(share / "onnx_models" / "scannet_depth_sem_320x416.onnx"))
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("image_depth_topic", "/camera/depth_cam/image_raw")
        self.declare_parameter("label_ids_topic", "/camera/seg_cam/labels_argmax")
        self.declare_parameter("image_semantic_topic", "/camera/seg_cam/image_raw")
        self.declare_parameter("input_queue_size", 256)
        self.declare_parameter("output_queue_size", 10)
        self.declare_parameter("warn_output_lag_s", 5.0)
        self.declare_parameter("max_output_lag_s", 0.0)
        self.declare_parameter("publish_synced_inputs", False)
        self.declare_parameter("synced_rgb_topic", "/mono_hydra_perception/synced/image_raw")
        self.declare_parameter("synced_camera_info_topic", "/mono_hydra_perception/synced/camera_info")
        self.declare_parameter("input_width", 416)
        self.declare_parameter("input_height", 320)
        self.declare_parameter("input_color_order", "rgb")
        self.declare_parameter("model_expects_rgb", True)
        self.declare_parameter("scale_input_to_01", True)
        self.declare_parameter("normalize_mean", [0.485, 0.456, 0.406])
        self.declare_parameter("normalize_std", [0.229, 0.224, 0.225])
        self.declare_parameter("output_original_size", True)
        self.declare_parameter("publish_color_semantic", False)
        self.declare_parameter("num_classes", 20)
        self.declare_parameter("semantic_encoding", "mono8")
        self.declare_parameter("depth_output_name", "")
        self.declare_parameter("semantic_output_name", "")
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("depth_min", 0.0)
        self.declare_parameter("depth_max", 10.0)
        self.declare_parameter("inference_provider", "auto")
        self.declare_parameter("skip_frequency", 3)
        self.declare_parameter("log_every_n_frames", 30)
        self.declare_parameter("intra_op_num_threads", 8)
        self.declare_parameter("inter_op_num_threads", 1)

        try:
            import onnxruntime as ort
        except Exception as exc:
            raise RuntimeError("ONNX Runtime is required for perception_backend:=onnx") from exc

        self.bridge = CvBridge()
        self.use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"ONNX model not found: {self.model_path}")
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.depth_topic = str(self.get_parameter("image_depth_topic").value)
        self.labels_topic = str(self.get_parameter("label_ids_topic").value)
        self.semantic_topic = str(self.get_parameter("image_semantic_topic").value)
        self.input_queue_size = max(1, int(self.get_parameter("input_queue_size").value))
        self.output_queue_size = max(1, int(self.get_parameter("output_queue_size").value))
        self.warn_output_lag_s = max(0.0, float(self.get_parameter("warn_output_lag_s").value))
        self.max_output_lag_s = max(0.0, float(self.get_parameter("max_output_lag_s").value))
        self.publish_synced_inputs = bool(self.get_parameter("publish_synced_inputs").value)
        self.synced_rgb_topic = str(self.get_parameter("synced_rgb_topic").value)
        self.synced_camera_info_topic = str(self.get_parameter("synced_camera_info_topic").value)
        self.input_width = int(self.get_parameter("input_width").value)
        self.input_height = int(self.get_parameter("input_height").value)
        self.input_color_order = str(self.get_parameter("input_color_order").value)
        self.model_expects_rgb = bool(self.get_parameter("model_expects_rgb").value)
        self.scale_input_to_01 = bool(self.get_parameter("scale_input_to_01").value)
        self.normalize_mean = list(self.get_parameter("normalize_mean").value)
        self.normalize_std = list(self.get_parameter("normalize_std").value)
        self.output_original_size = bool(self.get_parameter("output_original_size").value)
        self.publish_color_semantic = bool(self.get_parameter("publish_color_semantic").value)
        self.num_classes = int(self.get_parameter("num_classes").value)
        self.semantic_encoding = str(self.get_parameter("semantic_encoding").value)
        self.depth_output_name = str(self.get_parameter("depth_output_name").value)
        self.semantic_output_name = str(self.get_parameter("semantic_output_name").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_min = float(self.get_parameter("depth_min").value)
        self.depth_max = float(self.get_parameter("depth_max").value)
        self.skip_frequency = max(1, int(self.get_parameter("skip_frequency").value))
        self.log_every_n_frames = int(self.get_parameter("log_every_n_frames").value)
        self.intra_op_num_threads = max(0, int(self.get_parameter("intra_op_num_threads").value))
        self.inter_op_num_threads = max(0, int(self.get_parameter("inter_op_num_threads").value))
        self.color_table = _color_table(self.num_classes)

        options = ort.SessionOptions()
        options.log_severity_level = 2
        if self.intra_op_num_threads:
            options.intra_op_num_threads = self.intra_op_num_threads
        if self.inter_op_num_threads:
            options.inter_op_num_threads = self.inter_op_num_threads
        selected_providers = _providers(str(self.get_parameter("inference_provider").value))
        self.session = ort.InferenceSession(str(self.model_path), sess_options=options, providers=selected_providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

        self.depth_pub = self.create_publisher(Image, self.depth_topic, self.output_queue_size)
        self.labels_pub = self.create_publisher(Image, self.labels_topic, self.output_queue_size)
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
        self.latest_camera_info = None
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
        self.frame_count = 0
        self.processed_count = 0
        self.total_inference_ms = 0.0
        self.get_logger().info(
            f"ONNX M2H loaded model={self.model_path.name} providers={self.session.get_providers()} "
            f"size={self.input_width}x{self.input_height} skip={self.skip_frequency} "
            f"threads={self.intra_op_num_threads}/{self.inter_op_num_threads} "
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

    def _image_to_cv(self, msg: Image) -> Tuple[np.ndarray, str]:
        encoding = (msg.encoding or "").lower()
        if encoding == "rgb8":
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), "rgb"
        if encoding == "bgr8":
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"), "bgr"
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB), "rgb"
        return image[:, :, :3], self.input_color_order

    def _output_lag_s(self, msg: Image):
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
                "Dropping stale ONNX M2H frame %.1fs behind /clock."
                % lag_s,
                throttle_duration_sec=5.0,
            )
            return True
        if self.warn_output_lag_s > 0.0 and lag_s > self.warn_output_lag_s:
            self.get_logger().warn(
                "ONNX M2H output is %.1fs behind /clock; real-time replay is outrunning perception."
                % lag_s,
                throttle_duration_sec=5.0,
            )
        return False

    def _select_outputs(self, outputs: Dict[str, np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
        depth = outputs[self.depth_output_name] if self.depth_output_name else None
        semantic = outputs[self.semantic_output_name] if self.semantic_output_name else None
        if depth is None:
            for value in outputs.values():
                arr = np.asarray(value)
                if arr.ndim == 2 or (arr.ndim == 3 and arr.shape[0] == 1) or (arr.ndim == 4 and arr.shape[:2] == (1, 1)):
                    depth = arr
                    break
        if semantic is None:
            for value in outputs.values():
                arr = np.asarray(value)
                if depth is not None and arr is depth:
                    continue
                if arr.ndim in (2, 3, 4):
                    semantic = arr
                    break
        if depth is None or semantic is None:
            shapes = {name: tuple(np.asarray(value).shape) for name, value in outputs.items()}
            raise RuntimeError(f"could not identify depth and semantic outputs from {shapes}")
        return depth, semantic

    def _publish(self, header, depth: np.ndarray, semantic: np.ndarray, original_size: Tuple[int, int]) -> None:
        if self.output_original_size:
            original_h, original_w = original_size
            depth = cv2.resize(depth, (original_w, original_h), interpolation=cv2.INTER_LINEAR)
            semantic = cv2.resize(semantic, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

        depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding="32FC1")
        depth_msg.header = header
        self.depth_pub.publish(depth_msg)

        labels = np.clip(semantic, 0, 255).astype(np.uint8)
        label_msg = self.bridge.cv2_to_imgmsg(labels, encoding="mono8")
        label_msg.header = header
        self.labels_pub.publish(label_msg)

        if self.semantic_pub is not None:
            color_msg = self.bridge.cv2_to_imgmsg(self.color_table[labels % len(self.color_table)], encoding="rgb8")
            color_msg.header = header
            self.semantic_pub.publish(color_msg)

    def _on_image(self, msg: Image) -> None:
        self.frame_count += 1
        if self.frame_count % self.skip_frequency != 0:
            return
        if self._should_drop_for_lag(msg):
            return
        try:
            image, order = self._image_to_cv(msg)
            tensor = _preprocess_image(
                image,
                order,
                self.input_width,
                self.input_height,
                self.model_expects_rgb,
                self.scale_input_to_01,
                self.normalize_mean,
                self.normalize_std,
            )
            start = time.perf_counter()
            raw_outputs = self.session.run(self.output_names, {self.input_name: tensor})
            inference_ms = (time.perf_counter() - start) * 1000.0
            depth_raw, semantic_raw = self._select_outputs(dict(zip(self.output_names, raw_outputs)))
            depth = _parse_depth(depth_raw, self.depth_scale, self.depth_min, self.depth_max)
            semantic = _parse_semantic(semantic_raw, self.num_classes)
            self._publish(msg.header, depth, semantic, (msg.height, msg.width))
            self._publish_synced_inputs(msg)
            self.processed_count += 1
            self.total_inference_ms += inference_ms
            if self.log_every_n_frames > 0 and self.processed_count % self.log_every_n_frames == 0:
                avg = self.total_inference_ms / float(self.processed_count)
                self.get_logger().info(f"ONNX M2H average inference {avg:.2f} ms")
        except (CvBridgeError, ValueError, RuntimeError) as exc:
            self.get_logger().warn(f"ONNX M2H skipped frame: {exc}", throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2HOnnxRos2Node()
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
