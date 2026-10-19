"""ROS 2 deployment node for the GeoST-MAViSTA model."""

from __future__ import annotations

import os
import site
from pathlib import Path

import cv2
import message_filters
import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from .geometry import center_crop_resize_geometry, transform_intrinsics


MODEL_FILENAME = "mavista_geost_step025250_temporal_480x640.onnx"
MODEL_HEIGHT = 480
MODEL_WIDTH = 640
TEMPORAL_STATE_SHAPE = (1, 96, 30, 40)
OUTPUT_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def _cuda_library_path() -> str:
    roots: list[str] = []
    for getter in (site.getusersitepackages, site.getsitepackages):
        try:
            values = getter()
        except Exception:
            continue
        roots.extend(values if isinstance(values, list) else [values])
    paths = [str(path) for root in roots for path in sorted((Path(root) / "nvidia").glob("*/lib")) if path.is_dir()]
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    if existing:
        paths.append(existing)
    return ":".join(dict.fromkeys(paths))


def _providers(requested: str, available: list[str]) -> list[str]:
    requested = requested.lower()
    if requested == "cpu":
        return ["CPUExecutionProvider"]
    if requested == "cuda":
        if "CUDAExecutionProvider" not in available:
            raise RuntimeError(f"CUDAExecutionProvider unavailable; installed: {available}")
        return ["CUDAExecutionProvider", "CPUExecutionProvider"]
    if requested == "auto":
        return (["CUDAExecutionProvider", "CPUExecutionProvider"]
                if "CUDAExecutionProvider" in available else ["CPUExecutionProvider"])
    raise ValueError("provider must be one of auto, cuda, or cpu")


class MAViSTAHydraNode(Node):
    """Run stateful GeoST-MAViSTA and publish Hydra-compatible predictions."""

    def __init__(self) -> None:
        super().__init__("mavista")
        share = Path(get_package_share_directory("mavista"))
        self.declare_parameter("model_path", str(share / "models" / MODEL_FILENAME))
        self.declare_parameter("provider", "auto")
        self.declare_parameter("input_image_topic", "/camera/color/image_rect")
        self.declare_parameter("input_camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("input_qos_reliability", "best_effort")
        self.declare_parameter("output_depth_topic", "/hydra/camera/depth/image_rect")
        self.declare_parameter("output_semantic_topic", "/hydra/camera/semantic/image_raw")
        self.declare_parameter("output_semantic_visualization_topic", "/mavista/semantic/visualization")
        self.declare_parameter("output_semantic_confidence_topic", "/mavista/semantic/confidence")
        self.declare_parameter("semantic_confidence_threshold", 0.0)
        self.declare_parameter("min_depth_m", 0.1)
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("publish_diagnostics", False)
        self.declare_parameter("publish_semantic_visualization", False)
        self.declare_parameter("sync_slop_s", 0.03)

        requested = str(self.get_parameter("provider").value)
        available = ort.get_available_providers()
        providers = _providers(requested, available)
        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"GeoST-MAViSTA ONNX model not found: {model_path}")
        self.session = ort.InferenceSession(str(model_path), providers=providers)
        if requested.lower() == "cuda" and self.session.get_providers()[0] != "CUDAExecutionProvider":
            raise RuntimeError(f"CUDA requested but selected providers are {self.session.get_providers()}")
        input_names = {item.name for item in self.session.get_inputs()}
        expected_inputs = {"image", "previous_hidden_state", "reset_mask", "intrinsics"}
        if input_names != expected_inputs:
            raise RuntimeError(
                f"unexpected temporal GeoST-MAViSTA inputs: {input_names}; "
                f"expected {expected_inputs}"
            )
        output_names = {item.name for item in self.session.get_outputs()}
        expected_outputs = {
            "metric_depth",
            "semantic_logits",
            "semantic_confidence",
            "next_hidden_state",
        }
        if output_names != expected_outputs:
            raise RuntimeError(
                f"unexpected temporal GeoST-MAViSTA outputs: {output_names}; "
                f"expected {expected_outputs}"
            )
        state_input = next(
            item for item in self.session.get_inputs() if item.name == "previous_hidden_state"
        )
        if tuple(state_input.shape) != TEMPORAL_STATE_SHAPE:
            raise RuntimeError(
                f"unexpected temporal state shape {state_input.shape}; "
                f"expected {TEMPORAL_STATE_SHAPE}"
            )
        self.hidden_state = np.zeros(TEMPORAL_STATE_SHAPE, dtype=np.float32)
        self.reset_requested = True
        self.new_environment_service = self.create_service(
            Trigger, "~/new_environment", self._new_environment
        )
        self.get_logger().info(f"loaded {model_path.name}; providers={self.session.get_providers()}")
        self.get_logger().info(
            "temporal ConvGRU state initialized; it persists across frame gaps and "
            "resets only through ~/new_environment"
        )

        reliability = str(self.get_parameter("input_qos_reliability").value).lower()
        if reliability == "best_effort":
            input_qos = qos_profile_sensor_data
        elif reliability == "reliable":
            input_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        else:
            raise ValueError("input_qos_reliability must be best_effort or reliable")

        self.bridge = CvBridge()
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.threshold = float(self.get_parameter("semantic_confidence_threshold").value)
        self.publish_diagnostics = bool(self.get_parameter("publish_diagnostics").value)
        self.publish_visualization = bool(self.get_parameter("publish_semantic_visualization").value)

        image_topic = str(self.get_parameter("input_image_topic").value)
        info_topic = str(self.get_parameter("input_camera_info_topic").value)
        self.image_sub = message_filters.Subscriber(self, Image, image_topic, qos_profile=input_qos)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, info_topic, qos_profile=input_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub], 8, float(self.get_parameter("sync_slop_s").value)
        )
        self.sync.registerCallback(self._callback)

        self.depth_pub = self.create_publisher(Image, str(self.get_parameter("output_depth_topic").value), OUTPUT_QOS)
        self.semantic_pub = self.create_publisher(Image, str(self.get_parameter("output_semantic_topic").value), OUTPUT_QOS)
        self.confidence_pub = None
        if self.publish_diagnostics:
            self.confidence_pub = self.create_publisher(
                Image, str(self.get_parameter("output_semantic_confidence_topic").value), OUTPUT_QOS
            )
        self.visualization_pub = None
        if self.publish_visualization:
            self.visualization_pub = self.create_publisher(
                Image, str(self.get_parameter("output_semantic_visualization_topic").value), OUTPUT_QOS
            )

    def _new_environment(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.hidden_state.fill(0.0)
        self.reset_requested = True
        response.success = True
        response.message = "MAViSTA temporal state cleared for a new environment"
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _intrinsics(info: CameraInfo) -> tuple[float, float, float, float]:
        if info.p[0] > 0.0 and info.p[5] > 0.0:
            return info.p[0], info.p[5], info.p[2], info.p[6]
        if info.k[0] > 0.0 and info.k[4] > 0.0:
            return info.k[0], info.k[4], info.k[2], info.k[5]
        raise ValueError("CameraInfo has no valid P or K intrinsics")

    def _callback(self, image_msg: Image, info_msg: CameraInfo) -> None:
        try:
            rgb = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            height, width = rgb.shape[:2]
            geometry = center_crop_resize_geometry(width, height)
            crop = rgb[geometry.y0:geometry.y0 + geometry.crop_height, geometry.x0:geometry.x0 + geometry.crop_width]
            resized = cv2.resize(crop, (MODEL_WIDTH, MODEL_HEIGHT), interpolation=cv2.INTER_LINEAR)
            fx, fy, cx, cy = transform_intrinsics(*self._intrinsics(info_msg), geometry)
            model_image = np.ascontiguousarray(resized.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
            model_intrinsics = np.asarray([[fx, fy, cx, cy]], dtype=np.float32)
            reset_mask = np.asarray([self.reset_requested], dtype=np.bool_)
            depth, logits, confidence, next_hidden_state = self.session.run(
                [
                    "metric_depth",
                    "semantic_logits",
                    "semantic_confidence",
                    "next_hidden_state",
                ],
                {
                    "image": model_image,
                    "previous_hidden_state": self.hidden_state,
                    "reset_mask": reset_mask,
                    "intrinsics": model_intrinsics,
                },
            )
            if next_hidden_state.shape != TEMPORAL_STATE_SHAPE or not np.isfinite(next_hidden_state).all():
                raise RuntimeError("model returned an invalid temporal hidden state")
            self.hidden_state = next_hidden_state.astype(np.float32, copy=False)
            self.reset_requested = False
            depth = np.clip(depth[0, 0].astype(np.float32, copy=False), self.min_depth, self.max_depth)
            confidence = confidence[0].astype(np.float32, copy=False)
            labels = logits[0, 1:41].argmax(axis=0).astype(np.int32) + 1
            if self.threshold > 0.0:
                labels[confidence < self.threshold] = 0

            # The ONNX graph operates on a centered 640x480 crop, while Hydra
            # and Kimera consume the original RGB and CameraInfo directly. Put
            # predictions back into that source pixel geometry so RGB, depth,
            # labels, and calibration always have identical dimensions. Pixels
            # outside the model crop are explicitly invalid (depth/label 0).
            if (
                width != MODEL_WIDTH
                or height != MODEL_HEIGHT
                or geometry.crop_width != MODEL_WIDTH
                or geometry.crop_height != MODEL_HEIGHT
            ):
                crop_size = (geometry.crop_width, geometry.crop_height)
                crop_depth = cv2.resize(depth, crop_size, interpolation=cv2.INTER_LINEAR)
                crop_confidence = cv2.resize(
                    confidence, crop_size, interpolation=cv2.INTER_LINEAR
                )
                crop_labels = cv2.resize(
                    labels.astype(np.uint8), crop_size, interpolation=cv2.INTER_NEAREST
                ).astype(np.int32, copy=False)
                y_slice = slice(geometry.y0, geometry.y0 + geometry.crop_height)
                x_slice = slice(geometry.x0, geometry.x0 + geometry.crop_width)
                depth = np.zeros((height, width), dtype=np.float32)
                confidence = np.zeros((height, width), dtype=np.float32)
                labels = np.zeros((height, width), dtype=np.int32)
                depth[y_slice, x_slice] = crop_depth
                confidence[y_slice, x_slice] = crop_confidence
                labels[y_slice, x_slice] = crop_labels

            header = image_msg.header
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            semantic_msg = self.bridge.cv2_to_imgmsg(labels, encoding="32SC1")
            for message in (depth_msg, semantic_msg):
                message.header = header
            self.depth_pub.publish(depth_msg)
            self.semantic_pub.publish(semantic_msg)
            if self.confidence_pub is not None:
                confidence_msg = self.bridge.cv2_to_imgmsg(confidence, encoding="32FC1")
                confidence_msg.header = header
                self.confidence_pub.publish(confidence_msg)
            if self.visualization_pub is not None:
                visualization = cv2.applyColorMap(
                    np.asarray(labels * 6, dtype=np.uint8), cv2.COLORMAP_TURBO
                )
                visualization = cv2.cvtColor(visualization, cv2.COLOR_BGR2RGB)
                visualization_msg = self.bridge.cv2_to_imgmsg(visualization, encoding="rgb8")
                visualization_msg.header = header
                self.visualization_pub.publish(visualization_msg)
        except Exception as error:
            self.get_logger().error(f"GeoST-MAViSTA inference failed: {error}")


def main() -> None:
    rclpy.init()
    node = MAViSTAHydraNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
