#!/usr/bin/env python3
from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Iterable, List, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import CameraInfo, Image

try:
    from scipy.io import loadmat
except ImportError:
    loadmat = None  # type: ignore


@dataclass
class FrameData:
    stamp: Time
    depth: np.ndarray
    labels: np.ndarray
    frame_id: str
    pose_quality: float = 1.0


def _stamp_ns(stamp: Time) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class TemporalPoseWarpFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("temporal_pose_warp_filter")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("depth_topic", "/camera/depth_cam/image_raw")
        self.declare_parameter("label_topic", "/camera/seg_cam/labels_argmax")
        self.declare_parameter("output_depth_topic", "temporally_aligned/depth")
        self.declare_parameter("output_label_topic", "temporally_aligned/labels_argmax")
        self.declare_parameter("output_color_topic", "temporally_aligned/semantic")
        self.declare_parameter("output_mask_topic", "")
        self.declare_parameter("publish_color_semantic", True)
        self.declare_parameter("color_map_path", "")
        self.declare_parameter("history_size", 3)
        self.declare_parameter("max_history_age_s", 0.5)
        self.declare_parameter("history_use_fused", True)
        self.declare_parameter("min_depth", 0.05)
        self.declare_parameter("max_depth", 10.0)
        self.declare_parameter("input_depth_scale", 1.0)
        self.declare_parameter("depth_gate_abs", 0.2)
        self.declare_parameter("depth_gate_rel", 0.05)
        self.declare_parameter("depth_fusion_mode", "median")
        self.declare_parameter("depth_alpha", 0.5)
        self.declare_parameter("label_min_votes", 2)
        self.declare_parameter("dynamic_labels", "")
        self.declare_parameter("ignore_labels", "")
        self.declare_parameter("sync_queue", 5)
        self.declare_parameter("sync_slop", 0.02)
        self.declare_parameter("tf_timeout", 0.05)
        self.declare_parameter("tf_buffer_size", 10.0)
        self.declare_parameter("pose_quality_mode", "none")
        self.declare_parameter("pose_quality_odom_topic", "/external_odometry")
        self.declare_parameter("pose_quality_min", 0.2)
        self.declare_parameter("pose_quality_skip_threshold", 0.3)
        self.declare_parameter("pose_quality_alpha", 0.7)
        self.declare_parameter("pose_quality_lin_accel_ref", 2.0)
        self.declare_parameter("pose_quality_ang_accel_ref", 1.0)

        self.bridge = CvBridge()
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.label_topic = str(self.get_parameter("label_topic").value)
        self.output_depth_topic = str(self.get_parameter("output_depth_topic").value)
        self.output_label_topic = str(self.get_parameter("output_label_topic").value)
        self.output_color_topic = str(self.get_parameter("output_color_topic").value)
        self.output_mask_topic = str(self.get_parameter("output_mask_topic").value)
        self.publish_color_semantic = bool(self.get_parameter("publish_color_semantic").value)
        self.history_size = max(1, int(self.get_parameter("history_size").value))
        self.max_history_age = float(self.get_parameter("max_history_age_s").value)
        self.history_use_fused = bool(self.get_parameter("history_use_fused").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.input_depth_scale = float(self.get_parameter("input_depth_scale").value)
        self.depth_gate_abs = float(self.get_parameter("depth_gate_abs").value)
        self.depth_gate_rel = float(self.get_parameter("depth_gate_rel").value)
        self.depth_fusion_mode = str(self.get_parameter("depth_fusion_mode").value).lower()
        self.depth_alpha = float(self.get_parameter("depth_alpha").value)
        self.label_min_votes = int(self.get_parameter("label_min_votes").value)
        self.dynamic_labels = self._parse_label_list("dynamic_labels", [])
        self.ignore_labels = self._parse_label_list("ignore_labels", [])
        self.sync_queue = int(self.get_parameter("sync_queue").value)
        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.pose_quality_mode = str(self.get_parameter("pose_quality_mode").value).lower()
        self.pose_quality_odom_topic = str(self.get_parameter("pose_quality_odom_topic").value)
        self.pose_quality_min = float(self.get_parameter("pose_quality_min").value)
        self.pose_quality_skip_threshold = float(self.get_parameter("pose_quality_skip_threshold").value)
        self.pose_quality_alpha = float(self.get_parameter("pose_quality_alpha").value)
        self.pose_quality_lin_accel_ref = float(self.get_parameter("pose_quality_lin_accel_ref").value)
        self.pose_quality_ang_accel_ref = float(self.get_parameter("pose_quality_ang_accel_ref").value)

        self.camera_info: Optional[CameraInfo] = None
        self.intrinsics: Optional[Tuple[float, float, float, float, int, int]] = None
        self.pixel_grid: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.history: Deque[FrameData] = deque(maxlen=self.history_size)
        self.pose_quality = 1.0
        self.prev_odom_stamp_ns: Optional[int] = None
        self.prev_odom_linear: Optional[np.ndarray] = None
        self.prev_odom_angular: Optional[np.ndarray] = None

        color_map_path = str(self.get_parameter("color_map_path").value)
        self.colors = self._load_color_map(color_map_path) if self.publish_color_semantic else None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=float(self.get_parameter("tf_buffer_size").value)))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.depth_pub = self.create_publisher(Image, self.output_depth_topic, 1)
        self.label_pub = self.create_publisher(Image, self.output_label_topic, 1)
        self.color_pub = self.create_publisher(Image, self.output_color_topic, 1) if self.publish_color_semantic else None
        self.mask_pub = self.create_publisher(Image, self.output_mask_topic, 1) if self.output_mask_topic else None
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 1)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.label_sub = Subscriber(self, Image, self.label_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.label_sub], queue_size=self.sync_queue, slop=self.sync_slop
        )
        self.sync.registerCallback(self._sync_cb)
        if self.pose_quality_mode == "odom_twist_accel":
            self.create_subscription(Odometry, self.pose_quality_odom_topic, self._odom_cb, 20)

        self.get_logger().info(
            "temporal pose-warp filter ready depth=%s labels=%s output_depth=%s output_labels=%s history=%d"
            % (self.depth_topic, self.label_topic, self.output_depth_topic, self.output_label_topic, self.history_size)
        )

    def _parse_label_list(self, param_name: str, default: Iterable[int]) -> List[int]:
        value = self.get_parameter(param_name).value
        if isinstance(value, (list, tuple)):
            return [int(v) for v in value]
        if isinstance(value, str):
            tokens = [v for v in value.replace(",", " ").split() if v]
            return [int(v) for v in tokens]
        return list(default)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _compute_intrinsics(self, width: int, height: int) -> Optional[Tuple[float, float, float, float, int, int]]:
        if not self.camera_info or len(self.camera_info.k) != 9:
            return None
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if self.camera_info.width and self.camera_info.height:
            sx = width / float(self.camera_info.width)
            sy = height / float(self.camera_info.height)
            fx *= sx
            fy *= sy
            cx *= sx
            cy *= sy
        return fx, fy, cx, cy, width, height

    def _ensure_intrinsics(self, width: int, height: int) -> Optional[Tuple[float, float, float, float, int, int]]:
        if self.intrinsics and self.intrinsics[4] == width and self.intrinsics[5] == height:
            return self.intrinsics
        intrinsics = self._compute_intrinsics(width, height)
        if intrinsics is None:
            return None
        self.intrinsics = intrinsics
        self.pixel_grid = self._build_pixel_grid(width, height)
        return intrinsics

    def _build_pixel_grid(self, width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
        u = np.tile(np.arange(width, dtype=np.float32), height)
        v = np.repeat(np.arange(height, dtype=np.float32), width)
        return u, v

    def _lookup_relative_transform(
        self, cam_frame: str, prev_stamp: Time, cur_stamp: Time
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        try:
            transform = self.tf_buffer.lookup_transform_full(
                cam_frame,
                RclpyTime.from_msg(cur_stamp),
                cam_frame,
                RclpyTime.from_msg(prev_stamp),
                self.fixed_frame,
                Duration(seconds=self.tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TimeoutException,
        ) as exc:
            self.get_logger().warning(f"temporal pose-warp TF lookup failed: {exc}", throttle_duration_sec=2.0)
            return None
        return self._transform_to_rt(transform)

    def _transform_to_rt(self, transform) -> Tuple[np.ndarray, np.ndarray]:
        t = transform.transform.translation
        r = transform.transform.rotation
        qx, qy, qz, qw = float(r.x), float(r.y), float(r.z), float(r.w)
        norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if norm > 0.0:
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        rot = np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )
        return rot, np.array([t.x, t.y, t.z], dtype=np.float32)

    def _depth_from_msg(self, msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * self.input_depth_scale
        return depth.astype(np.float32)

    def _labels_from_msg(self, msg: Image) -> np.ndarray:
        labels = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        return labels if labels.dtype == np.uint8 else labels.astype(np.int32)

    def _depth_valid(self, depth: np.ndarray) -> np.ndarray:
        return np.isfinite(depth) & (depth > self.min_depth) & (depth < self.max_depth)

    def _label_mask(self, labels: np.ndarray) -> np.ndarray:
        mask = labels >= 0
        if self.dynamic_labels:
            mask &= ~np.isin(labels, self.dynamic_labels)
        if self.ignore_labels:
            mask &= ~np.isin(labels, self.ignore_labels)
        return mask

    def _warp_frame(
        self,
        depth: np.ndarray,
        labels: np.ndarray,
        intrinsics: Tuple[float, float, float, float, int, int],
        transform: Tuple[np.ndarray, np.ndarray],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        fx, fy, cx, cy, width, height = intrinsics
        if self.pixel_grid is None:
            self.pixel_grid = self._build_pixel_grid(width, height)
        grid_u, grid_v = self.pixel_grid
        flat_depth = depth.reshape(-1)
        valid = self._depth_valid(flat_depth)
        if not np.any(valid):
            return (
                np.full((height, width), np.inf, dtype=np.float32),
                np.full((height, width), -1, dtype=np.int32),
                np.zeros((height, width), dtype=bool),
            )
        valid_idx = np.nonzero(valid)[0]
        depth_valid = flat_depth[valid_idx]
        x = (grid_u[valid_idx] - cx) / fx * depth_valid
        y = (grid_v[valid_idx] - cy) / fy * depth_valid
        pts_cur = (transform[0] @ np.vstack((x, y, depth_valid))) + transform[1][:, None]
        z_cur = pts_cur[2]
        valid2 = z_cur > self.min_depth
        if not np.any(valid2):
            return (
                np.full((height, width), np.inf, dtype=np.float32),
                np.full((height, width), -1, dtype=np.int32),
                np.zeros((height, width), dtype=bool),
            )
        x_cur = pts_cur[0][valid2]
        y_cur = pts_cur[1][valid2]
        z_cur = z_cur[valid2]
        u = np.round((x_cur / z_cur) * fx + cx).astype(np.int32)
        v = np.round((y_cur / z_cur) * fy + cy).astype(np.int32)
        in_bounds = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        if not np.any(in_bounds):
            return (
                np.full((height, width), np.inf, dtype=np.float32),
                np.full((height, width), -1, dtype=np.int32),
                np.zeros((height, width), dtype=bool),
            )
        u, v, z_cur = u[in_bounds], v[in_bounds], z_cur[in_bounds]
        src_labels = labels.reshape(-1)[valid_idx][valid2][in_bounds]
        idx = v * width + u
        warp_depth_flat = np.full(width * height, np.inf, dtype=np.float32)
        np.minimum.at(warp_depth_flat, idx, z_cur)
        warp_labels_flat = np.full(width * height, -1, dtype=np.int32)
        keep = np.isclose(z_cur, warp_depth_flat[idx], rtol=1e-4, atol=1e-4)
        if np.any(keep):
            warp_labels_flat[idx[keep]] = src_labels[keep]
        warp_depth = warp_depth_flat.reshape((height, width))
        return warp_depth, warp_labels_flat.reshape((height, width)), warp_depth < np.inf

    def _depth_consistent(
        self, warp_depth: np.ndarray, cur_depth: np.ndarray, cur_valid: Optional[np.ndarray] = None
    ) -> np.ndarray:
        if cur_valid is None:
            cur_valid = self._depth_valid(cur_depth)
        if self.depth_gate_abs <= 0.0 and self.depth_gate_rel <= 0.0:
            return np.ones_like(cur_depth, dtype=bool)
        thresh = np.maximum(self.depth_gate_abs, self.depth_gate_rel * np.maximum(cur_depth, 1e-6))
        return (np.abs(warp_depth - cur_depth) <= thresh) | ~cur_valid

    def _odom_cb(self, msg: Odometry) -> None:
        stamp_ns = _stamp_ns(msg.header.stamp)
        if stamp_ns <= 0:
            return
        linear = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z], dtype=np.float32)
        angular = np.array(
            [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z], dtype=np.float32
        )
        inst_quality = 1.0
        if self.prev_odom_stamp_ns is not None and self.prev_odom_linear is not None and self.prev_odom_angular is not None:
            dt = (stamp_ns - self.prev_odom_stamp_ns) * 1e-9
            if dt > 1e-6:
                lin_accel = np.linalg.norm(linear - self.prev_odom_linear) / dt
                ang_accel = np.linalg.norm(angular - self.prev_odom_angular) / dt
                lin_term = (lin_accel / max(self.pose_quality_lin_accel_ref, 1e-3)) ** 2
                ang_term = (ang_accel / max(self.pose_quality_ang_accel_ref, 1e-3)) ** 2
                inst_quality = float(np.exp(-0.5 * (lin_term + ang_term)))
        inst_quality = float(np.clip(inst_quality, self.pose_quality_min, 1.0))
        alpha = float(np.clip(self.pose_quality_alpha, 0.0, 1.0))
        self.pose_quality = float(np.clip(alpha * self.pose_quality + (1.0 - alpha) * inst_quality, self.pose_quality_min, 1.0))
        self.prev_odom_stamp_ns = stamp_ns
        self.prev_odom_linear = linear
        self.prev_odom_angular = angular

    def _fuse_depth(self, cur_depth: np.ndarray, warped_depths: List[Tuple[np.ndarray, np.ndarray, float]]) -> np.ndarray:
        cur_valid = self._depth_valid(cur_depth)
        if not warped_depths:
            return cur_depth
        if self.depth_fusion_mode == "ema":
            fused = cur_depth.copy()
            support = cur_valid.copy()
            for warp_depth, warp_valid, weight in warped_depths:
                valid = warp_valid & self._depth_consistent(warp_depth, cur_depth, cur_valid)
                support |= valid
                if np.any(valid):
                    alpha = self.depth_alpha * float(np.clip(weight, self.pose_quality_min, 1.0))
                    fused[valid] = (1.0 - alpha) * fused[valid] + alpha * warp_depth[valid]
            fused = fused.astype(np.float32)
            fused[support] = np.clip(fused[support], self.min_depth, self.max_depth)
            fused[~support] = 0.0
            return fused

        support = cur_valid.copy()
        accum = np.zeros_like(cur_depth, dtype=np.float32)
        weight_sum = np.zeros_like(cur_depth, dtype=np.float32)
        accum[cur_valid] = cur_depth[cur_valid]
        weight_sum[cur_valid] = 1.0
        for warp_depth, warp_valid, weight in warped_depths:
            valid = warp_valid & self._depth_consistent(warp_depth, cur_depth, cur_valid)
            support |= valid
            if np.any(valid):
                w = float(np.clip(weight, self.pose_quality_min, 1.0))
                accum[valid] += w * warp_depth[valid]
                weight_sum[valid] += w
        fused = np.where(weight_sum > 1e-6, accum / np.maximum(weight_sum, 1e-6), cur_depth).astype(np.float32)
        fused[support] = np.clip(fused[support], self.min_depth, self.max_depth)
        fused[~support] = 0.0
        return fused

    def _fuse_labels(
        self,
        cur_labels: np.ndarray,
        cur_depth: np.ndarray,
        warped_labels: List[Tuple[np.ndarray, np.ndarray, np.ndarray, float]],
    ) -> np.ndarray:
        if not warped_labels:
            return cur_labels.astype(np.uint8)
        cur_depth_valid = self._depth_valid(cur_depth)
        labels_list = [cur_labels]
        valid_list = [self._label_mask(cur_labels) & cur_depth_valid]
        frame_weights = [1.0]
        for warp_depth, warp_labels, warp_valid, weight in warped_labels:
            valid = warp_valid & self._depth_consistent(warp_depth, cur_depth, cur_depth_valid) & self._label_mask(warp_labels)
            labels_list.append(warp_labels)
            valid_list.append(valid)
            frame_weights.append(float(np.clip(weight, self.pose_quality_min, 1.0)))
        max_label = max((int(np.max(labels[valid])) for labels, valid in zip(labels_list, valid_list) if np.any(valid)), default=-1)
        if max_label < 0:
            return cur_labels.astype(np.uint8)
        counts = np.zeros((max_label + 1, cur_labels.size), dtype=np.float32)
        for labels, valid, weight in zip(labels_list, valid_list, frame_weights):
            idx = np.nonzero(valid.reshape(-1))[0]
            if idx.size:
                np.add.at(counts, (labels.reshape(-1)[idx], idx), weight)
        max_counts = counts.max(axis=0)
        best_labels = counts.argmax(axis=0)
        flat_cur = cur_labels.reshape(-1).copy()
        cur_counts = np.zeros(cur_labels.size, dtype=counts.dtype)
        in_range = (flat_cur >= 0) & (flat_cur <= max_label)
        idx_range = np.nonzero(in_range)[0]
        if idx_range.size:
            cur_counts[idx_range] = counts[flat_cur[idx_range], idx_range]
        block = np.zeros(cur_labels.shape, dtype=bool)
        if self.dynamic_labels:
            block |= np.isin(cur_labels, self.dynamic_labels)
        if self.ignore_labels:
            block |= np.isin(cur_labels, self.ignore_labels)
        use_best = (max_counts >= self.label_min_votes) & (max_counts > cur_counts) & ~block.reshape(-1)
        flat_cur[use_best] = best_labels[use_best]
        return flat_cur.reshape(cur_labels.shape).astype(np.uint8)

    def _load_color_map(self, path: str) -> Optional[np.ndarray]:
        if not path:
            return None
        path_obj = Path(path)
        if path_obj.suffix.lower() == ".mat" and loadmat is not None:
            colors = loadmat(str(path_obj)).get("colors")
            if colors is None:
                return None
            colors = np.asarray(colors)
            if colors.max() <= 1.0:
                colors = colors * 255.0
            return np.clip(colors, 0, 255).astype(np.uint8)
        if path_obj.suffix.lower() == ".csv":
            colors = {}
            with open(path_obj, newline="") as handle:
                for row in csv.DictReader(handle):
                    idx = int(float(row.get("id", row.get("class", 0))))
                    colors[idx] = (int(float(row.get("red", 0))), int(float(row.get("green", 0))), int(float(row.get("blue", 0))))
            if not colors:
                return None
            palette = np.zeros((max(colors.keys()) + 1, 3), dtype=np.uint8)
            for idx, rgb in colors.items():
                palette[idx] = rgb
            return palette
        return None

    def _pascal_palette(self, num_classes: int) -> np.ndarray:
        palette = np.zeros((num_classes, 3), dtype=np.uint8)
        for j in range(num_classes):
            lab = j
            for i in range(8):
                palette[j, 0] |= (((lab >> 0) & 1) << (7 - i))
                palette[j, 1] |= (((lab >> 1) & 1) << (7 - i))
                palette[j, 2] |= (((lab >> 2) & 1) << (7 - i))
                lab >>= 3
        return palette

    def _colorize_labels(self, labels: np.ndarray) -> np.ndarray:
        if self.colors is None:
            palette = self._pascal_palette(max(int(labels.max()) + 1, 1) if labels.size else 1)
            rgb = palette[labels.astype(np.int32) % len(palette)]
        else:
            rgb = self.colors[np.clip(labels.astype(np.int32), 0, self.colors.shape[0] - 1)]
        return rgb[:, :, ::-1]

    def _prune_history(self, stamp: Time, shape: Tuple[int, int]) -> None:
        if self.history and self.history[0].depth.shape != shape:
            self.history.clear()
            return
        now_ns = _stamp_ns(stamp)
        while self.history and (now_ns - _stamp_ns(self.history[0].stamp)) * 1e-9 > self.max_history_age:
            self.history.popleft()

    def _sync_cb(self, depth_msg: Image, label_msg: Image) -> None:
        if self.camera_info is None:
            self.get_logger().warning(
                f"temporal pose-warp filter waiting for CameraInfo on {self.camera_info_topic}",
                throttle_duration_sec=2.0,
            )
            return
        depth = self._depth_from_msg(depth_msg)
        labels = self._labels_from_msg(label_msg)
        if depth.shape[:2] != labels.shape[:2]:
            self.get_logger().warning(f"temporal pose-warp size mismatch depth={depth.shape} labels={labels.shape}", throttle_duration_sec=2.0)
            return
        height, width = depth.shape[:2]
        intrinsics = self._ensure_intrinsics(width, height)
        if intrinsics is None:
            self.get_logger().warning("temporal pose-warp filter missing intrinsics", throttle_duration_sec=2.0)
            return
        cam_frame = self.camera_frame or depth_msg.header.frame_id or label_msg.header.frame_id
        if not cam_frame:
            self.get_logger().warning("temporal pose-warp filter missing camera frame id", throttle_duration_sec=2.0)
            return

        self._prune_history(depth_msg.header.stamp, depth.shape)
        warped_frames: List[Tuple[np.ndarray, np.ndarray, np.ndarray, float]] = []
        for frame in list(self.history):
            if self.pose_quality_mode != "none" and frame.pose_quality < self.pose_quality_skip_threshold:
                continue
            transform = self._lookup_relative_transform(cam_frame, frame.stamp, depth_msg.header.stamp)
            if transform is not None:
                warped_frames.append((*self._warp_frame(frame.depth, frame.labels, intrinsics, transform), frame.pose_quality))

        fused_depth = self._fuse_depth(depth, [(d, valid, w) for d, _, valid, w in warped_frames])
        fused_labels = self._fuse_labels(labels, depth, warped_frames)

        depth_out = self.bridge.cv2_to_imgmsg(fused_depth, encoding="32FC1")
        depth_out.header = depth_msg.header
        self.depth_pub.publish(depth_out)
        label_out = self.bridge.cv2_to_imgmsg(fused_labels, encoding="mono8")
        label_out.header = label_msg.header
        self.label_pub.publish(label_out)
        if self.publish_color_semantic and self.color_pub:
            color_out = self.bridge.cv2_to_imgmsg(self._colorize_labels(fused_labels), encoding="bgr8")
            color_out.header = label_msg.header
            self.color_pub.publish(color_out)
        if self.mask_pub:
            support = np.zeros(depth.shape, dtype=np.uint8)
            cur_depth_valid = self._depth_valid(depth)
            for warp_depth, _, warp_valid, _ in warped_frames:
                support |= (warp_valid & self._depth_consistent(warp_depth, depth, cur_depth_valid)).astype(np.uint8)
            mask_msg = self.bridge.cv2_to_imgmsg((support * 255).astype(np.uint8), encoding="mono8")
            mask_msg.header = depth_msg.header
            self.mask_pub.publish(mask_msg)

        self.history.append(
            FrameData(
                stamp=depth_msg.header.stamp,
                depth=fused_depth if self.history_use_fused else depth,
                labels=fused_labels if self.history_use_fused else labels.astype(np.uint8),
                frame_id=cam_frame,
                pose_quality=float(np.clip(self.pose_quality, self.pose_quality_min, 1.0)),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TemporalPoseWarpFilterNode()
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
