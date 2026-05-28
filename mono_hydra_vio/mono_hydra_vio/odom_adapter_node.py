#!/usr/bin/env python3
from __future__ import annotations

import copy
import math
from pathlib import Path as FilesystemPath
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _stamp_is_zero(stamp) -> bool:
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0


def _yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _load_yaml(path: str) -> dict:
    text = FilesystemPath(path).read_text()
    if text.startswith("%YAML:"):
        text = "\n".join(text.splitlines()[1:])
    return yaml.safe_load(text) or {}


def _matrix_from_pose(pose) -> np.ndarray:
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1.0e-12:
        matrix = np.eye(4, dtype=np.float64)
    else:
        s = 2.0 / n
        xx, yy, zz = x * x * s, y * y * s, z * z * s
        xy, xz, yz = x * y * s, x * z * s, y * z * s
        wx, wy, wz = w * x * s, w * y * s, w * z * s
        matrix = np.array(
            [
                [1.0 - yy - zz, xy - wz, xz + wy, 0.0],
                [xy + wz, 1.0 - xx - zz, yz - wx, 0.0],
                [xz - wy, yz + wx, 1.0 - xx - yy, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix


def _quaternion_from_matrix(matrix: np.ndarray):
    m = matrix[:3, :3]
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def _assign_pose_from_matrix(pose, matrix: np.ndarray) -> None:
    qx, qy, qz, qw = _quaternion_from_matrix(matrix)
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)


class OdomAdapterNode(Node):
    """Unifies Mono Hydra odometry input for ROS 2.

    Normal ScanNet mode consumes /external_odometry and republishes it on the
    Mono Hydra namespaced topic. RVIO2 bridge mode consumes the newest pose from
    /rvio2/trajectory and exposes the same Odometry interface.
    """

    def __init__(self) -> None:
        super().__init__("mono_hydra_vio_odom_adapter")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter("input_odom_topic", "/external_odometry")
        self.declare_parameter("rvio2_trajectory_topic", "/rvio2/trajectory")
        self.declare_parameter("output_odom_topic", "/mono_hydra_vio/odometry")
        self.declare_parameter("output_path_topic", "/mono_hydra_vio/path")
        self.declare_parameter("publish_bridge_external_odom", True)
        self.declare_parameter("use_rvio2_bridge", False)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("publish_camera_tf", True)
        self.declare_parameter("publish_map_tf", False)
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "scannet_world")
        self.declare_parameter("base_link_frame_id", "base_link_kimera")
        self.declare_parameter("camera_frame_id", "scannet_camera")
        self.declare_parameter("path_max_length", 2000)
        self.declare_parameter("force_frame_ids", False)
        self.declare_parameter("left_camera_params_path", "")
        self.declare_parameter("input_pose_frame", "body")

        self.input_odom_topic = self.get_parameter("input_odom_topic").value
        self.rvio2_trajectory_topic = self.get_parameter("rvio2_trajectory_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.output_path_topic = self.get_parameter("output_path_topic").value
        self.use_rvio2_bridge = bool(self.get_parameter("use_rvio2_bridge").value)
        self.publish_bridge_external_odom = bool(self.get_parameter("publish_bridge_external_odom").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_camera_tf = bool(self.get_parameter("publish_camera_tf").value)
        self.publish_map_tf = bool(self.get_parameter("publish_map_tf").value)
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_link_frame_id = self.get_parameter("base_link_frame_id").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.path_max_length = int(self.get_parameter("path_max_length").value)
        self.force_frame_ids = bool(self.get_parameter("force_frame_ids").value)
        self.left_camera_params_path = self.get_parameter("left_camera_params_path").value
        self.input_pose_frame = self.get_parameter("input_pose_frame").value
        if self.input_pose_frame not in ("body", "sensor"):
            raise RuntimeError("input_pose_frame must be 'body' or 'sensor'")
        self.sensor_T_body = None
        if self.input_pose_frame == "sensor":
            if not self.left_camera_params_path:
                raise RuntimeError("left_camera_params_path is required for input_pose_frame='sensor'")
            left_cam_params = _load_yaml(self.left_camera_params_path)
            transform = left_cam_params.get("T_BS", {})
            data = transform.get("data")
            rows = int(transform.get("rows", 4))
            cols = int(transform.get("cols", 4))
            if not data or rows != 4 or cols != 4:
                raise RuntimeError(f"Invalid T_BS in {self.left_camera_params_path}")
            body_T_sensor = np.array(data, dtype=np.float64).reshape(rows, cols)
            self.sensor_T_body = np.linalg.inv(body_T_sensor)

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.output_path_topic, 10)
        self.bridge_pub = (
            self.create_publisher(Odometry, self.input_odom_topic, 10)
            if self.use_rvio2_bridge and self.publish_bridge_external_odom
            else None
        )

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame_id
        self.last_rvio2_stamp_ns: Optional[int] = None
        self.last_rvio2_path_len = 0
        self.received_count = 0

        if self.use_rvio2_bridge:
            self.sub = self.create_subscription(Path, self.rvio2_trajectory_topic, self._on_rvio2_path, 10)
            self.get_logger().info(
                f"RVIO2 bridge mode: {self.rvio2_trajectory_topic} -> {self.output_odom_topic}"
            )
        else:
            self.sub = self.create_subscription(Odometry, self.input_odom_topic, self._on_odom, 10)
            self.get_logger().info(f"ScanNet odometry mode: {self.input_odom_topic} -> {self.output_odom_topic}")

        self._publish_static_tfs()
        self.get_logger().info(
            f"frames: map={self.map_frame_id} odom={self.odom_frame_id} "
            f"base={self.base_link_frame_id} camera={self.camera_frame_id} publish_tf={self.publish_tf} "
            f"input_pose_frame={self.input_pose_frame} force_frame_ids={self.force_frame_ids}"
        )

    def _normalize_odom(self, msg: Odometry) -> Odometry:
        odom = copy.deepcopy(msg)
        if _stamp_is_zero(odom.header.stamp):
            odom.header.stamp = self.get_clock().now().to_msg()
        if self.force_frame_ids or not odom.header.frame_id:
            odom.header.frame_id = self.odom_frame_id

        if self.input_pose_frame == "sensor" and self.sensor_T_body is not None:
            world_T_sensor = _matrix_from_pose(odom.pose.pose)
            world_T_body = world_T_sensor @ self.sensor_T_body
            _assign_pose_from_matrix(odom.pose.pose, world_T_body)
            odom.child_frame_id = self.base_link_frame_id
        elif self.force_frame_ids or not odom.child_frame_id:
            odom.child_frame_id = self.base_link_frame_id
        return odom

    def _on_odom(self, msg: Odometry) -> None:
        self._publish_outputs(self._normalize_odom(msg))

    def _on_rvio2_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        latest = msg.poses[-1]
        stamp = latest.header.stamp if not _stamp_is_zero(latest.header.stamp) else msg.header.stamp
        if _stamp_is_zero(stamp):
            stamp = self.get_clock().now().to_msg()

        stamp_ns = _stamp_ns(stamp)
        if (
            self.last_rvio2_stamp_ns is not None
            and stamp_ns <= self.last_rvio2_stamp_ns
            and len(msg.poses) <= self.last_rvio2_path_len
        ):
            return

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_link_frame_id
        odom.pose.pose = copy.deepcopy(latest.pose)
        if self.input_pose_frame == "sensor" and self.sensor_T_body is not None:
            world_T_sensor = _matrix_from_pose(latest.pose)
            world_T_body = world_T_sensor @ self.sensor_T_body
            _assign_pose_from_matrix(odom.pose.pose, world_T_body)

        self.last_rvio2_stamp_ns = stamp_ns
        self.last_rvio2_path_len = len(msg.poses)
        if self.bridge_pub is not None:
            self.bridge_pub.publish(odom)
        self._publish_outputs(odom)

    def _publish_outputs(self, odom: Odometry) -> None:
        self.received_count += 1
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = copy.deepcopy(odom.header)
        pose.pose = copy.deepcopy(odom.pose.pose)
        self.path_msg.header.stamp = odom.header.stamp
        self.path_msg.header.frame_id = odom.header.frame_id or self.odom_frame_id
        self.path_msg.poses.append(pose)
        if self.path_max_length > 0 and len(self.path_msg.poses) > self.path_max_length:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_length :]
        self.path_pub.publish(self.path_msg)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = copy.deepcopy(odom.header)
            transform.header.frame_id = odom.header.frame_id or self.odom_frame_id
            transform.child_frame_id = odom.child_frame_id or self.base_link_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

        if self.received_count == 1:
            self.get_logger().info(
                f"first odometry forwarded with frame_id={odom.header.frame_id} "
                f"child_frame_id={odom.child_frame_id}"
            )

    def _publish_static_tfs(self) -> None:
        transforms = []
        stamp = self.get_clock().now().to_msg()
        if self.publish_map_tf:
            transforms.append(self._make_static_transform(stamp, self.map_frame_id, self.odom_frame_id))
        if self.publish_camera_tf and self.camera_frame_id:
            transforms.append(self._make_static_transform(stamp, self.base_link_frame_id, self.camera_frame_id))
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    @staticmethod
    def _make_static_transform(stamp, parent: str, child: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = (
            _yaw_to_quaternion(0.0)
        )
        return transform


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomAdapterNode()
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
