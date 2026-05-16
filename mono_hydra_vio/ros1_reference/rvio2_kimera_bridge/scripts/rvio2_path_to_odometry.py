#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
import tf2_ros
from tf.transformations import quaternion_from_matrix, quaternion_matrix


def _load_yaml(path: Path) -> dict:
    text = path.read_text()
    if text.startswith("%YAML:"):
        text = "\n".join(text.splitlines()[1:])
    return yaml.safe_load(text) or {}


def _matrix_from_pose_msg(msg: PoseStamped) -> np.ndarray:
    quat = msg.pose.orientation
    trans = msg.pose.position
    transform = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transform[0, 3] = trans.x
    transform[1, 3] = trans.y
    transform[2, 3] = trans.z
    return transform


def _pose_msg_from_matrix(matrix: np.ndarray) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    quat = quaternion_from_matrix(matrix)
    return (
        (float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3])),
        (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
    )


class Rvio2PathToOdometry:
    def __init__(self) -> None:
        self.trajectory_topic = rospy.get_param("~trajectory_topic", "/rvio2/trajectory")
        self.odometry_topic = rospy.get_param("~odometry_topic", "/external_odometry")
        self.left_camera_params_path = Path(rospy.get_param("~left_camera_params_path"))
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        self.body_frame_id = rospy.get_param("~body_frame_id", "base_link_kimera")
        # RVIO2 publishes q_kG/p_Gk as a body pose. Keep "sensor" available for
        # other path sources, but make the RVIO2-safe behavior the default.
        self.input_pose_frame = rospy.get_param("~input_pose_frame", "body")
        self.publish_velocity = bool(rospy.get_param("~publish_velocity", False))
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        if self.input_pose_frame not in ("body", "sensor"):
            raise RuntimeError("~input_pose_frame must be 'body' or 'sensor'")

        left_cam_params = _load_yaml(self.left_camera_params_path)
        transform = left_cam_params.get("T_BS", {})
        data = transform.get("data")
        rows = int(transform.get("rows", 4))
        cols = int(transform.get("cols", 4))
        if not data or rows != 4 or cols != 4:
            raise RuntimeError(f"Invalid T_BS in {self.left_camera_params_path}")

        self.body_T_sensor = np.array(data, dtype=np.float64).reshape(rows, cols)
        self.sensor_T_body = np.linalg.inv(self.body_T_sensor)

        self.publisher = rospy.Publisher(self.odometry_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.subscriber = rospy.Subscriber(self.trajectory_topic, PathMsg, self.callback, queue_size=1)

        self.last_stamp_ns: Optional[int] = None
        self.last_position: Optional[np.ndarray] = None
        self.last_rotation: Optional[np.ndarray] = None
        self.last_path_size: int = 0

        rospy.loginfo(
            "rvio2_path_to_odometry forwarding %s -> %s using %s, input_pose_frame=%s, publish_tf=%s",
            self.trajectory_topic,
            self.odometry_topic,
            self.left_camera_params_path,
            self.input_pose_frame,
            self.publish_tf,
        )

    def callback(self, msg: PathMsg) -> None:
        if not msg.poses:
            return

        latest = msg.poses[-1]
        stamp = latest.header.stamp
        if stamp.to_nsec() == 0:
            stamp = msg.header.stamp
        if stamp.to_nsec() == 0:
            stamp = rospy.Time.now()

        stamp_ns = stamp.to_nsec()
        path_size = len(msg.poses)
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns and path_size <= self.last_path_size:
            return

        world_T_input = _matrix_from_pose_msg(latest)
        if self.input_pose_frame == "sensor":
            world_T_body = world_T_input @ self.sensor_T_body
        else:
            world_T_body = world_T_input

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame_id
        odom.child_frame_id = self.body_frame_id

        position, quat = _pose_msg_from_matrix(world_T_body)
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        if self.publish_velocity and self.last_stamp_ns is not None and self.last_position is not None and self.last_rotation is not None:
            dt = (stamp_ns - self.last_stamp_ns) * 1e-9
            if dt > 1e-6:
                curr_position = world_T_body[:3, 3]
                world_R_body = world_T_body[:3, :3]
                linear_world = (curr_position - self.last_position) / dt
                linear_body = world_R_body.T @ linear_world
                odom.twist.twist.linear.x = float(linear_body[0])
                odom.twist.twist.linear.y = float(linear_body[1])
                odom.twist.twist.linear.z = float(linear_body[2])

                delta_R = self.last_rotation.T @ world_R_body
                angle = math.acos(max(-1.0, min(1.0, (np.trace(delta_R) - 1.0) * 0.5)))
                if angle > 1e-9:
                    axis = np.array(
                        [
                            delta_R[2, 1] - delta_R[1, 2],
                            delta_R[0, 2] - delta_R[2, 0],
                            delta_R[1, 0] - delta_R[0, 1],
                        ],
                        dtype=np.float64,
                    )
                    axis_norm = np.linalg.norm(axis)
                    if axis_norm > 1e-9:
                        axis = axis / axis_norm
                        angular_body = axis * (angle / dt)
                        odom.twist.twist.angular.x = float(angular_body[0])
                        odom.twist.twist.angular.y = float(angular_body[1])
                        odom.twist.twist.angular.z = float(angular_body[2])

        self.publisher.publish(odom)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

        self.last_stamp_ns = stamp_ns
        self.last_position = world_T_body[:3, 3].copy()
        self.last_rotation = world_T_body[:3, :3].copy()
        self.last_path_size = path_size


def main() -> None:
    rospy.init_node("rvio2_path_to_odometry")
    Rvio2PathToOdometry()
    rospy.spin()


if __name__ == "__main__":
    main()
