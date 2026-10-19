#!/usr/bin/env python3
"""Refine only Hydra's RGB-D branch; raw M2H continues to feed Kimera."""
import json
import os
from collections import deque
from pathlib import Path
from threading import Condition, Thread
import time

os.environ.setdefault('OMP_NUM_THREADS', '2')

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener, TransformException

from mono_hydra_perception.ptc_refinement import PtcRefiner, RefinementConfig
from mono_hydra_perception.multi_reference_ptc import MultiReferencePtc


def transform_matrix(msg):
    q, t = msg.rotation, msg.translation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = quat / norm
    T = np.eye(4)
    T[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class PtcRefinementNode(Node):
    def __init__(self):
        super().__init__('ptc_depth_refinement')
        defaults = dict(rgb_topic='/mono_hydra_perception/synced/image_raw',
                        camera_info_topic='/mono_hydra_perception/synced/camera_info',
                        depth_topic='/camera/depth_cam/image_raw',
                        labels_topic='/camera/seg_cam/labels_argmax',
                        output_prefix='/mono_hydra_refinement',
                        fixed_frame='odom', camera_frame='left_cam_kimera',
                        width=424, local=True, flow_preset='fast', min_baseline=0.05,
                        references=1,
                        max_scale_change=0.10, max_age_s=1.5, max_depth=10.0,
                        pose_wait_s=1.0, max_queue=12, cpu_threads=2, metrics_path='')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.fixed_frame, self.camera_frame = p('fixed_frame'), p('camera_frame')
        self.pose_wait_s, self.max_queue = float(p('pose_wait_s')), int(p('max_queue'))
        if self.pose_wait_s < 0 or self.max_queue < 1 or int(p('cpu_threads')) < 1:
            raise ValueError('Invalid refinement queue/thread settings')
        cv2.setNumThreads(int(p('cpu_threads')))
        refinement_config = RefinementConfig(
            width=int(p('width')), local=bool(p('local')), flow_preset=p('flow_preset'),
            min_baseline=float(p('min_baseline')), max_scale_change=float(p('max_scale_change')),
            max_age_s=float(p('max_age_s')), max_depth=float(p('max_depth')))
        references = int(p('references'))
        if references not in (1, 2):
            raise ValueError('references must be 1 (existing PTC) or 2 (experimental)')
        self.refiner = (PtcRefiner(refinement_config) if references == 1 else
                        MultiReferencePtc(refinement_config, references=references))
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.metrics = None
        if p('metrics_path'):
            path = Path(p('metrics_path')).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.metrics = path.open('x', buffering=1)
        prefix = p('output_prefix').rstrip('/')
        self.pubs = [self.create_publisher(typ, prefix + suffix, 10) for typ, suffix in
                     [(Image, '/rgb'), (Image, '/depth'), (Image, '/labels'), (CameraInfo, '/camera_info')]]
        self.condition = Condition()
        self.pending = deque()
        self.stopping = False
        self.received = self.published = self.overflow = self.refined = 0
        # These are selected inference outputs, not the high-rate camera stream.
        # Losing one member under best-effort delivery loses the entire bundle.
        qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        self.subs = [message_filters.Subscriber(self, typ, p(name), qos_profile=qos) for typ, name in
                     [(Image, 'rgb_topic'), (Image, 'depth_topic'), (Image, 'labels_topic'), (CameraInfo, 'camera_info_topic')]]
        self.sync = message_filters.TimeSynchronizer(self.subs, 30)
        self.sync.registerCallback(self.enqueue)
        self.worker = Thread(target=self.run, name='ptc-refinement', daemon=False)
        self.worker.start()
        self.create_timer(5.0, self.report)
        self.get_logger().info('PTC refinement ready; only Hydra uses the output bundle; raw M2H remains available')

    def enqueue(self, *messages):
        with self.condition:
            self.received += 1
            if len(self.pending) >= self.max_queue:
                self.pending.popleft()
                self.overflow += 1
            self.pending.append((time.monotonic(), messages))
            self.condition.notify()

    def report(self):
        if not self.worker.is_alive() and not self.stopping:
            raise RuntimeError('PTC worker stopped unexpectedly')
        self.get_logger().info(f'PTC received={self.received} published={self.published} '
                               f'refined={self.refined} overflow={self.overflow} queued={len(self.pending)}')

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending or self.stopping)
                if self.stopping:
                    return
                received_at, messages = self.pending.popleft()
            rgb_msg, depth_msg, labels_msg, info_msg = messages
            stamp = Time.from_msg(depth_msg.header.stamp)
            stats = {'stamp_ns': stamp.nanoseconds, 'reason': 'missing_pose',
                     'corrected_fraction': 0.0, 'elapsed_ms': 0.0}
            pose = None
            deadline = received_at + self.pose_wait_s
            while not self.stopping:
                try:
                    tf = self.tf_buffer.lookup_transform(self.fixed_frame,
                        self.camera_frame or info_msg.header.frame_id, stamp, timeout=Duration())
                    pose = transform_matrix(tf.transform)
                    break
                except (TransformException, ValueError):
                    if time.monotonic() >= deadline:
                        break
                    with self.condition:
                        self.condition.wait(timeout=0.01)
            if self.stopping:
                return
            if pose is not None:
                try:
                    rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
                    depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
                    if depth_msg.encoding == '16UC1':
                        depth = depth.astype(np.float32) * 0.001
                    elif depth_msg.encoding != '32FC1':
                        raise ValueError('Depth must be 32FC1 metres or 16UC1 millimetres')
                    labels = self.bridge.imgmsg_to_cv2(labels_msg, 'passthrough')
                    if info_msg.width != depth.shape[1] or info_msg.height != depth.shape[0]:
                        raise ValueError('CameraInfo and depth resolution differ')
                    refined, stats = self.refiner.refine(rgb, depth, labels,
                        np.array(info_msg.k).reshape(3, 3), pose, stamp.nanoseconds)
                    if stats['reason'] == 'refined':
                        depth_msg = self.bridge.cv2_to_imgmsg(refined, '32FC1')
                        depth_msg.header = rgb_msg.header
                        self.refined += 1
                except Exception as exc:
                    self.refiner.reset()
                    stats['reason'] = 'exception'
                    stats['error'] = str(exc)
                    self.get_logger().error(f'PTC fallback to raw M2H: {exc}', throttle_duration_sec=2.0)
            else:
                self.refiner.reset()
            stats['queue_and_refinement_ms'] = (time.monotonic() - received_at) * 1000
            try:
                for pub, msg in zip(self.pubs, (rgb_msg, depth_msg, labels_msg, info_msg)):
                    pub.publish(msg)
                self.published += 1
                if self.metrics:
                    self.metrics.write(json.dumps(stats, allow_nan=False) + '\n')
            except Exception:
                if not self.stopping and rclpy.ok():
                    raise

    def destroy_node(self):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        self.worker.join()
        if self.metrics:
            self.metrics.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PtcRefinementNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
