# mono_hydra_vio

`mono_hydra_vio` is the VIO package for Mono Hydra. It combines the ROS 2 R-VIO2
frontend, the RVIO2-to-Hydra odometry interface, RGB-to-mono conversion for
monocular tracking, the ROS 1 Kimera VIO configuration set used for the
benchmarks, and VIO feature-conditioning utilities.

## Nodes

| Node | Purpose |
| --- | --- |
| `rvio2_mono_node` | Monocular RGB/IMU R-VIO2 trajectory estimation |
| `rgb_to_mono_node` | Converts RGB image streams to the mono image stream used by R-VIO2 |
| `odom_adapter_node` | Converts `/rvio2/trajectory` or `/external_odometry` into `/mono_hydra_vio/odometry`, `/external_odometry`, path, and TF |
| `mono_hydra_vio_ros_node` | Native ROS 2 port of the benchmarked Kimera VIO RGB-D/external-odometry pipeline and LCD pose-graph publisher |
| `kimera_pose_graph_bridge_node` | Optional bridge for forwarding loop edges already produced by Kimera VIO pose graph topics |
| `vio_feature_interface_node` | Publishes semantic feature masks and sparse depth images for VIO ablations and Kimera-compatible integrations |

## VIO Feature Interfaces

The feature interface preserves the Mono Hydra ROS 1 research hooks:

- sparse depth factors from learned dense depth, published as a sparse `32FC1`
  image on `/mono_hydra_vio/sparse_depth`;
- semantic feature masking from M2H label IDs, published as a `mono8` keep mask
  on `/mono_hydra_vio/semantic_feature_mask`;
- SuperPoint support through `superpoint_keypoint_mask_topic`, allowing a
  learned keypoint mask to gate sparse-depth sampling.

The RVIO2 path remains the default pose source for ITC and uHumans2. Kimera VIO
LCD is expected to produce the benchmarked pose graph and loop-closure edges.
Hydra consumes those pose graph messages and owns the final scene-graph
optimization.

## RVIO2 + Kimera Loop-Closure Flow

The ROS 2 flow mirrors the ROS 1 proposal-only contract:

```text
RGB + IMU
  -> R-VIO2 monocular frontend
  -> /rvio2/trajectory
  -> odom_adapter_node
  -> /external_odometry
  -> mono_hydra_vio_ros_node
  -> /mono_hydra_vio_ros/pose_graph
  -> /mono_hydra_vio_ros/pose_graph_incremental
  -> Hydra backend optimization with Kimera-PGMO/Kimera-RPGO
```

The ROS 1 Kimera VIO core is built in this package with the legacy DBoW2,
OpenGV, Kimera-RPGO, and ONNX Runtime dependencies vendored under
`mono_hydra_utils`. The ROS 2 `mono_hydra_vio_ros_node` feeds ROS 2 image, IMU,
depth, semantic-mask, and external-odometry topics into the original Kimera
pipeline and publishes the same pose-graph topics used by the ROS 1 stack.

## Configuration

R-VIO2 calibration presets:

```text
config/itc_realsense_848x480.yaml
config/uhumans2_office_720x480.yaml
```

Important launch arguments:

```text
use_rvio2_backend
use_rvio2_bridge
use_kimera_vio_ros_node
use_kimera_pose_graph_bridge
kimera_params_folder
kimera_sensor_params_folder
kimera_flags_folder
kimera_pose_graph_topic
kimera_pose_graph_incremental_topic
external_loop_closures_topic
use_sparse_depth_factors
use_semantic_masking
semantic_mask_topic
semantic_mask_inflate_px
sparse_depth_stride
sparse_depth_min_m
sparse_depth_max_m
superpoint_keypoint_mask_topic
kimera_rgbd_sync_queue_size
kimera_stereo_sync_queue_size
kimera_do_coarse_imu_camera_temporal_sync
kimera_do_fine_imu_camera_temporal_sync
```

Benchmark-matched assets imported from ROS 1:

```text
params/
vocabulary/ORBvoc.yml
rviz/
ros1_reference/
```

## Citations

This package builds on:

- Zheng Huai and Guoquan Huang, “Robocentric visual-inertial odometry,” IJRR,
  2022.
- Daniel DeTone, Tomasz Malisiewicz, and Andrew Rabinovich, “SuperPoint:
  Self-Supervised Interest Point Detection and Description,” CVPR Workshops,
  2018.
- Antoni Rosinol et al., “Kimera: From SLAM to spatial perception with 3D
  dynamic scene graphs,” IJRR, 2021.

The sparse-depth-factor and semantic-masking contributions should be cited with
the Mono Hydra paper entries.
