# mono_hydra_vio

`mono_hydra_vio` is the VIO package for Mono Hydra. It combines the ROS 2 R-VIO2
frontend, the RVIO2-to-Hydra odometry interface, RGB-to-mono conversion for
monocular tracking, the ROS 1 Kimera VIO configuration set used for the
benchmarks, and VIO feature-conditioning utilities.

## Nodes

| Node | Purpose |
| --- | --- |
| `rvio2_mono_node` | Monocular RGB/IMU R-VIO2 trajectory estimation |
| `rvio2_sp_mono_node` | R-VIO2 with SuperPoint guided feature tracking (same topics as `rvio2_mono_node`) |
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

## RVIO2-SP: SuperPoint Guided Tracking

`rvio2_sp_mono_node` is built from a separate copy of R-VIO2 in `src/rvio2_sp`
(namespace `RVIO2SP`). The classical `src/rvio2` sources and `rvio2_mono_node`
are unchanged. Only the tracker's association step differs; RANSAC, track
bookkeeping and the filter update are shared code.

Per image, one SuperPoint pass (ONNX Runtime, CUDA) produces NMS keypoints and
a dense descriptor map. `Tracker.Method` selects the association:

- `2` **SuperPoint dense** (default in the SuperPoint configs): predict each
  track with the IMU rotation, correlate its descriptor over a window of the
  dense descriptor field (no re-detection needed, ambiguous peaks rejected),
  LK-align from the previous image starting at the peak, and keep the result
  only if the descriptor at the aligned point still matches. Geometry comes from
  image alignment, appearance decides association.
- `1` SuperPoint guided: match to re-detected keypoints near the prediction,
  refine with LK, KLT fallback. Snapping to pixel-quantised keypoints breaks
  sub-pixel continuity, so RVIO2's 0.6 px RANSAC gate drops many tracks; kept
  for comparison.
- `0` KLT (classical).

New features come from SuperPoint keypoints (`Tracker.Detector: 1`) through the
classical grid logic.

Select the implementation with `rvio2_variant`:

```bash
# classical R-VIO2 fallback
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py rvio2_variant:=classical ...
# R-VIO2 with SuperPoint tracking (ITC default)
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py rvio2_variant:=superpoint ...
# same, as a wrapper
ros2 launch mono_hydra mono_hydra_itc_rosbag_rvio2_sp.launch.py ...
```

Switches live in `config/itc_realsense_848x480_superpoint.yaml` and can be
overridden per launch (`-1` / empty keeps the file value):

| Launch argument | Config key | Values |
| --- | --- | --- |
| `rvio2_sp_tracker_method` | `Tracker.Method` | 0 KLT, 1 SuperPoint guided, 2 SuperPoint dense |
| `rvio2_sp_detector` | `Tracker.Detector` | 0 Shi-Tomasi, 1 SuperPoint |
| `rvio2_sp_gyro_prediction` | `SPTrack.GyroPrediction` | 0/1 |
| `rvio2_sp_klt_fallback` | `SPTrack.KltFallback` | 0/1 |
| `rvio2_sp_refine` | `SPTrack.Refine` | 0 none, 1 LK, 2 cornerSubPix |
| `rvio2_sp_update_descriptor` | `SPTrack.UpdateDescriptor` | 0 first-frame, 1 latest |
| `rvio2_sp_klt_gyro_prediction` | `Tracker.KltGyroPrediction` | 0/1, KLT method only |
| `rvio2_sp_use_gpu` | `SuperPoint.UseGpu` | 0/1 |
| `rvio2_sp_model_path` | `SuperPoint.ModelPath` | path |
| `rvio2_sp_stats_csv` | `Stats.CsvPath` | per-frame tracking statistics |
| `rvio2_sp_config_overrides` | any key | `"Key=value;Key=value"` |

`Tracker.Method: 0` with `Tracker.Detector: 0` reproduces classical R-VIO2
inside `rvio2_sp_mono_node`, which is useful for A/B runs with identical
statistics logging.

The SuperPoint target is enabled by default for the revised ITC flow.
Export the ITC model from separately obtained SuperPoint v1 weights, from the
workspace root:

```bash
python3 src/mono_hydra_vio/scripts/export_superpoint_onnx.py \
  --weights /path/to/superpoint_v1.pth --height 480 --width 848 \
  --output src/mono_hydra_vio/models/superpoint_480x848.onnx
```

Install ONNX Runtime C/C++ headers and its matching runtime library, export the
model below, and build explicitly:

```bash
colcon build --packages-select mono_hydra_vio --cmake-args \
  -DMONO_HYDRA_BUILD_RVIO2_SP=ON \
  -DRVIO2_SP_ORT_INCLUDE_DIR=/path/to/onnxruntime/include \
  -DRVIO2_SP_ORT_LIB_DIR=/path/to/onnxruntime/lib
```

The existing Kimera core also expects the ONNX Runtime 1.18 distribution at
`src/mono_hydra_vio/ros1_reference/mono_hydra_vio/third_party/onnxruntime-linux-x64-gpu-1.18.0/`
(from the workspace root), containing `include/` and `lib/`. These runtime files
are ignored by Git and must be installed locally. The `RVIO2_SP_ORT_*` settings
above apply to the SuperPoint target; they do not relocate Kimera's dependency.

For a classical-only build, set `-DMONO_HYDRA_BUILD_RVIO2_SP=OFF` and launch
with `rvio2_variant:=classical`.

The model `models/superpoint_480x848.onnx` exposes the raw `semi` and `desc`
heads and can be regenerated for other resolutions with
`scripts/export_superpoint_onnx.py`. CMake links the pip `onnxruntime-gpu`
runtime (CUDA 12) when it is importable, otherwise the bundled ONNX Runtime
1.18; override with `-DRVIO2_SP_ORT_LIB_DIR=... -DRVIO2_SP_CUDA_LIB_DIRS=...`.
The CUDA library paths are embedded in the binary's RPATH, so no
`LD_LIBRARY_PATH` setup is required. The SuperPoint v1 weights are licensed by
Magic Leap for non-commercial research use.

## Configuration

R-VIO2 calibration presets:

```text
config/itc_realsense_848x480.yaml
config/itc_realsense_848x480_superpoint.yaml   (rvio2_sp_mono_node)
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
vocabulary/ORBvoc.yml  generated locally by scripts/download_orb_vocabulary.sh
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
