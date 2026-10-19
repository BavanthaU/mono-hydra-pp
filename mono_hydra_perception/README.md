# mono_hydra_perception

`mono_hydra_perception` contains the learned perception models used by Mono
Hydra. It publishes metric depth and semantic label IDs in the topic layout
expected by Hydra and the VIO feature interface.

## Backends

| Backend | Launch value | Model source | Resolution |
| --- | --- | --- | --- |
| Original ITC M2H | `perception_backend:=m2h` | ROS 1 `m2h_indoor.pt` and `m2h_core` port | `256x256` |
| Stock M2H-HMX-Large | `perception_backend:=torch` | vendored PyTorch configs and checkpoints | dataset-configured, overrideable |
| ONNX export | `perception_backend:=onnx` | packaged ONNX exports, including M2H-MX-B ScanNet/NYUDv2 | fixed graph input |

The original ITC M2H backend is the default for ITC parity because the ROS 1
benchmark launches `roslaunch m2h m2h.launch`. The HMX-Large backend supports
ITC, ScanNet, and NYUD/uHumans research profiles. The ONNX backend uses the
same ROS 2 topic contract as the torch backend and is the workstation/Jetson
path for fixed-shape exports.

All inference backends can publish synchronized RGB and CameraInfo alongside
each depth/label prediction. Dataset launches use that stream for Kimera RGBD
and Hydra so perception latency does not break RGB-depth pairing:

```text
/mono_hydra_perception/synced/image_raw
/mono_hydra_perception/synced/camera_info
```

The ROS 1 temporal pose-warp filter is ported as
`temporal_pose_warp_filter_node` and is enabled from launch with
`use_temporal_alignment:=true`.

For simulated rosbag runs, both backends also report when output frames are
behind `/clock`. `perception_max_output_lag_s:=0.0` preserves every processed
frame for dense parity; setting it to a positive value drops stale frames and is
useful for responsive RViz debugging when playback outruns inference.

## M2H-MX-B ONNX

Place the MX base ONNX files under:

```text
mono_hydra_perception/onnx_models/m2h_mx_b_scannet_480x640.onnx
mono_hydra_perception/onnx_models/m2h_mx_b_nyudv2_480x640.onnx
```

The existing `m2h_onnx_node` reads these models directly. It infers the static
`480x640` input shape and 40-logit semantic head from the graph, publishes the
standard Mono Hydra depth and label topics, and can optionally publish NYUDv2
normals/edge outputs.

ScanNet workstation run:

```bash
ros2 launch mono_hydra mono_hydra_scannet.launch.py \
  perception_backend:=onnx \
  onnx_model_preset:=m2h_mx_b_scannet_480x640 \
  perception_skip_frequency:=3
```

NYUDv2 node-only smoke run:

```bash
ros2 run mono_hydra_perception m2h_onnx_node --ros-args \
  -p dataset:=nyudv2 \
  -p model_path:="$(ros2 pkg prefix mono_hydra_perception)/share/mono_hydra_perception/onnx_models/m2h_mx_b_nyudv2_480x640.onnx" \
  -p image_topic:=/camera/color/image_raw \
  -p publish_color_semantic:=true \
  -p publish_normals:=true \
  -p publish_edge:=true
```

## Topics

| Topic | Type | Encoding |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `rgb8` or compatible |
| `/camera/depth_cam/image_raw` | `sensor_msgs/msg/Image` | `32FC1` |
| `/camera/seg_cam/labels_argmax` | `sensor_msgs/msg/Image` | `mono8` |
| `/camera/seg_cam/image_raw` | `sensor_msgs/msg/Image` | `bgr8` or `rgb8`, optional |
| `/mono_hydra_perception/synced/image_raw` | `sensor_msgs/msg/Image` | input RGB, same header as M2H outputs |
| `/mono_hydra_perception/synced/camera_info` | `sensor_msgs/msg/CameraInfo` | CameraInfo restamped to the M2H output frame |
| `/temporally_aligned/depth` | `sensor_msgs/msg/Image` | `32FC1`, optional |
| `/temporally_aligned/labels_argmax` | `sensor_msgs/msg/Image` | `mono8`, optional |

## Model Runtime

Install the tested runtime with:

```bash
MAX_JOBS=2 src/mono_hydra_utils/mono_hydra_utils/scripts/install_m2h_hmx_large_python_deps.sh
```

The current tested stack is Torch 2.10 CUDA 12.8, ONNX Runtime GPU, Transformers,
Timm, Einops, and Mamba-SSM.

## Citations

This package builds on the following external components:

- Oriane Siméoni et al., “DINOv3,” arXiv:2508.10104, 2025.
- Albert Gu and Tri Dao, “Mamba: Linear-Time Sequence Modeling with Selective
  State Spaces,” arXiv:2312.00752, 2023.
- ONNX Runtime developers, “ONNX Runtime,” 2021.

The M2H-HMX-Large and Mono Hydra model citations should be added with the
project papers used for the journal submission.
