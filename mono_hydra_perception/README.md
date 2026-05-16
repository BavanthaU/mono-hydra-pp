# mono_hydra_perception

`mono_hydra_perception` contains the learned perception models used by Mono
Hydra. It publishes metric depth and semantic label IDs in the topic layout
expected by Hydra and the VIO feature interface.

## Backends

| Backend | Launch value | Model source | Resolution |
| --- | --- | --- | --- |
| Stock M2H-HMX-Large | `perception_backend:=torch` | vendored PyTorch configs and checkpoints | dataset-configured, overrideable |
| ONNX export | `perception_backend:=onnx` | imported `scannet_depth_sem_320x416.onnx` model | fixed `320x416` input |

The stock backend supports the ITC, ScanNet, and NYUD/uHumans model profiles.
The ONNX backend was imported from the older `m2h_inference_ros` package and is
kept here so the ROS 2 workspace is independent from ROS 1 workspaces.

Both inference backends can publish synchronized RGB and CameraInfo alongside
each depth/label prediction. Dataset launches use that stream for Kimera RGBD
and Hydra so full-model inference latency does not break RGB-depth pairing:

```text
/mono_hydra_perception/synced/image_raw
/mono_hydra_perception/synced/camera_info
```

The ROS 1 temporal pose-warp filter is ported as
`temporal_pose_warp_filter_node` and is enabled from launch with
`use_temporal_alignment:=true`.

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
