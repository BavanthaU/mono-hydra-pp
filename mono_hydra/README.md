# mono_hydra

`mono_hydra` is the ROS 2 bridge between Mono Hydra datasets, the perception
and VIO packages in this workspace, and the official MIT SPARK Hydra ROS 2
backend. It owns launch files, dataset-specific topic/frame mappings, Hydra
input configuration, label-space selection, and output-path policy.

## Launch Files

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py
ros2 launch mono_hydra mono_hydra_itc.launch.py
ros2 launch mono_hydra mono_hydra_uhumans.launch.py
ros2 launch mono_hydra mono_hydra_7scenes.launch.py
ros2 launch mono_hydra mono_hydra_scannet.launch.py
```

The main launch accepts `perception_backend:=m2h` for the original ROS 1 ITC
M2H model, `perception_backend:=torch` for the newer M2H-HMX-Large model, and
`perception_backend:=onnx` for the fixed-resolution ONNX export.

## MAViSTA layout

MAViSTA means **Metric-Aware Visual Intelligence for Semantics and Temporal
Alignment**. The current deployed checkpoint is GeoST-MAViSTA step 025250;
MAViSTA remains the short name used by the ROS package and launch workflow.

The MAViSTA-Q model and inference node live together in the dedicated ROS 2
package at:

```text
src/mono_hydra_perception/mavista/
├── mavista/                 # ONNX Runtime node and geometry code
├── launch/                  # standalone perception launch
├── config/                  # node defaults
├── models/                  # step-2500 ONNX model and metadata
└── scripts/                 # model verification utilities
```

The package name exposed to ROS 2 is `mavista`. Its model file is
installed from `models/mavista_geost_step025250_temporal_480x640.onnx`. The ROS
node carries the model's recurrent ConvGRU hidden state from one accepted frame
to the next; no external temporal fusion or VIO pose input is used.

Mono Hydra owns only the integration launch files in
`src/mono_hydra/launch/`:

- `mono_hydra_itc_mavista_hydra.launch.py` is the canonical ITC Hydra + RViz
  terminal used by `run_guide_mavista.md`.
- `mono_hydra_mavista_nyuv2.launch.py` is the standalone perception-plus-Hydra
  convenience wrapper for a camera with an existing pose/TF source.
- `mono_hydra_scannet.launch.py` contains the shared backend switch used by
  both MAViSTA and the other perception backends; it is not a second MAViSTA
  node.

The four-terminal ITC workflow is the source of truth for the complete run.
The MAViSTA package consumes the existing RGB/CameraInfo topics and publishes
only `32FC1` metric depth and `32SC1` NYU40 labels; it does not republish the
camera stream or create odometry/camera poses.

Run the convenience perception + Hydra path with an existing TF/odometry source:

```bash
ros2 launch mono_hydra mono_hydra_mavista_nyuv2.launch.py \
  input_image_topic:=/camera/color/image_raw \
  input_camera_info_topic:=/camera/color/camera_info \
  provider:=cuda \
  use_rviz:=false
```

Run MAViSTA alone for topic inspection:

```bash
ros2 launch mavista mavista.launch.py \
  input_image_topic:=/camera/color/image_raw \
  input_camera_info_topic:=/camera/color/camera_info \
  provider:=cuda
```

If the camera's raw stream is distorted, run `image_proc` and pass its
rectified image and matching CameraInfo topics instead. The node uses the
input CameraInfo `P` matrix when valid and never substitutes NYUv2 intrinsics.

ITC defaults to `perception_backend:=m2h` because the benchmarked ROS 1 ITC
stack used the original `m2h` package with `m2h_indoor.pt`. HMX-Large and ONNX
remain explicit overrides for research and inference checks.

## Runtime Graph

```text
RGB + IMU -> mono_hydra_vio / R-VIO2 -> /mono_hydra_vio/odometry + TF
/external_odometry -> Kimera VIO LCD-equivalent pose graph topics
RGB + CameraInfo -> MAViSTA -> learned depth + semantic labels
original RGB + CameraInfo + depth + labels + pose graph + LC proposals + TF -> hydra_ros -> DSG outputs
```

Hydra LCD is enabled by default. The benchmarked RVIO2 + Kimera flow is kept as
the source of truth: loop closures come from Kimera VIO LCD pose graph outputs,
and final loop-closure optimization is delegated to the official Hydra ROS 2
backend using Kimera-PGMO and Kimera-RPGO.

## Dataset Mapping

| Dataset | Perception | Pose source | Label space |
| --- | --- | --- | --- |
| ITC | original M2H `m2h_indoor.pt` | R-VIO2 | NYUD20 |
| uHumans2 office | M2H-HMX-Large NYUD | R-VIO2 | NYUD20 |
| 7Scenes | M2H-HMX-Large ScanNet | dataset odometry until calibrated R-VIO2 config is added | ScanNet20 |
| ScanNet | M2H-HMX-Large ScanNet | dataset odometry or supplied R-VIO2 bridge | ScanNet20 |

The original ROS 1 launch, config, and RViz assets are copied into
`ros1_reference/` and `rviz/` so topic and visualization changes can be checked
against the benchmarked stack.

## Commands

Install model runtimes:

```bash
MAX_JOBS=2 src/mono_hydra_utils/mono_hydra_utils/scripts/install_m2h_hmx_large_python_deps.sh
```

Convert an ITC ROS 1 bag:

```bash
src/mono_hydra_utils/mono_hydra_utils/scripts/convert_itc_ros1_bag_to_ros2.sh \
  $HOME/ros1_workspaces/hydra2_ws/data/itc/ITC_2nd_floor_full_loop.bag
```

Run ITC with the ROS 2 RViz visualizer:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py
```

Run ITC headless:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  use_rviz:=false visualize:=false
```

ITC launches open `rviz/mono_hydra_ros2.rviz` by default and start the official
Hydra DSG marker visualizer. The RViz layout follows the ROS 1 workflow: the
Displays/config tree is docked on the left, RGB/depth/semantic image panels are
docked on the right, the 3D scene graph stays in the center, and the Time panel
is synchronized to the RGB input. It shows RGB, learned depth, semantic labels,
semantic color overlays, RVIO2/VIO trajectories, Kimera pose graph markers, and
Hydra DSG/backend marker outputs.

Play the converted bag:

```bash
$(ros2 pkg prefix mono_hydra_utils)/share/mono_hydra_utils/scripts/play_itc_full_loop_ros2.sh
```

Converted ROS 1 bags should not replay their original `/tf` or `/tf_static`
into the Mono Hydra stack. The launch files publish the benchmark frame chain
used by Hydra and RViz.

The helper plays at the normal real-time rosbag rate.

For ITC, allow roughly 30-60 seconds of playback before checking the DSG marker
topic because RVIO2 starts publishing usable poses after the first few RGB/IMU
frames:

```bash
ros2 topic echo --once /hydra_dsg_visualizer/dsg_markers \
  --qos-durability transient_local --qos-reliability reliable --no-arr
```

## Citations

This bridge uses the official Hydra ROS 2 stack and should be cited with:

- Nathan Hughes, Yun Chang, and Luca Carlone, “Hydra: A Real-time Spatial
  Perception System for 3D Scene Graph Construction and Optimization,” RSS,
  2022.
- Nathan Hughes et al., “Foundations of Spatial Perception for Robotics:
  Hierarchical Representations and Real-time Systems,” IJRR, 2024.
- Antoni Rosinol et al., “Kimera: From SLAM to spatial perception with 3D
  dynamic scene graphs,” IJRR, 2021.

Mono Hydra and M2H project citations should be added with the corresponding
paper entries for publication.
