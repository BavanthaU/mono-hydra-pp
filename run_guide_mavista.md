# MAViSTA + Mono Hydra Run Guide

This is the four-step test flow for the GeoST-MAViSTA model in the existing ROS
2 Jazzy workspace. The ROS package is named `mavista` and lives at
`src/mono_hydra_perception/mavista`.

The deployed ONNX graph is stateful and temporal. The MAViSTA node carries its
`[1,96,30,40]` ConvGRU hidden state from one accepted frame to the next and
publishes the model's temporally consistent predictions directly. It does not
use the separate Mono Hydra temporal filter, VIO pose warping, or pose-triggered
resets. Camera intrinsics come from `/camera/color/camera_info` (`P` first,
with `K` as fallback).

## Four-terminal full Hydra run

This preserves the existing ITC Hydra + RVIO2/Kimera workflow. MAViSTA replaces
only the M2H HMX-Large perception terminal. Start Terminals 1–3 first, then
start the bag paused in Terminal 4.

### Terminal 1: Hydra + RViz

```bash
cd $HOME/ros1_workspaces/ros2_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_mavista_hydra.launch.py \
  sequence_name:=itc_mavista_manual
```

This uses the ITC Hydra backend configuration, switches the label space to
NYU40, and consumes the raw MAViSTA prediction topics.

### Terminal 2: RVIO2 -> external odometry -> Kimera RGBD

```bash
cd $HOME/ros1_workspaces/ros2_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_vio.launch.py \
  sequence_name:=itc_mavista_manual \
  enable_lcd:=true \
  kimera_lcd_no_optimize:=true \
  rvio2_input_pose_frame:=body \
  odom_adapter_publish_tf:=false \
  kimera_left_cam_topic:=/camera/color/image_raw \
  depth_topic:=/hydra/camera/depth/image_rect \
  label_topic:=/hydra/camera/semantic/image_raw
```

`odom_adapter_publish_tf:=false` lets Kimera own the odom TF tree, matching the
existing ROS 1-style run. The explicit Kimera topics are required because
MAViSTA replaces the old M2H synchronized RGB/depth/label topics.

### Terminal 3: MAViSTA CUDA perception

```bash
cd $HOME/ros1_workspaces/ros2_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mavista mavista.launch.py \
  input_image_topic:=/camera/color/image_raw \
  input_camera_info_topic:=/camera/color/camera_info \
  output_depth_topic:=/hydra/camera/depth/image_rect \
  output_semantic_topic:=/hydra/camera/semantic/image_raw \
  provider:=cuda \
  input_qos_reliability:=reliable \
  publish_diagnostics:=false \
  publish_semantic_visualization:=true
```

The RGB and CameraInfo remain on their original `/camera/color/...` topics;
MAViSTA does not republish them. Only the prediction topics below are consumed
by Hydra and Kimera. MAViSTA restores its 640x480 model predictions into the
original RGB dimensions before publication, keeping RGB, depth, labels, and
CameraInfo pixel-aligned without another RGB copy. Pixels outside the centered
model crop use depth 0 and semantic label 0. `input_qos_reliability:=reliable`
matches the ITC bag's reliable RGB and CameraInfo publishers and prevents input
sample loss. For a live camera that publishes best-effort sensor data, use
`input_qos_reliability:=best_effort` instead. MAViSTA publishes NYU40
integer labels for Hydra; use `/mavista/semantic/visualization` for a
display-compatible color image.

### Terminal 4: ITC bag replay

```bash
cd $HOME/ros1_workspaces/ros2_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play test_data/itc_ros2_bags/ITC_2nd_floor_full_loop_ros2 \
  --clock --start-paused \
  --qos-profile-overrides-path \
  "$(ros2 pkg prefix mono_hydra_utils)/share/mono_hydra_utils/config/tf_overrides.yaml" \
  --remap /tf:=/tf_ignore /tf_static:=/tf_static_ignore
```

Press `Space` in Terminal 4 to unpause after the first three terminals have
initialized.

If the workspace was changed, rebuild once before starting the terminals:

```bash
cd $HOME/ros1_workspaces/ros2_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --base-paths src src/mono_hydra_perception/mavista \
  --packages-select mavista mono_hydra_perception mono_hydra
source install/setup.bash
```

## Outputs and inspection

The main outputs are:

```text
/camera/color/image_raw            rgb8, original bag image
/camera/color/camera_info          original bag calibration
/hydra/camera/depth/image_rect     32FC1, MAViSTA metres
/hydra/camera/semantic/image_raw   32SC1, MAViSTA NYU40 labels
/mavista/semantic/visualization     rgb8, optional display-only colors
```

If enabled, use `/mavista/semantic/visualization` in an RViz/rqt Image display. Keep
`/hydra/camera/semantic/image_raw` connected to Hydra; `32SC1` is the correct
label format even though some image viewers cannot display it.

## Depth flicker investigation

The GPU is loading CUDA correctly, but GPU utilization is not a measure of
prediction stability. The deployed ONNX contract is stateful:

```text
rgb + intrinsics + previous_hidden_state + reset_mask
  -> metric_depth + semantic outputs + next_hidden_state
```

The node initializes hidden state once, passes `next_hidden_state` into the next
frame, and does not clear it because of a timing gap or dropped frame. This is
the model's trained temporal path; no external temporal smoothing is enabled.

The node clips finite predictions to the configured deployment range before
publishing:

```text
0.1 m <= depth <= 10.0 m
```

Values outside that range become the nearest boundary value. The current model
bundle does not expose a depth log-variance output.

Check whether the issue is prediction instability or transport/synchronization
with:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /hydra/camera/depth/image_rect
ros2 topic hz /camera/color/camera_info
ros2 topic info /hydra/camera/depth/image_rect
```

The input image and CameraInfo must have close timestamps. The node currently
uses an approximate synchronizer with a 30 ms slop and a queue of eight
messages.

The depth and semantic output dimensions must equal the original RGB dimensions
(848x480 for the ITC bag). Kimera aborts its RGB-D frontend on a size mismatch,
which removes `odom_T_base_link_kimera`; Hydra then drops all input while the
pose is unavailable.

To deliberately clear temporal memory before a different environment, call:

```bash
ros2 service call /mavista/new_environment std_srvs/srv/Trigger {}
```

MAViSTA processes every synchronized RGB/CameraInfo pair in the normal command
and advances recurrent state exactly once for each successful inference. There
is no MAViSTA rate limiter or intentional frame-skipping parameter. If the
output rate is lower than expected, compare the input and output rates with
`ros2 topic hz`; the limiting factor is then the camera/bag rate,
RGB/CameraInfo synchronization, ROS executor/backpressure, or downstream Hydra
processing.
