# MAViSTA ROS 2 / Hydra deployment bundle

MAViSTA means **Metric-Aware Visual Intelligence for Semantics and Temporal
Alignment**. The deployed checkpoint is the GeoST-MAViSTA architecture at
training step 025250; “MAViSTA” is the short package and system name used in
ROS 2, launch files, and Hydra integration.

This bundle deploys the GeoST-MAViSTA checkpoint `step_025250.pt` as the fixed
480x640 stateful temporal ONNX model
`mavista_geost_step025250_temporal_480x640.onnx`.

Inputs are synchronized rectified RGB (`sensor_msgs/Image`) and matching
`CameraInfo`. The node uses the RGB and CameraInfo internally, center-crops and
resizes the model input to 640x480, and publishes only depth in metres
(`32FC1`), NYU40 labels (`32SC1`), and optional confidence (`32FC1`). Before
publication, predictions are projected back into the original input image
dimensions; pixels outside the centered model crop use invalid depth and label
0. The original RGB and CameraInfo topics remain the Hydra and VIO image
sources and are not republished by MAViSTA. It uses `CameraInfo.P` first and
falls back to `K` only when needed.

The graph inputs are `image` `[1,3,480,640]`, pixel `intrinsics`
`[fx,fy,cx,cy]`, `previous_hidden_state` `[1,96,30,40]`, and a boolean
`reset_mask`. It returns metric depth, 64 canonical semantic logits, semantic
confidence, and `next_hidden_state`. The ROS node feeds the next state back into
the following frame and takes semantic argmax only over channels 1..40.

Temporal state starts at zero and persists for the entire process, including
across dropped frames and timestamp gaps. It is not connected to VIO or an
external pose-warp filter. Reset it only when deliberately starting a new
environment:

```bash
ros2 service call /mavista/new_environment std_srvs/srv/Trigger {}
```

## Install and verify

Extract this folder under a ROS 2 workspace `src/`, install the ROS dependencies plus an ONNX Runtime compatible with the target CUDA/JetPack stack, then build:

```bash
rosdep install --from-paths src src/mono_hydra_perception/mavista --ignore-src -r -y
colcon build --base-paths src src/mono_hydra_perception/mavista \
  --packages-select mavista
source install/setup.bash
python3 src/mono_hydra_perception/mavista/scripts/verify_model.py
```

For a desktop NVIDIA system, install a CUDA-matched `onnxruntime-gpu`; the node selects CUDA when available and explicitly warns if it must use CPU. For Jetson, install the wheel that matches the installed JetPack release.

## Run

Start standalone with explicit remaps or edit `config/mavista.yaml`:

```bash
ros2 launch mavista mavista.launch.py \
  params_file:=/absolute/path/to/mavista.yaml \
  -r /camera/color/image_rect:=/real/rectified/rgb \
  -r /camera/color/camera_info:=/real/rectified/camera_info
```

The existing Mono Hydra launch keeps the input RGB and CameraInfo topics and
publishes MAViSTA predictions to `/hydra/camera/depth/image_rect` and
`/hydra/camera/semantic/image_raw`; standalone diagnostic confidence is
published under `/mavista`. Keep the workspace's existing TF and odometry
source: MAViSTA supplies perception only.

Use rectified camera input. Do not use NYUv2 intrinsics or assume a particular
camera driver. Confirm output image dimensions, timestamps, and frame IDs match
the original RGB input, valid depth is within 0.1--10 m, and semantic IDs are in
0--40 before enabling a mapping run.
