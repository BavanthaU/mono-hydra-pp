# ONNX Models

The public Mono Hydra ROS 2 repository intentionally does not include ONNX model
exports.

For local ONNX runs, place the exported models here with the filenames expected
by the launch defaults:

```text
scannet_depth_sem_320x416.onnx
scannet_depth_sem_192x256_trt_clean.onnx
```

ONNX inference is opt-in:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  perception_backend:=onnx onnx_model_path:=/path/to/model.onnx
```
