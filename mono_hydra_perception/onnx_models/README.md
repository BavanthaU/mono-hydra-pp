# ONNX Models

The public Mono Hydra ROS 2 repository intentionally does not include ONNX model
exports.

For local ONNX runs, place the exported models here with the filenames expected
by the launch defaults:

```text
m2h_mx_b_scannet_480x640.onnx
m2h_mx_b_nyudv2_480x640.onnx
scannet_depth_sem_320x416.onnx
scannet_depth_sem_192x256_trt_clean.onnx
```

ONNX inference is opt-in:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  perception_backend:=onnx onnx_model_path:=/path/to/model.onnx
```

The M2H-MX-B exports use static `1x3x480x640` RGB input, ImageNet
normalization, and named outputs:

```text
ScanNet: semseg, depth
NYUDv2:  semseg, depth, normals, edge
```

The ROS node infers static input size and class count from the graph. Use
`onnx_model_preset:=m2h_mx_b_scannet_480x640` or
`onnx_model_preset:=m2h_mx_b_nyudv2_480x640` from `mono_hydra` launch files.
