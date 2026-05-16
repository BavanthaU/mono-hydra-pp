# Model Weights

The public Mono Hydra ROS 2 repository intentionally does not include trained
M2H-HMX-Large checkpoints.

For local runs, place the checkpoints here with the filenames expected by the
launch defaults:

```text
itc_large__miou_0.393_rmse_0.523_weights.pt
scannet_large__miou_0.761_rmse_0.221_weights.pt
nyudv2_large__miou_0.656_rmse_0.380_weights.pt
```

Alternatively, pass an explicit checkpoint at launch:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  perception_checkpoint_path:=/path/to/model.pt
```
