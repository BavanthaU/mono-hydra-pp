# Model Weights

Download [M2H-MX checkpoints](https://huggingface.co/Bavantha11/m2h-mx).
Choose the ScanNet or NYUDv2 weights for your scene and use the matching
`perception_dataset:=scannet` or `perception_dataset:=nyudv2`.
See the [model manifest](../../reproducibility/model-manifest.json) for pinned
versions and the local ITC checkpoint hash.

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
  perception_backend:=torch perception_checkpoint_path:=/path/to/model.pt
```
