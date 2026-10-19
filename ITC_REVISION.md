# ITC setup and component switches

Follow the [setup and build instructions](README.md#setup), then run from the
workspace root.

## Run ITC

Terminal 1, using a ScanNet checkpoint:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  sequence_name:=itc_$(date +%Y%m%d_%H%M%S) \
  perception_dataset:=scannet \
  perception_checkpoint_path:=/path/to/m2h_mx_large_scannet.pt
```

For NYUDv2, use `perception_dataset:=nyudv2` with its matching checkpoint.
Download [checkpoints](https://huggingface.co/Bavantha11/m2h-mx) or
[ONNX exports](https://huggingface.co/Bavantha11/vio-stack-jetson-weights).
For a local ITC checkpoint, select `perception_dataset:=itc`.
Pass `perception_config_path:=/path/to/config.yml` when using a model
configuration other than the preset's default Large model.

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
src/mono_hydra_utils/mono_hydra_utils/scripts/play_itc_full_loop_ros2.sh
```

The playback helper reads
`test_data/itc_ros2_bags/ITC_2nd_floor_full_loop_ros2`. Press **Space** after
initialization. After playback, allow about 45 seconds for processing to drain,
press **Ctrl-C** once in terminal 1, and wait for export.

Results are saved under `output/<sequence_name>/`. Set
`output_root:=/path/to/results` to choose another location.

## Default components

RVIO2–SuperPoint, M2H-MX, temporal depth alignment, temporal label fusion,
sparse depth factors, semantic feature masking, geometric reliability weighting,
temporal object association, and Kimera visual loop closure are enabled by
default. The [switch table](README.md#component-switches) lists the controls.

Append `:=false` to a boolean switch to disable that component, or `:=true` to
enable it. Restart with a new `sequence_name` when changing settings. For example:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  sequence_name:=itc_no_depth_factors \
  perception_dataset:=scannet \
  perception_checkpoint_path:=/path/to/m2h_mx_large_scannet.pt \
  use_sparse_depth_factors:=false
```

### Temporal alignment settings

| Argument | Default | Purpose |
| --- | --- | --- |
| `temporal_history_size` | `3` | Number of historical observations |
| `temporal_max_history_age_s` | `0.5` | Maximum history age in seconds |
| `temporal_depth_gate_abs` | `0.30` | Absolute depth-consistency gate in metres |
| `temporal_depth_gate_rel` | `0.05` | Relative depth-consistency gate |
| `temporal_depth_fusion_mode` | `median` | Depth fusion rule |
| `temporal_label_min_votes` | `2` | Minimum votes for label fusion |

### Switch PTC on or off

PTC is an optional test and defaults to **off**. Install it from the workspace root:

```bash
bash src/mono_hydra_perception/scripts/install_ptc_depth.sh
# Source the env.sh path printed by the installer in terminal 1.
```

Enable PTC in place of temporal alignment:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  sequence_name:=itc_ptc \
  perception_dataset:=scannet \
  perception_checkpoint_path:=/path/to/m2h_mx_large_scannet.pt \
  use_ptc_refinement:=true use_temporal_alignment:=false \
  ptc_python_path:="$PTC_DEPTH_PYTHONPATH" \
  ptc_width:=424 ptc_local:=true ptc_max_scale_change:=0.10 ptc_references:=1
```

Use `ptc_references:=2` for the experimental two-reference mode. To return to
the default pipeline, restart with
`use_ptc_refinement:=false use_temporal_alignment:=true`.
See [PTC setup](mono_hydra_perception/PTC_REFINEMENT.md) for more options.

### Frontend and inference options

- Classical RVIO2: `rvio2_variant:=classical`. For a classical-only build,
  pass `--cmake-args -DMONO_HYDRA_BUILD_RVIO2_SP=OFF` to `colcon build`.
- ONNX: `perception_backend:=onnx perception_dataset:=itc
  onnx_model_path:=/path/to/model.onnx`. Use an export with the ITC label mapping.
- Latest-frame inference: `perception_target_fps:=5.0`. The default `0.0`
  uses frame skipping with `perception_skip_frequency:=3`.
- Mapping: see [geometric reliability and object association settings](mono_hydra/JOURNAL_REVISION.md).
