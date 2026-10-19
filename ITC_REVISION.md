# ITC first-review source update

This source snapshot synchronizes the local ITC full-loop implementation for
the first journal review cycle. It includes source changes and regression
checks; it does not replace the submitted manuscript or its result tables.

## Scope

- Timestamp-preserving M2H inputs, bounded perception queues, and optional
  latest-frame inference scheduling.
- Robust TSDF influence, object geometry/remapping fixes, and optional
  geometric reliability and temporal object association.
- Independent Kimera/Hydra LCD switches, detection-only Kimera loop reporting,
  and complete shutdown exports to Hydra PGMO.
- RVIO2–SuperPoint dense tracking as the main ITC flow, with classical RVIO2
  available as a fallback. PTC refinement and MAViSTA remain optional.

The ITC launcher includes `mono_hydra_scannet.launch.py`: despite that filename,
it implements the shared perception/VIO/Hydra wiring required by ITC. Shared
semantic mappings also support optional model presets on the ITC sensor stream.
Separate ScanNet/7 Scenes workflows and experiment results are outside this
update. Existing dataset support in the repository is retained.

## Run the main ITC RVIO2–SuperPoint + M2H-HMX flow

First follow the [SuperPoint build and model setup](mono_hydra_vio/README.md#rvio2-sp-superpoint-guided-tracking).
The SuperPoint ONNX file and ONNX Runtime headers/libraries must be supplied
locally before building. From the workspace root, start terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  sequence_name:=itc_revision_$(date +%Y%m%d_%H%M%S) \
  rvio2_variant:=superpoint rvio2_sp_tracker_method:=2 rvio2_sp_detector:=1 \
  perception_backend:=torch perception_dataset:=itc \
  perception_skip_frequency:=3 perception_target_fps:=0.0 \
  perception_preserve_aspect_ratio:=false use_temporal_alignment:=false \
  use_ptc_refinement:=false \
  revision_fusion_weighting:=false revision_object_association:=false \
  enable_lcd:=true enable_hydra_lcd:=false kimera_lcd_no_optimize:=true \
  hydra_exit_after_clock:=false use_rviz:=true
```

Terminal 2, from the same workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
src/mono_hydra_utils/mono_hydra_utils/scripts/play_itc_full_loop_ros2.sh
```

The helper expects the converted ITC full-loop bag under
`test_data/itc_ros2_bags/ITC_2nd_floor_full_loop_ros2`. It starts paused; wait for
initialization, then press Space. After playback, allow about 45 seconds for
processing to drain, press Ctrl-C once in terminal 1, and wait for clean export.

The explicit `torch` argument selects M2H-HMX. Omitting it uses the older M2H
launch default. Place checkpoints locally as described in the main README.
Model weights, ONNX exports, runtime libraries, bags, private commands, and
generated experiment outputs are not included in this source update.

## Optional components

For reliability weighting and temporal object association, append
`revision_fusion_weighting:=true revision_object_association:=true`.
Their algorithms, parameters, and limitations are documented in
[JOURNAL_REVISION.md](mono_hydra/JOURNAL_REVISION.md).

### Switch PTC on or off

PTC is disabled by default and remains an optional test. Install it once from
the workspace root:

```bash
bash src/mono_hydra_perception/scripts/install_ptc_depth.sh
# Source the env.sh path printed by the installer in terminal 1.
```

In the main terminal-1 command above, replace `use_ptc_refinement:=false` with
this block to turn on single-reference PTC:

```bash
  use_ptc_refinement:=true ptc_python_path:="$PTC_DEPTH_PYTHONPATH" \
  ptc_width:=424 ptc_local:=true ptc_max_scale_change:=0.10 ptc_references:=1 \
```

Set `ptc_references:=2` for the experimental two-reference test. Keep
`use_temporal_alignment:=false`. The bag playback command is identical.

To turn PTC off, stop the launch cleanly, choose a new `sequence_name`, and
restart with `use_ptc_refinement:=false`. No uninstall or rebuild is required.
`perception_target_fps:=0.0` preserves the original frame-skipping scheduler;
it is independent of whether PTC is enabled. Detailed installation, adapter
behavior, and limitations are in [PTC_REFINEMENT.md](mono_hydra_perception/PTC_REFINEMENT.md).

| Component | Main flow | Switch / fallback |
| --- | --- | --- |
| RVIO2 frontend | SuperPoint dense | `rvio2_variant:=classical` restores classical tracking |
| PTC | Off | `use_ptc_refinement:=true` after installation; `false` disables it |
| PTC references | One when enabled | `ptc_references:=2` enables the experimental pair |
| Reliability weighting | Off | `revision_fusion_weighting:=true` / `false` |
| Temporal object association | Off | `revision_object_association:=true` / `false` |
| Latest-frame scheduler | Off | `perception_target_fps:=5.0`; `0.0` restores frame skipping |

For a classical-only build, use `-DMONO_HYDRA_BUILD_RVIO2_SP=OFF` and explicitly
launch with `rvio2_variant:=classical`. MAViSTA has a separate
[ITC guide](run_guide_mavista.md).

## Validation scope

Publication checks on 2026-10-19 passed:

- 17 Python tests covering perception scheduling, label mapping, native PTC
  adapters, and MAViSTA geometry.
- 19 rebuilt Hydra tests covering robust fusion, geometric reliability, mesh
  clustering, and object association.
- Builds of the RVIO2–SuperPoint and Kimera nodes in the existing workspace;
  launch/perception/MAViSTA package builds from the publication checkout.
- 12 launch combinations covering the default, SuperPoint, and classical
  frontend selections with PTC and revision switches on/off; source syntax
  and whitespace checks.
These are implementation checks; no complete ITC replay or manuscript metric
reproduction is implied by this publication update.
