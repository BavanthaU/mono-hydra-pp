<div align="center">

# Mono Hydra++

**Monocular 3D scene graphs from RGB and IMU**

ROS 2 Jazzy · RVIO2–SuperPoint · M2H-MX · Hydra

[Paper](https://arxiv.org/abs/2605.17661) · [Code version](https://github.com/BavanthaU/mono-hydra-pp/tree/mono_hydrapp_v1.0) · [Setup](#setup) · [Run ITC](#run-itc) · [Component switches](#component-switches) · [Citation](#bibliography-and-citation)

</div>

Mono Hydra++ combines visual–inertial odometry, learned metric depth and
semantics, local geometric refinement, and Hydra mapping to build semantic
meshes and layered 3D scene graphs.

![Mono Hydra++ pipeline](images/system_overview_journal_v14.png)

The ITC launch enables RVIO2–SuperPoint tracking, M2H-MX perception, temporal
alignment, sparse depth factors, geometric reliability weighting, temporal
object association, and Kimera loop closure with Hydra PGMO. Use the
[component switches](#component-switches) to adjust the pipeline.

## Setup

Use Ubuntu 24.04 with ROS 2 Jazzy. Create a workspace:

```bash
mkdir -p ~/mono_hydra_ros2_ws
cd ~/mono_hydra_ros2_ws
git clone https://github.com/BavanthaU/mono-hydra-pp.git src
```

### Model weights

Download [M2H-MX checkpoints](https://huggingface.co/Bavantha11/m2h-mx) or
[ONNX exports](https://huggingface.co/Bavantha11/vio-stack-jetson-weights).
Choose **ScanNet** or **NYUDv2** weights for your scene, and set
`perception_dataset:=scannet` or `perception_dataset:=nyudv2` to match.

Complete the [SuperPoint model and ONNX Runtime setup](mono_hydra_vio/README.md#rvio2-sp-superpoint-guided-tracking),
then download the loop-closure vocabulary:

```bash
src/mono_hydra_vio/scripts/download_orb_vocabulary.sh
```

### Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src src/mono_hydra_perception/mavista --ignore-src -r -y
MAKEFLAGS="-j2" colcon build --base-paths src src/mono_hydra_perception/mavista \
  --symlink-install --parallel-workers 2
source install/setup.bash
```

## Run ITC

Convert the ROS 1 bag once:

```bash
src/mono_hydra_utils/mono_hydra_utils/scripts/convert_itc_ros1_bag_to_ros2.sh \
  /path/to/ITC_2nd_floor_full_loop.bag
```

**Terminal 1 — start the pipeline.** Select your downloaded checkpoint and its
dataset preset; this example uses ScanNet:

```bash
cd ~/mono_hydra_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  sequence_name:=itc_full_loop \
  perception_dataset:=scannet \
  perception_checkpoint_path:=/path/to/m2h_mx_large_scannet.pt
```

**Terminal 2 — play the bag:**

```bash
cd ~/mono_hydra_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
src/mono_hydra_utils/mono_hydra_utils/scripts/play_itc_full_loop_ros2.sh
```

Playback starts paused. Wait for initialization, then press **Space**. After the
bag finishes, let processing drain, then press **Ctrl-C** once in terminal 1
and wait for export. Outputs are saved under `output/itc_full_loop/`.
Use a new `sequence_name` for each run. See the [ITC guide](ITC_REVISION.md) for
model options, output settings, and PTC setup.

## Component switches

The paper components are **on by default** in the ITC launch. Append any of
these arguments to the launch command to change the configuration:

| Component | Default | Launch argument |
| --- | --- | --- |
| RVIO2–SuperPoint tracking | SuperPoint dense | `rvio2_variant:=superpoint` / `classical` |
| M2H-MX perception | PyTorch | `perception_backend:=torch` / `onnx` |
| Temporal depth alignment | On | `use_temporal_alignment:=true` / `false` |
| Temporal label fusion | On | `temporal_fuse_labels:=true` / `false` |
| Sparse depth factors | On | `use_sparse_depth_factors:=true` / `false` |
| Semantic feature masking | On | `use_semantic_masking:=true` / `false` |
| Geometric reliability weighting | On | `revision_fusion_weighting:=true` / `false` |
| Temporal object association | On | `revision_object_association:=true` / `false` |
| Kimera visual loop closure | On | `enable_lcd:=true` / `false` |
| Hydra semantic loop detector | Off | `enable_hydra_lcd:=true` / `false` |
| PTC refinement | Off, optional | `use_ptc_refinement:=true` / `false` |

For example, disable temporal alignment and sparse depth factors:

```bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  perception_dataset:=scannet \
  perception_checkpoint_path:=/path/to/m2h_mx_large_scannet.pt \
  use_temporal_alignment:=false use_sparse_depth_factors:=false
```

For PTC, follow [installation and switching instructions](ITC_REVISION.md#switch-ptc-on-or-off)
and use `use_ptc_refinement:=true use_temporal_alignment:=false`.

## Pipeline components

### Temporal alignment

Past depth and label predictions are warped into the current camera frame.
Visibility and consistency gates select observations for depth and label fusion.
Control this stage with `use_temporal_alignment` and `temporal_fuse_labels`.

![Temporal alignment and fusion](images/temporal_alignment_journal_v14.png)

### Depth sampling and local refinement

Depth at tracked keypoints is filtered by geometry and semantics before it
contributes to the local optimization. `use_sparse_depth_factors` controls the
depth constraints; `use_semantic_masking` controls semantic feature filtering.

![Depth sampling and filtering](images/depth_sampling_journal_v5.png)

The depth factor relates a camera-frame depth measurement to a landmark and
camera pose, alongside visual, IMU, and RVIO2 motion constraints.

![Depth-factor geometry](images/depth_factor_geometry_journal_v3.png)

### Mapping and loop closure

Hydra integrates depth and semantics into the mesh and scene graph.
`revision_fusion_weighting` applies geometric consistency weights during TSDF
integration. `revision_object_association` enables temporal object association.
Kimera supplies visual loop closures to Hydra PGMO with `enable_lcd:=true`.
See [mapping parameters](mono_hydra/JOURNAL_REVISION.md) for tuning options.

## Code version

The ISPRS paper version is tagged **`mono_hydrapp_v1.0`**.
[Checkout and checksum instructions](REPRODUCIBILITY.md) describe how to use
that version with the matching model files.

## License

[MIT](LICENSE) for the Mono Hydra integration. Third-party components retain
their respective licenses; see [Third-party notices](THIRD_PARTY_NOTICES.md).

## Acknowledgements

This work builds on major open-source robotics systems:

- MIT SPARK Hydra and Hydra-ROS: https://github.com/MIT-SPARK/Hydra and
  https://github.com/MIT-SPARK/Hydra-ROS. Cite Hughes et al., “Hydra: A
  Real-time Spatial Perception System for 3D Scene Graph Construction and
  Optimization,” RSS, 2022.
- Kimera, Kimera-PGMO, and Kimera-RPGO:
  https://github.com/MIT-SPARK/Kimera-VIO,
  https://github.com/MIT-SPARK/Kimera-PGMO, and
  https://github.com/MIT-SPARK/Kimera-RPGO. The `mono_hydra_vio` pose-graph and
  loop-closure implementation follows the Kimera formulation; cite Rosinol et
  al., “Kimera: an Open-Source Library for Real-Time Metric-Semantic
  Localization and Mapping,” ICRA, 2020, and Rosinol et al., “Kimera: From SLAM
  to Spatial Perception with 3D Dynamic Scene Graphs,” IJRR, 2021.
- R-VIO / R-VIO2: https://github.com/rpng/R-VIO. Cite Huai and Huang,
  “Robocentric Visual-Inertial Odometry,” IJRR, and the square-root R-VIO2
  online spatiotemporal calibration work where applicable.
- GTSAM: https://github.com/borglab/gtsam. Cite Dellaert, “Factor Graphs and
  GTSAM: A Hands-on Introduction,” 2012.

## Bibliography and citation

If you use this repository, please cite Mono-Hydra++ and the relevant perception
and mapping papers below, alongside the upstream systems acknowledged above.

1. **Mono-Hydra++** — U. V. B. L. Udugama, George Vosselman, and Francesco Nex.
   *Mono-Hydra++: Real-Time Monocular Scene Graph Construction with Multi-Task
   Learning for 3D Indoor Mapping.* 2026. Under review at the **ISPRS Journal of
   Photogrammetry and Remote Sensing**; revised manuscript submitted following
   the first review cycle. [Paper](https://arxiv.org/abs/2605.17661).

2. **M2H-MX** — U. V. B. L. Udugama, George Vosselman, and Francesco Nex.
   *M2H-MX: Multi-Task Semantic and Geometric Perception for Real-Time Monocular
   3D Scene Graph Construction.* **ICRA 2026, SRRA Workshop**, 2026 (accepted).
   [Publication details](https://bavanthau.github.io/publications/)
   · [Paper](https://arxiv.org/abs/2603.29236).

3. **M2H** — U. V. B. L. Udugama, George Vosselman, and Francesco Nex.
   *M2H: Multi-Task Learning with Efficient Window-Based Cross-Task Attention
   for Monocular Spatial Perception.* **IEEE/RSJ International Conference on
   Intelligent Robots and Systems (IROS)**, 2025.
   [DOI](https://doi.org/10.1109/IROS60139.2025.11246974)
   · [Paper](https://arxiv.org/abs/2510.17363).

4. **Mono-Hydra** — U. V. B. L. Udugama, G. Vosselman, and F. Nex.
   *Mono-Hydra: Real-Time 3D Scene Graph Construction from Monocular Camera
   Input with IMU.* **ISPRS Annals of the Photogrammetry, Remote Sensing and
   Spatial Information Sciences**, X-1/W1-2023, 439–445, 2023.
   [DOI](https://doi.org/10.5194/isprs-annals-X-1-W1-2023-439-2023)
   · [Paper](https://arxiv.org/abs/2308.05515).

<details>
<summary><strong>BibTeX — copy the relevant entries</strong></summary>

```bibtex
@unpublished{udugama2026monohydrapp,
  title         = {{Mono-Hydra++}: Real-Time Monocular Scene Graph Construction with Multi-Task Learning for {3D} Indoor Mapping},
  author        = {Udugama, U. V. B. L. and Vosselman, George and Nex, Francesco},
  year          = {2026},
  note          = {Under review at ISPRS Journal of Photogrammetry and Remote Sensing},
  eprint        = {2605.17661},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  doi           = {10.48550/arXiv.2605.17661},
  url           = {https://arxiv.org/abs/2605.17661}
}

@inproceedings{udugama2026m2hmx,
  title         = {{M2H-MX}: Multi-Task Semantic and Geometric Perception for Real-Time Monocular {3D} Scene Graph Construction},
  author        = {Udugama, U. V. B. L. and Vosselman, George and Nex, Francesco},
  booktitle     = {IEEE International Conference on Robotics and Automation (ICRA), SRRA Workshop},
  year          = {2026},
  note          = {Accepted},
  eprint        = {2603.29236},
  archivePrefix = {arXiv},
  url           = {https://arxiv.org/abs/2603.29236}
}

@inproceedings{udugama2025m2h,
  title         = {{M2H}: Multi-Task Learning with Efficient Window-Based Cross-Task Attention for Monocular Spatial Perception},
  author        = {Udugama, U. V. B. L. and Vosselman, George and Nex, Francesco},
  booktitle     = {2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year          = {2025},
  doi           = {10.1109/IROS60139.2025.11246974},
  eprint        = {2510.17363},
  archivePrefix = {arXiv},
  url           = {https://doi.org/10.1109/IROS60139.2025.11246974}
}

@article{udugama2023monohydra,
  title   = {{Mono-Hydra}: Real-Time {3D} Scene Graph Construction from Monocular Camera Input with {IMU}},
  author  = {Udugama, U. V. B. L. and Vosselman, G. and Nex, F.},
  journal = {ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences},
  volume  = {X-1/W1-2023},
  pages   = {439--445},
  year    = {2023},
  doi     = {10.5194/isprs-annals-X-1-W1-2023-439-2023},
  url     = {https://doi.org/10.5194/isprs-annals-X-1-W1-2023-439-2023}
}
```

</details>
