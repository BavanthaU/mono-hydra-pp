# mono_hydra_vio

This repository is the Mono-Hydra fork of
[Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO). The upstream project is
licensed under MIT; all original authorship and credit remain with the
MIT-SPARK team. Please cite their work when using this fork:

```
@InProceedings{Rosinol20icra-Kimera,
  title     = {Kimera: an Open-Source Library for Real-Time Metric-Semantic
               Localization and Mapping},
  author    = {Rosinol, Antoni and Abate, Marcus and Chang, Yun and Carlone, Luca},
  booktitle = {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
  year      = {2020},
  url       = {https://github.com/MIT-SPARK/Kimera}
}

@article{Rosinol21arxiv-Kimera,
  title   = {Kimera: from SLAM to Spatial Perception with 3D Dynamic Scene Graphs},
  author  = {Rosinol, Antoni and Violette, Andrew and Abate, Marcus and Hughes,
             Nathan and Chang, Yun and Shi, Jingnan and Gupta, Arjun and Carlone, Luca},
  journal = {arXiv preprint},
  volume  = {2101.06894},
  year    = {2021}
}
```

## What changed for Mono-Hydra?

- SuperPoint-based feature detection driven by ONNX Runtime (CPU/GPU).
- Sensor/parameter packages for RealSense RGB-D, ZED-X mono/RGB-D, and uHumans2
  datasets.
- Vendored ONNX Runtime 1.18 binaries in
  `third_party/onnxruntime-linux-x64-gpu-1.18.0`.

These additions are tested only with the revisions cloned by the Mono-Hydra
workspace setup script.

The current branch corresponds to the base version of Kimera-VIO used in the Mono-Hydra pipeline. SuperPoint support is included (used in the RealSense_RGBD_sp and ZEDXMono_RGBD parameter sets) and will be documented in the upcoming Mono-Hydra journal article. Updates aligned with that publication will be tagged once the paper is available.

When using this fork in publications, please cite both the Kimera papers above and the Mono-Hydra paper:

```bibtex
@article{Udugama_2023,
   title={MONO-HYDRA: REAL-TIME 3D SCENE GRAPH CONSTRUCTION FROM MONOCULAR CAMERA INPUT WITH IMU},
   volume={X-1/W1-2023},
   ISSN={2194-9050},
   url={http://dx.doi.org/10.5194/isprs-annals-X-1-W1-2023-439-2023},
   DOI={10.5194/isprs-annals-x-1-w1-2023-439-2023},
   journal={ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences},
   publisher={Copernicus GmbH},
   author={Udugama, U. V. B. L. and Vosselman, G. and Nex, F.},
   year={2023},
   month=dec,
   pages={439--445}
}
```

## How to use this fork

1. Run the workspace helper in `mono_hydra/scripts/setup_workspace.sh` (or the
   hosted version in the Mono-Hydra README) to clone `mono_hydra_vio`,
   `mono_hydra_vio_ros`, `Mono_Hydra`, `M2H`, and all required dependencies at
   known-good commits.
2. Build the catkin workspace (`catkin build`) after sourcing
   `/opt/ros/noetic/setup.bash`.
3. Launch the pipeline through the Mono-Hydra bring-up launch files (for
   example `roslaunch mono_hydra hydra_v2_d435i.launch`).

## Acknowledgements

Maintained by **Bavantha Udugama** (University of Twente / UAV Centre) for the
Mono-Hydra project. Massive thanks to the MIT-SPARK authors for releasing
Kimera-VIO and to the broader open-source community for the dependencies this
fork reuses.
