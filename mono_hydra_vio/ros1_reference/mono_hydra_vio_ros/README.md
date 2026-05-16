# mono_hydra_vio_ros

This repository is the Mono-Hydra fork of the
[Kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) wrapper released
by MIT-SPARK. The upstream project is MIT-licensed; all original authors and
contributors retain full credit. Please cite the Kimera papers when using this
software (see the `mono_hydra_vio` README for BibTeX entries).

## What changed for Mono-Hydra?

When you publish results with this fork, cite the Kimera papers and the Mono-Hydra paper:

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

- Launch files for RealSense D435i, ZED-X mono/RGB-D, and uHumans2 datasets.
- CMake integration with the SuperPoint/ONNX Runtime support introduced in the
  `mono_hydra_vio` core fork.
- Updated RViz configs and logging paths aligned with the Mono-Hydra workflow.

## How to use this fork

1. Clone the curated workspace using the script provided in the
   [`mono_hydra`](https://github.com/UAV-Centre-ITC/Mono_Hydra) repository. The
   script pulls `mono_hydra_vio_ros`, `mono_hydra_vio`, and all dependencies at
   known-good commits.
2. Build the catkin workspace (`catkin build`).
3. Launch one of the dataset-specific bring-up files, e.g.
   `roslaunch mono_hydra_vio_ros mono_hydra_vio_ros_realsense_rgbd.launch`.

   > **Note:** Kimera-VIO-ROS logs frontend/backend summaries under
   > `mono_hydra_vio_ros/output_logs/<config_name>/`. If you introduce a new
   > configuration (for example, `RealSense_RGBD_sp`), create the matching
   > subfolder in `output_logs/` ahead of time so the CSV writers can open their
   > files.

## Acknowledgements

Maintained by **Bavantha Udugama** for the Mono-Hydra project. Many thanks to
MIT-SPARK for releasing Kimera-VIO-ROS and to the broader community for the
packages reused here.
