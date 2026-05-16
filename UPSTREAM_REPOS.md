# Upstream Repositories and Provenance

This workspace is published as a single ROS 2 repository for Mono Hydra. The
third-party source trees are vendored under `mono_hydra_utils` so the stack
can be built without Git submodules.

Before publication, the nested upstream `.git` metadata and nested `.gitignore`
files were removed. The source revisions below were recorded for provenance, and
the public ignore policy is now controlled by the repository-root `.gitignore`.

| Component | Repository | Recorded revision |
| --- | --- | --- |
| Hydra | https://github.com/MIT-SPARK/Hydra.git | `2e58a35b` |
| Hydra-ROS | https://github.com/MIT-SPARK/Hydra-ROS.git | `8f3b7e3` |
| Spark-DSG | https://github.com/MIT-SPARK/Spark-DSG.git | `3c40997` |
| Kimera-PGMO | https://github.com/MIT-SPARK/Kimera-PGMO.git | `b7f2811` |
| Kimera-RPGO | https://github.com/MIT-SPARK/Kimera-RPGO.git | `f1fee09` |
| config_utilities | https://github.com/MIT-SPARK/config_utilities.git | `629688a` |
| Ianvs | https://github.com/MIT-SPARK/Ianvs.git | `e76fc7c` |
| pose_graph_tools | https://github.com/MIT-SPARK/pose_graph_tools.git | `965dfe8` |
| Spatial-Hash | https://github.com/MIT-SPARK/Spatial-Hash.git | `8045892` |
| TEASER++ | https://github.com/MIT-SPARK/TEASER-plusplus.git | `52a9c52` |

Additional imported or adapted components:

- Kimera-VIO / Kimera-VIO-ROS reference source and benchmark configuration are
  kept under `mono_hydra_vio/ros1_reference`.
- R-VIO2 source is built into `mono_hydra_vio/src/rvio2` and wrapped by the
  ROS 2 `rvio2_mono_node`.
- GTSAM is consumed as the factor-graph backend dependency through the ROS/system
  dependency chain.

Third-party license files are retained in their respective source directories.
