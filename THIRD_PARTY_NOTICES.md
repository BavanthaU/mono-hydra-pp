# Third-Party Notices

This repository contains original Mono Hydra ROS 2 integration code together
with vendored upstream robotics libraries needed for reproducible builds. The
top-level MIT License applies to the original Mono Hydra code only. Vendored
projects retain their upstream licenses and copyright notices.

The upstream license files are included in place. The main vendored components
are:

| Component | Location | License file |
| --- | --- | --- |
| MIT SPARK Hydra | `mono_hydra_utils/hydra` | `mono_hydra_utils/hydra/LICENSE` |
| MIT SPARK Hydra-ROS | `mono_hydra_utils/hydra_ros` | `mono_hydra_utils/hydra_ros/LICENSE` |
| Spark-DSG | `mono_hydra_utils/spark_dsg` | `mono_hydra_utils/spark_dsg/LICENSE` |
| Kimera-PGMO | `mono_hydra_utils/kimera_pgmo` | `mono_hydra_utils/kimera_pgmo/LICENSE` |
| Kimera-RPGO | `mono_hydra_utils/kimera_rpgo_legacy` | `mono_hydra_utils/kimera_rpgo_legacy/LICENSE.BSD` |
| pose_graph_tools | `mono_hydra_utils/pose_graph_tools` | `mono_hydra_utils/pose_graph_tools/LICENSE.BSD` |
| Spatial-Hash | `mono_hydra_utils/spatial_hash` | `mono_hydra_utils/spatial_hash/LICENSE` |
| TEASER++ | `mono_hydra_utils/teaser_plusplus` | `mono_hydra_utils/teaser_plusplus/LICENSE` |
| DBoW2 | `mono_hydra_utils/dbow2` | `mono_hydra_utils/dbow2/LICENSE.txt` |
| OpenGV | `mono_hydra_utils/opengv` | `mono_hydra_utils/opengv/License.txt` |
| config_utilities | `mono_hydra_utils/config_utilities` | `mono_hydra_utils/config_utilities/LICENSE` |
| Ianvs | `mono_hydra_utils/ianvs` | `mono_hydra_utils/ianvs/LICENSE` |
| ORB vocabulary assets | `mono_hydra_vio/vocabulary` | `mono_hydra_vio/vocabulary/LICENSE.txt`, `mono_hydra_vio/vocabulary/License-gpl.txt` |

Some ROS package manifests inside vendored subdirectories use short license
strings such as `BSD`. The authoritative terms are the license files shipped
with each upstream project.

When redistributing source or binaries built from this repository, retain the
copyright notices, license texts, and attribution requirements from every
vendored project in addition to the Mono Hydra MIT License.
