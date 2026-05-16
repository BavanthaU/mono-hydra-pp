# Mono Hydra Utility Sources

This directory keeps support packages and upstream source dependencies together
so the workspace top level stays focused on the three main Mono Hydra runtime
packages.

## ROS Package

```text
mono_hydra_utils/
```

The `mono_hydra_utils` ROS package contains reproducibility helpers, bag
conversion tools, runtime checks, and packaged QoS configuration.

## Upstream And Support Sources

```text
config_utilities/
hydra/
hydra_ros/
ianvs/
kimera_pgmo/
kimera_rpgo/
pose_graph_tools/
semantic_inference_msgs/
spark_dsg/
spatial_hash/
teaser_plusplus/
```

These are kept as normal colcon-discoverable ROS or CMake packages, but grouped
under `mono_hydra_utils` because they support the Mono Hydra runtime rather
than being project-facing packages.

`semantic_inference_msgs` is retained only because `hydra_ros` uses those
message definitions for optional feature-image interfaces. The TensorRT
`semantic_inference` runtime and `semantic_inference_ros` node are intentionally
not part of this workspace; Mono Hydra semantics come from
`mono_hydra_perception`.

The Kimera-PGMO RViz plugin is also intentionally omitted. The retained
visualization libraries are the ones `hydra_ros` links against; the Mono Hydra
runtime is configured for headless launch by default. The `mono_hydra_utils`
package provides a lightweight mesh-marker renderer so Hydra's
`kimera_pgmo_msgs/msg/Mesh` output is still visible in stock RViz2.
