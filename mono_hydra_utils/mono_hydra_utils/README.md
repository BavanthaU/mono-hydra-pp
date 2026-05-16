# mono_hydra_utils

`mono_hydra_utils` contains support modules, configuration assets, and command
wrappers that are useful for reproducing Mono Hydra experiments, plus small
visualization helpers used by the launch files.

## Python Modules

```text
mono_hydra_utils.workspace
mono_hydra_utils.bag_conversion
mono_hydra_utils.runtime_checks
```

`workspace` centralizes workspace path discovery. `bag_conversion` owns the
ROS 1 to ROS 2 bag conversion workflow used by ITC. `runtime_checks` validates
the deep-learning runtime used by the M2H-HMX-Large backends.
`mesh_marker_node` converts Hydra's Kimera-PGMO mesh topic to a standard RViz2
`visualization_msgs/Marker` triangle list for the Mono Hydra RViz layout.

## Configuration

```bash
src/mono_hydra_utils/mono_hydra_utils/config/tf_overrides.yaml
```

The TF QoS override is packaged here so bag-playback commands do not depend on
loose files in the workspace root.

## Command Wrappers

```bash
src/mono_hydra_utils/mono_hydra_utils/scripts/install_m2h_hmx_large_python_deps.sh
src/mono_hydra_utils/mono_hydra_utils/scripts/convert_itc_ros1_bag_to_ros2.sh
```

The dependency installer pins the tested deep-learning runtime and includes
ONNX Runtime GPU for the fixed-resolution perception backend.

The ITC converter uses `rosbags-convert` so ROS 1 bags can be played directly
into ROS 2 Jazzy without keeping the runtime workspace coupled to ROS 1.

## Citation Notes

This package does not implement a research algorithm. Cite the datasets,
models, and SLAM/perception backends used by the experiment launched from
`mono_hydra`.
