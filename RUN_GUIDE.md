# Mono Hydra ROS 2 Run Guide

This workspace keeps the ROS 1 benchmarked RVIO2 + Kimera flow as the reference
contract. The ROS 2 stack should not replace Kimera VIO LCD with proximity or
geometry-only loop proposals.

The Git repository is expected to be cloned as the `src/` directory of a ROS 2
workspace. Commands below are run from the workspace root unless a section says
otherwise.

## Reference Flow

```text
RGB + IMU
  -> R-VIO2
  -> /rvio2/trajectory
  -> rvio2_path_to_odometry equivalent
  -> /external_odometry
  -> Kimera VIO RGB-D/external-odometry pipeline with lcd_no_optimize:=true
  -> /mono_hydra_vio_ros/pose_graph
  -> /mono_hydra_vio_ros/pose_graph_incremental
  -> Hydra GraphBuilder / BackendModule
  -> Kimera-PGMO / Kimera-RPGO optimized 3D scene graph
```

The important ROS 1 behavior is that Kimera VIO publishes loop-closure
proposals and does not own the final scene-graph optimization. Hydra consumes
the pose graph and performs the backend optimization.

The ROS 1 Kimera VIO core and configs are now built in this workspace. The
native ROS 2 C++ `mono_hydra_vio_ros_node` feeds ROS 2 topics into the
benchmarked Kimera RGB-D/external-odometry pipeline and publishes
`/mono_hydra_vio_ros/pose_graph_incremental`; no alternate loop-closure
generator is enabled as a substitute.

## Imported ROS 1 Assets

```text
mono_hydra_vio/params/
mono_hydra_vio/vocabulary/ORBvoc.yml
mono_hydra_vio/rviz/
mono_hydra_vio/ros1_reference/
mono_hydra/rviz/
mono_hydra/ros1_reference/
```

The ORB vocabulary is downloaded locally and is not committed to Git:

```bash
src/mono_hydra_vio/scripts/download_orb_vocabulary.sh
```

Key benchmark parameter folders:

```text
RealSense_RGBD_RVIO2
uHumans2_RGBD_RVIO2_depth_factors_off
uHumans2_RGBD_RVIO2_depth_factors_on
ScanNet
ScanNet_depth_factors_off
ScanNet_depth_factors_on
7Scenes
```

## Build

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j2" colcon build --symlink-install --continue-on-error \
  --parallel-workers 2
source install/setup.bash
```

## ITC ROS 1 Bag Conversion

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
src/mono_hydra_utils/mono_hydra_utils/scripts/convert_itc_ros1_bag_to_ros2.sh \
  /home/bavantha/ros1_workspaces/hydra2_ws/data/itc/ITC_2nd_floor_full_loop.bag
```

## ITC Run

By default, ITC uses the original ROS 1 M2H model path:

```text
perception_backend: m2h
perception_dataset: itc
M2H weights: mono_hydra_perception/weights/m2h_indoor.pt
model input size: 256x256
depth_output_scale: 0.967
skip_frequency: 5
Hydra LCD config: mono_hydra/config/hydra_itc_lcd.yaml
```

`perception_backend:=torch` still launches the newer M2H-HMX-Large research
model, but it is not the ITC production default because the ROS 1 benchmark used
the original `m2h` package.

For ITC, RVIO2 uses the raw RGB stream, while Kimera RGBD and Hydra consume the
M2H-synchronized RGB/CameraInfo stream:

```text
RVIO2 image input: /camera/color/image_raw
M2H depth output: /camera/depth_cam/image_raw
M2H semantic output: /camera/seg_cam/labels_argmax
Synced RGB for Kimera/Hydra: /mono_hydra_perception/synced/image_raw
Synced CameraInfo for Hydra: /mono_hydra_perception/synced/camera_info
```

The default TF ownership follows the ROS 1 ITC launch: the RVIO2 adapter only
creates `/external_odometry` and `/mono_hydra_vio/odometry`; Kimera publishes
`odom -> base_link_kimera`; Kimera LCD publishes `map -> odom`; and Kimera
publishes `base_link_kimera -> left_cam_kimera`. The static `map -> odom`
helper is disabled for ITC so it cannot mask the LCD correction.

The ITC ROS 2 defaults preserve dense ROS 1 parity: M2H reports output lag but
does not drop already-processed RGB-D/semantic frames, Hydra's RGB-D receiver
queue and Kimera sync queues are large enough to avoid starving mesh
integration, and RViz uses the native `kimera_pgmo_rviz/MeshDisplay` plugin on
Hydra's optimized backend mesh. The compiled mesh-marker bridge remains
available only as an explicit fallback, not as part of the default ITC
production run. Use a positive `perception_max_output_lag_s` only for
low-latency debug views where incomplete/sparser mesh output is acceptable.

### ROS 1-Style Four-Terminal Manual Run

Use this when you want the same control split as the ROS 1 launch stack. Start
Terminals 1-3 first, then start the bag paused in Terminal 4 and unpause when
the nodes are ready. These commands run at recorded time; there is no `--rate`
throttling. The small launch wrappers below hide component on/off plumbing so
each terminal only exposes the knobs that belong to that component.

Terminal 1: Hydra + RViz

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_hydra.launch.py \
  sequence_name:=itc_ros1_style_manual
```

Hydra loads `mono_hydra/config/hydra_itc_topics.yaml`; its backend keeps
`optimize_on_lc: true`, matching the ROS 1 `optimize_on_lc:=true` behavior.
RViz opens by default from this terminal.

Terminal 2: RVIO2 -> external odom -> Kimera RGBD

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_vio.launch.py \
  sequence_name:=itc_ros1_style_manual \
  enable_lcd:=true \
  kimera_lcd_no_optimize:=true \
  rvio2_input_pose_frame:=body \
  odom_adapter_publish_tf:=false
```

`odom_adapter_publish_tf:=false` is the ROS 2 equivalent of the ROS 1
`rvio2_publish_tf:=false`; Kimera owns the odom TF tree and publishes the
Kimera TF links by default.

Terminal 3: M2H HMX-Large

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_m2h_hmx_large.launch.py \
  dataset:=itc \
  device:=auto \
  half:=true \
  skip_frequency:=3
```

This mirrors the explicit ROS 1 HMX-Large command. The launch wrapper keeps the
ITC topics, simulated time, 640x480 output, `depth_scale:=1.0`, and synchronized
RGB/CameraInfo publication fixed for the ITC stack.

Terminal 4: bag replay

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 bag play test_data/itc_ros2_bags/ITC_2nd_floor_full_loop_ros2 \
  --clock --start-paused \
  --qos-profile-overrides-path \
  "$(ros2 pkg prefix mono_hydra_utils)/share/mono_hydra_utils/config/tf_overrides.yaml" \
  --remap /tf:=/tf_ignore /tf_static:=/tf_static_ignore
```

### Minimized ITC Run

Use this only after the four-terminal run is behaving. It starts the stack in
one launch and opens RViz by default.

Terminal 1:

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 launch mono_hydra mono_hydra_itc_rosbag.launch.py \
  sequence_name:=itc_ros1_m2h_realtime
```

Terminal 2:

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
$(ros2 pkg prefix mono_hydra_utils)/share/mono_hydra_utils/scripts/play_itc_full_loop_ros2.sh
```

The reference commands use normal bag timing. ONNX remains opt-in via
`perception_backend:=onnx`; HMX-Large remains opt-in via
`perception_backend:=torch`.

## ScanNet Run

The ROS 1 ScanNet bag has been converted locally to:

```text
test_data/scannet_ros2_bags/scene0000_00_ros2
```

Terminal 1: ScanNet stack + RViz

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 launch mono_hydra mono_hydra_scannet.launch.py \
  sequence_name:=scene0000_00 \
  hydra_exit_after_clock:=true
```

Terminal 2: bag replay

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 bag play test_data/scannet_ros2_bags/scene0000_00_ros2 \
  --clock --start-paused \
  --remap /tf:=/tf_ignore /tf_static:=/tf_static_ignore
```

The ScanNet launch treats `/external_odometry` as a camera-pose input and
normalizes it to `scannet_world -> base_link_kimera`, while keeping
`base_link_kimera -> scannet_camera` as a static extrinsic. This avoids the
camera-frame TF conflict that leaves Hydra waiting for sensor extrinsics and
prevents RViz from showing the DSG.

## uHumans2 Run

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 launch mono_hydra mono_hydra_uhumans.launch.py \
  use_sparse_depth_factors:=true \
  perception_backend:=torch
```

For a headless run, add `use_rviz:=false visualize:=false`.

Temporal alignment uses the same default RViz/native-mesh path:

```bash
ros2 launch mono_hydra mono_hydra_uhumans.launch.py \
  use_sparse_depth_factors:=true perception_backend:=torch \
  use_temporal_alignment:=true temporal_history_size:=3 \
  temporal_label_min_votes:=2
```

Play the matching uHumans2 ROS 2 bag with `--clock` and the same TF QoS
override file.

## 7Scenes Run

```bash
cd /home/bavantha/ros1_workspaces/ros2_jazzy_ws
source install/setup.bash
ros2 launch mono_hydra mono_hydra_7scenes.launch.py \
  perception_backend:=torch
```

7Scenes keeps the dataset/external-odometry path until a calibrated RVIO2
camera/IMU profile is added. For a headless run, add
`use_rviz:=false visualize:=false`.

Temporal alignment can be enabled in the same way:

```bash
ros2 launch mono_hydra mono_hydra_7scenes.launch.py \
  perception_backend:=torch use_temporal_alignment:=true
```

## RViz

The ROS 2 visualizer config is:

```text
mono_hydra/rviz/mono_hydra_ros2.rviz
```

The documented dataset commands leave `use_rviz` at its default `true`, so they
start `rviz2` with this config and also start the official Hydra DSG marker
visualizer. The mesh display is the native `kimera_pgmo_rviz/MeshDisplay` on
`/hydra/backend/dsg_mesh`; the marker conversion bridge is not part of the
default path. The Hydra visualizer publishes:

```text
/hydra_visualizer/graph
/hydra_visualizer/agent_poses
```

The default panel layout mirrors the ROS 1 visualizer flow:

```text
left dock      Displays/config tree, expanded at Scene Graph, Input, VIO, Kimera Pose Graph
center         3D map view in the map frame
right dock     RGB, Depth, and Semantics image panels
bottom dock    Time panel synchronized to RGB
```

TF is available but disabled by default to keep the DSG readable. The factor
graph deformation-edge group is also available but disabled by default, matching
the ROS 1 configs where it was used as an inspection layer rather than the main
view.

The Mono Hydra RViz config shows the ROS 1 visualizer-equivalent inputs and
outputs:

```text
/camera/color/image_raw
/camera/depth_cam/image_raw
/camera/seg_cam/labels_argmax
/camera/seg_cam/image_raw
/mono_hydra_vio/path
/mono_hydra_vio/odometry
/rvio2/trajectory
/hydra_dsg_visualizer/dsg_markers
/hydra_dsg_visualizer/agent_poses
/hydra/backend/dsg_mesh
/hydra_dsg_visualizer/dsg_mesh_marker   optional marker fallback
/mono_hydra_vio_ros/graph_nodes
/mono_hydra_vio_ros/graph_nodes_ids
/mono_hydra_vio_ros/odometry_edges
/mono_hydra_vio_ros/loop_edges
/mono_hydra_vio_ros/rejected_loop_edges
/mono_hydra_vio_ros/optimized_trajectory
/incremental_dsg_builder_node/pgmo/deformation_graph_mesh_mesh
/incremental_dsg_builder_node/pgmo/deformation_graph_pose_mesh
```

Dataset launch files remap the input image topics into this config, so the same
RViz file works for ITC, ScanNet, 7Scenes, and uHumans2.

When playing converted ROS 1 dataset bags, keep the bag TF out of the stack TF
tree. Otherwise the dataset camera frame can conflict with
`base_link_kimera -> camera_color_optical_frame`, Hydra cannot resolve camera
extrinsics, and the DSG marker topic stays empty.

The original ROS 1 RViz files are retained as reference assets:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix mono_hydra)/share/mono_hydra/rviz/scannet.rviz"
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix mono_hydra)/share/mono_hydra/rviz/uHumans_m2h.rviz"
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix mono_hydra_vio)/share/mono_hydra_vio/rviz/mono_hydra_vio_tesse.rviz"
```

## Topic Checks

```bash
ros2 topic hz /camera/depth_cam/image_raw
ros2 topic hz /camera/seg_cam/labels_argmax
ros2 topic hz /mono_hydra_perception/synced/image_raw
ros2 topic hz /mono_hydra_perception/synced/camera_info
ros2 topic hz /external_odometry
ros2 topic hz /mono_hydra_vio/odometry
ros2 topic echo --once /hydra_dsg_visualizer/dsg_markers \
  --qos-durability transient_local --qos-reliability reliable --no-arr
ros2 topic info -v /hydra/backend/dsg_mesh
ros2 topic list | grep pose_graph
ros2 topic info -v /mono_hydra_vio_ros/pose_graph_incremental
ros2 run tf2_tools view_frames
```

Expected Kimera pose graph topics:

```text
/mono_hydra_vio_ros/pose_graph
/mono_hydra_vio_ros/pose_graph_incremental
```

Expected startup check:

```text
/mono_hydra_vio_ros/pose_graph_incremental: Publisher count 1
```

Kimera-only smoke test:

```bash
timeout 35s ros2 launch mono_hydra mono_hydra_scannet.launch.py \
  use_hydra_backend:=false use_perception:=false use_rviz:=false \
  visualize:=false use_rvio2_backend:=false use_rvio2_bridge:=false \
  start_mesh_marker:=false use_kimera_vio_ros_node:=true
ros2 topic info /mono_hydra_vio_ros/pose_graph_incremental
```

For the ITC bag, the first RVIO2 pose may arrive several seconds after the
first RGB frame. Give Hydra roughly 30-60 seconds of playback before judging
the DSG marker topic.

For the full-loop parity run, also check the saved counts after Hydra exits:

```bash
tail -n 1 output/<sequence_name>/backend/pgmo/dsg_pgmo_status.csv
wc -l output/<sequence_name>/vio_logs/output_lcd_result.csv
wc -l output/<sequence_name>/backend/loop_closures.csv
```

The important sanity check is `trajectory_len`: it should be in the ROS 1 range
of roughly 1690-1800 for the ITC full loop. A much smaller value means the RGB,
depth, semantic, odometry, and pose-graph streams are not staying synchronized
through the complete loop.

For a complete scene graph in RViz, the expected marker namespaces include:

```text
layer_2_*      objects
layer_3_*      places
layer_3p1_*    surface places
layer_4_*      rooms
layer_5_*      building
```

Hydra publishes the optimized backend mesh as `/hydra/backend/dsg_mesh`
(`kimera_pgmo_msgs/msg/Mesh`). This workspace now builds the native ROS 2
`kimera_pgmo_rviz/MeshDisplay` plugin from the MIT-SPARK/Kimera-PGMO `ros2`
branch, so the default RViz config renders that backend mesh directly instead
of through a marker conversion node or visualizer mesh republish.

The compiled marker bridge remains available only as a fallback:

```bash
ros2 launch mono_hydra mono_hydra_itc_hydra.launch.py start_mesh_marker:=true
```

The fallback uses `mesh_alpha:=0.92`; lower it only if the mesh starts hiding
the DSG graph.

Loop closure is valid only when the benchmarked Kimera VIO ROS node publishes
`/mono_hydra_vio_ros/pose_graph_incremental`.
If `ros2 topic info -v /mono_hydra_vio_ros/pose_graph_incremental` reports
`Publisher count: 0`, Hydra has no Kimera loop-closure proposal to optimize.
The ITC rosbag launch defaults to the ROS 1 production perception path:
`perception_backend:=m2h`, `m2h_indoor.pt`, `256x256`,
`m2h_depth_output_scale:=0.967`, `perception_skip_frequency:=5`, Kimera
`RealSense_RGBD_RVIO2`,
Hydra `RosPoseGraphs` under `/mono_hydra_vio_ros`, and
`use_kimera_external_lc_bridge:=false`. It also loads the ROS 1 ITC Hydra LCD
profile from `hydra_itc_lcd.yaml` instead of the generic default LCD profile.
The ONNX and HMX-Large backends remain available only when explicitly provided.
`hydra_exit_after_clock:=true` lets Hydra save `backend/loop_closures.csv`
after the bag clock finishes. A manual `Ctrl-C` during backend optimization can
kill Hydra before it writes the final CSV.

Validated full-loop ITC result after restoring the ROS 1 pose-graph remap:

```text
sequence: output/itc_rosbag_lc_cuda_184805
Kimera LCD rows: 535
Kimera accepted loop closures: 150
Hydra backend loop_closures.csv rows: 300
Hydra PGMO status rows with nonzero loop-closure counts: 1090
first Hydra loop timestamp: 1681748736976062775
last Hydra loop timestamp: 1681748978084907293
```

The critical fix is the ROS 1-equivalent remap:

```text
/mono_hydra_vio_ros/pose_graph -> /mono_hydra_vio_ros/pose_graph_incremental
```

Without this remap, Hydra subscribes to the full pose graph stream, logs
duplicate/invalid edge warnings, and saves an empty `backend/loop_closures.csv`
even though Kimera VIO LCD detected loops.

The launch also exports pip-installed NVIDIA CUDA library directories to the
ONNX node. In this workspace the ONNX provider check now reports:

```text
providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
average inference: 17 ms
```

Latest full-model ITC diagnostic run before the synchronized-perception routing
fix:

```text
sequence: output/itc_ros1_parity_torch_lcd_201201
Kimera VIO LCD rows: 247
Kimera accepted loop closures: 48
Hydra backend loop_closures.csv rows: 255
Hydra PGMO final total_lc: 207
last saved Hydra loop timestamp: 1681748977851407528
backend mesh: 212 MB
backend dsg_with_mesh: 309 MB
rooms layer: 9 nodes
building layer: 1 node
```

That run is kept as a failure reference: Kimera and Hydra did receive loop
closures, but Hydra saved only `trajectory_len=247`, far below the ROS 1
full-loop reference. The updated launch routes Kimera/Hydra through
`/mono_hydra_perception/synced/*`; after re-running, compare against
`ITC_PARITY_ANALYSIS.md`.

## Config Equivalence Checklist

- `lcd_no_optimize:=true` for RVIO2 + Kimera proposal-only runs.
- `use_lcd:=true`.
- `use_external_odom:=true`.
- RVIO2 bridge input pose frame defaults to `body`, matching the ROS 1 launch.
- ITC Kimera params: `RealSense_RGBD_RVIO2`.
- uHumans2 RVIO2/Kimera params:
  `uHumans2_RGBD_RVIO2_depth_factors_off` or
  `uHumans2_RGBD_RVIO2_depth_factors_on`.
- Hydra `frontend.pose_graph_tracker.ns` points to `/mono_hydra_vio_ros`.
- The ROS 2 launch remaps Hydra's `/mono_hydra_vio_ros/pose_graph`
  subscription to `/mono_hydra_vio_ros/pose_graph_incremental`, matching the
  ROS 1 `hydra.launch` remap.
- Hydra backend keeps `optimize_on_lc: true`.
- ITC, 7Scenes, and uHumans route Kimera/Hydra RGB input through
  `/mono_hydra_perception/synced/image_raw` so delayed full-model depth/labels
  stay paired with the exact RGB frame.
- ITC dense replay leaves `perception_max_output_lag_s:=0.0`, keeps large
  Kimera/Hydra synchronization queues, and warns if M2H falls behind instead of
  silently dropping RGB-D frames that Hydra needs for a complete mesh.
- Sparse depth, semantic masking, SuperPoint mask input, Kimera temporal
  calibration, and temporal pose-warp filtering all remain launch-time switches
  for ablation runs.
