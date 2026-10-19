# Optional PTC-Depth refinement for M2H

This uses the released [PTC-Depth](https://github.com/ajou-arrl/ptc-depth) C++ core
at revision `6f071370396bf2fd50b4bd6815f0a5012b06e119`. M2H remains the depth and
semantic model. No replacement model or weights are installed.

PTC is an optional experiment, disabled by default. It receives synchronized raw
M2H predictions and camera poses on a separate Hydra-only branch. Kimera still
receives raw M2H depth and publishes loop edges to Hydra PGMO. Set `use_temporal_alignment:=false` when enabling PTC.

## Install and build

Inside the ROS container, from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
bash src/mono_hydra_perception/scripts/install_ptc_depth.sh
# Source the env.sh path printed by that command.
colcon build --symlink-install --packages-select mono_hydra_perception mono_hydra --parallel-workers 2
source install/setup.bash
```

Requires the existing NumPy, OpenCV Python/C++ development files, Eigen, a C++
compiler and pip. Pip installs the small build tools in isolation and builds
only PTC; `--no-deps` avoids replacing the ROS OpenCV/NumPy environment. The
installer uses a revision/patch/Python/architecture-specific directory outside
the source repository. It leaves previous installations untouched. The optional
PTC source is BSD-3-Clause; its license remains in the cloned source directory.

## ITC launch

Add these arguments to the existing working ITC command:

```bash
use_ptc_refinement:=true \
ptc_python_path:="$PTC_DEPTH_PYTHONPATH" \
ptc_width:=424 ptc_local:=true ptc_max_scale_change:=0.10 \
perception_target_fps:=0.0 use_temporal_alignment:=false
```

`perception_target_fps` enables a single inference worker selected by source
timestamps, with one pending latest frame, for the Torch and ONNX M2H wrappers.
Zero preserves the legacy `skip_frequency` behavior. It is a requested cadence,
not a throughput guarantee. Inference logs expose received, selected, completed,
replaced and failed callback counts to distinguish scheduling from input loss.
RGB and CameraInfo are retained with their selected frame. This scheduler is independent of PTC and can be tested with refinement off.

Hydra receives `/mono_hydra_refinement/{rgb,depth,labels}`. Its CameraInfo stays on
the original synchronized M2H topic so Hydra can initialize its TF listener
before the delayed first refined frame. The original M2H topics are unchanged.
PTC preserves the source stamp, uses optical camera
poses in `odom`, and republishes the four images/info as one synchronized bundle.
The fixed-size input queue and a separate worker prevent a TF wait from blocking
raw perception or Kimera. Missing poses and invalid/unsupported corrections pass
through raw depth; queue overflow is counted explicitly. Per-frame statistics
are written to `<output_root>/<sequence_name>/ptc_metrics.jsonl` without overwriting.

## Adapter behavior and upstream corrections

- Call the native core with inverse metric M2H depth, bypassing the upstream
  Python wrapper's per-frame maximum normalization.
- Use RVIO-derived camera-to-camera transforms; do not estimate another pose.
- Hold a useful past reference until enough translation is available. Reset on
  calibration changes, timestamp rewind, excessive gaps or pose jumps.
- Compute forward/backward DIS correspondences, with texture, parallax,
  visibility and geometric consistency checks. Spatial validation blocks are
  withheld from PTC's current triangulation input.
- PTC supplies global or connected-region scale proposals. The adapter bounds
  their effect and accepts only regions improving held-out median and tail
  reprojection errors. Local regions are separated by semantic/depth boundaries;
  a semantic class is not treated as one physical object.
- Back off a proposed correction when it increases median or tail disagreement
  with the previously emitted surface by more than 5 mm. This is an acceptance
  check, not temporal depth averaging or a frame-dropping filter. It can reject
  a valid abrupt scale correction when the existing surface is biased, and
  fallback transitions can still increase disagreement between some frames.
- Upsample scale corrections instead of depth, retaining raw M2H detail and
  invalid pixels. Integrate the current observation once, not an accumulated
  posterior replayed as independent measurements. Existing Hydra robust TSDF
  weighting remains active.

The small tracked `third_party/ptc_depth.patch` fixes the triangulator's Sampson
matrix direction (triangulation uses the reverse transform), skips nonfinite
flow before integer projection in depth warping, and releases the Python GIL
during native refinement. The installer runs a rotated-camera regression for
the transform fix. It does not replace PTC's triangulation or Bayesian fusion.

## Verify and revert

```bash
PYTHONPATH="$PTC_DEPTH_PYTHONPATH:${PYTHONPATH:-}" OMP_NUM_THREADS=2 \
  python3 -m pytest -q src/mono_hydra_perception/test
```

Compare fixed M2H/RGB/pose inputs with refinement off, global only, and local
corrections. Report coverage and metric error where reference geometry exists,
not just consistency among retained pixels. Test a complete loop run and inspect
PGMO latency as well as perception timing. Pose/flow errors and poor parallax can
make triangulation unreliable; this is not a guarantee of flat floors, complete
doors, or ground-truth object identities.

The initial ITC capture trial did **not** demonstrate improved metric geometry.
The native core substantially reduced scale; the guarded local adapter's
common-support depth discrepancy was 6.16 cm versus 5.91 cm for raw M2H.
These are consistency measurements without ground truth, not accuracy scores.
Keep refinement opt-in while collecting metric validation. Test the optional
`perception_target_fps:=5.0` scheduler separately from this geometry experiment;
zero retains the existing cadence for the first comparison. Jetson performance
has not been measured.

To restore the previous flow, use `use_ptc_refinement:=false
perception_target_fps:=0.0`. No TSDF/PGMO code change is needed. The PTC dependency
can remain installed and unused. Machine-specific checkpoint and replay results
are documented separately in the workspace's diagnostic output directory.
