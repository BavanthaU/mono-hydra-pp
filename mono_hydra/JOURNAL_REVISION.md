# Optional journal-revision components

All new components default **off**. Keep the frozen M2H prediction model, RVIO poses and Kimera LCD → Hydra PGMO. No training, plane constraints, alternate global optimizer, or new TF wait was added to Hydra. Dataset and journal-result reruns are owned by the researcher; implementation regression checks are separate from scientific validation.

## Switches

Append these arguments to the existing ITC launch:

```bash
revision_fusion_weighting:=true \
revision_object_association:=true
```

PTC retains its existing single-reference behavior with `ptc_references:=1`. The bounded experimental two-reference path requires both:

```bash
use_ptc_refinement:=true ptc_references:=2
```

The existing optional PTC native installation and `PTC_DEPTH_PYTHONPATH` are required for either PTC mode. No additional model weights are used. Leave `use_temporal_alignment:=false` when using PTC. Keep `enable_lcd:=true enable_hydra_lcd:=false kimera_lcd_no_optimize:=true` for the existing ITC ownership.

## What changed

### Geometric reliability in fusion

`GeometricReliability` runs once per received reconstruction input, before parallel voxel integration. It compares current axial depth with a previous posed depth image. Smooth, visible, temporally ordered correspondences receive a bounded range-aware consistency weight. Image footprint variation broadens the tolerance on sloping surfaces. The optional image multiplies the existing sensor weight after its minimum-weight floor and before the existing robust TSDF influence.

Unknown visibility, large occlusion/disocclusion residuals, depth boundaries, missing samples, insufficient translation, stale history, or excessive motion preserve the existing sensor weight. History resets on clock rewind or sensor change. A small-baseline reference is retained until useful translation accumulates. This does not drop depth frames or replay a temporal posterior. The existing raw prediction still enters the TSDF once.

This is **a bounded projected-depth consistency score**, not a calibrated variance, independent triangulation measurement, or correlation-aware Bayesian estimator. It cannot detect a stable bias shared by successive predictions and it does not explicitly account for pose covariance. Large residuals are deliberately treated as unknown to preserve new-surface discovery. These limits must remain explicit in the paper.

Configuration lives under `active_window.geometric_reliability`: `enabled`, `min_factor` (0.2), `scale_m` (0.05), `scale_relative` (0.03), `min_baseline_m` (0.02), `max_baseline_m` (0.8), and `max_age_s` (1.5). Defaults are starting points, not dataset-validated optima. Timing is logged as `reconstruction/geometric_reliability`; verbosity 2 logs supported pixel counts and mean weight.

### Modest temporal object association

With `frontend.objects.use_temporal_overlap`, surviving shared mesh vertices provide direct identity support. Geometry-only matches require same-class objects, a bounded 3D box gap, current-depth visibility, projected footprint overlap, and three consecutive distinct-timestamp confirmations. Projection work is capped at 512 sampled vertices per object. Competing matches are not merged. Evidence expires and is cleared on clock rewind. Explicitly initialize cluster centroids to zero.

The launch also enables `backend.update_functors.objects.require_mesh_overlap`, so a subsequent centroid-only backend merge cannot bypass the frontend confirmation rule. PGMO trajectory and mesh optimization remain active.

This is active-window association, not an image-feature tracker or a complete persistent dynamic-object system. It can retain a single identity across nearby overlapping fragments; it does not fill missing geometry. Fully disjoint fragments, fully archived objects, motion, closely overlapping same-class objects, and incorrect depth visibility remain limitations. A backend merge across a loop now needs shared mesh support in this mode; evaluate the resulting recall/precision tradeoff.

Relevant parameters under `frontend.objects`: `association_max_gap_m` (0.2), `association_depth_tolerance_m` (0.2), `association_min_overlap` (0.6), `association_max_age_s` (2.0), and `association_confirmations` (3). Verbosity 2 logs confirmed `TemporalObjectMerge` events.

### Multiple-reference PTC (experimental)

`MultiReferencePtc` keeps at most six raw RGB/depth/label/pose records, selects two references using projected overlap and rotation-compensated parallax, and requires separation between the selected camera centers. Each pair initializes a separate released PTC core; its internal posterior is never switched to an unrelated reference. Corrections must be supported by both pairs, agree in sign and within 0.03 log-scale, remain within the existing correction bound, and pass both pairs' withheld spatial reprojection checks. Unsupported pixels and frames retain raw M2H depth. The current output is published once at its original timestamp.

This checks two supporting views; it does not claim a third independent validation view. Their poses and model errors can remain correlated. The default temporal guard remains enabled. Compute can approach two pair refinements per frame, and improvements on the captured real sequence have not been established. Keep this option conditional on your evaluation. Attribute the native method to [PTC-Depth](https://github.com/ajou-arrl/ptc-depth); the adapter and acceptance logic are separate modifications.

## Correctness baseline and regression checks

Previously implemented fixes remain: exact source timestamps, bounded robust TSDF weighting, refreshed object geometry after mesh remapping, enclosing AABBs in ITC, separated LCD switches, and detection-only Kimera loop reporting. The LCD exclusion window is 30 keyframes and accepted loops require at least 10 3D inliers. These are not ground-truth guarantees. The historical 50/87 reconstruction publication count did not mean 37 discarded input depths.

The affected Hydra/Hydra ROS/perception/launch packages were built. Focused C++ checks cover reliability weighting reaching voxel measurements, axial-depth translation, reset/fallback behavior, existing object remapping, repeated-overlap confirmation and occlusion rejection. Python checks exercise the actual patched native PTC, bounded two-view corrections, unchanged inputs, stationary fallback and rewind. Passing these establishes implementation behavior, not improved map accuracy.

## Running later evaluations

Use unique output sequence names. Compare corrected raw-depth baseline, reliability alone, object association alone, and both. Compare single- and two-reference PTC separately before combining them with other changes. Keep inputs, poses, source cadence, calibration, semantic label space and mapping parameters fixed when isolating components.

`scripts/run_revision_ablation.py` replays a **normalized ITC prediction/pose capture** into an isolated ROS domain and saves commands, stage logs, process CPU/RSS samples, and exports. It does not run perception or VIO and therefore does not measure whole-stack throughput. Example in a sourced ROS2 environment, with paths appropriate to your workspace:

```bash
python3 src/mono_hydra/scripts/run_revision_ablation.py \
  --workspace "$PWD" --capture debug_dumps/itc_fusion_live_20260913_0607 \
  --output debug_dumps/my_unique_fusion_run --variant fusion
```

`scripts/evaluate_revision.py` runs in the existing NumPy/SciPy/Open3D evaluation environment. It reads exports without changing them and refuses an existing output filename. It reports stage timing distributions and mesh ownership; optional GT enables sampled surface accuracy/completeness/F-score and ScanNet instance-support fragmentation/false-merge candidates. Provide a separately established rigid transform and a GT mesh with matching observed coverage. It rejects scale transformations. Semantic correctness and temporal ID-switch evaluation are not supplied by these static geometric instance metrics.

```bash
python3 src/mono_hydra/scripts/evaluate_revision.py \
  --run output/YOUR_RUN --output debug_dumps/YOUR_NEW_METRICS.json \
  --gt-mesh /path/to/gt.ply --transform /path/to/fixed_pred_to_gt.txt
```

Do not infer metric improvement from a smoother mesh, fewer objects, or lower self-consistency residual alone. Rerun affected scientific comparisons before updating manuscript claims. Existing archived experiment data and result files were not modified.

## Reverting

The quickest behavioral rollback is to omit the two `revision_*` switches and set `ptc_references:=1`, retaining the previously working PTC command. To restore the exact pre-edit source, use the guarded checkpoint in the workspace's `debug_dumps/journal_revision_20260913_01`. That backup includes the pre-existing uncommitted work; a Git reset would not preserve it.
