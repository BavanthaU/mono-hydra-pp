# Mapping components and tuning

Geometric reliability weighting and temporal object association are enabled by
default in the ITC launch. Disable either with
`revision_fusion_weighting:=false` or `revision_object_association:=false`.

## Geometric reliability in fusion

`GeometricReliability` runs once per received reconstruction input, before parallel voxel integration. It compares current axial depth with a previous posed depth image. Smooth, visible, temporally ordered correspondences receive a bounded range-aware consistency weight. Image footprint variation broadens the tolerance on sloping surfaces. The optional image multiplies the existing sensor weight after its minimum-weight floor and before the existing robust TSDF influence.

Unknown visibility, large occlusion/disocclusion residuals, depth boundaries, missing samples, insufficient translation, stale history, or excessive motion preserve the existing sensor weight. History resets on clock rewind or sensor change. A small-baseline reference is retained until useful translation accumulates. This does not drop depth frames or replay a temporal posterior. The existing raw prediction still enters the TSDF once.

This is **a bounded projected-depth consistency score**, not a calibrated variance, independent triangulation measurement, or correlation-aware Bayesian estimator. It cannot detect a stable bias shared by successive predictions and it does not explicitly account for pose covariance. Large residuals are deliberately treated as unknown to preserve new-surface discovery.

Configuration lives under `active_window.geometric_reliability`: `enabled`, `min_factor` (0.2), `scale_m` (0.05), `scale_relative` (0.03), `min_baseline_m` (0.02), `max_baseline_m` (0.8), and `max_age_s` (1.5). Timing is logged as `reconstruction/geometric_reliability`; verbosity 2 logs supported pixel counts and mean weight.

## Temporal object association

With `frontend.objects.use_temporal_overlap`, surviving shared mesh vertices provide direct identity support. Geometry-only matches require same-class objects, a bounded 3D box gap, current-depth visibility, projected footprint overlap, and three consecutive distinct-timestamp confirmations. Projection work is capped at 512 sampled vertices per object. Competing matches are not merged. Evidence expires and is cleared on clock rewind. Explicitly initialize cluster centroids to zero.

The launch also enables `backend.update_functors.objects.require_mesh_overlap`, so a subsequent centroid-only backend merge cannot bypass the frontend confirmation rule. PGMO trajectory and mesh optimization remain active.

This is active-window association, not an image-feature tracker or a complete persistent dynamic-object system. It can retain a single identity across nearby overlapping fragments; it does not fill missing geometry. Fully disjoint fragments, fully archived objects, motion, closely overlapping same-class objects, and incorrect depth visibility remain limitations. A backend merge across a loop now needs shared mesh support in this mode; evaluate the resulting recall/precision tradeoff.

Relevant parameters under `frontend.objects`: `association_max_gap_m` (0.2), `association_depth_tolerance_m` (0.2), `association_min_overlap` (0.6), `association_max_age_s` (2.0), and `association_confirmations` (3). Verbosity 2 logs confirmed `TemporalObjectMerge` events.

## Multiple-reference PTC (experimental)

`MultiReferencePtc` keeps at most six raw RGB/depth/label/pose records, selects two references using projected overlap and rotation-compensated parallax, and requires separation between the selected camera centers. Each pair initializes a separate released PTC core; its internal posterior is never switched to an unrelated reference. Corrections must be supported by both pairs, agree in sign and within 0.03 log-scale, remain within the existing correction bound, and pass both pairs' withheld spatial reprojection checks. Unsupported pixels and frames retain raw M2H depth. The current output is published once at its original timestamp.

This checks two supporting views; it does not claim a third independent validation view. Their poses and model errors can remain correlated. The default temporal guard remains enabled. Compute can approach two pair refinements per frame, and improvements on the captured real sequence have not been established. Attribute the native method to [PTC-Depth](https://github.com/ajou-arrl/ptc-depth); the adapter and acceptance logic are separate modifications.
