"""Bounded adapter for the released PTC-Depth core and metric M2H predictions.

PTC supplies the scale proposal. Independent held-out image matches decide where
to apply it. Only the current raw M2H observation is emitted, never a replay of
the accumulated posterior as an independent TSDF measurement.
"""
from dataclasses import dataclass
import math
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class RefinementConfig:
    width: int = 424
    min_baseline: float = 0.05
    max_baseline: float = 0.8
    max_age_s: float = 1.5
    max_depth: float = 10.0
    min_parallax_deg: float = 1.0
    max_flow_error: float = 0.75
    min_improvement_px: float = 0.05
    min_improvement_fraction: float = 0.10
    max_scale_change: float = 0.10
    min_train: int = 20
    min_validation: int = 8
    cell_size: int = 40
    local: bool = True
    flow_preset: str = 'fast'
    temporal_check: bool = True
    max_temporal_increase_m: float = 0.005

    def __post_init__(self):
        if (self.width < 64 or self.min_baseline <= 0 or
                self.max_baseline <= self.min_baseline or self.max_age_s <= 0 or
                self.max_depth <= 0 or not 0 < self.max_scale_change < 1 or
                self.min_parallax_deg <= 0 or self.max_flow_error <= 0 or
                self.min_improvement_px < 0 or
                not 0 <= self.min_improvement_fraction < 1 or
                self.min_train < 3 or self.min_validation < 3 or self.cell_size < 8 or
                self.flow_preset not in ('fast', 'medium') or self.max_temporal_increase_m < 0):
            raise ValueError('Invalid PTC refinement configuration')


def scaled_intrinsics(K, source_shape, target_shape):
    """OpenCV resize pixel-center convention, including anisotropic scaling."""
    sy, sx = np.asarray(target_shape) / np.asarray(source_shape)
    out = np.array(K, dtype=np.float64, copy=True)
    out[0] *= sx
    out[1] *= sy
    out[0, 2] += (sx - 1) / 2
    out[1, 2] += (sy - 1) / 2
    return out


def valid_pose(T):
    T = np.asarray(T)
    return (T.shape == (4, 4) and np.isfinite(T).all() and
            np.allclose(T[3], [0, 0, 0, 1], atol=1e-6) and
            np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-4) and
            np.linalg.det(T[:3, :3]) > 0.999)


def _remap(array, x, y, interpolation=cv2.INTER_LINEAR):
    return cv2.remap(array, x.astype(np.float32), y.astype(np.float32),
                     interpolation, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def _regions(depth, labels, gray, cell_size):
    """Connected small regions; class IDs alone must never join separate doors."""
    valid = np.isfinite(depth) & (depth > 0)
    edge = np.zeros(depth.shape, dtype=bool)
    for axis in (0, 1):
        shifted = np.roll(depth, 1, axis)
        edge |= np.abs(depth - shifted) > 0.05 + 0.05 * np.minimum(depth, shifted)
        edge |= labels != np.roll(labels, 1, axis)
    # Texture edges alone are not necessarily geometric boundaries.
    edge = cv2.dilate(edge.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    valid &= ~edge
    valid[::cell_size, :] = False
    valid[:, ::cell_size] = False
    _, components = cv2.connectedComponents(valid.astype(np.uint8), connectivity=4)
    return np.ascontiguousarray(components.astype(np.int32) - 1)


def reprojection_error(depth, x1, y1, x0, y0, K, T_current_previous):
    """Current axial depth projected back into the independent reference image."""
    rays = np.stack(((x1 - K[0, 2]) / K[0, 0],
                     (y1 - K[1, 2]) / K[1, 1], np.ones_like(x1)), axis=-1)
    points = rays * depth[..., None]
    prev = (points - T_current_previous[:3, 3]) @ T_current_previous[:3, :3]
    z = prev[..., 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = K[0, 0] * prev[..., 0] / z + K[0, 2]
        v = K[1, 1] * prev[..., 1] / z + K[1, 2]
    error = np.hypot(u - x0, v - y0)
    return np.where(np.isfinite(error) & (z > 0), error, np.inf)


class PtcRefiner:
    def __init__(self, config=RefinementConfig(), core_module=None):
        # Optional dependency: importing this module does not load another model.
        if core_module is None:
            import ptc_depth_cpp as core_module
        self.core_module = core_module
        self.config = config
        preset = (cv2.DISOPTICAL_FLOW_PRESET_FAST if config.flow_preset == 'fast'
                  else cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
        self.forward = cv2.DISOpticalFlow_create(preset)
        self.backward = cv2.DISOpticalFlow_create(preset)
        self.reset()

    def reset(self):
        self.core = None
        self.reference = None
        self.geometry = None
        self.validation_context = None

    def validate_candidate(self, candidate):
        """Check a combined proposal on this pair's withheld spatial samples."""
        if self.validation_context is None:
            return {'accepted': False, 'samples': 0}
        shape, x1, y1, x0, y0, k, relative, before_depth, mask = self.validation_context
        depth = cv2.resize(candidate, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
        samples = _remap(depth, x1, y1, cv2.INTER_NEAREST)
        changed = mask & np.isfinite(samples) & (samples > 0) & (np.abs(samples-before_depth) > 1e-5)
        count = int(changed.sum())
        if count < 24:
            return {'accepted': False, 'samples': count}
        before = reprojection_error(before_depth, x1, y1, x0, y0, k, relative)[changed]
        after = reprojection_error(samples, x1, y1, x0, y0, k, relative)[changed]
        finite = np.isfinite(before).all() and np.isfinite(after).all()
        if not finite:
            return {'accepted': False, 'samples': count}
        accepted = (np.median(after) < np.median(before)-0.01 and
                    np.percentile(after, 90) <= np.percentile(before, 90)+0.01)
        return dict(accepted=bool(accepted), samples=count,
                    before_px=float(np.median(before)), after_px=float(np.median(after)))

    def _initialize(self, gray, depth, K, pose, stamp):
        cfg = self.core_module.PTCDepthConfig()
        cfg.H, cfg.W = depth.shape
        cfg.fx, cfg.fy, cfg.cx, cfg.cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        cfg.outdoor = False
        cfg.verbose = False
        cfg.max_depth = self.config.max_depth
        cfg.min_baseline = self.config.min_baseline
        cfg.iterative = 0
        self.core = self.core_module.PTCDepth(cfg)
        self._call_core(gray, depth, np.eye(4), None, None)
        self.reference = (gray.copy(), depth.copy(), pose.copy(), stamp, depth.copy())

    def _call_core(self, gray, depth, relative, segments, flow):
        # Bypass the upstream Python wrapper's per-frame max normalization.
        inv = np.divide(1.0, depth, out=np.zeros_like(depth),
                        where=np.isfinite(depth) & (depth > 0))
        return self.core.refine(
            np.ascontiguousarray(gray), np.ascontiguousarray(inv),
            float(np.linalg.norm(relative[:3, 3])),
            np.ascontiguousarray(segments) if segments is not None else np.empty(0, np.int32),
            np.ascontiguousarray(relative[:3, :3]), np.ascontiguousarray(relative[:3, 3]),
            np.ascontiguousarray(flow) if flow is not None else np.empty(0, np.float32))

    def refine(self, rgb, raw_depth, labels, K, world_T_camera, stamp_ns):
        start = time.perf_counter()
        self.validation_context = None
        output = np.array(raw_depth, dtype=np.float32, copy=True)
        stats = {'stamp_ns': int(stamp_ns), 'reason': 'initialization',
                 'corrected_fraction': 0.0, 'accepted_regions': 0,
                 'validation_before_px': None, 'validation_after_px': None}

        def finish(reason):
            stats['reason'] = reason
            stats['elapsed_ms'] = (time.perf_counter() - start) * 1000
            return output, stats

        if (output.ndim != 2 or rgb.shape[:2] != output.shape or
                labels.shape != output.shape or not valid_pose(world_T_camera) or
                np.asarray(K).shape != (3, 3) or not np.isfinite(K).all() or
                K[0, 0] <= 0 or K[1, 1] <= 0):
            self.reset()
            return finish('invalid_input')
        pose = np.asarray(world_T_camera, dtype=np.float64)
        scale = min(1., self.config.width / output.shape[1])
        shape = (max(32, round(output.shape[0] * scale)), max(64, round(output.shape[1] * scale)))
        size = (shape[1], shape[0])
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) if rgb.ndim == 3 else rgb
        gray = cv2.resize(gray, size, interpolation=cv2.INTER_AREA)
        depth = cv2.resize(output, size, interpolation=cv2.INTER_NEAREST)
        semantic = cv2.resize(labels.astype(np.float32), size, interpolation=cv2.INTER_NEAREST).astype(np.int32)
        k = scaled_intrinsics(K, output.shape, shape)
        geometry = (output.shape, tuple(np.asarray(K).ravel()))
        if self.reference is None or self.geometry != geometry:
            self._initialize(gray, depth, k, pose, stamp_ns)
            self.geometry = geometry
            return finish('initialization')
        prev_gray, prev_depth, prev_pose, prev_stamp, prev_output = self.reference
        age = (stamp_ns - prev_stamp) * 1e-9
        if age <= 0 or age > self.config.max_age_s:
            self._initialize(gray, depth, k, pose, stamp_ns)
            return finish('history_reset')
        relative = np.linalg.inv(pose) @ prev_pose
        baseline = float(np.linalg.norm(relative[:3, 3]))
        stats['baseline_m'] = baseline
        if baseline < self.config.min_baseline:
            # Keep the reference so translation can accumulate; never emit a
            # warped posterior while stationary as another independent depth.
            return finish('low_baseline')
        if baseline > self.config.max_baseline:
            self._initialize(gray, depth, k, pose, stamp_ns)
            return finish('pose_jump')

        fwd = self.forward.calc(prev_gray, gray, None)
        bwd = self.backward.calc(gray, prev_gray, None)
        y0, x0 = np.indices(shape, dtype=np.float32)
        x1, y1 = x0 + fwd[..., 0], y0 + fwd[..., 1]
        back = _remap(bwd, x1, y1)
        d1 = _remap(depth, x1, y1, cv2.INTER_NEAREST)
        valid = ((x1 >= 2) & (x1 < shape[1] - 2) & (y1 >= 2) & (y1 < shape[0] - 2) &
                 (np.linalg.norm(fwd + back, axis=2) <= self.config.max_flow_error) &
                 np.isfinite(prev_depth) & (prev_depth > 0) & np.isfinite(d1) & (d1 > 0))
        texture = cv2.Sobel(prev_gray, cv2.CV_32F, 1, 0) ** 2 + cv2.Sobel(prev_gray, cv2.CV_32F, 0, 1) ** 2
        valid &= texture > 100
        valid &= np.abs(_remap(gray.astype(np.float32), x1, y1) - prev_gray.astype(np.float32)) < 30
        ray0 = np.stack(((x0-k[0,2])/k[0,0], (y0-k[1,2])/k[1,1], np.ones(shape)), axis=-1)
        ray1 = np.stack(((x1-k[0,2])/k[0,0], (y1-k[1,2])/k[1,1], np.ones(shape)), axis=-1)
        a = ray0 @ relative[:3, :3].T
        aa, bb, ab = np.sum(a*a, axis=2), np.sum(ray1*ray1, axis=2), np.sum(a*ray1, axis=2)
        cos = np.clip(ab / np.sqrt(aa*bb), -1, 1)
        valid &= cos < math.cos(math.radians(self.config.min_parallax_deg))
        det = aa*bb - ab*ab
        e0, e1 = -np.sum(a*relative[:3,3], axis=2), np.sum(ray1*relative[:3,3], axis=2)
        z1 = np.divide(ab*e0+aa*e1, det, out=np.zeros_like(det), where=det>1e-10)
        valid &= (z1 > 0.1) & (z1 < self.config.max_depth)
        geometric_error = reprojection_error(z1, x1, y1, x0, y0, k, relative)
        valid &= geometric_error < self.config.max_flow_error
        # Held-out spatial blocks are excluded from PTC's current observations.
        validation = valid & (((x0.astype(int)//8 + y0.astype(int)//8) % 4) == 0)
        self.validation_context = (shape, x1, y1, x0, y0, k, relative, d1, validation)
        training = valid & ~validation
        stats.update(train_matches=int(training.sum()), validation_matches=int(validation.sum()))
        if training.sum() < 80 or validation.sum() < 24:
            return finish('insufficient_matches')
        segments = (_regions(depth, semantic, gray, self.config.cell_size)
                    if self.config.local else np.zeros(shape, dtype=np.int32))
        train_flow = fwd.copy()
        train_flow[~training] = np.nan
        result = self._call_core(gray, depth, relative, segments, train_flow)
        self.reference = (gray.copy(), depth.copy(), pose.copy(), stamp_ns, depth.copy())
        proposal = result['depth']
        ratio = np.divide(proposal, depth, out=np.ones_like(depth), where=np.isfinite(depth)&(depth>0))
        good_proposal = np.isfinite(proposal) & (proposal > 0) & np.isfinite(ratio) & (ratio > 0)
        log_limit = math.log1p(self.config.max_scale_change)
        log_ratio = np.where(good_proposal, np.clip(np.log(np.maximum(ratio, 1e-8)), -log_limit, log_limit), 0).astype(np.float32)
        error_before = reprojection_error(d1, x1, y1, x0, y0, k, relative)
        groups = _remap(segments.astype(np.float32), x1, y1, cv2.INTER_NEAREST).astype(np.int32)
        # A proposal must also agree with the last emitted surface. Otherwise
        # alternating accepted/rejected scale estimates create new TSDF ghosts.
        current_points = (ray0 * prev_output[..., None]) @ relative[:3, :3].T + relative[:3, 3]
        pz = current_points[..., 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            pu = np.rint(k[0,0]*current_points[...,0]/pz+k[0,2])
            pv = np.rint(k[1,1]*current_points[...,1]/pz+k[1,2])
        visible = (np.isfinite(prev_output)&(prev_output>0)&np.isfinite(pz)&(pz>0)&
                   (pu>=0)&(pu<shape[1])&(pv>=0)&(pv<shape[0]))
        warped = np.full(depth.size, np.inf, dtype=np.float32)
        np.minimum.at(warped, pv[visible].astype(int)*shape[1]+pu[visible].astype(int), pz[visible])
        warped = warped.reshape(shape)
        common = np.isfinite(warped)&np.isfinite(depth)&(depth>0)
        common &= np.abs(warped-depth) < 0.2+0.1*depth
        correction = np.zeros(shape, dtype=np.float32)
        for group in np.unique(groups[validation]):
            if group < 0:
                continue
            test = validation & (groups == group)
            train = training & (groups == group)
            if test.sum() < self.config.min_validation or train.sum() < self.config.min_train:
                continue
            pixels = (segments == group) & good_proposal
            temporal = pixels & common
            before = np.median(error_before[test])
            for gain in (1.0, 0.5, 0.25):
                sample_log = _remap(log_ratio, x1[test].reshape(-1,1), y1[test].reshape(-1,1), cv2.INTER_NEAREST).ravel()
                error_after = reprojection_error(d1[test]*np.exp(gain*sample_log),
                    x1[test],y1[test],x0[test],y0[test],k,relative)
                after = np.median(error_after)
                if (after > before*(1-self.config.min_improvement_fraction) or
                        before-after < self.config.min_improvement_px or
                        np.percentile(error_after,90) > np.percentile(error_before[test],90)):
                    continue
                if self.config.temporal_check:
                    if temporal.sum() < 30:
                        continue
                    old_error = np.abs(depth[temporal]-warped[temporal])
                    new_error = np.abs(depth[temporal]*np.exp(gain*log_ratio[temporal])-warped[temporal])
                    allowance = self.config.max_temporal_increase_m
                    if (np.median(new_error) > np.median(old_error)+allowance or
                            np.percentile(new_error,90) > np.percentile(old_error,90)+allowance):
                        continue
                correction[pixels] = gain*log_ratio[pixels]
                stats['accepted_regions'] += 1
                break
        accepted = np.abs(correction) > 1e-6
        if not accepted.any():
            self._initialize(gray, depth, k, pose, stamp_ns)
            return finish('validation_rejected')
        # Upsample corrections rather than smoothed depth to retain M2H detail.
        full_correction = cv2.resize(correction, (output.shape[1], output.shape[0]), interpolation=cv2.INTER_LINEAR)
        nearest_depth = cv2.resize(depth, (output.shape[1], output.shape[0]), interpolation=cv2.INTER_NEAREST)
        valid_output = np.isfinite(output) & (output > 0)
        supported = valid_output & (np.abs(output-nearest_depth) <= 0.05+0.03*output)
        output[supported] *= np.exp(full_correction[supported])
        used = depth * np.exp(correction)
        self.reference = (gray.copy(), depth.copy(), pose.copy(), stamp_ns,
                          cv2.resize(output, size, interpolation=cv2.INTER_NEAREST))
        actual_error = reprojection_error(_remap(used,x1,y1,cv2.INTER_NEAREST), x1,y1,x0,y0,k,relative)
        stats['validation_before_px'] = float(np.median(error_before[validation]))
        stats['validation_after_px'] = float(np.median(actual_error[validation]))
        stats['corrected_fraction'] = float(np.count_nonzero(supported & (np.abs(full_correction)>1e-6))/output.size)
        return finish('refined')
