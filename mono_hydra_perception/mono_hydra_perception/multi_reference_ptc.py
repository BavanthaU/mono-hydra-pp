"""Opt-in bounded multiple-reference experiment around the released PTC core.

Network weights and raw inputs remain fixed. Native PTC is initialized separately
for each selected pair so its hidden posterior never uses an unrelated reference.
The current frame is emitted once, only where both references support correction.
"""
from collections import deque
import time

import cv2
import numpy as np

from .ptc_refinement import PtcRefiner, scaled_intrinsics, valid_pose, _remap, reprojection_error


class MultiReferencePtc:
    def __init__(self, config, references=2, history_size=6, core_module=None):
        if references != 2 or history_size < 2 or history_size > 12:
            raise ValueError('This bounded experiment uses two references and 2–12 history frames')
        self.config = config
        self.history = deque(maxlen=history_size)
        self.fitters = [PtcRefiner(config, core_module) for _ in range(references)]
        self.geometry = None

    def reset(self):
        self.history.clear()
        self.geometry = None
        for fitter in self.fitters:
            fitter.reset()

    def _score(self, previous, K, pose, shape):
        _, depth, _, old_pose, _ = previous
        relative = np.linalg.inv(pose) @ old_pose
        baseline = np.linalg.norm(relative[:3, 3])
        if not self.config.min_baseline <= baseline <= self.config.max_baseline:
            return -1.0
        y, x = np.mgrid[4:shape[0]:16, 4:shape[1]:16]
        z = depth[y, x]
        rays = np.stack(((x-K[0, 2])/K[0, 0], (y-K[1, 2])/K[1, 1], np.ones_like(z)), -1)
        points = (rays*z[..., None]) @ relative[:3, :3].T + relative[:3, 3]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = K[0, 0]*points[..., 0]/points[..., 2]+K[0, 2]
            v = K[1, 1]*points[..., 1]/points[..., 2]+K[1, 2]
        good = np.isfinite(z) & (z > 0) & (points[..., 2] > 0) & (u >= 0) & (v >= 0) & (u < shape[1]) & (v < shape[0])
        if good.mean() < 0.4:
            return -1.0
        rotated = rays @ relative[:3, :3].T
        cosine = np.sum(rotated*points, -1)/(np.linalg.norm(rotated, axis=-1)*np.linalg.norm(points, axis=-1)+1e-12)
        angles = np.degrees(np.arccos(np.clip(cosine[good], -1, 1)))
        # Reward informative parallax, not arbitrarily old or distant references.
        return float(good.mean() * np.minimum(angles, 5.0).mean())

    def refine(self, rgb, raw_depth, labels, K, world_T_camera, stamp_ns):
        start = time.perf_counter()
        raw = np.array(raw_depth, dtype=np.float32, copy=True)
        stats = dict(stamp_ns=int(stamp_ns), reason='insufficient_references',
                     corrected_fraction=0.0, selected_references=[], elapsed_ms=0.0)
        if (raw.ndim != 2 or rgb.shape[:2] != raw.shape or labels.shape != raw.shape or
                not valid_pose(world_T_camera) or np.shape(K) != (3, 3) or
                not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0):
            self.reset()
            stats['reason'] = 'invalid_input'
            return raw, stats
        geometry = (raw.shape, tuple(np.asarray(K).ravel()))
        if geometry != self.geometry or (self.history and stamp_ns <= self.history[-1][-1]):
            self.reset()
        self.geometry = geometry
        while self.history and (stamp_ns-self.history[0][-1])*1e-9 > self.config.max_age_s:
            self.history.popleft()
        ranked = sorted(((self._score(h, K, world_T_camera, raw.shape), i) for i, h in enumerate(self.history)), reverse=True)
        selected = []
        for score, i in ranked:
            if score < self.config.min_parallax_deg * 0.4:
                continue
            # Do not count two almost identical camera centers as two useful views.
            if selected and np.linalg.norm(self.history[i][3][:3, 3]-self.history[selected[0]][3][:3, 3]) < self.config.min_baseline:
                continue
            selected.append(i)
            if len(selected) == 2:
                break
        output = raw
        if len(selected) == 2:
            proposals = []
            for fitter, index in zip(self.fitters, selected):
                old_rgb, old_depth, old_labels, old_pose, old_stamp = self.history[index]
                fitter.reset()
                fitter.refine(old_rgb, old_depth, old_labels, K, old_pose, old_stamp)
                proposal, detail = fitter.refine(rgb, raw, labels, K, world_T_camera, stamp_ns)
                stats['selected_references'].append(dict(stamp_ns=int(old_stamp), reason=detail['reason']))
                proposals.append(proposal)
            valid = np.isfinite(raw) & (raw > 0)
            ratios = [np.divide(p, raw, out=np.ones_like(raw), where=valid) for p in proposals]
            logs = [np.log(np.maximum(r, 1e-8)) for r in ratios]
            supported = (valid & (np.abs(logs[0]) > 1e-5) & (np.abs(logs[1]) > 1e-5) &
                         (logs[0]*logs[1] > 0) & (np.abs(logs[0]-logs[1]) <= 0.03))
            candidate = raw.copy()
            candidate[supported] *= np.exp(0.5*(logs[0][supported]+logs[1][supported]))
            # Each selected pair supplies a held-out check of the combined field.
            checks = [f.validate_candidate(candidate) for f in self.fitters]
            stats['validation'] = checks
            if supported.any() and all(c['accepted'] for c in checks):
                output = candidate
                stats['reason'] = 'refined'
                stats['corrected_fraction'] = float(supported.mean())
            else:
                stats['reason'] = 'multi_reference_rejected'
        if (not self.history or
                np.linalg.norm(world_T_camera[:3, 3]-self.history[-1][3][:3, 3]) >= self.config.min_baseline or
                (stamp_ns-self.history[-1][-1])*1e-9 >= 0.5):
            self.history.append((rgb.copy(), raw.copy(), labels.copy(), world_T_camera.copy(), int(stamp_ns)))
        stats['elapsed_ms'] = (time.perf_counter()-start)*1000
        return output, stats
