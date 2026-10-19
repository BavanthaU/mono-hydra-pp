import cv2
import numpy as np
import pytest

from mono_hydra_perception.multi_reference_ptc import MultiReferencePtc
from mono_hydra_perception.ptc_refinement import RefinementConfig


def data():
    rng = np.random.default_rng(31)
    gray = cv2.GaussianBlur(rng.integers(0, 256, (120, 160), dtype=np.uint8), (3, 3), 0.5)
    rgb = np.repeat(gray[..., None], 3, axis=2)
    return rgb, np.full((120, 160), 3.3, np.float32), np.ones((120, 160), np.int32), np.array([[100., 0, 80], [0, 100, 60], [0, 0, 1]])


def make():
    core = pytest.importorskip('ptc_depth_cpp')
    cv2.setNumThreads(2)
    return MultiReferencePtc(RefinementConfig(width=160, local=False, temporal_check=False), core_module=core)


def test_two_useful_views_correct_bias_and_leave_raw_unchanged():
    f = make(); rgb, depth, labels, K = data(); original = depth.copy()
    for i in range(3):
        pose = np.eye(4); pose[0, 3] = i * 0.09
        image = cv2.warpAffine(rgb, np.array([[1., 0, -3*i], [0, 1., 0]]), (160, 120))
        output, stats = f.refine(image, depth, labels, K, pose, 1000000000 + i*200000000)
    assert len(stats['selected_references']) == 2, stats
    assert stats['reason'] == 'refined', stats
    assert np.mean(np.abs(output[:, 20:-20]-3)) < np.mean(np.abs(depth[:, 20:-20]-3))
    np.testing.assert_array_equal(depth, original)
    assert np.nanmin(output/depth) >= 1/1.1-1e-6


def test_stationary_and_rewind_do_not_accumulate_false_support():
    f = make(); rgb, depth, labels, K = data(); pose = np.eye(4)
    depth[0, :3] = [0, np.nan, np.inf]
    for i in range(12):
        result, stats = f.refine(rgb, depth, labels, K, pose, 1000000000+i*200000000)
        np.testing.assert_array_equal(result, depth)
        assert stats['reason'] == 'insufficient_references'
        assert len(f.history) <= 6
    _, stats = f.refine(rgb, depth, labels, K, pose, 1)
    assert len(f.history) == 1
    assert stats['reason'] == 'insufficient_references'
