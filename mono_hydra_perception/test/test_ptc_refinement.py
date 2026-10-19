import cv2
import numpy as np
import pytest

from mono_hydra_perception.ptc_refinement import (
    PtcRefiner, RefinementConfig, scaled_intrinsics, reprojection_error, valid_pose)


def scene():
    rng = np.random.default_rng(31)
    gray = rng.integers(0, 256, (120, 160), dtype=np.uint8)
    gray = cv2.GaussianBlur(gray, (3, 3), 0.5)
    image = np.repeat(gray[..., None], 3, axis=2)
    depth = np.full(gray.shape, 3.3, dtype=np.float32)  # true surface at 3 m
    labels = np.ones(gray.shape, np.int32)
    K = np.array([[100., 0, 80], [0, 100, 60], [0, 0, 1]])
    return image, depth, labels, K


def make_refiner(local=False, temporal_check=True):
    pytest.importorskip('ptc_depth_cpp')
    cv2.setNumThreads(2)
    return PtcRefiner(RefinementConfig(width=160, local=local, temporal_check=temporal_check))


def test_resize_intrinsics_preserve_pixel_centers():
    K = scene()[-1]
    resized = scaled_intrinsics(K, (120, 160), (60, 80))
    assert resized[0, 2] == 39.75
    assert resized[1, 2] == 29.75
    assert resized[0, 0] == 50


def test_pose_reprojection_includes_translation_and_rotation():
    K = scene()[-1]
    R, _ = cv2.Rodrigues(np.array([0.01, 0.12, -0.03]))
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [-0.2, 0.01, 0.02]
    prev = np.array([0.1, 0.2, 3.0]); curr = R @ prev + T[:3, 3]
    x0, y0 = 100*prev[:2]/prev[2]+K[:2, 2]
    x1, y1 = 100*curr[:2]/curr[2]+K[:2, 2]
    assert reprojection_error(np.array(curr[2]), np.array(x1), np.array(y1),
                              x0, y0, K, T) < 1e-8
    assert valid_pose(T)
    T[0, 0] = np.nan
    assert not valid_pose(T)


@pytest.mark.parametrize('local', [False, True])
def test_real_ptc_corrects_biased_metric_depth_without_changing_input(local):
    # Isolate metric triangulation. A previously emitted biased surface can
    # deliberately veto a sudden global calibration change in the safe mode.
    refiner = make_refiner(local, temporal_check=False)
    image, depth, labels, K = scene()
    original = depth.copy(); pose = np.eye(4)
    first, stats = refiner.refine(image, depth, labels, K, pose, 10**9)
    np.testing.assert_array_equal(first, depth)
    assert stats['reason'] == 'initialization'
    # Camera translates +18 cm: image moves left by 6 px for a surface at 3 m.
    moved = cv2.warpAffine(image, np.array([[1., 0, -6], [0, 1., 0]]), (160, 120))
    pose[0, 3] = 0.18
    result, stats = refiner.refine(moved, depth, labels, K, pose, 1200000000)
    assert stats['reason'] == 'refined', stats
    assert np.mean(np.abs(result[:, 10:-10]-3.0)) < np.mean(np.abs(depth[:, 10:-10]-3.0))
    assert stats['validation_after_px'] < stats['validation_before_px']
    assert np.min(result/depth) >= 1/1.1-1e-6
    np.testing.assert_array_equal(depth, original)


def test_stationary_invalid_pixels_and_pose_fallback_are_lossless():
    refiner = make_refiner()
    image, depth, labels, K = scene(); depth[0, :3] = [0, np.nan, np.inf]
    pose = np.eye(4)
    refiner.refine(image, depth, labels, K, pose, 10**9)
    result, stats = refiner.refine(image, depth, labels, K, pose, 1200000000)
    assert stats['reason'] == 'low_baseline'
    np.testing.assert_array_equal(result, depth)
    pose[0, 0] = np.nan
    result, stats = refiner.refine(image, depth, labels, K, pose, 1300000000)
    assert stats['reason'] == 'invalid_input'
    np.testing.assert_array_equal(result, depth)


def test_timestamp_and_calibration_changes_reset_history():
    refiner = make_refiner(); image, depth, labels, K = scene(); pose = np.eye(4)
    for stamp in (10**9, 0, 3*10**9):
        result, stats = refiner.refine(image, depth, labels, K, pose, stamp)
        assert stats['reason'] in ('initialization', 'history_reset')
        np.testing.assert_array_equal(result, depth)
    K[0, 0] = 101
    _, stats = refiner.refine(image, depth, labels, K, pose, 3100000000)
    assert stats['reason'] == 'initialization'


def test_configuration_rejects_invalid_limits():
    with pytest.raises(ValueError):
        RefinementConfig(max_scale_change=1)


def test_temporal_guard_rejects_scale_jump_from_consistent_surface():
    refiner = make_refiner()
    image, depth, labels, K = scene(); pose = np.eye(4)
    refiner.refine(image, depth, labels, K, pose, 10**9)
    moved = cv2.warpAffine(image, np.array([[1., 0, -6], [0, 1., 0]]), (160, 120))
    pose[0, 3] = 0.18
    result, stats = refiner.refine(moved, depth, labels, K, pose, 1200000000)
    assert stats['reason'] == 'validation_rejected'
    np.testing.assert_array_equal(result, depth)
