import numpy as np

from mono_hydra_perception.m2h_hmx_large_node import _map_labels


def test_partial_mapping_preserves_unknown_labels_and_input():
    labels = np.array([[0, 4, 19], [40, 128, 255]], dtype=np.uint8)
    original = labels.copy()
    actual = _map_labels(labels, {4: 0, 40: 7})
    np.testing.assert_array_equal(actual, [[0, 0, 19], [7, 128, 255]])
    np.testing.assert_array_equal(labels, original)
    assert actual.dtype == np.uint8


def test_mapping_supports_strided_and_empty_images():
    labels = np.array([[1, 2, 3, 4]], dtype=np.uint8)[:, ::2]
    np.testing.assert_array_equal(_map_labels(labels, {3: 8}), [[1, 8]])
    empty = np.empty((0, 2), dtype=np.uint8)
    assert _map_labels(empty, {1: 2}).shape == (0, 2)


def test_non_uint8_mapping_and_unmapped_conversion():
    labels = np.array([[-1, 300, 7]], dtype=np.int32)
    np.testing.assert_array_equal(_map_labels(labels, {-1: 2, 300: 9}), [[2, 9, 7]])
    np.testing.assert_array_equal(_map_labels(np.array([[1, 2]], dtype=np.int32), {}), [[1, 2]])
