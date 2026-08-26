"""Ceiling-camera calibration compatibility and scaling tests."""

from pathlib import Path

import numpy as np
import pytest

from cooperative_parking_robot.camera_calibration import (
    load_camera_calibration,
    scale_camera_matrix,
)


ROOT = Path(__file__).resolve().parents[1]


def test_packaged_ceiling_calibration_loads():
    path = ROOT / 'config' / 'cctv_camera_calibration.npz'
    camera_matrix, dist_coeffs, keys = load_camera_calibration(str(path))
    assert keys == ('camera_matrix', 'dist_coeffs')
    assert camera_matrix.shape == (3, 3)
    assert dist_coeffs.shape == (1, 5)
    assert camera_matrix[0, 0] == pytest.approx(708.48633456)
    assert camera_matrix[1, 1] == pytest.approx(707.63853756)
    assert camera_matrix[0, 2] == pytest.approx(664.39994909)
    assert camera_matrix[1, 2] == pytest.approx(358.75645269)


@pytest.mark.parametrize(
    ('filename', 'expected_fx', 'expected_fy'),
    [
        ('cctv0_camera_calibration.npz', 436.84593725, 433.72630176),
        ('cctv2_camera_calibration.npz', 448.12854014, 445.36364374),
    ],
)
def test_packaged_provisional_dual_calibrations_are_640x480_compatible(
        filename, expected_fx, expected_fy):
    path = ROOT / 'config' / filename
    camera_matrix, dist_coeffs, keys = load_camera_calibration(str(path))
    assert keys == ('mtx', 'dist')
    assert camera_matrix.shape == (3, 3)
    assert dist_coeffs.shape == (1, 5)
    assert camera_matrix[0, 0] == pytest.approx(expected_fx)
    assert camera_matrix[1, 1] == pytest.approx(expected_fy)
    assert 0.0 <= camera_matrix[0, 2] < 640.0
    assert 0.0 <= camera_matrix[1, 2] < 480.0


def test_original_mtx_dist_keys_are_supported(tmp_path):
    path = tmp_path / 'legacy.npz'
    matrix = np.array([
        [700.0, 0.0, 640.0],
        [0.0, 700.0, 360.0],
        [0.0, 0.0, 1.0],
    ])
    distortion = np.array([[0.1, -0.1, 0.0, 0.0, 0.01]])
    np.savez(path, mtx=matrix, dist=distortion)
    loaded_matrix, loaded_distortion, keys = load_camera_calibration(str(path))
    assert keys == ('mtx', 'dist')
    np.testing.assert_allclose(loaded_matrix, matrix)
    np.testing.assert_allclose(loaded_distortion, distortion)


def test_resolution_scaling_preserves_normalized_intrinsics():
    matrix = np.array([
        [800.0, 0.0, 640.0],
        [0.0, 800.0, 360.0],
        [0.0, 0.0, 1.0],
    ])
    scaled = scale_camera_matrix(matrix, 1280, 720, 640, 360)
    np.testing.assert_allclose(scaled, np.array([
        [400.0, 0.0, 320.0],
        [0.0, 400.0, 180.0],
        [0.0, 0.0, 1.0],
    ]))


def test_aspect_ratio_change_is_rejected():
    with pytest.raises(ValueError, match='aspect ratio'):
        scale_camera_matrix(np.eye(3), 1280, 720, 640, 480)


def test_incomplete_calibration_is_rejected(tmp_path):
    path = tmp_path / 'bad.npz'
    np.savez(path, mtx=np.eye(3))
    with pytest.raises(KeyError):
        load_camera_calibration(str(path))
