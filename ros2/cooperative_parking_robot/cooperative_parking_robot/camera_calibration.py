#!/usr/bin/env python3
"""Camera intrinsic calibration loading and frame-size adaptation helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Tuple


_KEY_PAIRS = (
    ('camera_matrix', 'dist_coeffs'),
    ('mtx', 'dist'),
    ('K', 'D'),
)
_VALID_DISTORTION_COUNTS = {4, 5, 8, 12, 14}


def load_camera_calibration(path: str) -> Tuple[object, object, Tuple[str, str]]:
    """Load an OpenCV intrinsic calibration from an ``.npz`` file.

    Accepted key pairs are ``camera_matrix/dist_coeffs``, ``mtx/dist`` and
    ``K/D``. Returns finite float64 arrays and the selected source-key pair.
    """
    import numpy as np

    file_path = Path(path).expanduser()
    if not file_path.is_file():
        raise FileNotFoundError(f'camera calibration not found: {file_path}')
    if file_path.stat().st_size == 0:
        raise ValueError(f'camera calibration is empty: {file_path}')

    with np.load(file_path, allow_pickle=False) as data:
        selected = next(
            ((matrix_key, dist_key)
             for matrix_key, dist_key in _KEY_PAIRS
             if matrix_key in data and dist_key in data),
            None,
        )
        if selected is None:
            available = ', '.join(sorted(data.files)) or '<none>'
            expected = ' or '.join(
                f'{matrix_key}/{dist_key}'
                for matrix_key, dist_key in _KEY_PAIRS)
            raise KeyError(
                f'camera calibration keys missing; expected {expected}; '
                f'available: {available}')

        matrix_key, dist_key = selected
        camera_matrix = np.asarray(data[matrix_key], dtype=np.float64)
        dist_coeffs = np.asarray(data[dist_key], dtype=np.float64)

    if camera_matrix.shape != (3, 3):
        raise ValueError(
            f'camera matrix must be 3x3, got {camera_matrix.shape}')
    if not np.all(np.isfinite(camera_matrix)):
        raise ValueError('camera matrix contains non-finite values')
    if float(camera_matrix[0, 0]) <= 0.0 or float(camera_matrix[1, 1]) <= 0.0:
        raise ValueError('camera focal lengths fx/fy must be positive')
    if abs(float(camera_matrix[2, 2])) < 1e-12:
        raise ValueError('camera matrix K[2,2] must be non-zero')
    camera_matrix = camera_matrix / float(camera_matrix[2, 2])

    if dist_coeffs.ndim > 2:
        raise ValueError(
            f'distortion coefficients must be 1D or 2D, got {dist_coeffs.shape}')
    dist_coeffs = dist_coeffs.reshape(1, -1)
    if dist_coeffs.size not in _VALID_DISTORTION_COUNTS:
        raise ValueError(
            'distortion coefficient count must be one of '
            f'{sorted(_VALID_DISTORTION_COUNTS)}, got {dist_coeffs.size}')
    if not np.all(np.isfinite(dist_coeffs)):
        raise ValueError('distortion coefficients contain non-finite values')

    return camera_matrix, dist_coeffs, selected


def scale_camera_matrix(
        camera_matrix, source_width: int, source_height: int,
        target_width: int, target_height: int):
    """Scale intrinsics for a resolution-only resize with equal aspect ratio."""
    import numpy as np

    values = (source_width, source_height, target_width, target_height)
    if any(int(value) <= 0 for value in values):
        raise ValueError('source/target image dimensions must be positive')

    source_aspect = float(source_width) / float(source_height)
    target_aspect = float(target_width) / float(target_height)
    if abs(source_aspect - target_aspect) / source_aspect > 0.01:
        raise ValueError(
            'camera stream aspect ratio differs from calibration; '
            'cropping cannot be corrected by focal-length scaling')

    scaled = np.asarray(camera_matrix, dtype=np.float64).copy()
    sx = float(target_width) / float(source_width)
    sy = float(target_height) / float(source_height)
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled
