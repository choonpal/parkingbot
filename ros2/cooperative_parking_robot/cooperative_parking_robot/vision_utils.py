#!/usr/bin/env python3
"""Small, ROS-independent helpers shared by Jetson vision nodes."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Callable
from typing import Iterable, Optional, Sequence, Tuple


def directed_axis_yaw(axis_yaw, expected_yaw):
    """Resolve an undirected PCA axis to the heading nearest expectation."""
    def wrap(value):
        wrapped = math.atan2(math.sin(float(value)), math.cos(float(value)))
        return -math.pi if wrapped >= math.pi else wrapped

    axis = wrap(axis_yaw)
    expected = wrap(expected_yaw)
    opposite = wrap(axis + math.pi)
    axis_error = abs(math.atan2(
        math.sin(axis - expected), math.cos(axis - expected)))
    opposite_error = abs(math.atan2(
        math.sin(opposite - expected), math.cos(opposite - expected)))
    return opposite if opposite_error < axis_error else axis


def normalize_model_mode(value: str) -> str:
    """Return a validated YOLO interpretation mode."""
    mode = str(value).strip().lower()
    aliases = {
        # 차량 마스크만 학습한 YOLO11-Seg. 빈 주차면은 고정 슬롯 DB와
        # 차량 마스크 겹침률로 판단하므로 empty_slot 클래스가 필요 없다.
        'vehicle': 'vehicle_seg',
        'vehicle_seg': 'vehicle_seg',
        'vehicle_only': 'vehicle_seg',
        'parking': 'parking_seg',
        'custom': 'parking_seg',
        'custom_seg': 'parking_seg',
        'parking_seg': 'parking_seg',
        'coco': 'coco',
        'pretrained': 'coco',
        'generic': 'coco',
    }
    if mode not in aliases:
        raise ValueError(
            "model_mode must be 'vehicle_seg', 'parking_seg' or 'coco', "
            f'got {value!r}')
    return aliases[mode]


def load_yolo_model(
        yolo_factory: Callable[..., Any], model_path: str,
        model_mode: str):
    """Load a local Ultralytics model without triggering a network download."""
    path = Path(str(model_path)).expanduser()
    if not path.is_file():
        raise FileNotFoundError(
            'YOLO requires a local model file; network download is disabled: '
            f'{path}')
    mode = normalize_model_mode(model_mode)
    task = 'segment' if mode in ('vehicle_seg', 'parking_seg') else None
    if task is None:
        return yolo_factory(str(path)), task
    return yolo_factory(str(path), task=task), task


def polygon_area(points: Sequence[Sequence[float]]) -> float:
    """Compute the absolute area of a 2D polygon without OpenCV."""
    if len(points) < 3:
        return 0.0
    total = 0.0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        total += float(point[0]) * float(next_point[1])
        total -= float(next_point[0]) * float(point[1])
    return abs(total) * 0.5


def select_marker_by_id(
        corners: Sequence,
        ids,
        marker_id: int,
        min_area_px: float = 0.0,
        min_area_ratio: float = 0.0,
        frame_width: Optional[int] = None,
        frame_height: Optional[int] = None):
    """Select the largest valid detection for ``marker_id``."""
    if ids is None:
        return None, 0.0
    if min_area_px < 0.0 or min_area_ratio < 0.0:
        raise ValueError('marker area thresholds must be non-negative')
    if min_area_ratio > 0.0:
        if not frame_width or not frame_height:
            raise ValueError(
                'frame dimensions are required when min_area_ratio is used')
        ratio_threshold = (
            float(frame_width) * float(frame_height) * float(min_area_ratio))
    else:
        ratio_threshold = 0.0
    threshold = max(float(min_area_px), ratio_threshold)

    try:
        flat_ids = [int(value) for value in ids.flatten()]
    except AttributeError:
        flat_ids = [int(value) for value in ids]

    best = None
    best_area = 0.0
    for index, detected_id in enumerate(flat_ids):
        if detected_id != int(marker_id):
            continue
        candidate = corners[index]
        if len(candidate) == 1 and hasattr(candidate[0], '__len__'):
            points = candidate[0]
        else:
            points = candidate
        area = polygon_area(points)
        if area < threshold:
            continue
        if best is None or area > best_area:
            best = points
            best_area = area
    return best, best_area


def pnp_distance_m(tvec) -> float:
    """Return Euclidean camera-to-marker distance from a PnP translation."""
    try:
        flat = tvec.reshape(-1)
        values = [float(flat[index]) for index in range(3)]
    except AttributeError:
        values = []
        stack = [tvec]
        while stack and len(values) < 3:
            item = stack.pop(0)
            if isinstance(item, (list, tuple)):
                stack[0:0] = list(item)
                continue
            values.append(float(item))
    if len(values) < 3:
        raise ValueError('tvec must contain at least three numeric values')
    return math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)


def parse_class_ids(values: Iterable[int]) -> Tuple[int, ...]:
    """Validate and de-duplicate a sequence of non-negative class IDs."""
    result = []
    for value in values:
        class_id = int(value)
        if class_id < 0:
            raise ValueError('class IDs must be non-negative')
        if class_id not in result:
            result.append(class_id)
    if not result:
        raise ValueError('at least one class ID is required')
    return tuple(result)


def principal_axis_yaw(
        points: Sequence[Sequence[float]],
        min_eigenvalue_ratio: float = 1.25,
        yaw_limit_rad: float = math.pi / 2.0) -> Optional[float]:
    """Estimate an undirected 2D polygon's major-axis yaw using PCA."""
    # 회전사각형(4점)도 충분히 주축을 정의할 수 있다. 실제 Segmentation
    # contour는 보통 더 많은 점을 주지만 convexHull이 4~7점으로 줄 수 있다.
    if len(points) < 4:
        return None
    ratio_limit = float(min_eigenvalue_ratio)
    yaw_limit = float(yaw_limit_rad)
    if ratio_limit <= 1.0:
        raise ValueError('min_eigenvalue_ratio must exceed 1.0')
    if not 0.0 < yaw_limit <= math.pi / 2.0:
        raise ValueError('yaw_limit_rad must be in (0, pi/2]')
    xy = [(float(point[0]), float(point[1])) for point in points]
    if not all(math.isfinite(value) for point in xy for value in point):
        return None
    mean_x = sum(point[0] for point in xy) / len(xy)
    mean_y = sum(point[1] for point in xy) / len(xy)
    dx = [point[0] - mean_x for point in xy]
    dy = [point[1] - mean_y for point in xy]
    cxx = sum(value * value for value in dx) / len(xy)
    cyy = sum(value * value for value in dy) / len(xy)
    cxy = sum(x * y for x, y in zip(dx, dy)) / len(xy)
    trace = cxx + cyy
    determinant = cxx * cyy - cxy * cxy
    root = math.sqrt(max(0.0, trace * trace / 4.0 - determinant))
    major = trace / 2.0 + root
    minor = trace / 2.0 - root
    if minor <= 1e-12 or major / minor < ratio_limit:
        return None
    if abs(cxy) > 1e-12:
        yaw = math.atan2(major - cxx, cxy)
    else:
        yaw = 0.0 if cxx >= cyy else math.pi / 2.0
    while yaw > math.pi / 2.0:
        yaw -= math.pi
    while yaw < -math.pi / 2.0:
        yaw += math.pi
    return yaw if abs(yaw) <= yaw_limit else None


def correct_floor_projection(
        floor_x: float, floor_y: float,
        camera_ground_x: float, camera_ground_y: float,
        camera_height: float, object_height: float) -> Tuple[float, float]:
    """Move a floor-plane ray intersection onto a horizontal object plane.

    ``floor_x/floor_y`` are produced by a floor-calibrated homography. For an
    observed point ``object_height`` metres above that floor, a nadir pinhole
    camera sees the same ray intersect the floor farther from the optical-axis
    ground point. A zero object height leaves the point unchanged.
    """
    height = float(object_height)
    if height < 0.0:
        raise ValueError('object_height must be non-negative')
    if height == 0.0:
        return float(floor_x), float(floor_y)
    camera_height = float(camera_height)
    if camera_height <= height:
        raise ValueError('camera_height must exceed object_height')
    scale = (camera_height - height) / camera_height
    camera_x = float(camera_ground_x)
    camera_y = float(camera_ground_y)
    return (
        camera_x + scale * (float(floor_x) - camera_x),
        camera_y + scale * (float(floor_y) - camera_y),
    )
