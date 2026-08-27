#!/usr/bin/env python3
"""Segmentation-mask centre helpers shared by production YOLO wrappers.

The camera preview already reports the centre of ``cv2.minAreaRect(mask)``.
Production must use the same pixel centre before Homography/parallax so the
calibration measurements and the driving coordinates refer to the same point.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np


def mask_min_area_rect_center(mask_polygon) -> Optional[Tuple[float, float]]:
    """Return the preview-compatible centre of a segmentation mask."""
    if mask_polygon is None or len(mask_polygon) < 3:
        return None
    points = np.asarray(mask_polygon, dtype=np.float32).reshape(-1, 2)
    if points.shape[0] < 3 or not np.all(np.isfinite(points)):
        return None
    (cx, cy), (width, height), _angle = cv2.minAreaRect(points)
    values = (float(cx), float(cy), float(width), float(height))
    if not all(math.isfinite(value) for value in values):
        return None
    if width < 1e-3 or height < 1e-3:
        return None
    return float(cx), float(cy)


def recentered_xyxy(
        mask_polygon,
        xyxy: Sequence[float],
        frame_width: int,
        frame_height: int,
        min_half_extent_px: float = 1.0,
) -> Tuple[Tuple[float, float, float, float], bool]:
    """Re-centre a detection box on the mask centre without leaving the frame.

    The box is only an adapter for the legacy image callback: that callback
    derives vehicle world position from ``(x1+x2)/2,(y1+y2)/2``. Width/height
    are retained when possible; near an image border the crop is symmetrically
    shrunk so its centre remains exactly the segmentation-mask centre.
    """
    if len(xyxy) != 4:
        raise ValueError('xyxy must contain four values')
    fw = int(frame_width)
    fh = int(frame_height)
    if fw <= 1 or fh <= 1:
        raise ValueError('frame dimensions must exceed one pixel')
    original = tuple(float(value) for value in xyxy)
    if not all(math.isfinite(value) for value in original):
        return original, False

    center = mask_min_area_rect_center(mask_polygon)
    if center is None:
        return original, False
    cx, cy = center
    if not (0.0 <= cx <= fw - 1.0 and 0.0 <= cy <= fh - 1.0):
        return original, False

    x1, y1, x2, y2 = original
    half_w = max(0.0, (x2 - x1) * 0.5)
    half_h = max(0.0, (y2 - y1) * 0.5)
    half_w = min(half_w, cx, (fw - 1.0) - cx)
    half_h = min(half_h, cy, (fh - 1.0) - cy)
    if half_w < float(min_half_extent_px) or half_h < float(min_half_extent_px):
        return original, False

    return (
        cx - half_w,
        cy - half_h,
        cx + half_w,
        cy + half_h,
    ), True


def recenter_vehicle_result_boxes(
        results: Iterable,
        frame_width: int,
        frame_height: int,
        vehicle_class_id: int,
) -> int:
    """Move vehicle box centres using a mutable copy of box tensor data.

    Ultralytics keeps ``xyxy/conf/cls`` in ``result.boxes.data``. Only the
    first four coordinates are changed; confidence, class, masks and polygons
    are untouched. PyTorch inference tensors may not be mutated after leaving
    ``InferenceMode``, so the original is never written: a normal clone/copy
    replaces ``boxes.data`` only when a vehicle was actually recentered.
    """
    changed = 0
    wanted = int(vehicle_class_id)
    for result in results:
        boxes = getattr(result, 'boxes', None)
        masks = getattr(result, 'masks', None)
        masks_xy = None if masks is None else getattr(masks, 'xy', None)
        if boxes is None or masks_xy is None:
            continue
        data = getattr(boxes, 'data', None)
        classes = getattr(boxes, 'cls', None)
        if data is None or classes is None:
            continue
        if hasattr(data, 'clone'):
            # clone() while InferenceMode remains enabled produces another
            # inference tensor. Explicitly disable it for the adapter copy.
            try:
                import torch
                with torch.inference_mode(False):
                    mutable_data = data.clone()
            except ImportError:  # pragma: no cover - tensor implies torch
                mutable_data = data.clone()
        else:
            mutable_data = np.array(data, copy=True)
        result_changed = 0
        count = min(len(boxes), len(masks_xy))
        for index in range(count):
            class_value = classes[index]
            if hasattr(class_value, 'item'):
                class_value = class_value.item()
            if int(class_value) != wanted:
                continue
            current = []
            for coordinate_index in range(4):
                value = data[index, coordinate_index]
                if hasattr(value, 'item'):
                    value = value.item()
                current.append(float(value))
            replacement, used_mask = recentered_xyxy(
                masks_xy[index], current, frame_width, frame_height)
            if not used_mask:
                continue
            for coordinate_index, value in enumerate(replacement):
                mutable_data[index, coordinate_index] = value
            changed += 1
            result_changed += 1
        if result_changed:
            boxes.data = mutable_data
    return changed
