#!/usr/bin/env python3
"""Pure map-boundary checks shared by field runtime adapters."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OrientedMapFit:
    fits: bool
    reason: str
    half_extent_x_m: float
    half_extent_y_m: float
    left_clearance_m: float
    right_clearance_m: float
    bottom_clearance_m: float
    top_clearance_m: float


def _finite(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _positive(name, value):
    value = _finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def _nonnegative(name, value):
    value = _finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value


def check_oriented_box_inside_map(
        center_x_m,
        center_y_m,
        yaw_rad,
        length_m,
        width_m,
        map_width_m,
        map_height_m,
        boundary_margin_m=0.0):
    """Check an oriented rectangle against a zero-origin rectangular map.

    The map and all field Homographies use origin ``(0,0)``.  A small boundary
    margin can represent raster-cell rounding or desired localisation reserve.
    """

    cx = _finite("center_x_m", center_x_m)
    cy = _finite("center_y_m", center_y_m)
    yaw = _finite("yaw_rad", yaw_rad)
    length = _positive("length_m", length_m)
    width = _positive("width_m", width_m)
    map_width = _positive("map_width_m", map_width_m)
    map_height = _positive("map_height_m", map_height_m)
    margin = _nonnegative("boundary_margin_m", boundary_margin_m)

    c = abs(math.cos(yaw))
    s = abs(math.sin(yaw))
    half_x = 0.5 * (length * c + width * s) + margin
    half_y = 0.5 * (length * s + width * c) + margin

    left = cx - half_x
    right = map_width - (cx + half_x)
    bottom = cy - half_y
    top = map_height - (cy + half_y)
    clearances = (left, right, bottom, top)
    fits = min(clearances) >= -1e-9

    if fits:
        reason = "OK"
    else:
        labels = ("LEFT", "RIGHT", "BOTTOM", "TOP")
        failed = [
            label for label, clearance in zip(labels, clearances)
            if clearance < -1e-9
        ]
        reason = "LOADED_FOOTPRINT_OUTSIDE_MAP_" + "_".join(failed)

    return OrientedMapFit(
        fits=fits,
        reason=reason,
        half_extent_x_m=half_x,
        half_extent_y_m=half_y,
        left_clearance_m=left,
        right_clearance_m=right,
        bottom_clearance_m=bottom,
        top_clearance_m=top,
    )
