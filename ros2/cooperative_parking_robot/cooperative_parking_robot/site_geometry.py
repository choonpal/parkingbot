#!/usr/bin/env python3
"""Measured ceiling-camera geometry used by production vision wrappers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class CameraGeometry:
    optical_axis_ground_m: Tuple[float, float]
    optical_center_height_m: float


# 2026-08-28 replacement-camera 640x360 rectified calibration principal points
# projected through the final, post-swap registered homographies. These are
# optical-axis/floor intersections, not the physical vertical projection of
# the tilted camera housings. Production launch parameters take precedence;
# these values are only a fail-safe fallback for an otherwise unconfigured
# direct launch.
CAMERA_GEOMETRY = {
    'cam0': CameraGeometry((2.319423, 2.315810), 2.610),
    'cam2': CameraGeometry((1.891773, 1.296094), 2.610),
}

# Both overhead robot ArUco markers are centered on the robot base/rotation
# center. Only the marker plane height needs parallax correction.
ROBOT_MARKER_HEIGHT_M = 0.120
FRONT_MARKER_OFFSET_X_M = 0.0
REAR_MARKER_OFFSET_X_M = 0.0

# The vehicle top surface is sloped and 0.74 m is only its maximum height.
# A single 0.74 m plane would over-correct the segmentation centroid. Leave
# this disabled until the effective height is estimated from a known vehicle
# center versus the raw homography-projected YOLO center.
VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M: Optional[float] = None
