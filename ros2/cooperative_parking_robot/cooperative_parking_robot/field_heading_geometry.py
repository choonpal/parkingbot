#!/usr/bin/env python3
"""Heading helpers for resolving segmentation's 180-degree axis ambiguity."""

from __future__ import annotations

import math


def normalize_angle(angle_rad):
    angle = float(angle_rad)
    if not math.isfinite(angle):
        raise ValueError("angle_rad must be finite")
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error(target_yaw_rad, current_yaw_rad):
    return normalize_angle(
        float(target_yaw_rad) - float(current_yaw_rad))


def resolve_undirected_axis_yaw(axis_yaw_rad, reference_yaw_rad):
    """Choose ``axis`` or ``axis+pi`` closest to a directed reference yaw.

    A segmentation mask/PCA only identifies the vehicle's longitudinal axis;
    front and rear are indistinguishable.  WAITING orientation or the current
    Front/Rear odometry supplies the directed reference.
    """

    axis = normalize_angle(axis_yaw_rad)
    reference = normalize_angle(reference_yaw_rad)
    opposite = normalize_angle(axis + math.pi)
    axis_error = abs(angle_error(axis, reference))
    opposite_error = abs(angle_error(opposite, reference))
    return axis if axis_error <= opposite_error else opposite


def circular_mean(first_yaw_rad, second_yaw_rad):
    first = normalize_angle(first_yaw_rad)
    second = normalize_angle(second_yaw_rad)
    x = math.cos(first) + math.cos(second)
    y = math.sin(first) + math.sin(second)
    if math.hypot(x, y) <= 1e-9:
        raise ValueError("opposite headings have no stable circular mean")
    return math.atan2(y, x)
