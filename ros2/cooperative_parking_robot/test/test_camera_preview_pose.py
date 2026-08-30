#!/usr/bin/env python3
"""Regression tests for the camera preview relative-pose readout."""

import math
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

# Some ROS launch-test plugins import the installed package before test
# collection reaches this file.  Extend that already-loaded package path so
# this regression test still exercises the working tree, not a stale overlay.
import cooperative_parking_robot  # noqa: E402

LOCAL_PACKAGE = str(PACKAGE_ROOT / 'cooperative_parking_robot')
if LOCAL_PACKAGE not in cooperative_parking_robot.__path__:
    cooperative_parking_robot.__path__.insert(0, LOCAL_PACKAGE)

from cooperative_parking_robot.camera_preview_node import (  # noqa: E402
    map_pose_metrics,
    relative_pose_metrics,
)


def _pose(forward=0.306, lateral=0.009, yaw_deg=-7.2, scale=1.0):
    half = math.radians(yaw_deg) / 2.0
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='rear_base'),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=forward, y=lateral),
            orientation=SimpleNamespace(
                x=0.0, y=0.0,
                z=scale * math.sin(half),
                w=scale * math.cos(half))),
    )


def test_relative_pose_metrics_reports_distance_lateral_and_yaw():
    metrics = relative_pose_metrics(_pose())
    assert metrics['forward_m'] == pytest.approx(0.306)
    assert metrics['lateral_m'] == pytest.approx(0.009)
    assert metrics['yaw_deg'] == pytest.approx(-7.2)
    assert metrics['frame_id'] == 'rear_base'


def test_relative_pose_metrics_normalizes_quaternion():
    assert relative_pose_metrics(_pose(scale=3.0))['yaw_deg'] == \
        pytest.approx(-7.2)


def test_relative_pose_metrics_rejects_invalid_values():
    with pytest.raises(ValueError, match='non-finite'):
        relative_pose_metrics(_pose(forward=float('nan')))
    with pytest.raises(ValueError, match='norm'):
        relative_pose_metrics(_pose(scale=0.0))


def test_map_pose_metrics_reports_production_position_and_yaw():
    msg = _pose(forward=1.25, lateral=2.50, yaw_deg=148.2, scale=2.0)
    msg.header.frame_id = 'map'
    metrics = map_pose_metrics(msg)
    assert metrics['x_m'] == pytest.approx(1.25)
    assert metrics['y_m'] == pytest.approx(2.50)
    assert metrics['yaw_deg'] == pytest.approx(148.2)
    assert metrics['frame_id'] == 'map'
