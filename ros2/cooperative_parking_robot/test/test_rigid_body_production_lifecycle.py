#!/usr/bin/env python3
"""ROS-layer lifecycle regression for the production rigid-body entrypoint."""

from cooperative_parking_robot.rigid_body_sync_node import (
    RigidBodySyncNode as LegacyRigidBodySyncNode,
)
from cooperative_parking_robot.rigid_body_sync_production_node import (
    RigidBodySyncNode,
)


class _ResetCounter:
    def __init__(self):
        self.calls = 0

    def reset(self):
        self.calls += 1


class _ReferenceCapture:
    def __init__(self):
        self.reference = object()
        self.state = 'REFERENCE_READY'
        self.started_at = 10.0
        self.samples = [(0.79, 0.0, 0.0)]


def test_path_and_replan_preserve_locked_lift_reference(monkeypatch):
    node = object.__new__(RigidBodySyncNode)
    node.reference_capture = _ReferenceCapture()
    node.lateral_pid = _ResetCounter()
    node.lateral_error_since = 10.0
    legacy_calls = []

    monkeypatch.setattr(
        LegacyRigidBodySyncNode, 'path_cb',
        lambda _self, msg: legacy_calls.append(msg))

    first_path = object()
    node.path_cb(first_path)
    reference = node.reference_capture.reference
    samples = list(node.reference_capture.samples)

    second_path = object()
    node.path_cb(second_path)

    assert legacy_calls == [first_path, second_path]
    assert node.reference_capture.reference is reference
    assert node.reference_capture.state == 'REFERENCE_READY'
    assert node.reference_capture.samples == samples
    assert node.lateral_pid.calls == 2
    assert node.lateral_error_since is None
