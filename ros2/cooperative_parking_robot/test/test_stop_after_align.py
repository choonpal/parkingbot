"""Entry-only commissioning mode must never advance into grip/lift."""

import time
from types import SimpleNamespace

from cooperative_parking_robot.robot_state_machine_node import (
    RobotStateMachineNode,
)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def warn(self, *_args, **_kwargs):
        pass


def aligned_machine(*, stop_after_align=True):
    machine = RobotStateMachineNode.__new__(RobotStateMachineNode)
    machine.role = "front"
    machine.active_mission_id = "mission-1"
    machine.state = "ALIGN"
    machine.hardware_fault = None
    machine.fleet_receipt_time = time.monotonic()
    machine.fleet_timeout = 2.5
    machine.align_timeout = 120.0
    machine.wheel_aligned = True
    machine.alignment_announced = False
    machine.aligned_hold = False
    machine.stop_after_align = stop_after_align
    machine.committed_stages = set()
    machine.pub_align_done = Publisher()
    machine.pub_aligned_hold = Publisher()
    machine.pub_estop = Publisher()
    machine.get_logger = lambda: Logger()
    machine.publish_ready_calls = []
    machine.commit_calls = []
    machine.publish_ready_stage = machine.publish_ready_calls.append
    machine.maybe_publish_commit = machine.commit_calls.append
    machine.elapsed = lambda: 999.0
    machine.failures = []
    machine.fail = machine.failures.append
    machine.transitions = []
    machine.transition = machine.transitions.append
    return machine


def test_aligned_hold_publishes_alignment_without_lift_barrier_or_timeout():
    machine = aligned_machine()

    machine.state_machine()

    assert machine.state == "ALIGN"
    assert machine.alignment_announced is True
    assert machine.aligned_hold is True
    assert machine.pub_align_done.messages[-1].data is True
    assert machine.pub_aligned_hold.messages[-1].data is True
    assert machine.publish_ready_calls == []
    assert machine.commit_calls == []
    assert machine.transitions == []
    assert machine.failures == []


def test_normal_mode_still_uses_lift_two_phase_barrier():
    machine = aligned_machine(stop_after_align=False)

    machine.state_machine()

    assert machine.aligned_hold is False
    assert machine.publish_ready_calls == ["LIFT"]
    assert machine.commit_calls == ["LIFT"]
    assert machine.failures == ["ALIGN_TIMEOUT"]


def test_lift_commit_is_ignored_in_aligned_hold_mode():
    machine = RobotStateMachineNode.__new__(RobotStateMachineNode)
    machine.role = "front"
    machine.stop_after_align = True
    machine.last_commit_sequence = -1
    machine.committed_stages = set()
    machine.get_logger = lambda: Logger()
    machine.decode_coordination_event = lambda _msg, _role: {
        "stage": "LIFT", "sequence": 1}

    machine.commit_cb(SimpleNamespace(data="{}"))

    assert machine.last_commit_sequence == -1
    assert machine.committed_stages == set()


def test_launches_expose_and_forward_stop_after_align():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    for relative in (
            "launch/full_system.launch.py",
            "launch/front_robot.launch.py",
            "launch/rear_robot.launch.py"):
        source = (root / relative).read_text(encoding="utf-8")
        assert '"stop_after_align"' in source or "'stop_after_align'" in source
        assert "_bool(\"stop_after_align\")" in source or (
            "_bool('stop_after_align')" in source)
