import pytest

from cooperative_parking_robot.command_arbiter import (
    VelocityCommandArbiter,
    ZERO_VELOCITY,
)


def test_manual_override_never_falls_through_when_link_stales():
    arbiter = VelocityCommandArbiter(
        manual_timeout_s=0.25, release_guard_s=0.50)
    arbiter.update_auto((0.1, 0.0, 0.0), 1.0)
    arbiter.set_manual_enabled(True, 1.1)
    assert arbiter.update_manual((0.0, 0.05, 0.0), 1.1)
    assert arbiter.output(1.2) == (0.0, 0.05, 0.0)
    # 수동 PC가 끊겨도 저장된 auto 명령으로 돌아가지 않고 정지한다.
    assert arbiter.output(1.4) == ZERO_VELOCITY


def test_manual_release_has_zero_speed_guard_before_auto_resumes():
    arbiter = VelocityCommandArbiter(release_guard_s=0.50)
    arbiter.set_manual_enabled(True, 1.0)
    arbiter.set_manual_enabled(False, 2.0)
    arbiter.update_auto((0.1, 0.0, 0.0), 2.1)
    assert arbiter.output(2.49) == ZERO_VELOCITY
    assert arbiter.output(2.50) == pytest.approx((1.0 / 30.0, 0.0, 0.0))


def test_manual_command_is_rejected_when_override_is_off():
    arbiter = VelocityCommandArbiter()
    assert not arbiter.update_manual((0.1, 0.0, 0.0), 1.0)
    assert arbiter.output(1.0) == ZERO_VELOCITY
