import pytest

from cooperative_parking_robot.manual_control import (
    DEFAULT_ANGULAR_SPEED_RPS,
    DEFAULT_LINEAR_SPEED_MPS,
    KeyboardTeleopState,
    VelocityCommandArbiter,
    ZERO_VELOCITY,
)

# STM32 firmware 상수. 값을 바꾸면 펌웨어와 함께 갱신한다.
FIRMWARE_MANUAL_WHEEL_RAD_S = 1.2566371
FIRMWARE_WHEEL_RADIUS_M = 0.05
FIRMWARE_L_SUM_M = 0.20


def test_default_speeds_match_firmware_manual_operating_point():
    """기본 속도는 실차 검증된 12rpm 지점과 같은 바퀴 속도를 내야 한다.

    이보다 낮추면 feedforward PWM이 정상 구동 구간(186~216) 아래로 내려가
    모터가 스톨 근처에서 덜컥거린다. 실제로 0.03m/s에서 이 증상이 나왔다.
    """
    linear_wheel_rad_s = DEFAULT_LINEAR_SPEED_MPS / FIRMWARE_WHEEL_RADIUS_M
    assert linear_wheel_rad_s == pytest.approx(
        FIRMWARE_MANUAL_WHEEL_RAD_S, rel=1e-3)

    angular_wheel_rad_s = (DEFAULT_ANGULAR_SPEED_RPS * FIRMWARE_L_SUM_M
                           / FIRMWARE_WHEEL_RADIUS_M)
    assert angular_wheel_rad_s == pytest.approx(
        FIRMWARE_MANUAL_WHEEL_RAD_S, rel=1e-3)


def test_default_state_uses_default_speeds():
    state = KeyboardTeleopState()
    state.handle_key('w', 0.0)
    assert state.velocity(0.1) == (DEFAULT_LINEAR_SPEED_MPS, 0.0, 0.0)
    state.handle_key('q', 0.0)
    assert state.velocity(0.1) == (0.0, 0.0, -DEFAULT_ANGULAR_SPEED_RPS)


def test_keyboard_mapping_and_deadman_stop():
    state = KeyboardTeleopState(
        linear_speed=0.05, angular_speed=0.30, deadman_s=0.30)
    state.handle_key('w', 1.0)
    assert state.velocity(1.29) == (0.05, 0.0, 0.0)
    assert state.velocity(1.31) == ZERO_VELOCITY

    state.handle_key('a', 2.0)
    assert state.velocity(2.1) == (0.0, -0.05, 0.0)
    state.handle_key('q', 2.2)
    assert state.velocity(2.3) == (0.0, 0.0, -0.30)
    state.handle_key(' ', 2.31)
    assert state.velocity(2.31) == ZERO_VELOCITY


def test_keyboard_grip_keys_stop_before_action():
    state = KeyboardTeleopState()
    state.handle_key('d', 1.0)
    assert state.handle_key('t', 1.1) == 'grip'
    assert state.velocity(1.1) == ZERO_VELOCITY
    assert state.handle_key('g', 1.2) == 'release'


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
