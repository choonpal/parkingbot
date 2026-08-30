"""시간·속도 지정 단발 주행 검증.

키보드 조종은 사람이 키를 누르고 있어야 해서 같은 조건을 다시 만들 수 없다.
이 노드는 '명령 속도 x 시간 = 거리'가 성립해야 엔코더 보정에 쓸 수 있고,
잘못된 입력에서는 **아무 명령도 내보내지 않아야** 한다.
"""

import ast
import math
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'drive_pulse_node.py')

FUNCTIONS = {'validate_pulse', 'pulse_velocity', 'expected_distance_m'}
CONSTANTS = {'MAX_LINEAR_MPS', 'MAX_ANGULAR_RPS', 'MAX_SECONDS',
             'NOMINAL_LINEAR_MPS'}


def _load():
    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    namespace = {'math': math}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            exec(compile(ast.Module([node], []), SOURCE, 'exec'), namespace)
        elif (isinstance(node, ast.Assign)
              and any(getattr(t, 'id', None) in CONSTANTS
                      for t in node.targets)):
            exec(compile(ast.Module([node], []), SOURCE, 'exec'), namespace)
    missing = FUNCTIONS - set(namespace)
    assert not missing, f'drive_pulse_node 에서 못 찾은 함수: {missing}'
    return namespace


NS = _load()
validate_pulse = NS['validate_pulse']
pulse_velocity = NS['pulse_velocity']
expected_distance_m = NS['expected_distance_m']


# --------------------------------------------------------------- 입력 검증
def test_nominal_pulse_is_accepted():
    plan = validate_pulse(NS['NOMINAL_LINEAR_MPS'], 0.0, 0.0, 1.6)
    assert plan['vx'] == pytest.approx(0.0628)
    assert plan['seconds'] == pytest.approx(1.6)


@pytest.mark.parametrize('vx,vy,wz,seconds', [
    (10.0, 0.0, 0.0, 1.0),        # 선속도 상한 초과 (오타 방지)
    (0.0, 0.0, 5.0, 1.0),         # 각속도 상한 초과
    (0.05, 0.0, 0.0, 60.0),       # 시간 상한 초과
    (0.05, 0.0, 0.0, 0.0),        # 시간 0
    (0.05, 0.0, 0.0, -1.0),       # 음수 시간
    (0.0, 0.0, 0.0, 1.0),         # 움직일 명령 없음
    (float('nan'), 0.0, 0.0, 1.0),
    (float('inf'), 0.0, 0.0, 1.0),
])
def test_bad_input_is_rejected(vx, vy, wz, seconds):
    with pytest.raises(ValueError):
        validate_pulse(vx, vy, wz, seconds)


def test_diagonal_speed_uses_the_magnitude_not_each_axis():
    """vx, vy 각각은 상한 안이어도 합성 속도는 넘을 수 있다."""
    with pytest.raises(ValueError):
        validate_pulse(0.12, 0.12, 0.0, 1.0)      # 합성 0.170 > 0.15
    validate_pulse(0.10, 0.10, 0.0, 1.0)          # 합성 0.141 -> 통과


def test_limits_can_be_raised_explicitly():
    """상한은 현장에서 올릴 수 있어야 하되, 기본값은 보수적이어야 한다."""
    with pytest.raises(ValueError):
        validate_pulse(0.30, 0.0, 0.0, 1.0)
    plan = validate_pulse(0.30, 0.0, 0.0, 1.0, max_linear=0.5)
    assert plan['vx'] == pytest.approx(0.30)


def test_over_limit_is_rejected_not_clamped():
    """조용히 줄이면 사용자가 넣은 값대로 갔다고 믿는다."""
    with pytest.raises(ValueError) as excinfo:
        validate_pulse(0.20, 0.0, 0.0, 1.0)
    assert '상한' in str(excinfo.value)


# ------------------------------------------------------------ 구간 진행
def test_velocity_holds_until_the_deadline():
    for elapsed in (0.0, 0.5, 1.59):
        assert pulse_velocity(elapsed, 1.6, 0.0628, 0.0, 0.0) == (
            0.0628, 0.0, 0.0)


def test_velocity_stops_exactly_at_the_deadline():
    assert pulse_velocity(1.6, 1.6, 0.0628, 0.0, 0.0) is None
    assert pulse_velocity(99.0, 1.6, 0.0628, 0.0, 0.0) is None


def test_no_ramp_so_distance_stays_predictable():
    """가감속을 넣으면 '명령 속도 x 시간'이 거리와 안 맞는다."""
    samples = [pulse_velocity(t / 100.0, 1.6, 0.0628, 0.0, 0.0)
               for t in range(0, 160)]
    assert all(sample == (0.0628, 0.0, 0.0) for sample in samples)


def test_negative_elapsed_is_a_bug_not_a_silent_zero():
    with pytest.raises(ValueError):
        pulse_velocity(-0.1, 1.6, 0.0628, 0.0, 0.0)


# ------------------------------------------------------------- 예상 거리
def test_expected_distance_matches_the_10cm_recipe():
    assert expected_distance_m(0.0628, 0.0, 1.6) == pytest.approx(0.1005,
                                                                  abs=1e-4)


def test_expected_distance_uses_both_axes():
    assert expected_distance_m(0.03, 0.04, 2.0) == pytest.approx(0.10)


def test_pulse_uses_role_scoped_base_frame():
    with open(SOURCE, encoding='utf-8') as handle:
        source = handle.read()
    assert "msg.header.frame_id = f'{self.role}_base'" in source


def test_pulse_waits_for_bridge_manual_ack_before_timing():
    with open(SOURCE, encoding='utf-8') as handle:
        source = handle.read()
    assert "f'/{self.role}/manual_active'" in source
    assert 'self.bridge_is_ready()' in source
    assert 'self.started_at = time.monotonic()' in source


def test_pure_rotation_travels_no_distance():
    assert expected_distance_m(0.0, 0.0, 3.0) == 0.0


# ------------------------------------------------------------ 안전 기본값
def test_defaults_are_conservative():
    assert NS['MAX_LINEAR_MPS'] <= 0.15
    assert NS['MAX_SECONDS'] <= 10.0
    # 확인된 구동점보다 낮은 기본 상한은 쓸모가 없다.
    assert NS['NOMINAL_LINEAR_MPS'] < NS['MAX_LINEAR_MPS']


def test_confirm_clear_defaults_to_false_in_source():
    """기본이 true 면 실행만으로 로봇이 움직인다."""
    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    declarations = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, 'attr', None) == 'declare_parameter'
        and node.args
        and getattr(node.args[0], 'value', None) == 'confirm_clear']
    assert declarations, 'confirm_clear 파라미터 선언을 못 찾았습니다'
    assert declarations[0].args[1].value is False
