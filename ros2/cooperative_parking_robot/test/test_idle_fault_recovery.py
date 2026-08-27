"""IDLE 과도 FAULT 의 복구 조건 검증.

기동 시 heartbeat 한 번을 놓치면 상태머신이 FAULT 로 떨어지는데, FAULT 가
``/emergency_stop`` 을 계속 발행해 STM32 를 latch 시키고, 그러면 전원 재인가
전까지 복구가 불가능한 고리가 만들어졌다. 실제로 하루 동안 이 고리에서
못 빠져나왔다.

동작 중 FAULT 는 여전히 ESTOP 을 걸어야 한다. 멈출 대상이 있기 때문이다.
IDLE 에서 빠진 FAULT 만 자동 복구 대상이다.
"""

import ast
import os
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'robot_state_machine_node.py')

METHODS = {'_idle_fault_recovered'}


def _load():
    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    methods = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            methods += [n for n in node.body
                        if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    missing = METHODS - {m.name for m in methods}
    assert not missing, f'robot_state_machine_node 에서 못 찾은 메서드: {missing}'
    holder = ast.ClassDef(name='Machine', bases=[], keywords=[],
                          body=methods, decorator_list=[])
    module = ast.Module([holder], [])
    ast.fix_missing_locations(module)
    namespace = {'time': time}
    exec(compile(module, SOURCE, 'exec'), namespace)
    return namespace


NS = _load()


def _machine(fault='ERR,HEARTBEAT_TIMEOUT', age=10.0, ready=True,
             recovery_s=3.0, require_ready=True):
    machine = NS['Machine']()
    machine.hardware_fault = fault
    machine.hardware_ready = ready
    machine.require_hardware_ready = require_ready
    machine.idle_fault_recovery_s = recovery_s
    machine.last_fault_time = time.monotonic() - age
    return machine


# --------------------------------------------------------------- 복구 허용
def test_quiet_transient_recovers():
    assert _machine().            _idle_fault_recovered() is True


def test_heartbeat_timeout_is_recoverable():
    """기동 경쟁으로 생긴 대표적인 과도현상."""
    assert _machine(fault='ERR,HEARTBEAT_TIMEOUT')._idle_fault_recovered()


def test_command_timeout_is_recoverable():
    assert _machine(fault='ERR,COMMAND_TIMEOUT')._idle_fault_recovered()


# --------------------------------------------------------------- 복구 거부
def test_estop_never_auto_recovers():
    """ESTOP latch 는 전원 재인가/리셋이 필요하다.

    소프트웨어가 임의로 풀렸다고 판단하면, 실제로는 latch 된 STM32 를
    정상이라고 믿게 된다.
    """
    for fault in ('ERR,ESTOP_LATCHED', 'ESTOP', 'ERR,estop_latched'):
        assert _machine(fault=fault)._idle_fault_recovered() is False


def test_recent_fault_does_not_recover():
    """오류가 계속 나는 중에는 복구하면 안 된다."""
    assert _machine(age=0.5)._idle_fault_recovered() is False


def test_boundary_is_inclusive():
    assert _machine(age=3.0)._idle_fault_recovered() is True
    assert _machine(age=2.9)._idle_fault_recovered() is False


def test_hardware_not_ready_blocks_recovery():
    assert _machine(ready=False)._idle_fault_recovered() is False


def test_hardware_ready_ignored_when_not_required():
    assert _machine(ready=False, require_ready=False)._idle_fault_recovered()


def test_zero_disables_auto_recovery():
    """현장에서 자동 복구를 완전히 끌 수 있어야 한다."""
    assert _machine(recovery_s=0.0)._idle_fault_recovered() is False


# ------------------------------------------------------- ESTOP 발행 규칙
def test_idle_fault_does_not_publish_estop():
    """IDLE 에서 ESTOP 을 쏘면 STM32 가 latch 되어 복구가 막힌다.

    소스에서 FAULT 진입 조건을 직접 확인한다.
    """
    with open(SOURCE, encoding='utf-8') as handle:
        text = handle.read()
    assert 'if self.state != "IDLE":\n                self.pub_estop.publish' in text, (
        'IDLE 에서 빠진 FAULT 는 ESTOP 을 발행하지 않아야 합니다')
    assert 'if self.fault_origin_state != "IDLE":\n                self.pub_estop.publish' in text, (
        'FAULT 유지 중에도 IDLE 출신이면 ESTOP 을 반복 발행하지 않아야 합니다')


def test_motion_fault_still_publishes_estop():
    """동작 중 FAULT 는 멈출 대상이 있으므로 ESTOP 을 유지해야 한다."""
    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    fail = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == 'fail')
    published = [
        node for node in ast.walk(fail)
        if isinstance(node, ast.Call)
        and getattr(node.func, 'attr', None) == 'publish']
    assert published, 'fail() 은 반드시 ESTOP 을 발행해야 합니다'
