"""Regression: current hardware faults are latched even while IDLE."""

import ast
from pathlib import Path
from types import SimpleNamespace

from std_msgs.msg import Bool


SOURCE = Path(__file__).parents[1] / (
    'cooperative_parking_robot/robot_state_machine_node.py')


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _method(name):
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    method = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name)
    holder = ast.ClassDef(
        name='Machine', bases=[], keywords=[], body=[method], decorator_list=[])
    module = ast.Module([holder], [])
    ast.fix_missing_locations(module)
    namespace = {'Bool': Bool}
    exec(compile(module, str(SOURCE), 'exec'), namespace)
    return namespace['Machine']


def test_previous_session_information_is_not_promoted_to_fault():
    machine = _method('hardware_cb')()
    machine.hardware_fault = None
    machine.get_logger = lambda: Logger()
    machine.hardware_cb(SimpleNamespace(
        data='INFO,PREVIOUS_SESSION_FAULT:HEARTBEAT_TIMEOUT'))
    assert machine.hardware_fault is None


def test_current_communication_timeouts_are_hardware_faults():
    for code in ('ERR,HEARTBEAT_TIMEOUT', 'ERR,COMMAND_TIMEOUT',
                 'ERR,ESTOP_LATCHED', 'ESTOP'):
        machine = _method('hardware_cb')()
        machine.hardware_fault = None
        machine.role = 'front'
        machine.get_logger = lambda: Logger()
        machine.hardware_cb(SimpleNamespace(data=code))
        assert machine.hardware_fault == code


def test_idle_communication_fault_enters_fault_without_blocking_uart_recovery():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'idle_fault_recovery_s' not in source
    assert '_idle_fault_recovered' not in source
    assert 'fault_origin_state' not in source
    state_method = _method('state_machine')
    estop_method = _method('_fault_requires_estop')
    machine = state_method()
    machine._fault_requires_estop = estop_method._fault_requires_estop
    machine.active_mission_id = ''
    machine.state = 'IDLE'
    machine.hardware_fault = 'ERR,HEARTBEAT_TIMEOUT'
    machine.pub_estop = Publisher()
    machine.transition = lambda new: setattr(machine, 'state', new)
    machine.state_machine()
    assert machine.state == 'FAULT'
    assert machine.pub_estop.messages == []


def test_catastrophic_hardware_fault_still_latches_estop():
    state_method = _method('state_machine')
    estop_method = _method('_fault_requires_estop')
    machine = state_method()
    machine._fault_requires_estop = estop_method._fault_requires_estop
    machine.active_mission_id = ''
    machine.state = 'IDLE'
    machine.hardware_fault = 'ERR,WHEEL_DIR_MISMATCH'
    machine.pub_estop = Publisher()
    machine.transition = lambda new: setattr(machine, 'state', new)
    machine.state_machine()
    assert machine.state == 'FAULT'
    assert len(machine.pub_estop.messages) >= 2
    assert all(message.data for message in machine.pub_estop.messages)


def test_motion_and_mission_timeout_faults_do_not_latch_estop():
    estop_method = _method('_fault_requires_estop')
    for fault in ('MOTION,ULTRASONIC_STALE', 'APPROACH_TIMEOUT',
                  'ERR,CLOCK_SKEW:auto:STALE_STAMP'):
        assert estop_method._fault_requires_estop(fault) is False


def test_lift_while_moving_remains_a_retryable_rejection():
    machine = _method('hardware_cb')()
    machine.hardware_fault = None
    machine.role = 'front'
    machine.get_logger = lambda: Logger()
    machine.hardware_cb(SimpleNamespace(data='ERR,LIFT_WHILE_MOVING'))
    assert machine.hardware_fault is None
