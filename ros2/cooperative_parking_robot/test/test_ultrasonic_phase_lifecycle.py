"""Deterministic phase-scoped ultrasonic lifecycle regressions."""

from pathlib import Path
from types import SimpleNamespace

from cooperative_parking_robot.individual_move_node import IndividualMoveNode
from cooperative_parking_robot.stm32_bridge_node import Stm32BridgeNode
from cooperative_parking_robot.ultrasonic_phase_health import (
    UltrasonicPhaseHealth,
)
from cooperative_parking_robot.uart_protocol import UartProtocol
from cooperative_parking_robot.uart_tx_scheduler import (
    P0_EMERGENCY, P3_ACTION, UartTxScheduler,
)


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = (ROOT.parents[1] /
            'stm32/parking_robot/Core/Src/parking_robot_firmware.c')


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def info(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def health():
    return UltrasonicPhaseHealth(
        required_valid_samples=3,
        invalid_samples_to_drop=3,
        max_sample_age_s=0.35,
        activation_timeout_s=1.0,
    )


def activate_ready(state, start=10.0):
    state.start(start)
    state.acknowledge_enabled(start + 0.01)
    now = start + 0.02
    for _ in range(3):
        for side in ('left', 'right'):
            state.observe(side, True, now)
            now += 0.01
    return now


def test_startup_hardware_ready_does_not_require_ultrasonic():
    node = object.__new__(Stm32BridgeNode)
    node.ser = SimpleNamespace(is_open=True)
    node.hello_acknowledged = True
    node.last_heartbeat_ack_time = 10.0
    node.heartbeat_ack_timeout = 0.30
    node.zero_command_acknowledged = True
    node.servo_attached = True
    node.estop_latched = False
    node.active_fault = None
    node.transport_fault = None
    node.last_ultrasonic_valid = {'left': 0.0, 'right': 0.0}
    conditions = node.hardware_ready_conditions(10.1)
    assert 'ultrasonic_fresh' not in conditions
    assert all(conditions.values())


def test_startup_ultrasonic_flapping_cannot_change_global_ready():
    source = (ROOT / 'cooperative_parking_robot' /
              'stm32_bridge_node.py').read_text()
    hardware_method = source.split(
        'def hardware_ready_conditions', 1)[1].split(
        'def publish_hardware_state', 1)[0]
    assert 'ultrasonic' not in hardware_method


def test_enable_requires_ack_and_three_valid_samples_per_side():
    state = health()
    state.start(1.0)
    for side in ('left', 'right'):
        state.observe(side, True, 1.1)
    assert state.ready is False
    state.acknowledge_enabled(1.2)
    for sample in range(2):
        for side in ('left', 'right'):
            state.observe(side, True, 1.3 + sample * 0.1)
    assert state.ready is False
    state.observe('left', True, 1.5)
    assert state.ready is False
    state.observe('right', True, 1.51)
    assert state.ready is True


def test_one_invalid_does_not_flap_ready_but_three_do():
    state = health()
    activate_ready(state)
    state.observe('left', False, 10.2)
    assert state.ready is True
    state.observe('left', False, 10.3)
    assert state.ready is True
    state.observe('left', False, 10.4)
    assert state.ready is False


def test_stale_sample_drops_ready():
    state = health()
    activate_ready(state)
    assert state.update(10.39) is True
    assert state.update(10.42) is False


def test_new_activation_cannot_reuse_previous_samples():
    state = health()
    activate_ready(state)
    old_generation = state.generation
    state.start(20.0)
    assert state.generation == old_generation + 1
    assert state.ready is False
    assert state.last_valid_at == {'left': 0.0, 'right': 0.0}


def test_front_and_rear_readiness_are_independent():
    front = health()
    rear = health()
    activate_ready(front)
    rear.start(10.0)
    rear.acknowledge_enabled(10.01)
    assert front.ready is True
    assert rear.ready is False
    assert (front.ready and rear.ready) is False


def motion_for_phase(phase, simultaneous=False):
    node = object.__new__(IndividualMoveNode)
    node.role = 'front'
    node.is_front = True
    node.phase = phase
    node.active_target = (0.0, 0.0, 0.0)
    node.simultaneous_entry = simultaneous
    node.peer_robot_state = 'ALIGN'
    node.peer_motion_phase = 'PREALIGNED'
    node.peer_ultrasonic_ready = False
    node.ultrasonic_ready = False
    node.ultrasonic_requested = False
    node.ultrasonic_activation_timeout = 1.5
    node.phase_enter_time = 10.0
    node.phase_timed_out = lambda: False
    node.current_vehicle_pose = lambda: (0.0, 0.0, 0.0)
    node.command_vehicle_axis = lambda speed: None
    node.stop = lambda: None
    node.get_logger = lambda: Logger()
    node.phases = []
    node.set_phase = lambda value: (
        node.phases.append(value), setattr(node, 'phase', value))
    node.enable_requests = []
    node.request_ultrasonic = lambda enabled, **_kwargs: (
        node.enable_requests.append(bool(enabled)))
    node.faults = []
    node.fault = lambda reason: node.faults.append(reason)
    return node


def test_prealign_completion_requests_enable_once_before_scan():
    node = motion_for_phase('PRE_ALIGN')
    node.centerline_tolerance = 0.01
    node.yaw_tolerance = 0.1
    node.prealign_hold_n = 3
    node.prealign_ok_n = 2
    node.deviation_cnt = 0
    IndividualMoveNode.run_align(node)
    assert node.enable_requests == [True]
    assert node.phase == 'WAIT_ULTRASONIC_READY'
    assert 'SCAN_IN' not in node.phases


def test_entry_waits_for_local_and_in_simultaneous_mode_peer_ready(monkeypatch):
    node = motion_for_phase('WAIT_ULTRASONIC_READY')
    monkeypatch.setattr(
        'cooperative_parking_robot.individual_move_node.time.monotonic',
        lambda: 10.1)
    IndividualMoveNode.run_align(node)
    assert node.phases == []
    node.ultrasonic_ready = True
    IndividualMoveNode.run_align(node)
    assert node.phase == 'SCAN_IN'

    pair = motion_for_phase('PREALIGNED', simultaneous=True)
    pair.ultrasonic_ready = True
    IndividualMoveNode.run_align(pair)
    assert pair.phases == []
    pair.peer_ultrasonic_ready = True
    IndividualMoveNode.run_align(pair)
    assert pair.phase == 'SCAN_IN'


def test_scan_and_centering_loss_stop_with_phase_fault():
    for phase, expected in (
            ('SCAN_IN', 'ULTRASONIC_LOST_DURING_SCAN'),
            ('CENTER_AXLE', 'ULTRASONIC_LOST_DURING_CENTERING')):
        node = motion_for_phase(phase)
        node.ultrasonic_ready = False
        IndividualMoveNode.run_align(node)
        assert node.faults == [expected]


def test_fault_and_final_alignment_disable_ultrasonic():
    node = motion_for_phase('FAULT')
    node.fault_sent = False
    node.pub_fault = Publisher()
    IndividualMoveNode.fault(node, 'ABORT')
    assert node.enable_requests == [False]

    source = (ROOT / 'cooperative_parking_robot' /
              'individual_move_node.py').read_text()
    center = source.split('if self.phase == "CENTER_AXLE":', 1)[1].split(
        'def enter_prealign', 1)[0]
    assert 'self.request_ultrasonic(False)' in center
    assert center.index('self.request_ultrasonic(False)') < center.index(
        'self.set_phase("ALIGNED")')


def test_estop_preempts_ultrasonic_control():
    tx = UartTxScheduler(lambda: None, clock=lambda: 0.0)
    tx.enqueue(b'@U,ON\n', kind='ultrasonic_control', priority=P3_ACTION)
    tx.enqueue(b'@ESTOP\n', kind='estop', priority=P0_EMERGENCY)
    assert tx.pop_next().payload == b'@ESTOP\n'


def test_firmware_physically_stops_trigger_and_pending_uart_frames():
    source = FIRMWARE.read_text()
    assert 'strcmp(cmd, "U,ON") == 0' in source
    assert 'strcmp(cmd, "U,OFF") == 0' in source
    assert 'if (!g_ultrasonic.enabled) return;' in source
    assert 'g_ultrasonic_tx_pending[side] = 0U;' in source
    assert UartProtocol().encode_ultrasonic(True) == '@U,ON\n'
    assert UartProtocol().encode_ultrasonic(False) == '@U,OFF\n'
