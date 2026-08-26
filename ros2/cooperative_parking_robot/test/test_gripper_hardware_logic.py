import math
from pathlib import Path

import pytest

from cooperative_parking_robot.uart_protocol import UartProtocol
from cooperative_parking_robot.wheel_edge_detector import (
    DualWheelEdgeDetector,
    SideEdgeDetector,
)


def test_uart_protocol_rejects_bad_servo_action():
    with pytest.raises(ValueError):
        UartProtocol().encode_servo('close')


def test_uart_protocol_parses_hardware_frames_strictly():
    protocol = UartProtocol()
    assert protocol.parse('E,1,2,3,4')['values'] == [1, 2, 3, 4]
    assert protocol.parse('E,1,2,3,4,5') is None
    assert protocol.parse('LIFT,GRIP_DONE')['status'] == 'GRIP_DONE'
    assert protocol.parse('ACK,123.0')['value'] == '123.0'
    assert protocol.parse('ERR,STALL,FL')['code'] == 'STALL,FL'
    left = protocol.parse('U,L,83')
    assert left == {
        'type': 'ultrasonic', 'side': 'left', 'valid': True,
        'status': 'OK', 'distance_mm': 83, 'distance_m': 0.083,
    }
    timeout = protocol.parse('U,R,TIMEOUT')
    assert timeout['type'] == 'ultrasonic'
    assert timeout['side'] == 'right' and not timeout['valid']
    assert math.isinf(timeout['distance_m'])
    assert protocol.parse('U,X,83') is None
    assert protocol.parse('U,L,-1') is None


def test_dual_ultrasonic_edges_return_average_center():
    detector = DualWheelEdgeDetector(
        threshold_m=0.10, exit_hysteresis_m=0.02, window_size=1)
    assert detector.update('left', 0.08, 1.00, 0.0) is None
    assert detector.update('right', 0.07, 1.02, 0.1) is None
    assert detector.update('left', 0.14, 1.20, 0.2) is None
    center = detector.update('right', 0.13, 1.18, 0.3)
    assert math.isclose(center, 1.10, abs_tol=1e-9)


def test_dual_ultrasonic_rejects_unpaired_edges():
    detector = DualWheelEdgeDetector(window_size=1, pair_timeout_s=0.5)
    detector.update('left', 0.05, 0.0, 0.0)
    detector.update('left', 0.20, 0.2, 0.1)
    assert detector.update('right', 0.20, 0.2, 0.7) is None


def test_range_infinity_closes_real_hc_sr04_wheel_edge():
    detector = SideEdgeDetector(window_size=1)
    assert detector.update(0.05, 1.00, 0.0) is None
    assert detector.update(math.inf, 1.20, 0.1) == pytest.approx(1.10)


def test_f401re_firmware_timer_map_and_startup_watchdog_guard():
    source = (Path(__file__).resolve().parents[3] /
              'stm32/parking_robot/Core/Src/'
              'parking_robot_firmware.c').read_text()
    assert 'htim8' not in source
    assert 'htim10' in source and 'htim11' in source
    assert 'heartbeat_seen' in source and 'command_seen' in source
    assert 'if (!g_robot.heartbeat_seen || !g_robot.command_seen)' in source
    assert 'kMotorCommandSign' in source and 'kEncoderSign' in source
    assert 'extern TIM_HandleTypeDef htim9' in source
    assert 'HAL_TIM_Base_Start(&htim9)' in source
    assert 'HAL_GPIO_EXTI_Callback' in source
    assert 'U,%c,%ld' in source and 'U,%c,TIMEOUT' in source


def test_firmware_estop_holds_servo_at_current_angle():
    source = (Path(__file__).resolve().parents[3] /
              'stm32/parking_robot/Core/Src/'
              'parking_robot_firmware.c').read_text()
    assert 'Robot_HoldServosImmediate();' in source
    assert 'g_robot.servo_target[i] = g_robot.servo_current[i];' in source
    assert 'g_robot.servo_motion_active = 0;' in source


def test_nominal_100mm_wheel_radius_is_consistent_across_stack():
    root = Path(__file__).resolve().parents[1]
    firmware = (
        root.parents[1] /
        'stm32/parking_robot/Core/Src/parking_robot_firmware.c').read_text()
    bridge = (root / 'cooperative_parking_robot/stm32_bridge_node.py').read_text()
    quote_map = str.maketrans({chr(34): chr(39)})
    front_launch = (
        root / 'launch/front_robot.launch.py').read_text().translate(quote_map)
    rear_launch = (
        root / 'launch/rear_robot.launch.py').read_text().translate(quote_map)
    assert '#define WHEEL_RADIUS    0.05f' in firmware
    assert "self.declare_parameter('wheel_radius', 0.05)" in bridge
    assert "'wheel_radius', default_value='0.05'" in front_launch
    assert "'wheel_radius', default_value='0.05'" in rear_launch
    assert "'wheel_radius': wheel_radius" in front_launch
    assert "'wheel_radius': wheel_radius" in rear_launch


def test_bridge_does_not_flood_velocity_frames_after_estop():
    source = (Path(__file__).resolve().parents[1] /
              'cooperative_parking_robot/stm32_bridge_node.py').read_text()
    assert 'if self.estop_latched:\n            # STM32는 ESTOP' in source
    assert 'if msg.data and not self.estop_latched:' in source
