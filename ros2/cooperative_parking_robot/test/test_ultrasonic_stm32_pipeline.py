"""STM32 ultrasonic -> UART -> ROS Range -> vehicle-frame edge contract."""

from pathlib import Path

import pytest

from cooperative_parking_robot.uart_protocol import UartProtocol
from cooperative_parking_robot.vehicle_entry import projected_robot_x_offset
from cooperative_parking_robot.wheel_edge_detector import gripper_target_base_x


ROOT = Path(__file__).resolve().parents[1]


def test_uart_ultrasonic_frames_are_strict_and_unit_safe():
    protocol = UartProtocol()
    assert protocol.parse("U,L,100")["distance_m"] == 0.1
    assert protocol.parse("U,R,4000")["distance_m"] == 4.0
    assert protocol.parse("U,L,TIMEOUT")["valid"] is False
    for invalid in (
            "U,L,0", "U,L,10001", "U,L,abc", "U,L,10,extra",
            "U,LEFT,100", "U,,100"):
        assert protocol.parse(invalid) is None


def test_bridge_publishes_exact_topics_consumed_by_edge_node():
    bridge = (
        ROOT / "cooperative_parking_robot/stm32_bridge_node.py"
    ).read_text()
    edge = (
        ROOT / "cooperative_parking_robot/ultrasonic_edge_node.py"
    ).read_text()
    for side in ("left", "right"):
        single_quoted = "f'/{self.role}/ultrasonic_{side}'"
        double_quoted = 'f"/{self.role}/ultrasonic_{side}"'
        assert single_quoted in bridge
        assert double_quoted in edge
    assert "elif parsed['type'] == 'ultrasonic':" in bridge
    assert "self.publish_ultrasonic(parsed)" in bridge
    assert "Range.ULTRASOUND" in bridge
    assert "RPi.GPIO" not in edge
    assert "declare_parameter('use_gpio'" not in edge


def test_launches_select_stm32_ultrasonic_path_only():
    for relative in (
            "launch/front_robot.launch.py",
            "launch/rear_robot.launch.py",
            "launch/full_system.launch.py"):
        source = (ROOT / relative).read_text()
        assert "use_gpio" not in source
        assert "left_trig_pin" not in source
        assert "right_echo_pin" not in source
    for relative in (
            "launch/front_robot.launch.py",
            "launch/rear_robot.launch.py"):
        source = (ROOT / relative).read_text()
        source = source.translate(str.maketrans({chr(34): chr(39)}))
        assert "'require_ultrasonic_for_ready'" in source
        assert "'left_sensor_to_gripper_x_m'" in source
        assert "'right_sensor_to_gripper_x_m'" in source

    full = (ROOT / "launch/full_system.launch.py").read_text()
    for name in (
        "ultrasonic_frame_timeout_s",
        "ultrasonic_threshold_m",
        "ultrasonic_exit_hysteresis_m",
        "front_left_sensor_to_gripper_x_m",
        "front_right_sensor_to_gripper_x_m",
        "rear_left_sensor_to_gripper_x_m",
        "rear_right_sensor_to_gripper_x_m",
    ):
        assert name in full


def test_sensor_to_gripper_offset_is_projected_before_edge_detection():
    # Legacy yaw=0 behavior remains numerically compatible.
    assert gripper_target_base_x(1.000, 0.025) == 0.975
    assert gripper_target_base_x(1.000, -0.015) == 1.015
    assert projected_robot_x_offset(
        0.025, 0.0, 0.0) == pytest.approx(0.025)
    source = (
        ROOT / "cooperative_parking_robot/ultrasonic_edge_node.py"
    ).read_text()
    assert "projected_robot_x_offset(" in source
    assert "offset_s" in source
    assert "corrected_s" in source


def test_firmware_alternates_sensors_and_uses_timer_exti():
    source = (
        ROOT.parents[1] /
        "stm32/parking_robot/Core/Src/parking_robot_firmware.c"
    ).read_text()
    required = (
        "ULTRASONIC_INTERVAL_MS 35U",
        "ULTRASONIC_TIMEOUT_MS  25U",
        "HAL_TIM_Base_Start(&htim9)",
        "HAL_GPIO_EXTI_Callback",
        "Ultrasonic_StartMeasurement",
        "UART_SendUltrasonicPending",
        "g_ultrasonic.next_side",
        "QueueUltrasonic(timed_out_side, -1)",
    )
    for text in required:
        assert text in source


def test_preflight_no_longer_requires_rpi_gpio_backend():
    source = (
        ROOT / "cooperative_parking_robot/hardware_preflight.py"
    ).read_text()
    assert "RPi.GPIO" not in source
    assert "--skip-gpio" not in source
