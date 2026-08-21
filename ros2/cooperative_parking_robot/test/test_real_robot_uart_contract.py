from pathlib import Path

from cooperative_parking_robot.uart_protocol import UartProtocol


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_SOURCE = (
    ROOT.parents[1] /
    'stm32/parking_robot/Core/Src/parking_robot_firmware.c'
)


def test_ros_frames_use_prefix_and_legacy_commands_remain_unambiguous():
    protocol = UartProtocol()
    assert protocol.encode_velocity(0.1, -0.2, 0.3) == \
        '@V,0.100,-0.200,0.300\n'
    assert protocol.encode_servo('grip') == '@S,grip\n'
    assert protocol.encode_heartbeat(1.25) == '@HB,1.250\n'
    assert protocol.encode_estop() == '@ESTOP\n'


def test_real_robot_telemetry_is_accepted_by_ros_parser():
    parsed = UartProtocol().parse(
        'T,W,120,119,121,118,200,-201,202,-203,2600,400,83,-1')
    assert parsed == {
        'type': 'telemetry',
        'command': 'W',
        'rpm_x10': [120, 119, 121, 118],
        'pwm': [200, -201, 202, -203],
        'servo_us': [2600, 400],
        'ultrasonic_mm': [83, -1],
    }


def test_production_firmware_uses_real_robot_hardware_baseline():
    source = FIRMWARE_SOURCE.read_text()
    assert '#define ENCODER_PPR     5182.0f' in source
    # 단일 바퀴 +/-120 시험에서 뒤쪽 PWM과 DIR의 논리 채널이 서로 다른 것이
    # 드러났다. DIR을 PWM과 같은 4/3 순서로 맞춘 뒤 표준 부호를 사용한다.
    assert 'kMotorCommandSign[MOTOR_NUM] = {1, -1, 1, -1}' in source
    # 2026-08-20 회전 시험으로 확정한 표준 배선 기준값.
    assert 'kEncoderSign[MOTOR_NUM] = {1, -1, 1, -1}' in source
    # 현재 보드에서 다시 실측: 물리 RL은 진단 RL(TIM4), 물리 RR은 진단
    # RR(TIM3)에만 나타났다. 직진으로는 교차를 판정할 수 없다.
    assert 'enc[] = {&htim5, &htim2, &htim4, &htim3}' in source
    assert 'kEncoder16Bit[MOTOR_NUM] = {0, 0, 1, 1}' in source
    assert 'encoder_prev[RL] = (int32_t)__HAL_TIM_GET_COUNTER(&htim4)' in source
    assert 'encoder_prev[RR] = (int32_t)__HAL_TIM_GET_COUNTER(&htim3)' in source
    # PA10=TIM1_CH3=RR, PA11=TIM1_CH4=RL이므로 뒤쪽 두 항목의 채널 순서가
    # 앞쪽과 반대인 것이 정상이다. 2026-08-20 모터 하네스는 두 번 교체되어
    # 원래 상태로 돌아왔으므로 이 표는 원래 순서를 유지한다. 이 표만 뒤집으면
    # index RL의 PWM이 RR 모터를 돌리게 되어 두 PID가 상대 바퀴를 제어한다.
    assert 'TIM_CHANNEL_4, TIM_CHANNEL_3' in source
    assert '#define MOTOR_RL_DIR_Pin       MOTOR4_DIR_Pin' in source
    assert '#define MOTOR_RR_DIR_Pin       MOTOR3_DIR_Pin' in source
    assert 'MOTOR_RL_DIR_GPIO_Port, MOTOR_RR_DIR_GPIO_Port' in source
    assert 'MOTOR_RL_DIR_Pin, MOTOR_RR_DIR_Pin' in source
    assert 'kServoOpenPulseUs[SERVO_NUM] = {2600.0f, 400.0f}' in source
    assert 'kServoOpenPulseUs[SERVO_NUM] = {400.0f, 2600.0f}' in source
    assert "if (uart_rx_byte == '@')" in source
    assert 'Legacy_ApplyCommand(uart_rx_byte)' in source
    assert 'ParseDecimalToken' in source
    assert 'sscanf(cmd, "V,%f,%f,%f"' not in source


def test_firmware_guards_against_inverted_encoder_runaway():
    """부호가 뒤집혀도 모터가 상한까지 가속하지 않고 멈춰야 한다.

    ramp가 끝난 정상상태에서만 판정하므로 정상적인 방향 전환에서는
    걸리지 않는다.
    """
    source = FIRMWARE_SOURCE.read_text()
    assert '#define WRONG_DIRECTION_LIMIT_CYCLES' in source
    assert 'wrong_direction_cycles[MOTOR_NUM]' in source
    assert 'QueueError("WHEEL_DIR_MISMATCH")' in source
    assert '#define MOTOR_TEST_PWM_MAX        120L' in source
    assert 'QueueError("BAD_MOTOR_TEST")' in source
    assert 'now - g_robot.last_cmd_time > COMMAND_TIMEOUT_MS' in source
    # ramp 중에는 판정하지 않는다는 조건이 빠지면 방향 전환에서 오작동한다.
    assert 'if (current == requested_rpm_x10 &&' in source


def test_cubemx_settings_match_real_robot_pwm_and_encoder_filter():
    project = ROOT.parents[1] / 'stm32/parking_robot'
    main = (project / 'Core/Src/main.c').read_text()
    ioc = (project / 'parking_robot.ioc').read_text()
    msp = (project / 'Core/Src/stm32f4xx_hal_msp.c').read_text()
    assert 'htim1.Init.Prescaler = 83;' in main
    assert 'htim1.Init.Period = 999;' in main
    # 2026-08-20: 뒷바퀴 엔코더가 모터 스위칭 잡음으로 가짜 펄스를 세어
    # PID가 발산했다. 입력 필터를 최대값 15로 올려 약 3.05us 이하 글리치를
    # 걸러낸다. 12rpm에서 실제 엣지 간격은 약 965us라 300배 여유가 있다.
    assert main.count('sConfig.IC1Filter = 15;') == 4
    assert main.count('sConfig.IC2Filter = 15;') == 4
    # CubeMX 재생성으로 되돌아가지 않도록 .ioc도 함께 유지한다.
    for timer in ('TIM2', 'TIM3', 'TIM4', 'TIM5'):
        assert f'{timer}.IC1Filter=15' in ioc
        assert f'{timer}.IC2Filter=15' in ioc
    assert 'TIM1.Period=999' in ioc and 'TIM1.Prescaler=83' in ioc
    assert msp.count('GPIO_InitStruct.Pull = GPIO_PULLUP;') >= 5
