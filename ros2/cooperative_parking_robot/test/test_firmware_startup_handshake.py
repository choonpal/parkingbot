"""Host-compiled regression harness for the authoritative STM32 C source."""

from pathlib import Path
import shutil
import subprocess

import pytest


REPOSITORY = Path(__file__).resolve().parents[3]
FIRMWARE_DIR = REPOSITORY / 'stm32/parking_robot/Core/Src'
STUB_DIR = Path(__file__).with_name('firmware_stub')


HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "parking_robot_firmware.c"

UART_HandleTypeDef huart2 = {USART2};
TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim9, htim10, htim11;

static uint32_t test_tick;
static HAL_StatusTypeDef next_tx_status = HAL_OK;
static char tx_frames[128][256];
static uint32_t tx_timeouts[128];
static int tx_count;

static void test_profile_constants(void) {
    assert((int)kServoOpenPulseUs[0] == EXPECTED_SERVO1_OPEN);
    assert((int)kServoGripPulseUs[0] == EXPECTED_SERVO1_GRIP);
    assert((int)kServoMinPulseUs[0] == EXPECTED_SERVO1_MIN);
    assert((int)kServoMaxPulseUs[0] == EXPECTED_SERVO1_MAX);
    assert((int)kServoOpenPulseUs[1] == EXPECTED_SERVO2_OPEN);
    assert((int)kServoGripPulseUs[1] == EXPECTED_SERVO2_GRIP);
    assert((int)kServoMinPulseUs[1] == EXPECTED_SERVO2_MIN);
    assert((int)kServoMaxPulseUs[1] == EXPECTED_SERVO2_MAX);
#if PARKING_ROBOT_PROFILE == PARKING_ROBOT_PROFILE_FRONT
    assert(&ENCODER_RL_TIMER == &htim4);
    assert(&ENCODER_RR_TIMER == &htim3);
#elif PARKING_ROBOT_PROFILE == PARKING_ROBOT_PROFILE_REAR
    assert(&ENCODER_RL_TIMER == &htim3);
    assert(&ENCODER_RR_TIMER == &htim4);
#else
#error "test requires a production firmware profile"
#endif
}

uint32_t HAL_GetTick(void) { return test_tick; }
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t c) {
    (void)h; (void)c; return 0;
}
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) {
    (void)h; (void)c; return 0;
}
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t c) {
    (void)h; (void)c; return 0;
}
int HAL_TIM_Base_Start(TIM_HandleTypeDef *h) { (void)h; return 0; }
HAL_StatusTypeDef HAL_UART_Receive_IT(
        UART_HandleTypeDef *h, uint8_t *p, uint16_t n) {
    (void)h; (void)p; (void)n; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t timeout) {
    (void)h;
    HAL_StatusTypeDef status = next_tx_status;
    next_tx_status = HAL_OK;
    if (status != HAL_OK) return status;
    assert(tx_count < 128);
    assert(n < sizeof(tx_frames[tx_count]));
    memcpy(tx_frames[tx_count], p, n);
    tx_frames[tx_count][n] = '\0';
    tx_timeouts[tx_count] = timeout;
    tx_count++;
    return HAL_OK;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t n, GPIO_PinState s) {
    (void)p; (void)n; (void)s;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n) {
    (void)p; (void)n; return GPIO_PIN_RESET;
}

static void reset_robot(void) {
    test_tick = 0U;
    tx_count = 0;
    next_tx_status = HAL_OK;
    Robot_Init();
    tx_count = 0;
}

static void command(const char *text) {
    char mutable[96];
    assert(strlen(text) < sizeof(mutable));
    strcpy(mutable, text);
    UART_ParseCommand(mutable);
    UART_SendPending();
}

static void expect_last(const char *text) {
    assert(tx_count > 0);
    if (strcmp(tx_frames[tx_count - 1], text) != 0) {
        fprintf(stderr, "expected [%s], got [%s]\n", text,
                tx_frames[tx_count - 1]);
        assert(0);
    }
}

static void establish(const char *session) {
    char frame[96];
    snprintf(frame, sizeof(frame), "HELLO,2,%s", session);
    command(frame);
    snprintf(frame, sizeof(frame), "ACK,HELLO:2:%s\n", session);
    expect_last(frame);
    snprintf(frame, sizeof(frame), "HB,%s:1", session);
    command(frame);
    snprintf(frame, sizeof(frame), "ACK,%s:1\n", session);
    expect_last(frame);
    snprintf(frame, sizeof(frame), "V,0.000,0.000,0.000,%s", session);
    command(frame);
    snprintf(frame, sizeof(frame), "ACK,V:%s\n", session);
    expect_last(frame);
}

static void test_fresh_boot_sequence(void) {
    reset_robot();
    establish("aaaaaaaaaaaaaaaa");
    assert(g_robot.protocol_session_active == 1U);
    assert(g_robot.heartbeat_seen == 1U);
    assert(g_robot.command_seen == 1U);
    assert(g_robot.servo_attached == 0U);
    char attach[64];
    snprintf(attach, sizeof(attach), "S,attach,%d,%d",
             EXPECTED_SERVO1_OPEN, EXPECTED_SERVO2_OPEN);
    command(attach);
    expect_last("ACK,SERVO_ATTACH\n");
    assert(g_robot.servo_attached == 1U);
    command("S,grip");
    expect_last("ACK,GRIP\n");
    assert((int)g_robot.servo_target[0] == EXPECTED_SERVO1_GRIP);
    assert((int)g_robot.servo_target[1] == EXPECTED_SERVO2_GRIP);
    command("S,release");
    expect_last("ACK,RELEASE\n");
    assert((int)g_robot.servo_target[0] == EXPECTED_SERVO1_OPEN);
    assert((int)g_robot.servo_target[1] == EXPECTED_SERVO2_OPEN);
}

static void test_other_profile_attach_is_rejected(void) {
    reset_robot();
    establish("aaaaaaaaaaaaaaaa");
#if PARKING_ROBOT_PROFILE == PARKING_ROBOT_PROFILE_FRONT
    command("S,attach,400,2600");
#else
    command("S,attach,2600,400");
#endif
    expect_last("ERR,BAD_SERVO_ATTACH\n");
    assert(g_robot.servo_attached == 0U);
}

static void test_previous_heartbeat_session_and_current_latch(void) {
    reset_robot();
    establish("aaaaaaaaaaaaaaaa");
    test_tick = 301U;
    Motor_PID_Task();
    UART_SendPending();
    expect_last("ERR,HEARTBEAT_TIMEOUT\n");
    command("HB,aaaaaaaaaaaaaaaa:2");
    expect_last("ERR,HEARTBEAT_TIMEOUT\n");
    command("HELLO,2,aaaaaaaaaaaaaaaa");
    expect_last("ERR,HEARTBEAT_TIMEOUT\n");

    command("HELLO,2,bbbbbbbbbbbbbbbb");
    expect_last("ACK,HELLO:2:bbbbbbbbbbbbbbbb\n");
    assert(g_robot.heartbeat_timed_out == 0U);
    assert(g_robot.command_timed_out == 0U);
    assert(g_robot.servo_attached == 0U);
}

static void test_current_command_timeout_latches(void) {
    reset_robot();
    establish("aaaaaaaaaaaaaaaa");
    test_tick = 200U;
    command("HB,aaaaaaaaaaaaaaaa:2");
    test_tick = 260U;
    command("V,0.000,0.000,0.000");
    expect_last("ERR,COMMAND_TIMEOUT\n");
    command("S,attach,2600,400");
    expect_last("ERR,STARTUP_SEQUENCE\n");
    assert(g_robot.servo_attached == 0U);
}

static void test_estop_and_noncommunication_fault_survive_hello(void) {
    reset_robot();
    command("ESTOP");
    assert(g_robot.estop_latched == 1U);
    command("HELLO,2,bbbbbbbbbbbbbbbb");
    expect_last("ERR,ESTOP_LATCHED\n");
    assert(g_robot.estop_latched == 1U);

    reset_robot();
    command("HELLO,1,badversion00000");
    expect_last("ERR,BAD_HELLO\n");
    assert(g_robot.safety_fault_latched == 1U);
    command("HELLO,2,bbbbbbbbbbbbbbbb");
    expect_last("ERR,BAD_HELLO\n");
}

static void test_complete_line_transport(void) {
    reset_robot();
    QueueAck("ONE");
    UART_SendPending();
    QueueError("COMMAND_TIMEOUT");
    UART_SendPending();
    UART_SendEncoders();
    UART_SendTelemetry();
    QueueUltrasonic(ULTRA_LEFT, 123);
    QueueUltrasonic(ULTRA_RIGHT, -1);
    UART_SendUltrasonicPending();
    assert(tx_count == 6);
    for (int index = 0; index < tx_count; index++) {
        size_t length = strlen(tx_frames[index]);
        assert(length > 0U);
        assert(tx_frames[index][length - 1U] == '\n');
        assert(strchr(tx_frames[index], '\n') ==
               &tx_frames[index][length - 1U]);
        uint32_t expected_timeout = (uint32_t)(
            (length * 10U * 1000U + UART_BAUD_RATE - 1U) /
            UART_BAUD_RATE) + UART_TX_MARGIN_MS;
        assert(tx_timeouts[index] == expected_timeout);
    }

    next_tx_status = HAL_TIMEOUT;
    UART_SendEncoders();
    assert(g_uart_tx_fault == 1U);
    assert(g_robot.target_vx == 0.0f);
}

int main(void) {
    test_profile_constants();
    test_fresh_boot_sequence();
    test_other_profile_attach_is_rejected();
    test_previous_heartbeat_session_and_current_latch();
    test_current_command_timeout_latches();
    test_estop_and_noncommunication_fault_survive_hello();
    test_complete_line_transport();
    return 0;
}
'''


@pytest.mark.parametrize(
    ('profile', 'expected'),
    (
        pytest.param(
            1, (2600, 1550, 1550, 2600, 400, 1450, 400, 1450),
            id='front-robot-2'),
        pytest.param(
            2, (400, 1600, 400, 1600, 2600, 1400, 1400, 2600),
            id='rear-robot-1'),
    ),
)
def test_authoritative_firmware_startup_and_uart_transport(
        tmp_path, profile, expected):
    compiler = shutil.which('gcc')
    if compiler is None:
        pytest.skip('host gcc unavailable')
    harness = tmp_path / f'firmware_handshake_harness_{profile}.c'
    binary = tmp_path / f'firmware_handshake_harness_{profile}'
    harness.write_text(HARNESS, encoding='utf-8')
    names = (
        'SERVO1_OPEN', 'SERVO1_GRIP', 'SERVO1_MIN', 'SERVO1_MAX',
        'SERVO2_OPEN', 'SERVO2_GRIP', 'SERVO2_MIN', 'SERVO2_MAX',
    )
    expected_defines = [
        f'-DEXPECTED_{name}={value}'
        for name, value in zip(names, expected)
    ]
    compile_result = subprocess.run([
        compiler, '-std=c11', '-D_GNU_SOURCE', '-Wall', '-Wextra', '-Werror',
        f'-DPARKING_ROBOT_PROFILE={profile}', *expected_defines,
        '-I', str(STUB_DIR), '-I', str(FIRMWARE_DIR),
        '-I', str(REPOSITORY / 'stm32/parking_robot/Core/Inc'),
        str(harness), '-lm', '-o', str(binary),
    ], text=True, capture_output=True)
    assert compile_result.returncode == 0, compile_result.stderr
    run_result = subprocess.run(
        [str(binary)], text=True, capture_output=True, timeout=10)
    assert run_result.returncode == 0, run_result.stderr


def test_firmware_profile_has_no_silent_default(tmp_path):
    compiler = shutil.which('gcc')
    if compiler is None:
        pytest.skip('host gcc unavailable')
    source = tmp_path / 'profile_must_be_explicit.c'
    source.write_text(
        '#include "parking_robot_firmware.h"\nint main(void) { return 0; }\n',
        encoding='utf-8')
    result = subprocess.run([
        compiler, '-std=c11', '-Wall', '-Werror',
        '-I', str(REPOSITORY / 'stm32/parking_robot/Core/Inc'),
        '-c', str(source), '-o', str(tmp_path / 'profile.o'),
    ], text=True, capture_output=True)
    assert result.returncode != 0
    assert 'must be explicitly defined' in result.stderr


def test_production_build_defines_two_named_profile_artifacts():
    project = REPOSITORY / 'stm32/parking_robot'
    cmake = (project / 'CMakeLists.txt').read_text(encoding='utf-8')
    presets = (project / 'CMakePresets.json').read_text(encoding='utf-8')
    build_script = (
        REPOSITORY / 'tools/build_stm32_firmware.sh').read_text(
            encoding='utf-8')
    assert 'parking_robot_front parking_robot_front 1' in cmake
    assert 'parking_robot_rear parking_robot_rear 2' in cmake
    assert 'build_front_firmware' in cmake
    assert 'build_rear_firmware' in cmake
    assert '"name": "front"' in presets
    assert '"name": "rear"' in presets
    assert 'front|rear|all' in build_script


def test_firmware_has_one_serialized_uart_writer():
    source = (FIRMWARE_DIR / 'parking_robot_firmware.c').read_text()
    assert source.count('HAL_UART_Transmit(') == 1
    assert 'UART_ProcessPendingCommands();' in source
    assert 'UART_QueueRxCommand(uart_rx_buf);' in source
