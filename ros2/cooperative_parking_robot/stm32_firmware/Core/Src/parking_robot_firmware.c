/*
 * ==================================================
 * [파트 3] STM32 펌웨어 — 주차 로봇 모터/서보 제어
 * ==================================================
 * 라즈베리파이(ROS2)와 UART로 통신하며 메카넘 모터 4개 +
 * arm 서보 2개를 제어.
 *
 * 4개 태스크 (FreeRTOS 또는 메인 루프 기반):
 *   3-1. uart_comm_task   : 라파 통신 (속도 수신, 엔코더 송신)
 *   3-2. motor_pid_task   : 모터 속도 PID 제어
 *   3-3. servo_lift_task  : arm 서보 Soft-start 제어
 *   3-4. ultrasonic_task  : HC-SR04 좌/우 교대 측정 + UART 송신
 *
 * UART 프로토콜:
 *   수신: "V,vx,vy,omega\n"  (속도 명령, m/s)
 *         "S,grip\n" / "S,release\n"  (arm 제어)
 *   송신: "E,fl,fr,rl,rr\n"  (엔코더 카운트)
 *         "U,L,83\n" / "U,R,86\n"  (초음파 거리 mm)
 *         "U,L,TIMEOUT\n"              (Echo timeout)
 *
 * 환경: STM32 Nucleo (F4/F7/G4), HAL 라이브러리
 * ==================================================
 */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

/* ===== 하드웨어 핸들 (CubeMX 생성) ===== */
extern UART_HandleTypeDef huart2;     // 라즈베리파이 통신
extern TIM_HandleTypeDef htim1;       // 모터 PWM CH1~4
extern TIM_HandleTypeDef htim2;       // 엔코더 (FL, 32-bit)
extern TIM_HandleTypeDef htim3;       // 엔코더 (FR, 16-bit)
extern TIM_HandleTypeDef htim4;       // 엔코더 (RL, 16-bit)
extern TIM_HandleTypeDef htim5;       // 엔코더 (RR, 32-bit)
extern TIM_HandleTypeDef htim9;       // 초음파 1 MHz free-running timebase
extern TIM_HandleTypeDef htim10;      // 좌 서보 PWM CH1
extern TIM_HandleTypeDef htim11;      // 우 서보 PWM CH1


/* CubeIDE 프로젝트의 MOTOR1~4를 FL/FR/RL/RR 순서로 사용한다. */
#ifndef MOTOR_FL_DIR_GPIO_Port
#define MOTOR_FL_DIR_GPIO_Port MOTOR1_DIR_GPIO_Port
#define MOTOR_FL_DIR_Pin       MOTOR1_DIR_Pin
#define MOTOR_FR_DIR_GPIO_Port MOTOR2_DIR_GPIO_Port
#define MOTOR_FR_DIR_Pin       MOTOR2_DIR_Pin
#define MOTOR_RL_DIR_GPIO_Port MOTOR3_DIR_GPIO_Port
#define MOTOR_RL_DIR_Pin       MOTOR3_DIR_Pin
#define MOTOR_RR_DIR_GPIO_Port MOTOR4_DIR_GPIO_Port
#define MOTOR_RR_DIR_Pin       MOTOR4_DIR_Pin
#endif

/* parking_robot.ioc의 초음파 배치를 그대로 사용한다.
 *   Left  = ULTRASONIC1: TRIG PC8, ECHO PC6
 *   Right = ULTRASONIC2: TRIG PC5, ECHO PC7
 * ECHO는 5V 신호이므로 외부 저항분압 또는 레벨시프터를 사용한다. */
#define ULTRASONIC_LEFT_TRIG_GPIO_Port  ULTRASONIC1_TRIG_GPIO_Port
#define ULTRASONIC_LEFT_TRIG_Pin        ULTRASONIC1_TRIG_Pin
#define ULTRASONIC_RIGHT_TRIG_GPIO_Port ULTRASONIC2_TRIG_GPIO_Port
#define ULTRASONIC_RIGHT_TRIG_Pin       ULTRASONIC2_TRIG_Pin
#define ULTRASONIC_LEFT_ECHO_GPIO_Port  ULTRASONIC1_ECHO_GPIO_Port
#define ULTRASONIC_LEFT_ECHO_Pin        ULTRASONIC1_ECHO_Pin
#define ULTRASONIC_RIGHT_ECHO_GPIO_Port ULTRASONIC2_ECHO_GPIO_Port
#define ULTRASONIC_RIGHT_ECHO_Pin       ULTRASONIC2_ECHO_Pin

/* ===== 상수 ===== */
#define WHEEL_RADIUS    0.05f      // 100mm 메카넘 명목 반경; 유효반경 실측 후 확정
#define LX              0.10f      // 좌우 바퀴 거리/2
#define LY              0.10f      // 전후 바퀴 거리/2
#define ENCODER_PPR     2600.0f    // 26PPR * 100 감속비
#define CONTROL_HZ      100.0f     // 제어 주기
#define DT              (1.0f / CONTROL_HZ)
#define HEARTBEAT_TIMEOUT_MS 300U
#define COMMAND_TIMEOUT_MS   300U
#define SERVO_TIMEOUT_MS    5000U
#define STALL_LIMIT_CYCLES    50U   // 100Hz에서 0.5초
#define STALL_TARGET_RAD_S   0.50f
#define MAX_LINEAR_MPS       0.25f
#define MAX_ANGULAR_RAD_S    1.00f
#define MOTOR_PWM_COMMAND_MAX 999.0f
#define SERVO_NUM               2
#define ULTRASONIC_NUM          2
#define ULTRASONIC_TRIGGER_US  10U
#define ULTRASONIC_INTERVAL_MS 35U
#define ULTRASONIC_TIMEOUT_MS  25U
#define ULTRASONIC_MIN_MM      20U
#define ULTRASONIC_MAX_MM    4000U

/* ===== 메카넘 모터 인덱스 ===== */
enum { FL = 0, FR, RL, RR, MOTOR_NUM };
enum { ULTRA_LEFT = 0, ULTRA_RIGHT, ULTRA_COUNT };

static GPIO_TypeDef * const kUltrasonicTrigPort[ULTRASONIC_NUM] = {
    ULTRASONIC_LEFT_TRIG_GPIO_Port, ULTRASONIC_RIGHT_TRIG_GPIO_Port
};
static const uint16_t kUltrasonicTrigPin[ULTRASONIC_NUM] = {
    ULTRASONIC_LEFT_TRIG_Pin, ULTRASONIC_RIGHT_TRIG_Pin
};
static GPIO_TypeDef * const kUltrasonicEchoPort[ULTRASONIC_NUM] = {
    ULTRASONIC_LEFT_ECHO_GPIO_Port, ULTRASONIC_RIGHT_ECHO_GPIO_Port
};
static const uint16_t kUltrasonicEchoPin[ULTRASONIC_NUM] = {
    ULTRASONIC_LEFT_ECHO_Pin, ULTRASONIC_RIGHT_ECHO_Pin
};

/* 실제 배선/모터 장착 방향에 맞춰 반드시 저속 잭업 시험으로 확정한다.
 * command sign: +1이면 양의 wheel target에서 DIR=SET, -1이면 반대.
 * encoder sign: 양의 wheel 회전 때 누적 count가 증가하도록 맞춘다. */
static const int8_t kMotorCommandSign[MOTOR_NUM] = {1, 1, 1, 1};
static const int8_t kEncoderSign[MOTOR_NUM] = {1, 1, 1, 1};

/* 좌/우 기구가 mirror라면 각 값을 독립적으로 반대로 튜닝한다. */
static const float kServoOpenDeg[SERVO_NUM] = {30.0f, 30.0f};
static const float kServoGripDeg[SERVO_NUM] = {90.0f, 90.0f};

/* ===== PID 구조체 ===== */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float out_limit;
} PID_t;

/* ===== 전역 상태 ===== */
typedef struct {
    float target_vx, target_vy, target_omega;  // 목표 속도 (UART 수신)
    float wheel_target[MOTOR_NUM];             // 각 바퀴 목표 속도
    float wheel_actual[MOTOR_NUM];             // 각 바퀴 실제 속도
    int32_t encoder_count[MOTOR_NUM];          // 엔코더 누적
    int32_t encoder_prev[MOTOR_NUM];           // 이전 엔코더
    int32_t encoder_delta[MOTOR_NUM];           // 이번 제어주기 델타
    PID_t pid[MOTOR_NUM];                       // 바퀴별 PID
    uint8_t servo_state;                        // 0=열림, 1=닫힘(grip)
    float servo_current[SERVO_NUM];             // 현재 서보 각도 (soft-start)
    float servo_target[SERVO_NUM];              // 목표 서보 각도
    uint8_t servo_motion_active;
    uint32_t servo_motion_start;
    uint32_t last_cmd_time;                     // 워치독용
    uint32_t last_heartbeat_time;
    uint16_t stall_cycles[MOTOR_NUM];
    uint8_t estop_latched;
    uint8_t heartbeat_seen;          // 첫 유효 HB 수신 후 timeout 감시 시작
    uint8_t command_seen;            // 첫 유효 V 수신 후 timeout 감시 시작
    uint8_t heartbeat_timed_out;
    uint8_t command_timed_out;
} RobotState_t;

RobotState_t g_robot;

typedef struct {
    uint8_t active_side;
    uint8_t next_side;
    volatile uint8_t waiting_echo;
    volatile uint8_t echo_high;
    volatile uint8_t measurement_complete;
    volatile uint16_t echo_rise_us;
    uint32_t last_trigger_ms;
    uint32_t measurement_start_ms;
} UltrasonicState_t;

static UltrasonicState_t g_ultrasonic;
/* -1은 TIMEOUT, 양수는 mm. ISR은 값을 먼저 쓰고 pending을 세운다. */
static volatile int32_t g_ultrasonic_tx_mm[ULTRASONIC_NUM];
static volatile uint8_t g_ultrasonic_tx_pending[ULTRASONIC_NUM];

/* ===== UART 수신 버퍼 ===== */
uint8_t uart_rx_byte;
char uart_rx_buf[64];
uint8_t uart_rx_idx = 0;

/* ISR에서는 blocking UART 송신을 하지 않고 main loop가 응답을 보낸다. */
#define TX_ACK          (1U << 0)
#define TX_ERR          (1U << 1)
#define TX_GRIP_DONE    (1U << 2)
#define TX_RELEASE_DONE (1U << 3)
static volatile uint8_t g_tx_flags = 0;
static char g_ack_value[32];
static char g_error_code[32];

/* main.h에 프로젝트별 prototype가 없더라도 C99에서 안전하게 컴파일되도록 선언. */
void UART_ParseCommand(char *cmd);
void Mecanum_InverseKinematics(float vx, float vy, float omega);
void Set_MotorPWM(int idx, float pwm);
void Set_ServoPWM(int idx, float angle);
static void Robot_StopMotorsImmediate(void);
static void Robot_HoldServosImmediate(void);
static bool Robot_IsStopped(void);
static void QueueAck(const char *value);
static void QueueError(const char *code);
static void UART_SendPending(void);
static void UART_SendUltrasonicPending(void);
static void QueueUltrasonic(uint8_t side, int32_t distance_mm);
static void Ultrasonic_Task(void);
static void Ultrasonic_StartMeasurement(uint8_t side);
static uint16_t Ultrasonic_Micros(void);

/* ==================================================
 * 초기화
 * ================================================== */
void Robot_Init(void)
{
    memset(&g_robot, 0, sizeof(g_robot));
    memset(&g_ultrasonic, 0, sizeof(g_ultrasonic));
    g_ultrasonic.next_side = ULTRA_LEFT;

    // 바퀴별 PID 게인 설정
    for (int i = 0; i < MOTOR_NUM; i++) {
        g_robot.pid[i].Kp = 2.0f;
        g_robot.pid[i].Ki = 0.5f;
        g_robot.pid[i].Kd = 0.1f;
        g_robot.pid[i].out_limit = MOTOR_PWM_COMMAND_MAX;
    }

    // 서보 초기값 (열림)
    g_robot.servo_state = 0;
    for (int i = 0; i < SERVO_NUM; i++) {
        g_robot.servo_current[i] = kServoOpenDeg[i];
        g_robot.servo_target[i] = kServoOpenDeg[i];
    }
    g_robot.last_cmd_time = HAL_GetTick();
    g_robot.last_heartbeat_time = HAL_GetTick();

    // 엔코더 타이머 시작
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    g_robot.encoder_prev[FL] = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    g_robot.encoder_prev[FR] = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_robot.encoder_prev[RL] = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    g_robot.encoder_prev[RR] = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

    // PWM 시작
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 모터
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); // 좌 서보
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); // 우 서보

    /* TIM9: prescaler를 1 MHz tick, period 65535로 설정한다.
     * ECHO GPIO는 EXTI rising+falling, TRIG GPIO는 push-pull output. */
    HAL_TIM_Base_Start(&htim9);
    __HAL_TIM_SET_COUNTER(&htim9, 0U);
    HAL_GPIO_WritePin(ULTRASONIC_LEFT_TRIG_GPIO_Port,
                      ULTRASONIC_LEFT_TRIG_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULTRASONIC_RIGHT_TRIG_GPIO_Port,
                      ULTRASONIC_RIGHT_TRIG_Pin, GPIO_PIN_RESET);
    g_ultrasonic.last_trigger_ms =
        HAL_GetTick() - ULTRASONIC_INTERVAL_MS;

    // UART 인터럽트 수신 시작
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

static void QueueAck(const char *value)
{
    strncpy(g_ack_value, value, sizeof(g_ack_value) - 1);
    g_ack_value[sizeof(g_ack_value) - 1] = '\0';
    g_tx_flags |= TX_ACK;
}

static void QueueError(const char *code)
{
    strncpy(g_error_code, code, sizeof(g_error_code) - 1);
    g_error_code[sizeof(g_error_code) - 1] = '\0';
    g_tx_flags |= TX_ERR;
}

static void UART_SendLine(const char *line)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)line, strlen(line), 10);
}

static void UART_SendPending(void)
{
    char buf[64];
    if (g_tx_flags & TX_ERR) {
        snprintf(buf, sizeof(buf), "ERR,%s\n", g_error_code);
        g_tx_flags &= (uint8_t)~TX_ERR;
        UART_SendLine(buf);
    } else if (g_tx_flags & TX_GRIP_DONE) {
        g_tx_flags &= (uint8_t)~TX_GRIP_DONE;
        UART_SendLine("LIFT,GRIP_DONE\n");
    } else if (g_tx_flags & TX_RELEASE_DONE) {
        g_tx_flags &= (uint8_t)~TX_RELEASE_DONE;
        UART_SendLine("LIFT,RELEASE_DONE\n");
    } else if (g_tx_flags & TX_ACK) {
        snprintf(buf, sizeof(buf), "ACK,%s\n", g_ack_value);
        g_tx_flags &= (uint8_t)~TX_ACK;
        UART_SendLine(buf);
    }
}

/* ==================================================
 * [3-1] uart_comm_task — 라즈베리파이 통신
 * ================================================== */

/* UART 수신 인터럽트 콜백 (한 바이트씩) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (uart_rx_byte == '\n') {
            uart_rx_buf[uart_rx_idx] = '\0';
            UART_ParseCommand(uart_rx_buf);
            uart_rx_idx = 0;
        } else if (uart_rx_idx < sizeof(uart_rx_buf) - 1) {
            uart_rx_buf[uart_rx_idx++] = uart_rx_byte;
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

/* 명령 파싱: V / S / HB / ESTOP. ESTOP은 전원 재인가 전까지 latch된다. */
void UART_ParseCommand(char *cmd)
{
    if (strcmp(cmd, "ESTOP") == 0) {
        g_robot.estop_latched = 1;
        Robot_StopMotorsImmediate();
        /* 하중을 갑자기 놓지 않고 현재 각도에서 servo motion을 동결한다. */
        Robot_HoldServosImmediate();
        QueueAck("ESTOP");
    } else if (strncmp(cmd, "HB,", 3) == 0 && cmd[3] != '\0') {
        g_robot.last_heartbeat_time = HAL_GetTick();
        g_robot.heartbeat_seen = 1;
        g_robot.heartbeat_timed_out = 0;
        QueueAck(&cmd[3]);
    } else if (cmd[0] == 'V') {
        // 속도 명령
        float vx, vy, omega;
        if (sscanf(cmd, "V,%f,%f,%f", &vx, &vy, &omega) == 3) {
            if (g_robot.estop_latched) {
                QueueError("ESTOP_LATCHED");
                return;
            }
            if (!isfinite(vx) || !isfinite(vy) || !isfinite(omega) ||
                fabsf(vx) > MAX_LINEAR_MPS ||
                fabsf(vy) > MAX_LINEAR_MPS ||
                fabsf(omega) > MAX_ANGULAR_RAD_S) {
                QueueError("BAD_VELOCITY");
                return;
            }
            g_robot.target_vx = vx;
            g_robot.target_vy = vy;
            g_robot.target_omega = omega;
            g_robot.last_cmd_time = HAL_GetTick();
            g_robot.command_seen = 1;
            Mecanum_InverseKinematics(vx, vy, omega);
        } else {
            QueueError("BAD_V_FRAME");
        }
    } else if (strncmp(cmd, "S,", 2) == 0) {
        if (g_robot.estop_latched) {
            QueueError("ESTOP_LATCHED");
            return;
        }
        if (!Robot_IsStopped()) {
            QueueError("LIFT_WHILE_MOVING");
            return;
        }
        uint8_t requested_state;
        uint8_t done_flag;
        if (strcmp(cmd, "S,grip") == 0) {
            requested_state = 1;
            done_flag = TX_GRIP_DONE;
        } else if (strcmp(cmd, "S,release") == 0) {
            requested_state = 0;
            done_flag = TX_RELEASE_DONE;
        } else {
            QueueError("BAD_SERVO_COMMAND");
            return;
        }

        /* 재전송은 idempotent. 진행 중 타이머를 리셋하지 않는다. */
        if (g_robot.servo_state == requested_state) {
            if (!g_robot.servo_motion_active) {
                g_tx_flags |= done_flag;
            }
        } else {
            g_robot.servo_state = requested_state;
            for (int i = 0; i < SERVO_NUM; i++) {
                g_robot.servo_target[i] = requested_state
                    ? kServoGripDeg[i] : kServoOpenDeg[i];
            }
            g_robot.servo_motion_active = 1;
            g_robot.servo_motion_start = HAL_GetTick();
        }
        QueueAck(requested_state ? "GRIP" : "RELEASE");
    } else {
        QueueError("UNKNOWN_COMMAND");
    }
}

/* 엔코더 값 송신: "E,fl,fr,rl,rr" */
void UART_SendEncoders(void)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "E,%ld,%ld,%ld,%ld\n",
                       (long)g_robot.encoder_count[FL],
                       (long)g_robot.encoder_count[FR],
                       (long)g_robot.encoder_count[RL],
                       (long)g_robot.encoder_count[RR]);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 10);
}

/* ==================================================
 * [3-4] ultrasonic_task — STM32 측정, RPi 판단
 * ================================================== */

static uint16_t Ultrasonic_Micros(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
}

static void QueueUltrasonic(uint8_t side, int32_t distance_mm)
{
    if (side >= ULTRASONIC_NUM) return;
    g_ultrasonic_tx_mm[side] = distance_mm;
    g_ultrasonic_tx_pending[side] = 1U;
}

static void UART_SendUltrasonicPending(void)
{
    char buf[32];
    static const char side_code[ULTRASONIC_NUM] = {'L', 'R'};
    for (uint8_t side = 0; side < ULTRASONIC_NUM; side++) {
        if (!g_ultrasonic_tx_pending[side]) continue;
        int32_t distance_mm = g_ultrasonic_tx_mm[side];
        g_ultrasonic_tx_pending[side] = 0U;
        if (distance_mm < 0) {
            snprintf(buf, sizeof(buf), "U,%c,TIMEOUT\n", side_code[side]);
        } else {
            snprintf(buf, sizeof(buf), "U,%c,%ld\n",
                     side_code[side], (long)distance_mm);
        }
        UART_SendLine(buf);
    }
}

static void Ultrasonic_StartMeasurement(uint8_t side)
{
    if (side >= ULTRASONIC_NUM) return;
    g_ultrasonic.active_side = side;
    g_ultrasonic.echo_high = 0U;
    g_ultrasonic.measurement_complete = 0U;
    g_ultrasonic.waiting_echo = 0U;

    /* Linux busy-wait 대신 STM32 1 MHz timer로 정확한 10 us pulse를 만든다. */
    HAL_GPIO_WritePin(kUltrasonicTrigPort[side],
                      kUltrasonicTrigPin[side], GPIO_PIN_SET);
    uint16_t start_us = Ultrasonic_Micros();
    while ((uint16_t)(Ultrasonic_Micros() - start_us) <
           ULTRASONIC_TRIGGER_US) {
        /* 최대 10 us만 blocking */
    }
    HAL_GPIO_WritePin(kUltrasonicTrigPort[side],
                      kUltrasonicTrigPin[side], GPIO_PIN_RESET);

    g_ultrasonic.measurement_start_ms = HAL_GetTick();
    g_ultrasonic.last_trigger_ms = g_ultrasonic.measurement_start_ms;
    g_ultrasonic.waiting_echo = 1U;
}

/* CubeMX에서 두 ECHO pin을 GPIO_EXTI rising+falling으로 설정한다. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t side;
    if (GPIO_Pin == kUltrasonicEchoPin[ULTRA_LEFT]) {
        side = ULTRA_LEFT;
    } else if (GPIO_Pin == kUltrasonicEchoPin[ULTRA_RIGHT]) {
        side = ULTRA_RIGHT;
    } else {
        return;
    }

    if (!g_ultrasonic.waiting_echo || side != g_ultrasonic.active_side) {
        return;
    }

    uint16_t now_us = Ultrasonic_Micros();
    GPIO_PinState level = HAL_GPIO_ReadPin(
        kUltrasonicEchoPort[side], kUltrasonicEchoPin[side]);
    if (level == GPIO_PIN_SET) {
        if (!g_ultrasonic.echo_high) {
            g_ultrasonic.echo_rise_us = now_us;
            g_ultrasonic.echo_high = 1U;
        }
        return;
    }

    if (g_ultrasonic.echo_high) {
        uint16_t pulse_us = (uint16_t)(now_us - g_ultrasonic.echo_rise_us);
        g_ultrasonic.echo_high = 0U;
        g_ultrasonic.measurement_complete = 1U;
        g_ultrasonic.waiting_echo = 0U;
        /* mm = pulse_us * 0.343 / 2. 반올림을 위해 +1000. */
        uint32_t distance_mm =
            ((uint32_t)pulse_us * 343U + 1000U) / 2000U;
        if (distance_mm >= ULTRASONIC_MIN_MM &&
            distance_mm <= ULTRASONIC_MAX_MM) {
            QueueUltrasonic(side, (int32_t)distance_mm);
        } else {
            QueueUltrasonic(side, -1);
        }
    }
}

static void Ultrasonic_Task(void)
{
    uint32_t now = HAL_GetTick();
    if (g_ultrasonic.waiting_echo &&
        now - g_ultrasonic.measurement_start_ms >= ULTRASONIC_TIMEOUT_MS) {
        uint8_t timed_out_side = g_ultrasonic.active_side;
        /* 먼저 waiting을 내려 이후 EXTI를 무시한다. 직전에 정상 falling
         * edge ISR이 실행됐다면 measurement_complete=1이라 timeout을 덮지 않는다. */
        g_ultrasonic.waiting_echo = 0U;
        g_ultrasonic.echo_high = 0U;
        if (!g_ultrasonic.measurement_complete) {
            QueueUltrasonic(timed_out_side, -1);
        }
    }

    /* 35 ms마다 좌/우를 교대한다. 센서 하나당 약 14.3 Hz이고 이전 Echo
     * timeout(25 ms)이 끝난 뒤 다음 센서를 울려 상호 간섭을 줄인다. */
    if (!g_ultrasonic.waiting_echo &&
        now - g_ultrasonic.last_trigger_ms >= ULTRASONIC_INTERVAL_MS) {
        uint8_t side = g_ultrasonic.next_side;
        g_ultrasonic.next_side =
            (side == ULTRA_LEFT) ? ULTRA_RIGHT : ULTRA_LEFT;
        Ultrasonic_StartMeasurement(side);
    }
}

/* ==================================================
 * [3-2] motor_pid_task — 모터 속도 PID 제어
 * ================================================== */

/* 메카넘 역기구학: (vx, vy, omega) → 4바퀴 속도 */
void Mecanum_InverseKinematics(float vx, float vy, float omega)
{
    float L = LX + LY;
    // 표준 메카넘 공식 (롤러 45도)
    g_robot.wheel_target[FL] = (vx - vy - L * omega) / WHEEL_RADIUS;
    g_robot.wheel_target[FR] = (vx + vy + L * omega) / WHEEL_RADIUS;
    g_robot.wheel_target[RL] = (vx + vy - L * omega) / WHEEL_RADIUS;
    g_robot.wheel_target[RR] = (vx - vy + L * omega) / WHEEL_RADIUS;
}

/* STM32F401RE에는 TIM8이 없다. 네 개의 quadrature encoder는
 * TIM2/TIM3/TIM4/TIM5에 각각 배치한다. TIM2/TIM5는 32비트,
 * TIM3/TIM4는 16비트다. enc[] 순서 FL=TIM2, FR=TIM3,
 * RL=TIM4, RR=TIM5 기준.
 * 16비트 카운터는 0~65535를 순환하는데, 이전엔 delta를 32비트 그대로
 * 빼서(cnt-prev) 카운터가 순환하는 순간마다 실제 회전량과 정반대의 거대한
 * 값(예: prev=65530,cnt=5(정상 +11회전)인데 delta=5-65530=-65525로 계산)이
 * 나오는 버그가 있었다. int16_t로 캐스팅한 차분은 2의 보수 표현 특성상
 * wraparound를 자동으로 올바르게 처리한다 — 단, 한 제어주기 실제 변화량이
 * ±32767틱을 넘지 않는다는 전제인데(CONTROL_HZ 주기당 이 정도 회전은 이
 * 로봇 최대속도로는 불가능하므로) 안전하다.
 * 실제 CubeMX .ioc에서 타이머 비트폭을 다르게 설정했다면 kEncoder16Bit를
 * 맞게 수정할 것. */
static const uint8_t kEncoder16Bit[MOTOR_NUM] = {0, 1, 1, 0};  // TIM2,3,4,5

void Update_WheelSpeeds(void)
{
    TIM_HandleTypeDef* enc[] = {&htim2, &htim3, &htim4, &htim5};
    for (int i = 0; i < MOTOR_NUM; i++) {
        uint32_t raw = __HAL_TIM_GET_COUNTER(enc[i]);
        int32_t delta;
        if (kEncoder16Bit[i]) {
            delta = (int16_t)((uint16_t)raw - (uint16_t)g_robot.encoder_prev[i]);
        } else {
            delta = (int32_t)raw - g_robot.encoder_prev[i];
        }
        delta *= kEncoderSign[i];
        g_robot.encoder_delta[i] = delta;
        g_robot.encoder_prev[i] = (int32_t)raw;
        g_robot.encoder_count[i] += delta;
        // 카운트 → rad/s
        float rev = (float)delta / ENCODER_PPR;
        g_robot.wheel_actual[i] = rev * 2.0f * M_PI * CONTROL_HZ;
    }
}

/* PID 계산 */
float PID_Compute(PID_t* pid, float target, float actual)
{
    float error = target - actual;
    pid->integral += error * DT;
    // 적분 와인드업 방지
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;
    float derivative = (error - pid->prev_error) / DT;
    pid->prev_error = error;

    float out = pid->Kp * error + pid->Ki * pid->integral
                + pid->Kd * derivative;
    if (out > pid->out_limit) out = pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    return out;
}

/* 모터 PWM 출력 (방향 + 크기) */
void Set_MotorPWM(int idx, float pwm)
{
    uint32_t ch[] = {TIM_CHANNEL_1, TIM_CHANNEL_2,
                     TIM_CHANNEL_3, TIM_CHANNEL_4};
    GPIO_TypeDef* dir_port[] = {
        MOTOR_FL_DIR_GPIO_Port, MOTOR_FR_DIR_GPIO_Port,
        MOTOR_RL_DIR_GPIO_Port, MOTOR_RR_DIR_GPIO_Port
    };
    uint16_t dir_pin[] = {
        MOTOR_FL_DIR_Pin, MOTOR_FR_DIR_Pin,
        MOTOR_RL_DIR_Pin, MOTOR_RR_DIR_Pin
    };
    if (idx < 0 || idx >= MOTOR_NUM) return;
    pwm *= (float)kMotorCommandSign[idx];

    // 방향 설정
    if (pwm >= 0) {
        HAL_GPIO_WritePin(dir_port[idx], dir_pin[idx], GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dir_port[idx], dir_pin[idx], GPIO_PIN_RESET);
        pwm = -pwm;
    }
    /* PID 출력 0~999를 CubeIDE TIM1의 실제 ARR 전체 범위로 환산한다.
     * 현재 ARR=65535이면 999 명령이 CCR=65535가 된다. ARR를 나중에
     * 바꾸더라도 duty 비율은 유지된다. */
    if (pwm > MOTOR_PWM_COMMAND_MAX) {
        pwm = MOTOR_PWM_COMMAND_MAX;
    }
    const uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t duty = (uint32_t)(
        (pwm * (float)auto_reload / MOTOR_PWM_COMMAND_MAX) + 0.5f);
    if (duty > auto_reload) {
        duty = auto_reload;
    }
    __HAL_TIM_SET_COMPARE(&htim1, ch[idx], duty);
}

static void Robot_StopMotorsImmediate(void)
{
    g_robot.target_vx = 0.0f;
    g_robot.target_vy = 0.0f;
    g_robot.target_omega = 0.0f;
    for (int i = 0; i < MOTOR_NUM; i++) {
        g_robot.wheel_target[i] = 0.0f;
        g_robot.pid[i].integral = 0.0f;
        g_robot.pid[i].prev_error = 0.0f;
        Set_MotorPWM(i, 0.0f);
    }
}

static void Robot_HoldServosImmediate(void)
{
    g_robot.servo_motion_active = 0;
    for (int i = 0; i < SERVO_NUM; i++) {
        g_robot.servo_target[i] = g_robot.servo_current[i];
        Set_ServoPWM(i, g_robot.servo_current[i]);
    }
}

static bool Robot_IsStopped(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (fabsf(g_robot.wheel_target[i]) > 0.10f ||
            fabsf(g_robot.wheel_actual[i]) > 0.20f) {
            return false;
        }
    }
    return true;
}

/* 모터 제어 주기 실행 (100Hz) */
void Motor_PID_Task(void)
{
    static const char *stall_error[MOTOR_NUM] = {
        "STALL_FL", "STALL_FR", "STALL_RL", "STALL_RR"
    };
    uint32_t now = HAL_GetTick();

    Update_WheelSpeeds();

    if (g_robot.estop_latched) {
        Robot_StopMotorsImmediate();
        return;
    }

    /* 전원 인가 직후 ROS2가 뜨기 전의 300ms를 fault로 오인하지 않는다.
     * 단, HB와 V를 각각 한 번 이상 받기 전에는 모터를 무조건 정지한다. */
    if (!g_robot.heartbeat_seen || !g_robot.command_seen) {
        Robot_StopMotorsImmediate();
        return;
    }

    if (now - g_robot.last_heartbeat_time > HEARTBEAT_TIMEOUT_MS) {
        if (!g_robot.heartbeat_timed_out) {
            g_robot.heartbeat_timed_out = 1;
            QueueError("HEARTBEAT_TIMEOUT");
        }
        Robot_StopMotorsImmediate();
        return;
    }

    if (now - g_robot.last_cmd_time > COMMAND_TIMEOUT_MS) {
        if (!g_robot.command_timed_out) {
            g_robot.command_timed_out = 1;
            QueueError("COMMAND_TIMEOUT");
        }
        Robot_StopMotorsImmediate();
        return;
    }
    g_robot.command_timed_out = 0;

    /* 명령이 있는데 엔코더가 0인 상태가 0.5초 지속되면 fault latch. */
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (fabsf(g_robot.wheel_target[i]) > STALL_TARGET_RAD_S &&
            labs(g_robot.encoder_delta[i]) <= 1) {
            if (g_robot.stall_cycles[i] < STALL_LIMIT_CYCLES) {
                g_robot.stall_cycles[i]++;
            }
        } else {
            g_robot.stall_cycles[i] = 0;
        }
        if (g_robot.stall_cycles[i] >= STALL_LIMIT_CYCLES) {
            g_robot.estop_latched = 1;
            QueueError(stall_error[i]);
            Robot_StopMotorsImmediate();
            return;
        }
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        if (fabsf(g_robot.wheel_target[i]) < 0.01f) {
            g_robot.pid[i].integral = 0.0f;
            g_robot.pid[i].prev_error = 0.0f;
            Set_MotorPWM(i, 0.0f);
            continue;
        }
        float pwm = PID_Compute(&g_robot.pid[i],
                                g_robot.wheel_target[i],
                                g_robot.wheel_actual[i]);
        Set_MotorPWM(i, pwm);
    }
}

/* ==================================================
 * [3-3] servo_lift_task — arm Soft-start 제어
 * ================================================== */

/* 서보 각도 → PWM (50Hz, 0.5~2.5ms 펄스).
 * CubeMX에서 TIM10 CH1/TIM11 CH1 활성화 필수. */
void Set_ServoPWM(int idx, float angle)
{
    TIM_HandleTypeDef *timer[SERVO_NUM] = {&htim10, &htim11};
    if (idx < 0 || idx >= SERVO_NUM) return;
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    /* TIM10/TIM11을 1 MHz tick, 20,000 period(50 Hz)로 설정한다. */
    uint32_t pulse = 500 + (uint32_t)(angle / 180.0f * 2000.0f);
    __HAL_TIM_SET_COMPARE(timer[idx], TIM_CHANNEL_1, pulse);
}

/* 서보 Soft-start: 급가동 방지 (목표까지 서서히) */
void Servo_Lift_Task(void)
{
    float step = 1.0f;   // 한 주기당 최대 1도씩 (부드럽게)
    bool all_done = true;

    for (int i = 0; i < SERVO_NUM; i++) {
        float diff = g_robot.servo_target[i] - g_robot.servo_current[i];
        if (fabsf(diff) > step) {
            g_robot.servo_current[i] += (diff > 0) ? step : -step;
            all_done = false;
        } else {
            g_robot.servo_current[i] = g_robot.servo_target[i];
        }
        Set_ServoPWM(i, g_robot.servo_current[i]);
    }

    if (!g_robot.servo_motion_active) {
        return;
    }
    if (HAL_GetTick() - g_robot.servo_motion_start > SERVO_TIMEOUT_MS) {
        g_robot.servo_motion_active = 0;
        QueueError("SERVO_TIMEOUT");
        return;
    }
    if (all_done) {
        g_robot.servo_motion_active = 0;
        g_tx_flags |= g_robot.servo_state ? TX_GRIP_DONE : TX_RELEASE_DONE;
    }
}

/* ==================================================
 * 메인 루프 (FreeRTOS 없이 타이머 기반)
 * ================================================== */
void Robot_MainLoop(void)
{
    static uint32_t last_control = 0;
    static uint32_t last_servo = 0;
    static uint32_t last_encoder_tx = 0;
    uint32_t now = HAL_GetTick();

    // 모터 PID: 100Hz (10ms)
    if (now - last_control >= 10) {
        Motor_PID_Task();
        last_control = now;
    }

    // 서보: 50Hz (20ms)
    if (now - last_servo >= 20) {
        Servo_Lift_Task();
        last_servo = now;
    }

    // 초음파: 좌/우 교대 trigger + Echo timeout 상태머신
    Ultrasonic_Task();

    /* ACK/ERR/LIFT/초음파 응답은 RX interrupt가 아니라 main loop에서 송신. */
    UART_SendPending();
    UART_SendUltrasonicPending();

    // 엔코더 송신: 50Hz (20ms)
    if (now - last_encoder_tx >= 20) {
        UART_SendEncoders();
        last_encoder_tx = now;
    }
}

/*
 * main()에서 호출 순서:
 *   Robot_Init();
 *   while (1) { Robot_MainLoop(); }
 *
 * 또는 FreeRTOS면 각 태스크를 별도 스레드로:
 *   xTaskCreate(uart_comm_task, ...);
 *   xTaskCreate(motor_pid_task, ...);
 *   xTaskCreate(servo_lift_task, ...);
 */
