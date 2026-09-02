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
 *   수신: "@V,vx,vy,omega\n"  (속도 명령, m/s)
 *         "@HELLO,2,session_id\n" (새 Linux bridge 세션 경계)
 *         "@V,0,0,0,session_id\n" (command channel startup probe)
 *         "@S,attach,pulse1_us,pulse2_us\n" (서보 기준 동기화)
 *         "@S,grip\n" / "@S,release\n"  (arm 제어)
 *         "@HB,session_id:sequence\n" / "@ESTOP\n"
 *         "@M,FL|FR|RL|RR,pwm\n" (정비용 단일 바퀴, |pwm|<=120)
 *   저수준 정비용 W/S/A/D/Q/E, U/J/I/K/T/G/O/X 단일문자 명령도
 *   지원한다. '@' prefix가 두 프로토콜의 S/E 충돌을 막는다.
 *   송신: "E,fl,fr,rl,rr\n"  (엔코더 카운트)
 *         "U,L,83\n" / "U,R,86\n"  (초음파 거리 mm)
 *         "U,L,TIMEOUT\n"              (Echo timeout)
 *
 * 환경: STM32 Nucleo (F4/F7/G4), HAL 라이브러리
 * ==================================================
 */

#include "main.h"
#include "parking_robot_firmware.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

/* ===== 하드웨어 핸들 (CubeMX 생성) ===== */
extern UART_HandleTypeDef huart2;     // 라즈베리파이 통신
extern TIM_HandleTypeDef htim1;       // 모터 PWM CH1~4
extern TIM_HandleTypeDef htim2;       // 엔코더 (FR, 32-bit)
extern TIM_HandleTypeDef htim3;       // 엔코더 (RR, 16-bit)
extern TIM_HandleTypeDef htim4;       // 엔코더 (RL, 16-bit)
extern TIM_HandleTypeDef htim5;       // 엔코더 (FL, 32-bit)
extern TIM_HandleTypeDef htim9;       // 초음파 1 MHz free-running timebase
extern TIM_HandleTypeDef htim10;      // 좌 서보 PWM CH1
extern TIM_HandleTypeDef htim11;      // 우 서보 PWM CH1

/* 두 실차의 뒤쪽 엔코더 하네스 순서가 다르다.
 * Front(robot-2)는 손 회전 실측으로 RL=TIM4, RR=TIM3을 확인했다.
 * Rear(robot-1)는 기존 실차 핀맵과 동작 코드의 RL=TIM3, RR=TIM4를 쓴다. */
#if PARKING_ROBOT_PROFILE == PARKING_ROBOT_PROFILE_FRONT
#define ENCODER_RL_TIMER htim4
#define ENCODER_RR_TIMER htim3
#elif PARKING_ROBOT_PROFILE == PARKING_ROBOT_PROFILE_REAR
#define ENCODER_RL_TIMER htim3
#define ENCODER_RR_TIMER htim4
#else
#error "PARKING_ROBOT_PROFILE must be FRONT(1) or REAR(2)"
#endif


/* 뒤쪽은 PCB 핀 배치가 PWM과 DIR 모두 4/3 순서다.
 *   RL = TIM1_CH4(PA11) + MOTOR4_DIR(PC3)
 *   RR = TIM1_CH3(PA10) + MOTOR3_DIR(PC2)
 * PWM만 4/3으로 두고 DIR을 3/4로 쓰면 각 바퀴가 상대 DIR을 보게 되어
 * 직진에서는 숨고 회전·횡이동에서만 방향 제어가 깨진다. */
#ifndef MOTOR_FL_DIR_GPIO_Port
#define MOTOR_FL_DIR_GPIO_Port MOTOR1_DIR_GPIO_Port
#define MOTOR_FL_DIR_Pin       MOTOR1_DIR_Pin
#define MOTOR_FR_DIR_GPIO_Port MOTOR2_DIR_GPIO_Port
#define MOTOR_FR_DIR_Pin       MOTOR2_DIR_Pin
#define MOTOR_RL_DIR_GPIO_Port MOTOR4_DIR_GPIO_Port
#define MOTOR_RL_DIR_Pin       MOTOR4_DIR_Pin
#define MOTOR_RR_DIR_GPIO_Port MOTOR3_DIR_GPIO_Port
#define MOTOR_RR_DIR_Pin       MOTOR3_DIR_Pin
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
#define ENCODER_PPR     5182.0f    // 실차 telemetry로 확인된 1회전 count
#define CONTROL_HZ      20.0f      // 실차에서 검증된 50 ms 제어 주기
#define DT              (1.0f / CONTROL_HZ)
#define HEARTBEAT_TIMEOUT_MS 300U
#define COMMAND_TIMEOUT_MS   250U
#define UART_PROTOCOL_VERSION   2U
#define UART_SESSION_ID_MAX    16U
#define UART_BAUD_RATE      115200U
#define UART_TX_MARGIN_MS        5U
#define SERVO_TIMEOUT_MS    5000U
#define MAX_LINEAR_MPS       0.25f
#define MAX_ANGULAR_RAD_S    1.00f
#define MOTOR_PWM_COMMAND_MAX 350.0f
#define SERVO_NUM               2
#define ULTRASONIC_NUM          2
#define ULTRASONIC_TRIGGER_US  10U
#define ULTRASONIC_INTERVAL_MS 35U
#define ULTRASONIC_TIMEOUT_MS  25U
#define ULTRASONIC_MIN_MM      20U
#define ULTRASONIC_MAX_MM    4000U
#define MANUAL_WHEEL_RAD_S     1.2566371f /* 실차 12 rpm */
#define OPEN_LOOP_PWM          180.0f
#define MOTOR_TEST_PWM_MAX        120L
#define SERVO_COMMAND_STEP_US   50.0f
#define SERVO_RAMP_STEP_US      30.0f
#define TELEMETRY_PERIOD_MS    200U
#define SPEED_KP                  3L
#define SPEED_KI_DIVISOR         32L
#define SPEED_INTEGRAL_LIMIT   3200L
#define PWM_FEEDFORWARD_AT_12RPM 200L
#define TARGET_RAMP_RPM_X10      15
/* 엔코더 부호가 뒤집히면 PID가 반대로 밀어 출력이 상한에 붙는다.
 * ramp가 끝난 정상상태에서 20cycle(1초) 동안 실제 회전이 목표와
 * 반대이면 정지시킨다. ramp 중에는 판정하지 않아 정상적인 방향
 * 전환과 구분된다. */
#define WRONG_DIRECTION_LIMIT_CYCLES 20U

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
/* 2026-08-20 오픈루프 실측으로 확정. 네 바퀴에 같은 논리 PWM +180을 줬을 때
 * FL +9.91, FR +10.02, RL -9.95, RR -10.12으로 앞뒤가 반대로 돌았다.
 * 육안으로도 앞바퀴는 뒤로, 뒷바퀴는 앞으로 돌아 서로 마주보는 상태였다.
 * 뒤쪽 두 항목을 반전해 네 바퀴가 같은 방향을 향하게 한다. */
static const int8_t kMotorCommandSign[MOTOR_NUM] = {1, -1, 1, -1};
/* enc[] 교차를 바로잡으면서 네 바퀴 규약이 통일됐으므로 표준값을 쓴다.
 * 손으로 네 바퀴를 전진 방향으로 돌린 실측으로 확인했다. 직진만으로는
 * 뒤쪽 두 채널의 교차를 볼 수 없고(두 바퀴가 같은 방향이라 값이 같다)
 * 회전을 10초 이상 유지해야 드러난다. 기존 실차 주행은 오픈루프라
 * 이 오류가 전혀 드러나지 않았다. */
static const int8_t kEncoderSign[MOTOR_NUM] = {1, -1, 1, -1};

/* 좌/우 기구가 mirror라면 각 값을 독립적으로 반대로 튜닝한다. */
#if PARKING_ROBOT_PROFILE == PARKING_ROBOT_PROFILE_FRONT
/* robot-2: ArUco가 달린 front 로봇에서 실차 검증된 pulse 범위. */
static const float kServoOpenPulseUs[SERVO_NUM] = {2600.0f, 400.0f};
static const float kServoGripPulseUs[SERVO_NUM] = {1550.0f, 1450.0f};
static const float kServoMinPulseUs[SERVO_NUM] = {1550.0f, 400.0f};
static const float kServoMaxPulseUs[SERVO_NUM] = {2600.0f, 1450.0f};
#elif PARKING_ROBOT_PROFILE == PARKING_ROBOT_PROFILE_REAR
/* robot-1: rear 로봇에서 실차 검증된 pulse 범위. */
static const float kServoOpenPulseUs[SERVO_NUM] = {400.0f, 2600.0f};
static const float kServoGripPulseUs[SERVO_NUM] = {1600.0f, 1400.0f};
static const float kServoMinPulseUs[SERVO_NUM] = {400.0f, 1400.0f};
static const float kServoMaxPulseUs[SERVO_NUM] = {1600.0f, 2600.0f};
#else
#error "PARKING_ROBOT_PROFILE must be FRONT(1) or REAR(2)"
#endif

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
    uint8_t servo_attached;                     // bridge와 pulse 기준 동기화 완료
    uint32_t servo_motion_start;
    uint32_t last_cmd_time;                     // 워치독용
    uint32_t last_heartbeat_time;
    uint16_t stall_cycles[MOTOR_NUM];
    uint8_t estop_latched;
    uint8_t heartbeat_seen;          // 첫 유효 HB 수신 후 timeout 감시 시작
    uint8_t command_seen;            // 첫 유효 V 수신 후 timeout 감시 시작
    uint8_t heartbeat_timed_out;
    uint8_t command_timed_out;
    uint8_t manual_mode;
    uint8_t manual_open_loop;
    char last_command;
    float motor_pwm[MOTOR_NUM];
    int16_t target_rpm_x10[MOTOR_NUM];
    int32_t speed_integral[MOTOR_NUM];
    uint8_t wrong_direction_cycles[MOTOR_NUM];
    uint8_t protocol_session_active;
    char session_id[UART_SESSION_ID_MAX + 1U];
    uint8_t safety_fault_latched;
    char safety_fault_code[32];
} RobotState_t;

RobotState_t g_robot;

typedef struct {
    uint8_t enabled;
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
static uint8_t uart_frame_active = 0U;

/* RX ISR only assembles bytes. Parsing and every hardware action happen in
 * the main loop, so UART RX cannot race a TX formatter or motor/servo state. */
#define RX_COMMAND_QUEUE_DEPTH 8U
static char g_rx_commands[RX_COMMAND_QUEUE_DEPTH][sizeof(uart_rx_buf)];
static volatile uint8_t g_rx_command_head = 0U;
static volatile uint8_t g_rx_command_tail = 0U;
static volatile uint8_t g_rx_command_count = 0U;
static volatile uint8_t g_rx_queue_overflow = 0U;
static volatile uint8_t g_uart_rx_error_pending = 0U;
static volatile uint8_t g_uart_rx_rearm_pending = 0U;

/* ISR에서는 blocking UART 송신을 하지 않고 main loop가 응답을 보낸다. */
#define TX_ACK          (1U << 0)
#define TX_ERR          (1U << 1)
#define TX_GRIP_DONE    (1U << 2)
#define TX_RELEASE_DONE (1U << 3)
#define TX_HEARTBEAT_ACK (1U << 4)
static volatile uint8_t g_tx_flags = 0;
static char g_ack_value[48];
static char g_heartbeat_ack_value[48];
static char g_error_code[32];
static uint8_t g_uart_tx_fault = 0U;

/* main.h에 프로젝트별 prototype가 없더라도 C99에서 안전하게 컴파일되도록 선언. */
void UART_ParseCommand(char *cmd);
void Mecanum_InverseKinematics(float vx, float vy, float omega);
void Set_MotorPWM(int idx, float pwm);
void Set_ServoPWM(int idx, float angle);
static void Robot_StopMotorsImmediate(void);
static void Robot_HoldServosImmediate(void);
static bool Robot_IsStopped(void);
static void QueueAck(const char *value);
static void QueueHeartbeatAck(const char *value);
static void QueueError(const char *code);
static bool ErrorIsCommunicationTimeout(const char *code);
static bool ErrorIsRecoverableRejection(const char *code);
static bool SessionIdIsValid(const char *session_id);
static bool ProtocolActuationAllowed(void);
static bool UART_ArmReceive(void);
static void UART_MaintainRx(void);
static void UART_ProcessPendingCommands(void);
static void UART_QueueRxCommand(const char *command);
static bool UART_TransmitFrame(const char *line);
static void UART_SendPending(void);
static void UART_SendUltrasonicPending(void);
static void Ultrasonic_SetEnabled(bool enabled);
static void QueueUltrasonic(uint8_t side, int32_t distance_mm);
static void Ultrasonic_Task(void);
static void Ultrasonic_StartMeasurement(uint8_t side);
static uint16_t Ultrasonic_Micros(void);
static void Legacy_ApplyCommand(uint8_t command);
static void Legacy_SetVector(float forward, float right, float clockwise);
static void Legacy_SetOpenLoop(float forward, float right, float clockwise);
static void UART_SendTelemetry(void);
static bool ParseDecimalToken(const char **cursor, char terminator, float *value);

/* newlib-nano는 기본 링크 설정에서 scanf의 %f 지원이 빠져 있다.
 * ROS가 보내는 짧은 10진수 토큰을 직접 읽어 CubeIDE 링크 옵션에 의존하지 않는다. */
static bool ParseDecimalToken(const char **cursor, char terminator, float *value)
{
    const char *p = *cursor;
    bool negative = false;
    bool has_digit = false;
    float parsed = 0.0f;

    if (*p == '-' || *p == '+') {
        negative = (*p == '-');
        p++;
    }

    while (*p >= '0' && *p <= '9') {
        has_digit = true;
        parsed = parsed * 10.0f + (float)(*p - '0');
        if (parsed > 1000.0f) {
            return false;
        }
        p++;
    }

    if (*p == '.') {
        float place = 0.1f;
        p++;
        while (*p >= '0' && *p <= '9') {
            has_digit = true;
            parsed += (float)(*p - '0') * place;
            place *= 0.1f;
            p++;
        }
    }

    if (!has_digit || *p != terminator) {
        return false;
    }

    *value = negative ? -parsed : parsed;
    *cursor = (terminator == '\0') ? p : p + 1;
    return true;
}

/* ==================================================
 * 초기화
 * ================================================== */
void Robot_Init(void)
{
    memset(&g_robot, 0, sizeof(g_robot));
    memset(&g_ultrasonic, 0, sizeof(g_ultrasonic));
    g_tx_flags = 0U;
    g_uart_tx_fault = 0U;
    g_ack_value[0] = '\0';
    g_heartbeat_ack_value[0] = '\0';
    g_error_code[0] = '\0';
    g_rx_command_head = 0U;
    g_rx_command_tail = 0U;
    g_rx_command_count = 0U;
    g_rx_queue_overflow = 0U;
    g_uart_rx_error_pending = 0U;
    g_uart_rx_rearm_pending = 0U;
    uart_rx_idx = 0U;
    uart_frame_active = 0U;
    g_ultrasonic.next_side = ULTRA_LEFT;

    // 서보 초기값 (열림)
    g_robot.servo_state = 0;
    for (int i = 0; i < SERVO_NUM; i++) {
        g_robot.servo_current[i] = kServoOpenPulseUs[i];
        g_robot.servo_target[i] = kServoOpenPulseUs[i];
    }
    g_robot.last_cmd_time = HAL_GetTick();
    g_robot.last_heartbeat_time = HAL_GetTick();
    g_robot.last_command = 'X';

    // 엔코더 타이머 시작
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    g_robot.encoder_prev[FL] = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    g_robot.encoder_prev[FR] = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    g_robot.encoder_prev[RL] =
        (int32_t)__HAL_TIM_GET_COUNTER(&ENCODER_RL_TIMER);
    g_robot.encoder_prev[RR] =
        (int32_t)__HAL_TIM_GET_COUNTER(&ENCODER_RR_TIMER);

    // PWM 시작
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 모터
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /* 실차 기준: 부팅만으로 arm이 움직이지 않게 서보 PWM은 명령 때 시작한다. */

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

    // UART 인터럽트 수신 시작. 실패하면 main loop에서 안전 정지 후 재시도한다.
    if (!UART_ArmReceive()) {
        g_uart_rx_error_pending = 1U;
    }
}

static void QueueAck(const char *value)
{
    strncpy(g_ack_value, value, sizeof(g_ack_value) - 1);
    g_ack_value[sizeof(g_ack_value) - 1] = '\0';
    g_tx_flags |= TX_ACK;
}

static void QueueHeartbeatAck(const char *value)
{
    /* Heartbeat evidence must not share the general ACK mailbox.  A later
     * action/handshake ACK may replace g_ack_value, but can never erase HB. */
    strncpy(g_heartbeat_ack_value, value,
            sizeof(g_heartbeat_ack_value) - 1U);
    g_heartbeat_ack_value[sizeof(g_heartbeat_ack_value) - 1U] = '\0';
    g_tx_flags |= TX_HEARTBEAT_ACK;
}

static bool ErrorIsCommunicationTimeout(const char *code)
{
    return strcmp(code, "HEARTBEAT_TIMEOUT") == 0 ||
           strcmp(code, "COMMAND_TIMEOUT") == 0;
}

static bool ErrorIsRecoverableRejection(const char *code)
{
    return strcmp(code, "LIFT_WHILE_MOVING") == 0 ||
           strcmp(code, "SERVO_NOT_ATTACHED") == 0 ||
           /* Attach is rejected before PWM/current/target are changed. A bad
            * profile is therefore a recoverable configuration error, not
            * evidence of uncontrolled servo movement or hardware damage. */
           strcmp(code, "BAD_SERVO_ATTACH") == 0 ||
           strcmp(code, "BAD_HELLO") == 0 ||
           strcmp(code, "BAD_HEARTBEAT_TOKEN") == 0 ||
           strcmp(code, "BAD_VELOCITY") == 0 ||
           strcmp(code, "BAD_ZERO_PROBE") == 0 ||
           strcmp(code, "BAD_V_FRAME") == 0 ||
           strcmp(code, "BAD_MOTOR_TEST") == 0 ||
           strcmp(code, "BAD_SERVO_COMMAND") == 0 ||
           strcmp(code, "RX_QUEUE_OVERFLOW") == 0 ||
           strcmp(code, "UART_RX_ERROR") == 0 ||
           strcmp(code, "TX_FRAME_INVALID") == 0 ||
           strcmp(code, "UNKNOWN_COMMAND") == 0 ||
           strcmp(code, "SERVO_TIMEOUT") == 0 ||
           strcmp(code, "HELLO_REQUIRED") == 0 ||
           strcmp(code, "HEARTBEAT_REQUIRED") == 0 ||
           strcmp(code, "COMMAND_REQUIRED") == 0 ||
           strcmp(code, "STARTUP_SEQUENCE") == 0;
}

static bool ErrorRequiresNewSession(const char *code)
{
    return strcmp(code, "RX_QUEUE_OVERFLOW") == 0 ||
           strcmp(code, "UART_RX_ERROR") == 0 ||
           strcmp(code, "UNKNOWN_COMMAND") == 0;
}

static bool SessionIdIsValid(const char *session_id)
{
    size_t length = strlen(session_id);
    if (length < 8U || length > UART_SESSION_ID_MAX) return false;
    for (size_t i = 0U; i < length; i++) {
        char value = session_id[i];
        if (!((value >= 'A' && value <= 'Z') ||
              (value >= 'a' && value <= 'z') ||
              (value >= '0' && value <= '9') ||
              value == '_' || value == '-')) {
            return false;
        }
    }
    return true;
}

/* 모든 비상정지 외 actuator 경로가 동일한 v2 session gate를 사용한다.
 * UART 오류 뒤 남은 byte가 legacy 명령으로 보이더라도 새 HELLO/HB/zero
 * probe 전에는 모터와 서보를 다시 움직일 수 없다. */
static bool ProtocolActuationAllowed(void)
{
    uint32_t now = HAL_GetTick();

    if (g_uart_rx_error_pending) {
        Robot_StopMotorsImmediate();
        Robot_HoldServosImmediate();
        return false;
    }
    if (g_robot.estop_latched) {
        Robot_StopMotorsImmediate();
        QueueError("ESTOP_LATCHED");
        return false;
    }
    if (g_robot.safety_fault_latched) {
        Robot_StopMotorsImmediate();
        QueueError(g_robot.safety_fault_code);
        return false;
    }
    if (!g_robot.protocol_session_active) {
        Robot_StopMotorsImmediate();
        QueueError("HELLO_REQUIRED");
        return false;
    }
    if (g_robot.heartbeat_seen &&
        now - g_robot.last_heartbeat_time > HEARTBEAT_TIMEOUT_MS) {
        g_robot.heartbeat_timed_out = 1U;
    }
    if (!g_robot.heartbeat_seen || g_robot.heartbeat_timed_out) {
        Robot_StopMotorsImmediate();
        QueueError(g_robot.heartbeat_timed_out
            ? "HEARTBEAT_TIMEOUT" : "HEARTBEAT_REQUIRED");
        return false;
    }
    if (g_robot.command_seen &&
        now - g_robot.last_cmd_time > COMMAND_TIMEOUT_MS) {
        g_robot.command_timed_out = 1U;
    }
    if (!g_robot.command_seen || g_robot.command_timed_out) {
        Robot_StopMotorsImmediate();
        QueueError(g_robot.command_timed_out
            ? "COMMAND_TIMEOUT" : "COMMAND_REQUIRED");
        return false;
    }
    return true;
}

static void QueueError(const char *code)
{
    /* Session/protocol/configuration rejections stop immediately but do not
     * become reset-required. Only explicitly physical/internal safety faults
     * survive HELLO in safety_fault_latched. */
    bool communication_timeout = ErrorIsCommunicationTimeout(code);
    bool recoverable_rejection = ErrorIsRecoverableRejection(code);
    bool requires_new_session = ErrorRequiresNewSession(code);
    bool recoverable = communication_timeout || recoverable_rejection;
    if (recoverable) {
        /* A recoverable rejection is still a safety stop. Discard target/PID
         * state immediately; a later valid session/command must start from
         * zero instead of resuming the rejected command's predecessor. */
        Robot_StopMotorsImmediate();
    }
    /* Loss of the controller/session is fail-closed for every actuator.
     * Hold an in-progress servo at its current pulse instead of allowing its
     * old trajectory to continue after communication authority is gone. */
    if (communication_timeout || requires_new_session) {
        Robot_StopMotorsImmediate();
        Robot_HoldServosImmediate();
    }
    if (requires_new_session) {
        /* A corrupted/unknown frame makes the current byte-stream boundary
         * untrustworthy.  Keep the robot stopped and reject HB/V/servo until
         * a complete HELLO establishes every startup gate again. */
        g_robot.protocol_session_active = 0U;
        g_robot.session_id[0] = '\0';
        g_robot.heartbeat_seen = 0U;
        g_robot.command_seen = 0U;
        g_robot.manual_mode = 0U;
        g_robot.manual_open_loop = 0U;
        g_robot.servo_attached = 0U;
        g_robot.last_command = 'X';
    }
    /* Physical/internal faults still survive HELLO and require manual reset. */
    if (!recoverable &&
        strcmp(code, "ESTOP_LATCHED") != 0) {
        if (!g_robot.safety_fault_latched) {
            strncpy(g_robot.safety_fault_code, code,
                    sizeof(g_robot.safety_fault_code) - 1U);
            g_robot.safety_fault_code[
                sizeof(g_robot.safety_fault_code) - 1U] = '\0';
        }
        g_robot.safety_fault_latched = 1U;
    }
    strncpy(g_error_code, code, sizeof(g_error_code) - 1);
    g_error_code[sizeof(g_error_code) - 1] = '\0';
    g_tx_flags |= TX_ERR;
}

static bool UART_TransmitFrame(const char *line)
{
    if (line == NULL || g_uart_tx_fault) return false;
    size_t length = strlen(line);
    if (length == 0U || length > UINT16_MAX || line[length - 1U] != '\n') {
        QueueError("TX_FRAME_INVALID");
        Robot_StopMotorsImmediate();
        return false;
    }
    /* 8N1 consumes ten serial bits per byte. The old hard-coded 10 ms
     * deadline could expire mid telemetry line; the next line then appeared
     * glued into the truncated frame. Every producer now uses this one
     * complete-line path with a length-derived deadline and checked result. */
    uint32_t wire_ms = (uint32_t)(
        (length * 10U * 1000U + UART_BAUD_RATE - 1U) / UART_BAUD_RATE);
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        &huart2, (uint8_t *)line, (uint16_t)length,
        wire_ms + UART_TX_MARGIN_MS);
    if (status != HAL_OK) {
        g_uart_tx_fault = 1U;
        Robot_StopMotorsImmediate();
        return false;
    }
    return true;
}

static void UART_SendLine(const char *line)
{
    (void)UART_TransmitFrame(line);
}

static void UART_SendPending(void)
{
    char buf[64];
    if (g_uart_tx_fault) return;
    if (g_tx_flags & TX_ERR) {
        snprintf(buf, sizeof(buf), "ERR,%s\n", g_error_code);
        if (UART_TransmitFrame(buf)) {
            g_tx_flags &= (uint8_t)~TX_ERR;
        }
    } else if (g_tx_flags & TX_HEARTBEAT_ACK) {
        snprintf(buf, sizeof(buf), "ACK,%s\n", g_heartbeat_ack_value);
        if (UART_TransmitFrame(buf)) {
            g_tx_flags &= (uint8_t)~TX_HEARTBEAT_ACK;
        }
    } else if (g_tx_flags & TX_ACK) {
        snprintf(buf, sizeof(buf), "ACK,%s\n", g_ack_value);
        if (UART_TransmitFrame(buf)) {
            g_tx_flags &= (uint8_t)~TX_ACK;
        }
    } else if (g_tx_flags & TX_GRIP_DONE) {
        if (UART_TransmitFrame("LIFT,GRIP_DONE\n")) {
            g_tx_flags &= (uint8_t)~TX_GRIP_DONE;
        }
    } else if (g_tx_flags & TX_RELEASE_DONE) {
        if (UART_TransmitFrame("LIFT,RELEASE_DONE\n")) {
            g_tx_flags &= (uint8_t)~TX_RELEASE_DONE;
        }
    }
}

/* ==================================================
 * [3-1] uart_comm_task — 라즈베리파이 통신
 * ================================================== */

static bool UART_ArmReceive(void)
{
    HAL_StatusTypeDef status =
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
    if (status == HAL_OK ||
        (status == HAL_BUSY &&
         huart2.RxState == HAL_UART_STATE_BUSY_RX)) {
        g_uart_rx_rearm_pending = 0U;
        return true;
    }
    g_uart_rx_rearm_pending = 1U;
    return false;
}

/* UART error callback은 ISR context이므로 actuator나 blocking TX를 건드리지
 * 않는다. main loop가 fault를 처리하며, 여기서는 끊어진 byte RX만 복구한다. */
static void UART_MaintainRx(void)
{
    uint8_t report_error = 0U;

    __disable_irq();
    if (g_uart_rx_error_pending) {
        g_uart_rx_error_pending = 0U;
        g_rx_queue_overflow = 0U;
        g_rx_command_head = 0U;
        g_rx_command_tail = 0U;
        g_rx_command_count = 0U;
        uart_rx_idx = 0U;
        uart_frame_active = 0U;
        report_error = 1U;
    }
    __enable_irq();

    if (report_error) {
        QueueError("UART_RX_ERROR");
    }

    if (g_uart_rx_rearm_pending) {
        if (huart2.RxState == HAL_UART_STATE_READY) {
            (void)UART_ArmReceive();
        } else if (huart2.RxState == HAL_UART_STATE_BUSY_RX) {
            /* Non-blocking FE/NE errors keep the existing receive alive. */
            g_uart_rx_rearm_pending = 0U;
        }
    }
}

static void UART_QueueRxCommand(const char *command)
{
    if (g_rx_command_count >= RX_COMMAND_QUEUE_DEPTH) {
        g_rx_queue_overflow = 1U;
        return;
    }
    strncpy(g_rx_commands[g_rx_command_head], command,
            sizeof(g_rx_commands[g_rx_command_head]) - 1U);
    g_rx_commands[g_rx_command_head][
        sizeof(g_rx_commands[g_rx_command_head]) - 1U] = '\0';
    g_rx_command_head = (uint8_t)(
        (g_rx_command_head + 1U) % RX_COMMAND_QUEUE_DEPTH);
    g_rx_command_count++;
}

static void UART_ProcessPendingCommands(void)
{
    /* Error ISR가 Maintain 이후에 끼어들면 현재 loop에서는 어떤 queued
     * command도 실행하지 않는다. 다음 Maintain이 queue/session을 폐기한다. */
    if (g_uart_rx_error_pending) {
        return;
    }
    if (g_rx_queue_overflow) {
        __disable_irq();
        g_rx_queue_overflow = 0U;
        g_rx_command_head = 0U;
        g_rx_command_tail = 0U;
        g_rx_command_count = 0U;
        __enable_irq();
        Robot_StopMotorsImmediate();
        QueueError("RX_QUEUE_OVERFLOW");
        return;
    }

    while (g_rx_command_count > 0U) {
        char command[sizeof(uart_rx_buf)];
        if (g_uart_rx_error_pending) {
            return;
        }
        __disable_irq();
        strncpy(command, g_rx_commands[g_rx_command_tail],
                sizeof(command) - 1U);
        command[sizeof(command) - 1U] = '\0';
        g_rx_command_tail = (uint8_t)(
            (g_rx_command_tail + 1U) % RX_COMMAND_QUEUE_DEPTH);
        g_rx_command_count--;
        __enable_irq();

        if (g_uart_rx_error_pending) {
            return;
        }

        if (command[0] != '\0' && command[1] == '\0') {
            Legacy_ApplyCommand((uint8_t)command[0]);
        } else {
            UART_ParseCommand(command);
        }
        /* Each parsed command gets its own complete response before the next
         * command can overwrite the small ACK/ERR mailbox. TX remains solely
         * in main-loop context and therefore cannot interleave frames. */
        UART_SendPending();
    }
}

/* UART 수신 인터럽트 콜백 (한 바이트씩). ISR은 완성 frame을
 * queue에 넣기만 하고 parser/모터/서보/TX는 호출하지 않는다. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* ErrorCallback 이후 main loop가 session을 무효화하기 전에는 손상
         * frame의 tail byte를 해석하거나 다음 byte RX를 열지 않는다. */
        if (g_uart_rx_error_pending) {
            uart_rx_idx = 0U;
            uart_frame_active = 0U;
            g_uart_rx_rearm_pending = 1U;
            return;
        }
        if (!uart_frame_active) {
            if (uart_rx_byte == '@') {
                uart_frame_active = 1U;
                uart_rx_idx = 0U;
            } else if (uart_rx_byte != '\r' && uart_rx_byte != '\n') {
                char legacy[2] = {(char)uart_rx_byte, '\0'};
                UART_QueueRxCommand(legacy);
            }
        } else if (uart_rx_byte == '\n') {
            uart_rx_buf[uart_rx_idx] = '\0';
            UART_QueueRxCommand(uart_rx_buf);
            uart_rx_idx = 0U;
            uart_frame_active = 0U;
        } else if (uart_rx_byte != '\r' &&
                   uart_rx_idx < sizeof(uart_rx_buf) - 1U) {
            uart_rx_buf[uart_rx_idx++] = (char)uart_rx_byte;
        } else if (uart_rx_byte != '\r') {
            uart_rx_idx = 0U;
            uart_frame_active = 0U;
            g_rx_queue_overflow = 1U;
        }
        (void)UART_ArmReceive();
    }
}

/* ORE는 STM32 HAL 내부에서 RX interrupt를 종료한 뒤 이 callback을 부른다.
 * ISR에서는 partial frame 폐기와 fault 표시만 한다. main loop가 먼저
 * actuator/session을 fail-closed 처리한 뒤 byte RX를 다시 건다. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) {
        return;
    }

    uart_rx_idx = 0U;
    uart_frame_active = 0U;
    g_uart_rx_error_pending = 1U;
    g_uart_rx_rearm_pending = 1U;
}

/* 명령 파싱: HELLO / V / S / HB / ESTOP.
 * ESTOP은 HELLO로 절대 해제하지 않으며 전원 재인가/수동 reset까지 latch된다. */
void UART_ParseCommand(char *cmd)
{
    if (strcmp(cmd, "ESTOP") == 0) {
        g_robot.estop_latched = 1;
        Robot_StopMotorsImmediate();
        Ultrasonic_SetEnabled(false);
        /* 하중을 갑자기 놓지 않고 현재 각도에서 servo motion을 동결한다. */
        Robot_HoldServosImmediate();
        QueueAck("ESTOP");
    } else if (strncmp(cmd, "HELLO,", 6) == 0) {
        const char expected_prefix[] = "HELLO,2,";
        const char *session_id = &cmd[sizeof(expected_prefix) - 1U];
        if (strncmp(cmd, expected_prefix, sizeof(expected_prefix) - 1U) != 0 ||
            !SessionIdIsValid(session_id)) {
            QueueError("BAD_HELLO");
            return;
        }

        /* Linux bridge가 재시작되는 동안 이전 세션에서 이미 활성화된
         * heartbeat/command watchdog이 timeout될 수 있다. 새 세션 경계에서
         * 모터를 먼저 정지하고 통신 watchdog 상태만 초기화한다. 물리 ESTOP과
         * 기타 hardware fault는 여기서 절대 지우지 않는다. */
        Robot_StopMotorsImmediate();
        if (g_robot.estop_latched) {
            QueueError("ESTOP_LATCHED");
            return;
        }
        if (g_uart_tx_fault) {
            return;
        }
        if (g_robot.safety_fault_latched) {
            QueueError(g_robot.safety_fault_code);
            return;
        }
        if (g_robot.protocol_session_active &&
            strcmp(g_robot.session_id, session_id) == 0) {
            uint32_t now = HAL_GetTick();
            if (g_robot.heartbeat_seen &&
                now - g_robot.last_heartbeat_time > HEARTBEAT_TIMEOUT_MS) {
                g_robot.heartbeat_timed_out = 1U;
            }
            if (g_robot.command_seen &&
                now - g_robot.last_cmd_time > COMMAND_TIMEOUT_MS) {
                g_robot.command_timed_out = 1U;
            }
        }
        if (g_robot.protocol_session_active &&
            strcmp(g_robot.session_id, session_id) == 0 &&
            (g_robot.heartbeat_timed_out || g_robot.command_timed_out)) {
            QueueError(g_robot.heartbeat_timed_out
                ? "HEARTBEAT_TIMEOUT" : "COMMAND_TIMEOUT");
            return;
        }
        g_robot.manual_mode = 0U;
        g_robot.manual_open_loop = 0U;
        g_robot.heartbeat_seen = 0U;
        g_robot.command_seen = 0U;
        g_robot.heartbeat_timed_out = 0U;
        g_robot.command_timed_out = 0U;
        g_robot.last_heartbeat_time = HAL_GetTick();
        g_robot.last_cmd_time = HAL_GetTick();
        g_robot.servo_attached = 0U;
        Ultrasonic_SetEnabled(false);
        g_robot.protocol_session_active = 1U;
        strncpy(g_robot.session_id, session_id,
                sizeof(g_robot.session_id) - 1U);
        g_robot.session_id[sizeof(g_robot.session_id) - 1U] = '\0';

        /* 아직 UART로 전송되지 않은 이전 세션의 통신 timeout만 폐기한다.
         * 센서/모터/서보 오류는 보존되어 새 bridge에 그대로 전달된다. */
        if ((g_tx_flags & TX_ERR) != 0U &&
            (strcmp(g_error_code, "HEARTBEAT_TIMEOUT") == 0 ||
             strcmp(g_error_code, "COMMAND_TIMEOUT") == 0)) {
            g_tx_flags &= (uint8_t)~TX_ERR;
        }
        g_tx_flags &= (uint8_t)~TX_ACK;
        char ack[48];
        snprintf(ack, sizeof(ack), "HELLO:%u:%s",
                 (unsigned)UART_PROTOCOL_VERSION, session_id);
        QueueAck(ack);
    } else if (strncmp(cmd, "HB,", 3) == 0 && cmd[3] != '\0') {
        const char *token = &cmd[3];
        size_t session_length = strlen(g_robot.session_id);
        if (!g_robot.protocol_session_active) {
            QueueError("HELLO_REQUIRED");
            return;
        }
        if (strncmp(token, g_robot.session_id, session_length) != 0 ||
            token[session_length] != ':' ||
            token[session_length + 1U] == '\0') {
            QueueError("BAD_HEARTBEAT_TOKEN");
            return;
        }
        if (g_robot.heartbeat_seen &&
            HAL_GetTick() - g_robot.last_heartbeat_time >
            HEARTBEAT_TIMEOUT_MS) {
            g_robot.heartbeat_timed_out = 1U;
        }
        if (g_robot.heartbeat_timed_out) {
            QueueError("HEARTBEAT_TIMEOUT");
            return;
        }
        g_robot.last_heartbeat_time = HAL_GetTick();
        g_robot.heartbeat_seen = 1U;
        QueueHeartbeatAck(token);
    } else if (strcmp(cmd, "U,OFF") == 0) {
        /* OFF is always accepted after HELLO, including a motion/ESTOP fault.
         * It cannot weaken safety and prevents needless acoustic/UART load. */
        if (!g_robot.protocol_session_active) {
            QueueError("HELLO_REQUIRED");
            return;
        }
        Ultrasonic_SetEnabled(false);
        QueueAck("ULTRASONIC:OFF");
    } else if (strcmp(cmd, "U,ON") == 0) {
        if (g_robot.estop_latched) {
            QueueError("ESTOP_LATCHED");
            return;
        }
        if (!g_robot.protocol_session_active) {
            QueueError("HELLO_REQUIRED");
            return;
        }
        if (!g_robot.heartbeat_seen || g_robot.heartbeat_timed_out) {
            QueueError(g_robot.heartbeat_timed_out
                ? "HEARTBEAT_TIMEOUT" : "HEARTBEAT_REQUIRED");
            return;
        }
        Ultrasonic_SetEnabled(true);
        QueueAck("ULTRASONIC:ON");
    } else if (cmd[0] == 'V') {
        // 속도 명령
        float vx, vy, omega;
        const char *cursor = cmd + 2;
        const char *session_id = NULL;
        const char *session_separator = strchr(cursor, ',');
        if (session_separator != NULL) {
            session_separator = strchr(session_separator + 1, ',');
        }
        if (session_separator != NULL) {
            session_separator = strchr(session_separator + 1, ',');
        }
        char omega_terminator = session_separator != NULL ? ',' : '\0';
        if (cmd[1] == ',' &&
            ParseDecimalToken(&cursor, ',', &vx) &&
            ParseDecimalToken(&cursor, ',', &vy) &&
            ParseDecimalToken(&cursor, omega_terminator, &omega)) {
            if (omega_terminator == ',') session_id = cursor;
            if (g_robot.estop_latched) {
                QueueError("ESTOP_LATCHED");
                return;
            }
            if (!g_robot.protocol_session_active) {
                QueueError("HELLO_REQUIRED");
                return;
            }
            if (!g_robot.heartbeat_seen || g_robot.heartbeat_timed_out) {
                QueueError(g_robot.heartbeat_timed_out
                    ? "HEARTBEAT_TIMEOUT" : "HEARTBEAT_REQUIRED");
                return;
            }
            if (g_robot.command_seen &&
                HAL_GetTick() - g_robot.last_cmd_time >
                COMMAND_TIMEOUT_MS) {
                g_robot.command_timed_out = 1U;
            }
            if (g_robot.command_timed_out) {
                QueueError("COMMAND_TIMEOUT");
                return;
            }
            if (!isfinite(vx) || !isfinite(vy) || !isfinite(omega) ||
                fabsf(vx) > MAX_LINEAR_MPS ||
                fabsf(vy) > MAX_LINEAR_MPS ||
                fabsf(omega) > MAX_ANGULAR_RAD_S) {
                QueueError("BAD_VELOCITY");
                return;
            }
            if (session_id != NULL &&
                (strcmp(session_id, g_robot.session_id) != 0 ||
                 vx != 0.0f || vy != 0.0f || omega != 0.0f)) {
                QueueError("BAD_ZERO_PROBE");
                return;
            }
            g_robot.target_vx = vx;
            g_robot.target_vy = vy;
            g_robot.target_omega = omega;
            g_robot.last_cmd_time = HAL_GetTick();
            g_robot.command_seen = 1;
            g_robot.manual_mode = 0U;
            g_robot.manual_open_loop = 0U;
            g_robot.last_command = 'V';
            Mecanum_InverseKinematics(vx, vy, omega);
            if (session_id != NULL) {
                char ack[32];
                snprintf(ack, sizeof(ack), "V:%s", g_robot.session_id);
                QueueAck(ack);
            }
        } else {
            QueueError("BAD_V_FRAME");
        }
    } else if (strncmp(cmd, "M,", 2) == 0) {
        /* 정비용 단일 바퀴 오픈루프 명령. bridge를 끈 잭업 상태에서만 쓴다.
         * 250ms마다 갱신하지 않으면 기존 command watchdog이 즉시 정지시킨다. */
        if (g_robot.estop_latched) {
            QueueError("ESTOP_LATCHED");
            return;
        }
        if (strcmp(cmd, "M,STOP") == 0) {
            Robot_StopMotorsImmediate();
            g_robot.manual_mode = 1U;
            g_robot.last_cmd_time = HAL_GetTick();
            g_robot.last_command = 'M';
            QueueAck("MOTOR_STOP");
            return;
        }
        if (!ProtocolActuationAllowed()) {
            return;
        }

        int motor_index = -1;
        const char *pwm_text = NULL;
        if (strncmp(&cmd[2], "FL,", 3) == 0) {
            motor_index = FL;
            pwm_text = &cmd[5];
        } else if (strncmp(&cmd[2], "FR,", 3) == 0) {
            motor_index = FR;
            pwm_text = &cmd[5];
        } else if (strncmp(&cmd[2], "RL,", 3) == 0) {
            motor_index = RL;
            pwm_text = &cmd[5];
        } else if (strncmp(&cmd[2], "RR,", 3) == 0) {
            motor_index = RR;
            pwm_text = &cmd[5];
        }

        char *end = NULL;
        long requested_pwm = (pwm_text != NULL)
            ? strtol(pwm_text, &end, 10) : 0L;
        if (motor_index < 0 || pwm_text == NULL || end == pwm_text ||
            *end != '\0' || requested_pwm < -MOTOR_TEST_PWM_MAX ||
            requested_pwm > MOTOR_TEST_PWM_MAX) {
            QueueError("BAD_MOTOR_TEST");
            return;
        }

        Robot_StopMotorsImmediate();
        Set_MotorPWM(motor_index, (float)requested_pwm);
        g_robot.manual_mode = 1U;
        g_robot.manual_open_loop = 1U;
        g_robot.command_seen = 1U;
        g_robot.last_cmd_time = HAL_GetTick();
        g_robot.last_command = 'M';
        QueueAck("MOTOR_TEST");
    } else if (strncmp(cmd, "S,attach,", 9) == 0) {
        if (g_robot.estop_latched) {
            QueueError("ESTOP_LATCHED");
            return;
        }
        if (!Robot_IsStopped()) {
            QueueError("LIFT_WHILE_MOVING");
            return;
        }
        if (!g_robot.protocol_session_active || !g_robot.heartbeat_seen ||
            !g_robot.command_seen || g_robot.heartbeat_timed_out ||
            g_robot.command_timed_out) {
            QueueError("STARTUP_SEQUENCE");
            return;
        }

        char *end = NULL;
        long pulse_1 = strtol(&cmd[9], &end, 10);
        if (end == &cmd[9] || *end != ',') {
            QueueError("BAD_SERVO_ATTACH");
            return;
        }
        const char *pulse_2_text = end + 1;
        long pulse_2 = strtol(pulse_2_text, &end, 10);
        if (end == pulse_2_text || *end != '\0' ||
            pulse_1 < (long)kServoMinPulseUs[0] ||
            pulse_1 > (long)kServoMaxPulseUs[0] ||
            pulse_2 < (long)kServoMinPulseUs[1] ||
            pulse_2 > (long)kServoMaxPulseUs[1]) {
            QueueError("BAD_SERVO_ATTACH");
            return;
        }

        /* 3선식 hobby servo는 재부팅 후 실제 각도를 읽을 수 없다. bridge가
         * 보존한 pulse를 compare/current/target에 먼저 복원한 뒤 PWM을 붙여
         * 부팅 기본값으로 한 프레임 튀는 동작을 막는다. */
        Robot_StopMotorsImmediate();
        g_robot.servo_current[0] = (float)pulse_1;
        g_robot.servo_current[1] = (float)pulse_2;
        g_robot.servo_target[0] = (float)pulse_1;
        g_robot.servo_target[1] = (float)pulse_2;
        g_robot.servo_motion_active = 0U;
        g_robot.servo_state = 2U;
        Set_ServoPWM(0, g_robot.servo_current[0]);
        Set_ServoPWM(1, g_robot.servo_current[1]);
        HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
        g_robot.servo_attached = 1U;
        g_robot.last_command = 'P';
        QueueAck("SERVO_ATTACH");
    } else if (strncmp(cmd, "S,", 2) == 0) {
        if (g_robot.estop_latched) {
            QueueError("ESTOP_LATCHED");
            return;
        }
        if (!Robot_IsStopped()) {
            QueueError("LIFT_WHILE_MOVING");
            return;
        }
        if (!g_robot.servo_attached) {
            QueueError("SERVO_NOT_ATTACHED");
            return;
        }
        HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
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
                    ? kServoGripPulseUs[i] : kServoOpenPulseUs[i];
            }
            g_robot.servo_motion_active = 1;
            g_robot.servo_motion_start = HAL_GetTick();
        }
        QueueAck(requested_state ? "GRIP" : "RELEASE");
    } else {
        QueueError("UNKNOWN_COMMAND");
    }
}

static void Legacy_SetVector(float forward, float right, float clockwise)
{
    float wheel[MOTOR_NUM] = {
        forward + right + clockwise,
        forward - right - clockwise,
        forward - right + clockwise,
        forward + right - clockwise
    };
    float max_magnitude = 1.0f;
    for (int i = 0; i < MOTOR_NUM; i++) {
        float magnitude = fabsf(wheel[i]);
        if (magnitude > max_magnitude) max_magnitude = magnitude;
    }
    for (int i = 0; i < MOTOR_NUM; i++) {
        g_robot.wheel_target[i] =
            wheel[i] * MANUAL_WHEEL_RAD_S / max_magnitude;
        g_robot.pid[i].integral = 0.0f;
        g_robot.pid[i].prev_error = 0.0f;
    }
    g_robot.manual_mode = 1U;
    g_robot.manual_open_loop = 0U;
    g_robot.last_cmd_time = HAL_GetTick();
}

static void Legacy_SetOpenLoop(float forward, float right, float clockwise)
{
    float wheel[MOTOR_NUM] = {
        forward + right + clockwise,
        forward - right - clockwise,
        forward - right + clockwise,
        forward + right - clockwise
    };
    float max_magnitude = 1.0f;
    for (int i = 0; i < MOTOR_NUM; i++) {
        float magnitude = fabsf(wheel[i]);
        if (magnitude > max_magnitude) max_magnitude = magnitude;
    }
    for (int i = 0; i < MOTOR_NUM; i++) {
        g_robot.wheel_target[i] = 0.0f;
        Set_MotorPWM(i, wheel[i] * OPEN_LOOP_PWM / max_magnitude);
    }
    g_robot.manual_mode = 1U;
    g_robot.manual_open_loop = 1U;
    g_robot.last_cmd_time = HAL_GetTick();
}

static void Legacy_ApplyCommand(uint8_t command)
{
    float *target;
    const char *safe_commands = "Xx 12";
    const char *actuation_commands = "WSADQEwsadqeUuJjIiKkTtGgOo";

    if (strchr(safe_commands, (int)command) == NULL &&
        strchr(actuation_commands, (int)command) == NULL) {
        return;
    }
    if (strchr(actuation_commands, (int)command) != NULL &&
        !ProtocolActuationAllowed()) {
        return;
    }
    if (strchr("UuJjIiKkTtGg", (int)command) != NULL &&
        !g_robot.servo_attached) {
        Robot_StopMotorsImmediate();
        QueueError("SERVO_NOT_ATTACHED");
        return;
    }
    switch (command) {
    case 'W': Legacy_SetVector(1, 0, 0); break;
    case 'S': Legacy_SetVector(-1, 0, 0); break;
    case 'A': Legacy_SetVector(0, -1, 0); break;
    case 'D': Legacy_SetVector(0, 1, 0); break;
    case 'Q': Legacy_SetVector(0, 0, -1); break;
    case 'E': Legacy_SetVector(0, 0, 1); break;
    case 'w': Legacy_SetOpenLoop(1, 0, 0); break;
    case 's': Legacy_SetOpenLoop(-1, 0, 0); break;
    case 'a': Legacy_SetOpenLoop(0, -1, 0); break;
    case 'd': Legacy_SetOpenLoop(0, 1, 0); break;
    case 'q': Legacy_SetOpenLoop(0, 0, -1); break;
    case 'e': Legacy_SetOpenLoop(0, 0, 1); break;
    case 'U': case 'u':
        Robot_StopMotorsImmediate();
        HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        target = &g_robot.servo_target[0];
        *target = fminf(kServoMaxPulseUs[0], *target + SERVO_COMMAND_STEP_US);
        break;
    case 'J': case 'j':
        Robot_StopMotorsImmediate();
        HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        target = &g_robot.servo_target[0];
        *target = fmaxf(kServoMinPulseUs[0], *target - SERVO_COMMAND_STEP_US);
        break;
    case 'I': case 'i':
        Robot_StopMotorsImmediate();
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
        target = &g_robot.servo_target[1];
        *target = fminf(kServoMaxPulseUs[1], *target + SERVO_COMMAND_STEP_US);
        break;
    case 'K': case 'k':
        Robot_StopMotorsImmediate();
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
        target = &g_robot.servo_target[1];
        *target = fmaxf(kServoMinPulseUs[1], *target - SERVO_COMMAND_STEP_US);
        break;
    case 'T': case 't':
        Robot_StopMotorsImmediate();
        HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
        for (int i = 0; i < SERVO_NUM; i++)
            g_robot.servo_target[i] = kServoGripPulseUs[i];
        break;
    case 'G': case 'g':
        Robot_StopMotorsImmediate();
        HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
        for (int i = 0; i < SERVO_NUM; i++)
            g_robot.servo_target[i] = kServoOpenPulseUs[i];
        break;
    case 'O': case 'o':
        Robot_StopMotorsImmediate();
        HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
        g_robot.servo_attached = 0U;
        break;
    case '1': case '2':
        /* 자동 초음파 상태머신이 계속 측정하므로 최신값은 U frame으로 확인한다. */
        Robot_StopMotorsImmediate();
        break;
    case 'X': case 'x': case ' ':
        Robot_StopMotorsImmediate();
        break;
    default:
        return;
    }
    g_robot.last_command = (char)command;
    g_robot.last_cmd_time = HAL_GetTick();
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
    if (len <= 0 || (size_t)len >= sizeof(buf)) {
        QueueError("TX_FRAME_INVALID");
        Robot_StopMotorsImmediate();
        return;
    }
    (void)UART_TransmitFrame(buf);
}

/* 기존 실차 모니터와 호환되는 14-field telemetry. */
static void UART_SendTelemetry(void)
{
    char buf[160];
    int len = snprintf(
        buf, sizeof(buf),
        "T,%c,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%ld,%ld\r\n",
        g_robot.last_command,
        (int)(g_robot.wheel_actual[FL] * 600.0f / (2.0f * M_PI)),
        (int)(g_robot.wheel_actual[FR] * 600.0f / (2.0f * M_PI)),
        (int)(g_robot.wheel_actual[RL] * 600.0f / (2.0f * M_PI)),
        (int)(g_robot.wheel_actual[RR] * 600.0f / (2.0f * M_PI)),
        (int)g_robot.motor_pwm[FL], (int)g_robot.motor_pwm[FR],
        (int)g_robot.motor_pwm[RL], (int)g_robot.motor_pwm[RR],
        (unsigned)g_robot.servo_current[0],
        (unsigned)g_robot.servo_current[1],
        (long)g_ultrasonic_tx_mm[ULTRA_LEFT],
        (long)g_ultrasonic_tx_mm[ULTRA_RIGHT]);
    if (len <= 0 || (size_t)len >= sizeof(buf)) {
        QueueError("TX_FRAME_INVALID");
        Robot_StopMotorsImmediate();
        return;
    }
    (void)UART_TransmitFrame(buf);
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
    if (side >= ULTRASONIC_NUM || !g_ultrasonic.enabled) return;
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

static void Ultrasonic_SetEnabled(bool enabled)
{
    g_ultrasonic.enabled = enabled ? 1U : 0U;
    g_ultrasonic.waiting_echo = 0U;
    g_ultrasonic.echo_high = 0U;
    g_ultrasonic.measurement_complete = 0U;
    g_ultrasonic.next_side = ULTRA_LEFT;
    for (uint8_t side = 0U; side < ULTRASONIC_NUM; side++) {
        g_ultrasonic_tx_pending[side] = 0U;
        HAL_GPIO_WritePin(kUltrasonicTrigPort[side],
                          kUltrasonicTrigPin[side], GPIO_PIN_RESET);
    }
    if (enabled) {
        g_ultrasonic.last_trigger_ms =
            HAL_GetTick() - ULTRASONIC_INTERVAL_MS;
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
    if (!g_ultrasonic.enabled) return;
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

/* 앞쪽 논리 순서는 FL=TIM5, FR=TIM2로 공통이며 뒤쪽은 로봇 프로필별로
 * ENCODER_RL_TIMER/ENCODER_RR_TIMER를 선택한다.
 * TIM2/TIM5는 32비트, TIM3/TIM4는 16비트다.
 * 16비트 카운터는 0~65535를 순환하는데, 이전엔 delta를 32비트 그대로
 * 빼서(cnt-prev) 카운터가 순환하는 순간마다 실제 회전량과 정반대의 거대한
 * 값(예: prev=65530,cnt=5(정상 +11회전)인데 delta=5-65530=-65525로 계산)이
 * 나오는 버그가 있었다. int16_t로 캐스팅한 차분은 2의 보수 표현 특성상
 * wraparound를 자동으로 올바르게 처리한다 — 단, 한 제어주기 실제 변화량이
 * ±32767틱을 넘지 않는다는 전제인데(CONTROL_HZ 주기당 이 정도 회전은 이
 * 로봇 최대속도로는 불가능하므로) 안전하다.
 * 실제 CubeMX .ioc에서 타이머 비트폭을 다르게 설정했다면 kEncoder16Bit를
 * 맞게 수정할 것. */
static const uint8_t kEncoder16Bit[MOTOR_NUM] = {0, 0, 1, 1};

void Update_WheelSpeeds(void)
{
    TIM_HandleTypeDef* enc[] = {
        &htim5, &htim2, &ENCODER_RL_TIMER, &ENCODER_RR_TIMER
    };
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
/* PA10=TIM1_CH3=RR, PA11=TIM1_CH4=RL이므로 뒤쪽 두 항목은 4/3 순서다.
 * DIR도 위 별칭에서 RL=MOTOR4(PC3), RR=MOTOR3(PC2)로 같은 채널끼리
 * 묶는다. 2026-08-20 단일 바퀴 ±120 실측으로 잘못된 3/4 DIR 순서가
 * 두 뒤바퀴 모두 방향을 바꾸지 못하게 했음을 확인했다. */
    uint32_t ch[] = {TIM_CHANNEL_1, TIM_CHANNEL_2,
                     TIM_CHANNEL_4, TIM_CHANNEL_3};
    GPIO_TypeDef* dir_port[] = {
        MOTOR_FL_DIR_GPIO_Port, MOTOR_FR_DIR_GPIO_Port,
        MOTOR_RL_DIR_GPIO_Port, MOTOR_RR_DIR_GPIO_Port
    };
    uint16_t dir_pin[] = {
        MOTOR_FL_DIR_Pin, MOTOR_FR_DIR_Pin,
        MOTOR_RL_DIR_Pin, MOTOR_RR_DIR_Pin
    };
    if (idx < 0 || idx >= MOTOR_NUM) return;
    g_robot.motor_pwm[idx] = pwm;
    pwm *= (float)kMotorCommandSign[idx];

    // 방향 설정
    if (pwm >= 0) {
        HAL_GPIO_WritePin(dir_port[idx], dir_pin[idx], GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dir_port[idx], dir_pin[idx], GPIO_PIN_RESET);
        pwm = -pwm;
    }
    /* 실차와 동일하게 ARR=999에서 compare를 0~350으로 제한한다.
     * 즉 최대 duty는 약 35%이며, 임의로 100%로 재스케일하지 않는다. */
    if (pwm > MOTOR_PWM_COMMAND_MAX) {
        pwm = MOTOR_PWM_COMMAND_MAX;
    }
    uint32_t duty = (uint32_t)(pwm + 0.5f);
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
        g_robot.target_rpm_x10[i] = 0;
        g_robot.speed_integral[i] = 0;
        g_robot.wrong_direction_cycles[i] = 0;
        Set_MotorPWM(i, 0.0f);
    }
    g_robot.manual_open_loop = 0U;
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

/* 실차에서 검증된 50 ms 정수 PI + feed-forward 제어. */
void Motor_PID_Task(void)
{
    uint32_t now = HAL_GetTick();

    Update_WheelSpeeds();

    if (g_robot.estop_latched || g_robot.safety_fault_latched ||
        g_uart_tx_fault) {
        Robot_StopMotorsImmediate();
        return;
    }

    /* 전원 인가 직후 ROS2가 뜨기 전의 300ms를 fault로 오인하지 않는다.
     * 단, HB와 V를 각각 한 번 이상 받기 전에는 모터를 무조건 정지한다. */
    if (!g_robot.manual_mode) {
        if (!g_robot.heartbeat_seen || !g_robot.command_seen) {
            Robot_StopMotorsImmediate();
            return;
        }
    }

    if (!g_robot.manual_mode &&
        now - g_robot.last_heartbeat_time > HEARTBEAT_TIMEOUT_MS) {
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
    /* 실차에서 먼저 검증되지 않은 stall latch는 안전한 정지를 오인해
     * 전원 재인가가 필요해질 수 있어 이번 통합에서는 사용하지 않는다. */
    if (g_robot.manual_open_loop) {
        return;
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        int16_t requested_rpm_x10 = (int16_t)(
            g_robot.wheel_target[i] * 600.0f / (2.0f * M_PI));
        int16_t current = g_robot.target_rpm_x10[i];
        if (current < requested_rpm_x10) {
            current += TARGET_RAMP_RPM_X10;
            if (current > requested_rpm_x10) current = requested_rpm_x10;
        } else if (current > requested_rpm_x10) {
            current -= TARGET_RAMP_RPM_X10;
            if (current < requested_rpm_x10) current = requested_rpm_x10;
        }
        g_robot.target_rpm_x10[i] = current;

        if (current == 0) {
            g_robot.speed_integral[i] = 0;
            g_robot.wrong_direction_cycles[i] = 0;
            Set_MotorPWM(i, 0.0f);
            continue;
        }

        int32_t target_delta =
            ((int32_t)current * (int32_t)ENCODER_PPR * 50L) / 600000L;
        int32_t error = target_delta - g_robot.encoder_delta[i];

        /* 정상상태에서 실제 회전이 목표와 반대면 부호 설정이 틀린
         * 것이다. 그대로 두면 PID가 출력을 상한까지 밀어올린다. */
        if (current == requested_rpm_x10 &&
            ((current > 0 && g_robot.encoder_delta[i] < 0) ||
             (current < 0 && g_robot.encoder_delta[i] > 0))) {
            g_robot.wrong_direction_cycles[i]++;
            if (g_robot.wrong_direction_cycles[i] >=
                WRONG_DIRECTION_LIMIT_CYCLES) {
                Robot_StopMotorsImmediate();
                QueueError("WHEEL_DIR_MISMATCH");
                return;
            }
        } else {
            g_robot.wrong_direction_cycles[i] = 0;
        }

        g_robot.speed_integral[i] += error;
        if (g_robot.speed_integral[i] > SPEED_INTEGRAL_LIMIT)
            g_robot.speed_integral[i] = SPEED_INTEGRAL_LIMIT;
        if (g_robot.speed_integral[i] < -SPEED_INTEGRAL_LIMIT)
            g_robot.speed_integral[i] = -SPEED_INTEGRAL_LIMIT;

        int32_t feedforward =
            ((int32_t)current * PWM_FEEDFORWARD_AT_12RPM) / 120L;
        int32_t output = feedforward + SPEED_KP * error +
            g_robot.speed_integral[i] / SPEED_KI_DIVISOR;
        if (output > (int32_t)MOTOR_PWM_COMMAND_MAX)
            output = (int32_t)MOTOR_PWM_COMMAND_MAX;
        if (output < -(int32_t)MOTOR_PWM_COMMAND_MAX)
            output = -(int32_t)MOTOR_PWM_COMMAND_MAX;
        Set_MotorPWM(i, (float)output);
    }
}

/* ==================================================
 * [3-3] servo_lift_task — arm Soft-start 제어
 * ================================================== */

/* 실차에서 검증한 microsecond pulse를 그대로 출력한다. */
void Set_ServoPWM(int idx, float pulse_us)
{
    TIM_HandleTypeDef *timer[SERVO_NUM] = {&htim10, &htim11};
    if (idx < 0 || idx >= SERVO_NUM) return;
    if (pulse_us < kServoMinPulseUs[idx]) pulse_us = kServoMinPulseUs[idx];
    if (pulse_us > kServoMaxPulseUs[idx]) pulse_us = kServoMaxPulseUs[idx];
    uint32_t pulse = (uint32_t)(pulse_us + 0.5f);
    __HAL_TIM_SET_COMPARE(timer[idx], TIM_CHANNEL_1, pulse);
}

/* 서보 Soft-start: 급가동 방지 (목표까지 서서히) */
void Servo_Lift_Task(void)
{
    float step = SERVO_RAMP_STEP_US;
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
    static uint32_t last_telemetry_tx = 0;
    uint32_t now = HAL_GetTick();

    /* RX fault를 먼저 fail-closed 처리하고, 완전한 frame만 실행한다. */
    UART_MaintainRx();
    UART_ProcessPendingCommands();
    /* Maintain 직후 또는 queue 처리 중 발생한 RX fault도 actuator task보다
     * 먼저 처리한다. */
    UART_MaintainRx();

    // 실차에서 검증된 모터 제어 주기: 20Hz (50ms)
    if (now - last_control >= 50) {
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

    if (now - last_telemetry_tx >= TELEMETRY_PERIOD_MS) {
        UART_SendTelemetry();
        last_telemetry_tx = now;
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
