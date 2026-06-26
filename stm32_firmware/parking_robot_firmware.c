/*
 * ==================================================
 * [파트 3] STM32 펌웨어 — 주차 로봇 모터/서보 제어
 * ==================================================
 * 라즈베리파이(ROS2)와 UART로 통신하며 메카넘 모터 4개 +
 * arm 서보 2개를 제어.
 *
 * 3개 태스크 (FreeRTOS 또는 메인 루프 기반):
 *   3-1. uart_comm_task   : 라파 통신 (속도 수신, 엔코더 송신)
 *   3-2. motor_pid_task   : 모터 속도 PID 제어
 *   3-3. servo_lift_task  : arm 서보 Soft-start 제어
 *
 * UART 프로토콜:
 *   수신: "V,vx,vy,omega\n"  (속도 명령, m/s)
 *         "S,grip\n" / "S,release\n"  (arm 제어)
 *   송신: "E,fl,fr,rl,rr\n"  (엔코더 카운트)
 *
 * 환경: STM32 Nucleo (F4/F7/G4), HAL 라이브러리
 * ==================================================
 */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ===== 하드웨어 핸들 (CubeMX 생성) ===== */
extern UART_HandleTypeDef huart2;     // 라즈베리파이 통신
extern TIM_HandleTypeDef htim1;       // 모터 PWM
extern TIM_HandleTypeDef htim2;       // 서보 PWM
extern TIM_HandleTypeDef htim3;       // 엔코더 (FL)
extern TIM_HandleTypeDef htim4;       // 엔코더 (FR)
extern TIM_HandleTypeDef htim5;       // 엔코더 (RL)
extern TIM_HandleTypeDef htim8;       // 엔코더 (RR)

/* ===== 상수 ===== */
#define WHEEL_RADIUS    0.03f      // 바퀴 반경 (m)
#define LX              0.10f      // 좌우 바퀴 거리/2
#define LY              0.10f      // 전후 바퀴 거리/2
#define ENCODER_PPR     2600.0f    // 26PPR * 100 감속비
#define CONTROL_HZ      100.0f     // 제어 주기
#define DT              (1.0f / CONTROL_HZ)

/* ===== 메카넘 모터 인덱스 ===== */
enum { FL = 0, FR, RL, RR, MOTOR_NUM };

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
    PID_t pid[MOTOR_NUM];                       // 바퀴별 PID
    uint8_t servo_state;                        // 0=열림, 1=닫힘(grip)
    float servo_current;                        // 현재 서보 각도 (soft-start)
    float servo_target;                         // 목표 서보 각도
    uint32_t last_cmd_time;                     // 워치독용
} RobotState_t;

RobotState_t g_robot;

/* ===== UART 수신 버퍼 ===== */
uint8_t uart_rx_byte;
char uart_rx_buf[64];
uint8_t uart_rx_idx = 0;

/* ==================================================
 * 초기화
 * ================================================== */
void Robot_Init(void)
{
    memset(&g_robot, 0, sizeof(g_robot));

    // 바퀴별 PID 게인 설정
    for (int i = 0; i < MOTOR_NUM; i++) {
        g_robot.pid[i].Kp = 2.0f;
        g_robot.pid[i].Ki = 0.5f;
        g_robot.pid[i].Kd = 0.1f;
        g_robot.pid[i].out_limit = 1000.0f;  // PWM 최대
    }

    // 서보 초기값 (열림)
    g_robot.servo_state = 0;
    g_robot.servo_current = 30.0f;   // 열림 각도
    g_robot.servo_target = 30.0f;

    // 엔코더 타이머 시작
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    // PWM 시작
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 모터
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // 서보

    // UART 인터럽트 수신 시작
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
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

/* 명령 파싱: "V,vx,vy,omega" 또는 "S,grip/release" */
void UART_ParseCommand(char *cmd)
{
    if (cmd[0] == 'V') {
        // 속도 명령
        float vx, vy, omega;
        if (sscanf(cmd, "V,%f,%f,%f", &vx, &vy, &omega) == 3) {
            g_robot.target_vx = vx;
            g_robot.target_vy = vy;
            g_robot.target_omega = omega;
            g_robot.last_cmd_time = HAL_GetTick();
            Mecanum_InverseKinematics(vx, vy, omega);
        }
    } else if (cmd[0] == 'S') {
        // 서보 명령
        if (strstr(cmd, "grip")) {
            g_robot.servo_state = 1;
            g_robot.servo_target = 90.0f;   // 닫힘 각도
        } else if (strstr(cmd, "release")) {
            g_robot.servo_state = 0;
            g_robot.servo_target = 30.0f;   // 열림 각도
        }
    }
}

/* 엔코더 값 송신: "E,fl,fr,rl,rr" */
void UART_SendEncoders(void)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "E,%ld,%ld,%ld,%ld\n",
                       g_robot.encoder_count[FL],
                       g_robot.encoder_count[FR],
                       g_robot.encoder_count[RL],
                       g_robot.encoder_count[RR]);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 10);
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

/* 엔코더로 실제 속도 계산 */
void Update_WheelSpeeds(void)
{
    TIM_HandleTypeDef* enc[] = {&htim3, &htim4, &htim5, &htim8};
    for (int i = 0; i < MOTOR_NUM; i++) {
        int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(enc[i]);
        int32_t delta = cnt - g_robot.encoder_prev[i];
        g_robot.encoder_prev[i] = cnt;
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
    GPIO_TypeDef* dir_port[] = {GPIOB, GPIOB, GPIOB, GPIOB};
    uint16_t dir_pin[] = {GPIO_PIN_0, GPIO_PIN_1,
                          GPIO_PIN_2, GPIO_PIN_3};

    // 방향 설정
    if (pwm >= 0) {
        HAL_GPIO_WritePin(dir_port[idx], dir_pin[idx], GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dir_port[idx], dir_pin[idx], GPIO_PIN_RESET);
        pwm = -pwm;
    }
    // PWM 크기 (0~999)
    uint32_t duty = (uint32_t)pwm;
    if (duty > 999) duty = 999;
    __HAL_TIM_SET_COMPARE(&htim1, ch[idx], duty);
}

/* 모터 제어 주기 실행 (100Hz) */
void Motor_PID_Task(void)
{
    // 워치독: 명령이 300ms 이상 없으면 정지
    if (HAL_GetTick() - g_robot.last_cmd_time > 300) {
        for (int i = 0; i < MOTOR_NUM; i++) {
            g_robot.wheel_target[i] = 0;
        }
    }

    Update_WheelSpeeds();

    for (int i = 0; i < MOTOR_NUM; i++) {
        float pwm = PID_Compute(&g_robot.pid[i],
                                g_robot.wheel_target[i],
                                g_robot.wheel_actual[i]);
        Set_MotorPWM(i, pwm);
    }
}

/* ==================================================
 * [3-3] servo_lift_task — arm Soft-start 제어
 * ================================================== */

/* 서보 각도 → PWM (50Hz, 1~2ms 펄스) */
void Set_ServoPWM(float angle)
{
    // 각도(0~180) → 펄스폭(500~2500us)
    // 타이머가 20ms 주기, 1us 단위라 가정
    uint32_t pulse = 500 + (uint32_t)(angle / 180.0f * 2000.0f);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

/* 서보 Soft-start: 급가동 방지 (목표까지 서서히) */
void Servo_Lift_Task(void)
{
    float diff = g_robot.servo_target - g_robot.servo_current;
    float step = 1.0f;   // 한 주기당 최대 1도씩 (부드럽게)

    if (fabsf(diff) > step) {
        g_robot.servo_current += (diff > 0) ? step : -step;
    } else {
        g_robot.servo_current = g_robot.servo_target;
    }
    Set_ServoPWM(g_robot.servo_current);
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
