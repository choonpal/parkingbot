/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Servo installation profile for the robot being built.
 * Set to 1 for the ArUco robot and 0 for the no-marker robot.
 */
#define ROBOT_HAS_ARUCO_MARKER       1U

#if ROBOT_HAS_ARUCO_MARKER
#define SERVO1_START_PULSE_US        2600U
#define SERVO2_START_PULSE_US        400U
#define SERVO1_MIN_PULSE_US          1550U
#define SERVO1_MAX_PULSE_US          2600U
#define SERVO2_MIN_PULSE_US          400U
#define SERVO2_MAX_PULSE_US          1450U
#else
#define SERVO1_START_PULSE_US        400U
#define SERVO2_START_PULSE_US        2600U
#define SERVO1_MIN_PULSE_US          400U
#define SERVO1_MAX_PULSE_US          1600U
#define SERVO2_MIN_PULSE_US          1400U
#define SERVO2_MAX_PULSE_US          2600U
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t servo1_pulse_us = SERVO1_START_PULSE_US;
volatile uint16_t servo2_pulse_us = SERVO2_START_PULSE_US;
volatile uint16_t servo1_target_pulse_us = SERVO1_START_PULSE_US;
volatile uint16_t servo2_target_pulse_us = SERVO2_START_PULSE_US;
volatile int32_t encoder_live_counts[4] = {0, 0, 0, 0};
volatile int32_t wheel_encoder_delta[4] = {0, 0, 0, 0};
volatile int16_t wheel_command_rpm_x10[4] = {0, 0, 0, 0};
volatile int16_t wheel_target_rpm_x10[4] = {0, 0, 0, 0};
volatile int16_t wheel_measured_rpm_x10[4] = {0, 0, 0, 0};
volatile int16_t wheel_pwm_output[4] = {0, 0, 0, 0};
volatile int32_t ultrasonic_distance_mm[2] = {-1, -1};
volatile uint8_t drive_command = 'X';
volatile uint8_t drive_watchdog_stopped = 1U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MOTOR_COUNT       4U
#define MOTOR_FL          0U
#define MOTOR_FR          1U
#define MOTOR_RL          2U
#define MOTOR_RR          3U
#define MOTOR_COMMAND_MAX            200
#define MOTOR_PWM_LIMIT              350
#define OPEN_LOOP_PWM                180
#define DRIVE_TIMEOUT_MS             250U
#define SPEED_CONTROL_PERIOD_MS      50U
#define TELEMETRY_PERIOD_MS          200U
#define ENCODER_COUNTS_PER_REV       5182L
#define MANUAL_TARGET_RPM_X10        120
#define PWM_FEEDFORWARD_AT_TARGET    200
#define SPEED_KP                     3
#define SPEED_KI_DIVISOR             32
#define SPEED_INTEGRAL_LIMIT         3200L
#define TARGET_RAMP_STEP_RPM_X10     10
#define SERVO_COMMAND_STEP_US        50
#define SERVO_RAMP_STEP_US           30U
#define SERVO_UPDATE_PERIOD_MS       20U
#define ULTRASONIC_MIN_INTERVAL_MS   60U
#define ULTRASONIC_TIMEOUT_US        30000U

/*
 * Logical order:
 *   0 = FL: TIM1_CH1, PC0, TIM5
 *   1 = FR: TIM1_CH2, PC1, TIM2
 *   2 = RL: TIM1_CH4, PC3, TIM3
 *   3 = RR: TIM1_CH3, PC2, TIM4
 */
static const uint32_t motor_pwm_channel[MOTOR_COUNT] =
{
  TIM_CHANNEL_1,
  TIM_CHANNEL_2,
  TIM_CHANNEL_4,
  TIM_CHANNEL_3
};

static const uint16_t motor_dir_pin[MOTOR_COUNT] =
{
  GPIO_PIN_0,
  GPIO_PIN_1,
  GPIO_PIN_3,
  GPIO_PIN_2
};

/*
 * Logical positive means that the wheel drives the robot forward.
 * Left and right motors are mirrored, so their raw DIR polarity is opposite.
 * This table is intentionally easy to adjust after the wheels are mounted.
 */
static const int8_t motor_forward_polarity[MOTOR_COUNT] =
{
   1,
  -1,
   1,
  -1
};

/* Encoder A/B polarity is independent from the motor output wiring. */
static const int8_t motor_encoder_polarity[MOTOR_COUNT] =
{
   1,
  -1,
   1,
  -1
};

static TIM_HandleTypeDef * const motor_encoder[MOTOR_COUNT] =
{
  &htim5,
  &htim2,
  &htim3,
  &htim4
};

static uint32_t encoder_previous_raw[MOTOR_COUNT] = {0U, 0U, 0U, 0U};
static int32_t speed_integral[MOTOR_COUNT] = {0, 0, 0, 0};
static uint8_t drive_open_loop_active = 0U;
static uint8_t servo1_active = 0U;
static uint8_t servo2_active = 0U;
static uint8_t ultrasonic_has_pinged = 0U;
static uint32_t last_ultrasonic_ping_ms = 0U;

static void MicrosecondTimer_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Delay_Microseconds(uint32_t delay_us)
{
  uint32_t start_cycles = DWT->CYCCNT;
  uint32_t wait_cycles =
      delay_us * (SystemCoreClock / 1000000U);

  while ((uint32_t)(DWT->CYCCNT - start_cycles) < wait_cycles)
  {
  }
}

static uint8_t Ultrasonic_WaitForPin(uint16_t pin,
                                     GPIO_PinState state,
                                     uint32_t timeout_us)
{
  uint32_t start_cycles = DWT->CYCCNT;
  uint32_t timeout_cycles =
      timeout_us * (SystemCoreClock / 1000000U);

  while (HAL_GPIO_ReadPin(GPIOC, pin) != state)
  {
    if ((uint32_t)(DWT->CYCCNT - start_cycles) >= timeout_cycles)
    {
      return 0U;
    }
  }

  return 1U;
}

static void Ultrasonic_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_8, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static int32_t Ultrasonic_Measure(uint8_t sensor)
{
  uint16_t trig_pin = (sensor == 0U) ? GPIO_PIN_8 : GPIO_PIN_5;
  uint16_t echo_pin = (sensor == 0U) ? GPIO_PIN_6 : GPIO_PIN_7;
  uint32_t pulse_start;
  uint32_t pulse_cycles;
  uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  uint32_t pulse_us;

  HAL_GPIO_WritePin(GPIOC, trig_pin, GPIO_PIN_RESET);
  if (Ultrasonic_WaitForPin(echo_pin, GPIO_PIN_RESET, 2000U) == 0U)
  {
    return -1;
  }

  Delay_Microseconds(3U);
  HAL_GPIO_WritePin(GPIOC, trig_pin, GPIO_PIN_SET);
  Delay_Microseconds(10U);
  HAL_GPIO_WritePin(GPIOC, trig_pin, GPIO_PIN_RESET);

  if (Ultrasonic_WaitForPin(echo_pin,
                            GPIO_PIN_SET,
                            ULTRASONIC_TIMEOUT_US) == 0U)
  {
    return -1;
  }

  pulse_start = DWT->CYCCNT;
  if (Ultrasonic_WaitForPin(echo_pin,
                            GPIO_PIN_RESET,
                            ULTRASONIC_TIMEOUT_US) == 0U)
  {
    return -1;
  }

  pulse_cycles = (uint32_t)(DWT->CYCCNT - pulse_start);
  pulse_us = pulse_cycles / cycles_per_us;

  return (int32_t)((pulse_us * 343U + 1000U) / 2000U);
}

static void Ultrasonic_RequestMeasurement(uint8_t sensor)
{
  uint32_t now_ms = HAL_GetTick();

  if ((ultrasonic_has_pinged == 0U) ||
      ((now_ms - last_ultrasonic_ping_ms) >=
       ULTRASONIC_MIN_INTERVAL_MS))
  {
    ultrasonic_distance_mm[sensor] = Ultrasonic_Measure(sensor);
    last_ultrasonic_ping_ms = HAL_GetTick();
    ultrasonic_has_pinged = 1U;
  }
}

static void Servo_Enable(uint8_t servo)
{
  if ((servo == 0U) && (servo1_active == 0U))
  {
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, servo1_pulse_us);
    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    servo1_active = 1U;
  }
  else if ((servo == 1U) && (servo2_active == 0U))
  {
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, servo2_pulse_us);
    if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    servo2_active = 1U;
  }
}

static uint16_t Servo_ClampPulse(uint8_t servo, int32_t pulse_us)
{
  uint16_t min_pulse_us =
      (servo == 0U) ? SERVO1_MIN_PULSE_US : SERVO2_MIN_PULSE_US;
  uint16_t max_pulse_us =
      (servo == 0U) ? SERVO1_MAX_PULSE_US : SERVO2_MAX_PULSE_US;

  if (pulse_us < (int32_t)min_pulse_us)
  {
    return min_pulse_us;
  }

  if (pulse_us > (int32_t)max_pulse_us)
  {
    return max_pulse_us;
  }

  return (uint16_t)pulse_us;
}

static uint16_t Servo_RampPulse(uint16_t current, uint16_t target)
{
  uint32_t next = current;

  if (next < target)
  {
    next += SERVO_RAMP_STEP_US;
    if (next > target)
    {
      next = target;
    }
  }
  else if (next > target)
  {
    if (next > (target + SERVO_RAMP_STEP_US))
    {
      next -= SERVO_RAMP_STEP_US;
    }
    else
    {
      next = target;
    }
  }

  return (uint16_t)next;
}

static void Servo_StopMotion(void)
{
  servo1_target_pulse_us = servo1_pulse_us;
  servo2_target_pulse_us = servo2_pulse_us;
}

static void Servo_DisableAll(void)
{
  Servo_StopMotion();

  if (servo1_active != 0U)
  {
    (void)HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
    servo1_active = 0U;
  }

  if (servo2_active != 0U)
  {
    (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
    servo2_active = 0U;
  }
}

static void Servo_Update(void)
{
  servo1_pulse_us =
      Servo_RampPulse(servo1_pulse_us, servo1_target_pulse_us);
  servo2_pulse_us =
      Servo_RampPulse(servo2_pulse_us, servo2_target_pulse_us);

  if (servo1_active != 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, servo1_pulse_us);
  }

  if (servo2_active != 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, servo2_pulse_us);
  }
}

static void Motor_SetRaw(uint8_t motor, GPIO_PinState direction, uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOC, motor_dir_pin[motor], direction);
  __HAL_TIM_SET_COMPARE(&htim1, motor_pwm_channel[motor], pwm);
}

static void Motor_StopAll(void)
{
  uint8_t motor;

  for (motor = 0U; motor < MOTOR_COUNT; motor++)
  {
    __HAL_TIM_SET_COMPARE(&htim1, motor_pwm_channel[motor], 0U);
  }
}

static uint32_t Encoder_ReadRaw(uint8_t motor)
{
  return __HAL_TIM_GET_COUNTER(motor_encoder[motor]);
}

static int32_t Encoder_GetDelta(uint8_t motor,
                                uint32_t current_raw,
                                uint32_t previous_raw)
{
  if ((motor == MOTOR_FL) || (motor == MOTOR_FR))
  {
    return (int32_t)(current_raw - previous_raw);
  }

  return (int32_t)(int16_t)((uint16_t)current_raw -
                            (uint16_t)previous_raw);
}

static void Motor_SetSigned(uint8_t motor, int32_t command)
{
  int32_t raw_command;
  uint16_t pwm;
  GPIO_PinState direction;

  if (command > MOTOR_PWM_LIMIT)
  {
    command = MOTOR_PWM_LIMIT;
  }
  else if (command < -MOTOR_PWM_LIMIT)
  {
    command = -MOTOR_PWM_LIMIT;
  }

  if (command == 0)
  {
    Motor_SetRaw(motor, GPIO_PIN_RESET, 0U);
    return;
  }

  raw_command = command * motor_forward_polarity[motor];
  direction = (raw_command > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  pwm = (uint16_t)((raw_command > 0) ? raw_command : -raw_command);
  Motor_SetRaw(motor, direction, pwm);
}

static void SpeedControl_Stop(void)
{
  uint8_t motor;

  drive_open_loop_active = 0U;

  for (motor = 0U; motor < MOTOR_COUNT; motor++)
  {
    wheel_command_rpm_x10[motor] = 0;
    wheel_target_rpm_x10[motor] = 0;
    wheel_pwm_output[motor] = 0;
    speed_integral[motor] = 0;
  }

  Motor_StopAll();
}

static int16_t SpeedControl_RampTarget(int16_t current, int16_t requested)
{
  int32_t next = current;

  if (next < requested)
  {
    next += TARGET_RAMP_STEP_RPM_X10;
    if (next > requested)
    {
      next = requested;
    }
  }
  else if (next > requested)
  {
    next -= TARGET_RAMP_STEP_RPM_X10;
    if (next < requested)
    {
      next = requested;
    }
  }

  return (int16_t)next;
}

static void SpeedControl_Update(uint32_t elapsed_ms)
{
  uint8_t motor;
  uint32_t current_raw;
  int32_t raw_delta;
  int32_t logical_delta;
  int32_t target_delta;
  int32_t error;
  int32_t feedforward;
  int32_t output;

  if (elapsed_ms == 0U)
  {
    return;
  }

  for (motor = 0U; motor < MOTOR_COUNT; motor++)
  {
    wheel_target_rpm_x10[motor] =
        SpeedControl_RampTarget(wheel_target_rpm_x10[motor],
                                wheel_command_rpm_x10[motor]);

    current_raw = Encoder_ReadRaw(motor);
    raw_delta = Encoder_GetDelta(motor,
                                 current_raw,
                                 encoder_previous_raw[motor]);
    encoder_previous_raw[motor] = current_raw;

    logical_delta = raw_delta * motor_encoder_polarity[motor];
    wheel_encoder_delta[motor] = logical_delta;
    encoder_live_counts[motor] += logical_delta;
    wheel_measured_rpm_x10[motor] =
        (int16_t)((logical_delta * 600000L) /
                  (ENCODER_COUNTS_PER_REV * (int32_t)elapsed_ms));

    /* Keep encoder telemetry active without letting PI overwrite raw PWM. */
    if (drive_open_loop_active != 0U)
    {
      continue;
    }

    if (wheel_target_rpm_x10[motor] == 0)
    {
      speed_integral[motor] = 0;
      wheel_pwm_output[motor] = 0;
      Motor_SetSigned(motor, 0);
      continue;
    }

    target_delta =
        ((int32_t)wheel_target_rpm_x10[motor] *
         ENCODER_COUNTS_PER_REV *
         (int32_t)elapsed_ms) /
        600000L;
    error = target_delta - logical_delta;

    speed_integral[motor] += error;
    if (speed_integral[motor] > SPEED_INTEGRAL_LIMIT)
    {
      speed_integral[motor] = SPEED_INTEGRAL_LIMIT;
    }
    else if (speed_integral[motor] < -SPEED_INTEGRAL_LIMIT)
    {
      speed_integral[motor] = -SPEED_INTEGRAL_LIMIT;
    }

    feedforward =
        ((int32_t)wheel_target_rpm_x10[motor] *
         PWM_FEEDFORWARD_AT_TARGET) /
        MANUAL_TARGET_RPM_X10;
    output = feedforward +
             (SPEED_KP * error) +
             (speed_integral[motor] / SPEED_KI_DIVISOR);

    if (output > MOTOR_PWM_LIMIT)
    {
      output = MOTOR_PWM_LIMIT;
    }
    else if (output < -MOTOR_PWM_LIMIT)
    {
      output = -MOTOR_PWM_LIMIT;
    }

    wheel_pwm_output[motor] = (int16_t)output;
    Motor_SetSigned(motor, output);
  }
}

static void Drive_SetVector(int32_t forward, int32_t right, int32_t clockwise)
{
  int32_t wheel[MOTOR_COUNT];
  int32_t new_target;
  int32_t max_magnitude = 0;
  int32_t magnitude;
  uint8_t motor;

  /*
   * Mecanum X-layout mixer.
   * Positive forward: robot forward
   * Positive right:   robot moves right
   * Positive turn:    robot rotates clockwise
   */
  wheel[MOTOR_FL] = forward + right + clockwise;
  wheel[MOTOR_FR] = forward - right - clockwise;
  wheel[MOTOR_RL] = forward - right + clockwise;
  wheel[MOTOR_RR] = forward + right - clockwise;

  for (motor = 0U; motor < MOTOR_COUNT; motor++)
  {
    magnitude = (wheel[motor] >= 0) ? wheel[motor] : -wheel[motor];
    if (magnitude > max_magnitude)
    {
      max_magnitude = magnitude;
    }
  }

  if (max_magnitude > MOTOR_COMMAND_MAX)
  {
    for (motor = 0U; motor < MOTOR_COUNT; motor++)
    {
      wheel[motor] = (wheel[motor] * MOTOR_COMMAND_MAX) / max_magnitude;
    }
  }

  for (motor = 0U; motor < MOTOR_COUNT; motor++)
  {
    new_target =
        (wheel[motor] * MANUAL_TARGET_RPM_X10) / MOTOR_COMMAND_MAX;
    if (wheel_command_rpm_x10[motor] != new_target)
    {
      speed_integral[motor] = 0;
    }
    wheel_command_rpm_x10[motor] = (int16_t)new_target;
  }
}

static void Drive_SetOpenLoopVector(int32_t forward,
                                    int32_t right,
                                    int32_t clockwise)
{
  int32_t wheel[MOTOR_COUNT];
  int32_t command;
  uint8_t motor;

  wheel[MOTOR_FL] = forward + right + clockwise;
  wheel[MOTOR_FR] = forward - right - clockwise;
  wheel[MOTOR_RL] = forward - right + clockwise;
  wheel[MOTOR_RR] = forward + right - clockwise;

  drive_open_loop_active = 1U;

  for (motor = 0U; motor < MOTOR_COUNT; motor++)
  {
    wheel_command_rpm_x10[motor] = 0;
    wheel_target_rpm_x10[motor] = 0;
    speed_integral[motor] = 0;

    if (wheel[motor] > 0)
    {
      command = OPEN_LOOP_PWM;
    }
    else if (wheel[motor] < 0)
    {
      command = -OPEN_LOOP_PWM;
    }
    else
    {
      command = 0;
    }

    wheel_pwm_output[motor] = (int16_t)command;
    Motor_SetSigned(motor, command);
  }
}

static uint8_t Drive_ApplyCommand(uint8_t command)
{
  switch (command)
  {
    case 'W':
      Servo_StopMotion();
      drive_open_loop_active = 0U;
      Drive_SetVector(MOTOR_COMMAND_MAX, 0, 0);
      return 'W';

    case 'w':
      Servo_StopMotion();
      Drive_SetOpenLoopVector(MOTOR_COMMAND_MAX, 0, 0);
      return 'w';

    case 'S':
      Servo_StopMotion();
      drive_open_loop_active = 0U;
      Drive_SetVector(-MOTOR_COMMAND_MAX, 0, 0);
      return 'S';

    case 's':
      Servo_StopMotion();
      Drive_SetOpenLoopVector(-MOTOR_COMMAND_MAX, 0, 0);
      return 's';

    case 'A':
      Servo_StopMotion();
      drive_open_loop_active = 0U;
      Drive_SetVector(0, -MOTOR_COMMAND_MAX, 0);
      return 'A';

    case 'a':
      Servo_StopMotion();
      Drive_SetOpenLoopVector(0, -MOTOR_COMMAND_MAX, 0);
      return 'a';

    case 'D':
      Servo_StopMotion();
      drive_open_loop_active = 0U;
      Drive_SetVector(0, MOTOR_COMMAND_MAX, 0);
      return 'D';

    case 'd':
      Servo_StopMotion();
      Drive_SetOpenLoopVector(0, MOTOR_COMMAND_MAX, 0);
      return 'd';

    case 'Q':
      Servo_StopMotion();
      drive_open_loop_active = 0U;
      Drive_SetVector(0, 0, -MOTOR_COMMAND_MAX);
      return 'Q';

    case 'q':
      Servo_StopMotion();
      Drive_SetOpenLoopVector(0, 0, -MOTOR_COMMAND_MAX);
      return 'q';

    case 'E':
      Servo_StopMotion();
      drive_open_loop_active = 0U;
      Drive_SetVector(0, 0, MOTOR_COMMAND_MAX);
      return 'E';

    case 'e':
      Servo_StopMotion();
      Drive_SetOpenLoopVector(0, 0, MOTOR_COMMAND_MAX);
      return 'e';

    case 'U':
    case 'u':
      SpeedControl_Stop();
      Servo_Enable(0U);
      servo1_target_pulse_us =
          Servo_ClampPulse(0U, (int32_t)servo1_target_pulse_us +
                           SERVO_COMMAND_STEP_US);
      return 'U';

    case 'J':
    case 'j':
      SpeedControl_Stop();
      Servo_Enable(0U);
      servo1_target_pulse_us =
          Servo_ClampPulse(0U, (int32_t)servo1_target_pulse_us -
                           SERVO_COMMAND_STEP_US);
      return 'J';

    case 'I':
    case 'i':
      SpeedControl_Stop();
      Servo_Enable(1U);
      servo2_target_pulse_us =
          Servo_ClampPulse(1U, (int32_t)servo2_target_pulse_us +
                           SERVO_COMMAND_STEP_US);
      return 'I';

    case 'K':
    case 'k':
      SpeedControl_Stop();
      Servo_Enable(1U);
      servo2_target_pulse_us =
          Servo_ClampPulse(1U, (int32_t)servo2_target_pulse_us -
                           SERVO_COMMAND_STEP_US);
      return 'K';

    case 'T':
    case 't':
      SpeedControl_Stop();
      Servo_Enable(0U);
      Servo_Enable(1U);
#if ROBOT_HAS_ARUCO_MARKER
      servo1_target_pulse_us =
          Servo_ClampPulse(0U, (int32_t)servo1_target_pulse_us -
                           SERVO_COMMAND_STEP_US);
      servo2_target_pulse_us =
          Servo_ClampPulse(1U, (int32_t)servo2_target_pulse_us +
                           SERVO_COMMAND_STEP_US);
#else
      servo1_target_pulse_us =
          Servo_ClampPulse(0U, (int32_t)servo1_target_pulse_us +
                           SERVO_COMMAND_STEP_US);
      servo2_target_pulse_us =
          Servo_ClampPulse(1U, (int32_t)servo2_target_pulse_us -
                           SERVO_COMMAND_STEP_US);
#endif
      return 'T';

    case 'G':
    case 'g':
      SpeedControl_Stop();
      Servo_Enable(0U);
      Servo_Enable(1U);
#if ROBOT_HAS_ARUCO_MARKER
      servo1_target_pulse_us =
          Servo_ClampPulse(0U, (int32_t)servo1_target_pulse_us +
                           SERVO_COMMAND_STEP_US);
      servo2_target_pulse_us =
          Servo_ClampPulse(1U, (int32_t)servo2_target_pulse_us -
                           SERVO_COMMAND_STEP_US);
#else
      servo1_target_pulse_us =
          Servo_ClampPulse(0U, (int32_t)servo1_target_pulse_us -
                           SERVO_COMMAND_STEP_US);
      servo2_target_pulse_us =
          Servo_ClampPulse(1U, (int32_t)servo2_target_pulse_us +
                           SERVO_COMMAND_STEP_US);
#endif
      return 'G';

    case 'O':
    case 'o':
      SpeedControl_Stop();
      Servo_DisableAll();
      return 'O';

    case '1':
      SpeedControl_Stop();
      Servo_StopMotion();
      Ultrasonic_RequestMeasurement(0U);
      return '1';

    case '2':
      SpeedControl_Stop();
      Servo_StopMotion();
      Ultrasonic_RequestMeasurement(1U);
      return '2';

    case 'X':
    case 'x':
    case ' ':
      SpeedControl_Stop();
      Servo_StopMotion();
      return 'X';

    default:
      return 0U;
  }
}

static uint8_t Command_IsDriveMotion(uint8_t command)
{
  return ((command == 'W') ||
          (command == 'S') ||
          (command == 'A') ||
          (command == 'D') ||
          (command == 'Q') ||
          (command == 'E') ||
          (command == 'w') ||
          (command == 's') ||
          (command == 'a') ||
          (command == 'd') ||
          (command == 'q') ||
          (command == 'e')) ? 1U : 0U;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  uint8_t motor;
  uint8_t rx_byte;
  uint8_t accepted_command;
  uint32_t now_ms;
  uint32_t last_command_ms;
  uint32_t last_control_ms;
  uint32_t last_servo_update_ms;
  uint32_t last_telemetry_ms;
  uint32_t control_elapsed_ms;
  char telemetry[128];
  int telemetry_length;

  if ((HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK))
  {
    Error_Handler();
  }

  if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK))
  {
    Error_Handler();
  }

  MicrosecondTimer_Init();
  Ultrasonic_GPIO_Init();

  Motor_StopAll();
  HAL_GPIO_WritePin(GPIOC,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  for (motor = 0U; motor < MOTOR_COUNT; motor++)
  {
    __HAL_TIM_SET_COUNTER(motor_encoder[motor], 0U);
    encoder_previous_raw[motor] = Encoder_ReadRaw(motor);
  }

  now_ms = HAL_GetTick();
  last_command_ms = now_ms;
  last_control_ms = now_ms;
  last_servo_update_ms = now_ms;
  last_telemetry_ms = now_ms;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now_ms = HAL_GetTick();

    if (HAL_UART_Receive(&huart2, &rx_byte, 1U, 1U) == HAL_OK)
    {
      accepted_command = Drive_ApplyCommand(rx_byte);
      if (accepted_command != 0U)
      {
        drive_command = accepted_command;
        last_command_ms = now_ms;
        drive_watchdog_stopped =
            (Command_IsDriveMotion(accepted_command) != 0U) ? 0U : 1U;
        HAL_GPIO_WritePin(LD2_GPIO_Port,
                          LD2_Pin,
                          (Command_IsDriveMotion(accepted_command) != 0U) ?
                              GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }

    if ((drive_watchdog_stopped == 0U) &&
        ((now_ms - last_command_ms) > DRIVE_TIMEOUT_MS))
    {
      SpeedControl_Stop();
      drive_command = 'X';
      drive_watchdog_stopped = 1U;
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }

    control_elapsed_ms = now_ms - last_control_ms;
    if (control_elapsed_ms >= SPEED_CONTROL_PERIOD_MS)
    {
      last_control_ms = now_ms;
      SpeedControl_Update(control_elapsed_ms);
    }

    if ((now_ms - last_servo_update_ms) >= SERVO_UPDATE_PERIOD_MS)
    {
      last_servo_update_ms = now_ms;
      Servo_Update();
    }

    if ((now_ms - last_telemetry_ms) >= TELEMETRY_PERIOD_MS)
    {
      last_telemetry_ms = now_ms;
      telemetry_length = snprintf(
          telemetry,
          sizeof(telemetry),
          "T,%c,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%ld,%ld\r\n",
          (char)drive_command,
          (int)wheel_measured_rpm_x10[MOTOR_FL],
          (int)wheel_measured_rpm_x10[MOTOR_FR],
          (int)wheel_measured_rpm_x10[MOTOR_RL],
          (int)wheel_measured_rpm_x10[MOTOR_RR],
          (int)wheel_pwm_output[MOTOR_FL],
          (int)wheel_pwm_output[MOTOR_FR],
          (int)wheel_pwm_output[MOTOR_RL],
          (int)wheel_pwm_output[MOTOR_RR],
          (unsigned int)servo1_pulse_us,
          (unsigned int)servo2_pulse_us,
          (long)ultrasonic_distance_mm[0],
          (long)ultrasonic_distance_mm[1]);

      if (telemetry_length > 0)
      {
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)telemetry,
                          (uint16_t)telemetry_length,
                          20U);
      }
    }

    HAL_Delay(5);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 19999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 83;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 19999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
