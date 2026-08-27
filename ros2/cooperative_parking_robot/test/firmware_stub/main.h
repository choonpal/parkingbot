#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stddef.h>
#include <stdint.h>

typedef struct { void *Instance; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3,
} HAL_StatusTypeDef;

#define USART2 ((void *)2)
#define GPIOB ((GPIO_TypeDef *)0)
#define GPIOC ((GPIO_TypeDef *)0)
#define GPIO_PIN_0 0U
#define GPIO_PIN_1 1U
#define GPIO_PIN_2 2U
#define GPIO_PIN_3 3U
#define GPIO_PIN_5 5U
#define GPIO_PIN_6 6U
#define GPIO_PIN_7 7U
#define GPIO_PIN_8 8U
#define GPIO_PIN_12 12U
#define GPIO_PIN_13 13U
#define GPIO_PIN_14 14U
#define GPIO_PIN_15 15U
#define TIM_CHANNEL_1 1U
#define TIM_CHANNEL_2 2U
#define TIM_CHANNEL_3 3U
#define TIM_CHANNEL_4 4U
#define TIM_CHANNEL_ALL 0xFFFFU

#define MOTOR1_DIR_Pin GPIO_PIN_0
#define MOTOR1_DIR_GPIO_Port GPIOC
#define MOTOR2_DIR_Pin GPIO_PIN_1
#define MOTOR2_DIR_GPIO_Port GPIOC
#define MOTOR3_DIR_Pin GPIO_PIN_2
#define MOTOR3_DIR_GPIO_Port GPIOC
#define MOTOR4_DIR_Pin GPIO_PIN_3
#define MOTOR4_DIR_GPIO_Port GPIOC
#define ULTRASONIC2_TRIG_Pin GPIO_PIN_5
#define ULTRASONIC2_TRIG_GPIO_Port GPIOC
#define ULTRASONIC1_ECHO_Pin GPIO_PIN_6
#define ULTRASONIC1_ECHO_GPIO_Port GPIOC
#define ULTRASONIC2_ECHO_Pin GPIO_PIN_7
#define ULTRASONIC2_ECHO_GPIO_Port GPIOC
#define ULTRASONIC1_TRIG_Pin GPIO_PIN_8
#define ULTRASONIC1_TRIG_GPIO_Port GPIOC

uint32_t HAL_GetTick(void);
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_Base_Start(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);

#define __HAL_TIM_GET_COUNTER(handle) ((void)(handle), 0U)
#define __HAL_TIM_GET_AUTORELOAD(handle) ((void)(handle), 999U)
#define __HAL_TIM_SET_COUNTER(handle, value) \
    do { (void)(handle); (void)(value); } while (0)
#define __HAL_TIM_SET_COMPARE(handle, channel, value) \
    do { (void)(handle); (void)(channel); (void)(value); } while (0)
#define __disable_irq() do {} while (0)
#define __enable_irq() do {} while (0)

#endif
