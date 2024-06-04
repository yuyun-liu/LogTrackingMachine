/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	GPIO_TypeDef* Group;
	uint16_t Pin;
}GPIO_FUNCTION_STRUCT;

typedef struct
{
	GPIO_FUNCTION_STRUCT* enable;
	GPIO_FUNCTION_STRUCT* direction;
	TIM_TypeDef* pulse;
	int motor_index;
	float current_degree;
	uint32_t motor_motion_start_cnt;
	uint32_t motor_motion_start_tick;
	int motor_motion_flag;
}STEPPER_MOTOR;

typedef struct
{
	float inch_per_second;
	float degree;
	uint32_t second;
	uint32_t minute;
	uint32_t hour;
}HMI_PARAMETERS;

typedef struct
{
	_Bool function_status;
	_Bool newBtnStatus;
	_Bool lastBtnStatus;
	_Bool reach;
	_Bool reset;
	uint32_t count;
	uint32_t target;
}BUTTON_STATUS;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define PULSE_PER_REV 6400
#define WHEEL_DIAMETER 66 	//Unit: mm
#define PI 3.1415926535897932384626433
#define COUNT_PER_REV 20

#define MSG_HEAD_SIZE 4
#define MAX_BUFFER_SIZE 36
#define HMI_PARAMETERS_LENGTH 2

#define PAGE_INIT 0
#define PAGE_MAIN 1

#define EVENT_INCREASE_RPM 0xA1
#define EVENT_DECREASE_RPM 0xA2
#define EVENT_TURN_RIGHT 0xA3
#define EVENT_TURN_LEFT 0xA4
#define EVENT_STOP 0xA5

#define PARAM_INCH_PER_SECOND 0
#define PARAM_DEGREE 1
#define VALUE_MOVEMENT 4
#define VALUE_EXECUTION_TIME 5

#define MOTOR_IDLE 0
#define MOTOR_READY_MOVING 1
#define MOTOR_MOVING 2

#define TASK_IDLE 0
#define TASK_INIT 1
#define TASK_MOVING_PLATE 2
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
_Bool isElapsedTime(uint32_t _TimeInterval ,uint32_t* _lastTimeTick);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
extern uint32_t new_tick;
extern int page_index;
extern int event_id;

extern float velocity_0;

extern uint32_t lastTimeTick_value_movement;
extern uint32_t lastTimeTick_execution_time;

extern float hmi_parameters_buffer[];
extern HMI_PARAMETERS hmi_parameters;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern STEPPER_MOTOR m1;
extern STEPPER_MOTOR m2;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
