/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "hmi.h"
#include "ui.h"
#include <stdlib.h>
#include <math.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
GPIO_FUNCTION_STRUCT m1_enable = {GPIOB, GPIO_PIN_0};
GPIO_FUNCTION_STRUCT m1_direction = {GPIOB, GPIO_PIN_1};
STEPPER_MOTOR m1 = {&m1_enable, &m1_direction, TIM3, 1, 0, 0, MOTOR_IDLE, 0, 0, 0};
GPIO_FUNCTION_STRUCT m2_enable = {GPIOB, GPIO_PIN_2};
GPIO_FUNCTION_STRUCT m2_direction = {GPIOB, GPIO_PIN_3};
GPIO_FUNCTION_STRUCT m2_pulse = {GPIOA, GPIO_PIN_4};
STEPPER_MOTOR m2 = {&m2_enable, &m2_direction, TIM14, 2, 90, 0, MOTOR_IDLE, 0, 0, 0};
GPIO_FUNCTION_STRUCT limit_sensor = {GPIOA, GPIO_PIN_15};

HMI_PARAMETERS hmi_parameters = {0.0, 90, 0, 0, 0};
AUTOMODE_PARAMETERS automode_parameters = {0, 0, 0};
float hmi_parameters_buffer[HMI_PARAMETERS_LENGTH];
float automode_parameters_buffer[AUTOMODE_PARAMETERS_LENGTH];

uint32_t new_tick = 0;
uint32_t init_working_tick = 0;
_Bool tick_overflow_flag = 0;
int page_index = 0;
int event_id = -1;
int main_loop_task = TASK_INIT;
uint32_t last_count = 0;
int actual_movement = 0; //unit: mm
_Bool auto_mode = 0;
int auto_mode_iteration = 0;
uint32_t target_execution_time_per_speed_interval = 0;
int target_iteration = 0;

float lastMotorSpeed = 0;
float velocity_0 = 0;
float acc = 0.25;
float dacc = 0.75;
uint32_t time_to_acc = 0;
uint32_t time_to_dacc = 0;
_Bool accelerating = 0;

uint32_t lastTimeTick_speed_change = 0;
uint32_t lastTimeTick_value_movement = 0;
uint32_t lastTimeTick_execution_time = 0;
uint32_t lastTimeTick_limit_sensor_detected = 0;
uint32_t lastTimeTick_target_deg_change = 0;
uint32_t lastTimeTick_compute_actual_movement = 0;
uint32_t lastTimeTick_auto_mode = 0;

uint32_t lastTimeTick_test_only = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
void Main_Loop_Task_Dispatcher();
void Compute_Execution_Time(uint32_t tick);
void set_motor_speed_dps(STEPPER_MOTOR* motor, float degree_per_second);
void set_motor_rotation_deg(STEPPER_MOTOR* motor, int target_degree);
void set_motor_speed_mmps(STEPPER_MOTOR* motor, float mm_per_second);
void set_motor_speed_mmps_with_acc(STEPPER_MOTOR* motor, float mm_per_second, _Bool force_refresh_target_speed);
void set_motor_speed_inchps_with_acc(STEPPER_MOTOR* motor, float inch_per_second, _Bool force_refresh_target_speed);
void set_motor_rotation_deg_by_pulses(STEPPER_MOTOR* motor, int target_degree);
void Compute_Actual_Movement();
void Auto_Mode();
void Handle_Message(MSG_RX_BUFFER buff);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  //MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
	MSG_InitializeQeueue();
	USART1_START_RX();
	USART3_START_RX();
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(m1.enable->Group, m1.enable->Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(m2.enable->Group, m2.enable->Pin, GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(&htim2);

	WritePage_to_HMI(PAGE_INIT);
	HAL_Delay(2000);
	WritePage_to_HMI(PAGE_MAIN);
	init_working_tick = HAL_GetTick();
	
	Write_Parameters_To_HMI(PARAM_INCH_PER_SECOND, hmi_parameters.inch_per_second);
	Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		new_tick = HAL_GetTick();
		
		Main_Loop_Task_Dispatcher();
		
		if(isElapsedTime(1000, &lastTimeTick_test_only))
		{
			//Write_MotorDriver(1, 2, 3);
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 15;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
	
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_NVIC_SetPriority(USART3_4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_4_IRQn);
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void Main_Loop_Task_Dispatcher()
{
	switch(main_loop_task)
	{
		case TASK_IDLE:
			set_motor_speed_inchps_with_acc(&m1, hmi_parameters.inch_per_second, FALSE);
			set_motor_rotation_deg_by_pulses(&m2, (int)hmi_parameters.degree);
			
			Compute_Actual_Movement();
			Compute_Execution_Time(new_tick);
			Indicate_Actual_Movement(actual_movement);
			Indicate_Execution_Time();
			
			Set_Parameters();
			Update_Up_Down_Btn_count();
			Handle_Message(msg_GetRXQData());
			break;
		case TASK_INIT:
			main_loop_task = TASK_IDLE;
			/*
			set_motor_rotation_deg_by_pulses(&m2, 360);
			if(m2.current_degree == 180)
			{
				main_loop_task = TASK_IDLE;
				hmi_parameters.degree = 90.0;
				Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
			}*/
			break;
		case TASK_AUTO_MODE:
			Auto_Mode();
			set_motor_rotation_deg_by_pulses(&m2, (int)hmi_parameters.degree);
		
			Compute_Actual_Movement();
			Compute_Execution_Time(new_tick);
			Indicate_Actual_Movement(actual_movement);
			Indicate_Execution_Time();
			
			Set_Parameters();
			Update_Up_Down_Btn_count();
			Handle_Message(msg_GetRXQData());
			break;
		case TASK_MOVING_PLATE:
			break;
		default:
			break;
	}
}

void Compute_Execution_Time(uint32_t tick)
{
	uint32_t elapsedTick = 0;

	if(tick < init_working_tick)
	{
		tick_overflow_flag = 1;
		elapsedTick = (uint32_t)((4294967295 - init_working_tick + tick)/1000);
	}
	else if((tick >= init_working_tick) && tick_overflow_flag)
	{
		elapsedTick = 359999999; //99H59M59S
	}
	else
	{
		elapsedTick = (uint32_t)((tick - init_working_tick)/1000);
	}
	hmi_parameters.second = elapsedTick % 60;
	elapsedTick = elapsedTick/60;
	hmi_parameters.minute = elapsedTick % 60;
	elapsedTick = elapsedTick/60;
	hmi_parameters.hour = elapsedTick % 60;
}

_Bool isElapsedTime(uint32_t _TimeInterval ,uint32_t* _lastTimeTick)
{
	uint32_t _elapsedTick = 0;
	if(*_lastTimeTick > new_tick)
	{
		_elapsedTick = (uint32_t)(4294967295 - *_lastTimeTick + new_tick);
	}
	else
	{
		_elapsedTick = (uint32_t)(new_tick - *_lastTimeTick);
	}
	
	if(_elapsedTick >= _TimeInterval)
	{
		*_lastTimeTick = new_tick;
		return 1;
	}
	else
	{
		return 0;
	}
}

_Bool isElapsedTimeNoReset(uint32_t _TimeInterval ,uint32_t _lastTimeTick)
{
	uint32_t _elapsedTick = 0;
	if(_lastTimeTick > new_tick)
	{
		_elapsedTick = (uint32_t)(4294967295 - _lastTimeTick + new_tick);
	}
	else
	{
		_elapsedTick = (uint32_t)(new_tick - _lastTimeTick);
	}
	
	if(_elapsedTick >= _TimeInterval)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint32_t set_timer_frequency(TIM_TypeDef* timer, uint32_t frequency)
{
	uint32_t arr_value = 0;
	if(frequency != 0)
	{
		arr_value = ((HSI48_VALUE) / (frequency * (timer->PSC + 1))) - 1;
	}
	timer->CCR1 = arr_value * 0.5;
	timer->ARR = arr_value;
	
	return arr_value;
}

void set_motor_speed_dps(STEPPER_MOTOR* motor, float degree_per_second)
{
	if((int)degree_per_second == 0)
	{
		HAL_GPIO_WritePin(motor->enable->Group, motor->enable->Pin, GPIO_PIN_SET);
	}
	else if(degree_per_second < 0)
	{
		HAL_GPIO_WritePin(motor->enable->Group, motor->enable->Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->direction->Group, motor->direction->Pin, GPIO_PIN_RESET);
		degree_per_second *= -1;
	}
	else
	{
		HAL_GPIO_WritePin(motor->enable->Group, motor->enable->Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->direction->Group, motor->direction->Pin, GPIO_PIN_SET);
	}
	set_timer_frequency(motor->pulse, (uint32_t)((float)(degree_per_second / 360.0) * (float)PULSE_PER_REV));
}

void set_motor_speed_mmps(STEPPER_MOTOR* motor, float mm_per_second)
{
	float _degree_per_second = ((mm_per_second / ((float)WHEEL_DIAMETER * PI)) * 360.0);
	set_motor_speed_dps(motor, _degree_per_second);
}

void set_motor_speed_mmps_with_acc(STEPPER_MOTOR* motor, float mm_per_second, _Bool force_refresh_target_speed)
{
	float _mm_per_second;
	float _acc = acc;
	uint32_t _time_to_acc;
#ifdef START_BY_START_BTN
	if(stopBtnStatus.newBtnStatus && !stopBtnStatus.lastBtnStatus)
	{
		if((motor->motor_motion_flag == MOTOR_MOVING) && !force_refresh_target_speed)
			return;
		
		time_to_dacc = (uint32_t)((float)(abs((int)(0 - motor->current_speed))) / dacc);
		motor->target_speed = 0;
		lastTimeTick_speed_change = new_tick;
		motor->motor_motion_flag = MOTOR_STOPING;
	}
	
	if(motor->motor_motion_flag == MOTOR_STOPING)
	{		
		if(motor->current_speed != 0)
		{
			_time_to_acc = time_to_dacc;
			if(isElapsedTimeNoReset(_time_to_acc, lastTimeTick_speed_change))
			{
				motor->current_speed = 0;
				_mm_per_second = 0;
			}
			else
			{
				uint32_t _elapsedTick = 0;
				if(lastTimeTick_speed_change > new_tick)
				{
					_elapsedTick = (uint32_t)(4294967295 - lastTimeTick_speed_change + new_tick);
				}
				else
				{
					_elapsedTick = (uint32_t)(new_tick - lastTimeTick_speed_change);
				}
				
				if(motor->current_speed >= 0)
				{
					_mm_per_second = (motor->current_speed - ((float)dacc * ((float)(((int)(_elapsedTick / 50)) * 50))));
				}
				else
				{
					_mm_per_second = (motor->current_speed + ((float)dacc * ((float)(((int)(_elapsedTick / 50)) * 50))));
				}
			}
			set_motor_speed_mmps(motor, _mm_per_second);
		}
		else
		{
			motor->motor_motion_flag = MOTOR_IDLE;
			motor->last_speed = 0;
		}
		return;
	}
	
	if((startBtnStatus.newBtnStatus && !startBtnStatus.lastBtnStatus) || force_refresh_target_speed)
	{
		if(motor->motor_motion_flag == MOTOR_IDLE)
		{
			motor->motor_motion_flag = MOTOR_MOVING;
			motor->target_speed = mm_per_second;
		}
	}
	
	if(motor->motor_motion_flag == MOTOR_MOVING)
#else
	if(!upBtnStatus.newBtnStatus && !downBtnStatus.newBtnStatus)
#endif
	{
		if((motor->target_speed != motor->last_speed) && (motor->current_speed != motor->target_speed))
		{
			time_to_acc = (uint32_t)((float)(abs((int)(motor->target_speed - motor->last_speed))) / acc);
			time_to_dacc = (uint32_t)((float)(abs((int)(motor->target_speed - motor->last_speed))) / dacc);
			motor->current_speed = motor->last_speed;
			motor->last_speed = motor->target_speed;
			lastTimeTick_speed_change = new_tick;
		}
		
		if(motor->current_speed != motor->target_speed)
		{
			_time_to_acc = (motor->current_speed > motor->target_speed) ? time_to_dacc : time_to_acc;
			if(isElapsedTimeNoReset(_time_to_acc, lastTimeTick_speed_change))
			{
				motor->current_speed = motor->target_speed;
				_mm_per_second = motor->target_speed;
			}
			else
			{
				uint32_t _elapsedTick = 0;
				if(lastTimeTick_speed_change > new_tick)
				{
					_elapsedTick = (uint32_t)(4294967295 - lastTimeTick_speed_change + new_tick);
				}
				else
				{
					_elapsedTick = (uint32_t)(new_tick - lastTimeTick_speed_change);
				}
				if(motor->current_speed > motor->target_speed)
				{
					_mm_per_second = (motor->current_speed - ((float)dacc * ((float)(((int)(_elapsedTick / 50)) * 50))));
				}
				else
				{
					_mm_per_second = (motor->current_speed + ((float)acc * ((float)(((int)(_elapsedTick / 50)) * 50))));
				}
			}
			set_motor_speed_mmps(motor, _mm_per_second);
		}
		else
		{
			m1.motor_motion_flag = MOTOR_IDLE;
		}
	}
}

void set_motor_speed_inchps_with_acc(STEPPER_MOTOR* motor, float inch_per_second, _Bool force_refresh_target_speed)
{
	set_motor_speed_mmps_with_acc(motor, inch_per_second * 25.4, force_refresh_target_speed);
}

void set_motor_rotation_deg(STEPPER_MOTOR* motor, int target_degree)
{
	if(motor->motor_motion_flag == MOTOR_IDLE)
	{
		if((int)motor->current_degree != target_degree)
		{
			motor->motor_motion_start_cnt = 0;
			set_motor_speed_dps(motor, 0);
			HAL_GPIO_WritePin(motor->enable->Group, motor->enable->Pin, GPIO_PIN_RESET);
			motor->motor_motion_flag = MOTOR_READY_MOVING;
			motor->motor_motion_start_tick = new_tick;
		}
	}
	else
	{
		if(motor->motor_motion_flag == MOTOR_READY_MOVING)
		{
			if(isElapsedTime(1000, &motor->motor_motion_start_tick))
			{
				motor->motor_motion_flag = MOTOR_MOVING;
				const float _degree_per_secnod = (target_degree - motor->current_degree) >= 0 ? 60 : -60;
				set_motor_speed_dps(motor, _degree_per_secnod);
			}
		}
		else
		{
			uint32_t target_execution_time = (uint32_t)(((float)(abs((int)(target_degree - motor->current_degree))) / 60.0) * 1000.0);
			motor->motor_motion_start_cnt = target_execution_time;
			if(isElapsedTime(target_execution_time, &m2.motor_motion_start_tick))
			{
				set_motor_speed_dps(motor, 0);
				motor->current_degree = target_degree;
				motor->motor_motion_flag = MOTOR_IDLE;
			}
		}
	}
}

void set_motor_rotation_deg_by_pulses(STEPPER_MOTOR* motor, int target_degree)
{
	if(motor->motor_motion_flag == MOTOR_IDLE)
	{
		if(upBtnStatus.newBtnStatus || downBtnStatus.newBtnStatus)
		{
			return;
		}
		
		if((int)motor->current_degree != target_degree)
		{
			if(!isElapsedTime(500, &lastTimeTick_target_deg_change))
			{
				return;
			}
			
			if((target_degree - motor->current_degree) < 0)
			{
				HAL_GPIO_WritePin(motor->enable->Group, motor->enable->Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor->direction->Group, motor->direction->Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(motor->enable->Group, motor->enable->Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor->direction->Group, motor->direction->Pin, GPIO_PIN_SET);
			}
			motor->motor_motion_start_cnt = 0;
			HAL_GPIO_WritePin(motor->enable->Group, motor->enable->Pin, GPIO_PIN_RESET);
			motor->motor_motion_flag = MOTOR_READY_MOVING;
			motor->motor_motion_start_tick = new_tick;
		}
		else
		{
			lastTimeTick_target_deg_change = new_tick;
		}
	}
	else
	{
		if(motor->motor_motion_flag == MOTOR_READY_MOVING)
		{
			if(isElapsedTime(200, &motor->motor_motion_start_tick))
			{
				motor->motor_motion_flag = MOTOR_MOVING;
			}
		}
		else
		{
			if((m2.motor_motion_start_cnt / 2) < (int)(((abs((int)(target_degree - motor->current_degree))) / 360.0) * (float)PULSE_PER_REV))
			{
				if(HAL_GPIO_ReadPin(limit_sensor.Group, limit_sensor.Pin) && ((target_degree - motor->current_degree) > 0))
				{
					if(isElapsedTime(100, &lastTimeTick_limit_sensor_detected))
					{
						m2.current_degree = 180.0;
						hmi_parameters.degree = m2.current_degree;
						motor->motor_motion_flag = MOTOR_IDLE;
						Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
					}
				}
				else
				{
					lastTimeTick_limit_sensor_detected = new_tick;
				}
				
				if(isElapsedTime(1, &m2.motor_motion_start_tick))
				{
					HAL_GPIO_WritePin(m2_pulse.Group, m2_pulse.Pin, m2.motor_motion_start_cnt % 2);
					m2.motor_motion_start_cnt += 1;
				}
			}
			else
			{
				m2.current_degree = target_degree;
				motor->motor_motion_flag = MOTOR_IDLE;
			}
		}
	}
}

void Compute_Actual_Movement()
{
	uint32_t _time_interval = 1000;
	if(isElapsedTime(_time_interval, &lastTimeTick_compute_actual_movement))
	{
		uint32_t _elapsedCnt = 0;
		uint32_t current_cnt = TIM2->CNT;
		if(last_count > current_cnt)
		{
			_elapsedCnt = (uint32_t)(htim2.Init.Period - last_count + current_cnt);
		}
		else
		{
			_elapsedCnt = (uint32_t)(current_cnt - last_count);
		}
		last_count = current_cnt;
		actual_movement = (int)(((float)(_elapsedCnt * (1000 / _time_interval)) / COUNT_PER_REV) * (float)WHEEL_DIAMETER * PI);
		actual_movement /= 25.4; // mm to inch
	}
}

void Auto_Mode()
{
	if(!auto_mode)
	{
		if((int)automode_parameters.hour | (int)automode_parameters.minute | (int)automode_parameters.second)
		{
			if(startBtnStatus.newBtnStatus && !startBtnStatus.lastBtnStatus)
			{
				// Skip if currect set target speed of auto mode is equal to auto start speed
				if(hmi_parameters.inch_per_second <= AUTO_MODE_START_SPEED)
				{
					return;
				}
				
				auto_mode = 1;
				auto_mode_iteration = 0;
				lastTimeTick_auto_mode = new_tick;
				init_working_tick = new_tick;
				
				// Set auto mode speed to start speed
				set_motor_speed_inchps_with_acc(&m1, AUTO_MODE_START_SPEED, TRUE);
			}
			else
			{
				set_motor_speed_inchps_with_acc(&m1, 0, FALSE);
			}
		}
	}
	
	if(auto_mode)
	{
		// Exit if the actual movement (speed) is not reach the start speed of auto mode.
		if(actual_movement < (AUTO_MODE_START_SPEED - 1))
		{
			set_motor_speed_inchps_with_acc(&m1, AUTO_MODE_START_SPEED, TRUE);
			lastTimeTick_auto_mode = new_tick;
			return;
		}
		
		if(stopBtnStatus.newBtnStatus && !stopBtnStatus.lastBtnStatus)
		{
			auto_mode = 0;
			auto_mode_iteration = 0;
			set_motor_speed_inchps_with_acc(&m1, 0, TRUE);
			return;
		}
		
		target_execution_time_per_speed_interval = (((uint32_t)automode_parameters.hour * 3600) + ((uint32_t)automode_parameters.minute * 60) + (uint32_t)automode_parameters.second) * 1000;
		target_iteration = (abs)((int)((hmi_parameters.inch_per_second - AUTO_MODE_START_SPEED) / (float)AUTO_MODE_SPEED_INTERVAL));
		if(isElapsedTime(target_execution_time_per_speed_interval, &lastTimeTick_auto_mode))
		{
			if(auto_mode_iteration >= target_iteration)
			{
				set_motor_speed_inchps_with_acc(&m1, 0, TRUE);
			}
			else
			{
				float _inch_per_second = (float)(AUTO_MODE_START_SPEED + (AUTO_MODE_SPEED_INTERVAL * (auto_mode_iteration + 1))) * (hmi_parameters.inch_per_second >= 0 ? 1 : -1);
				set_motor_speed_inchps_with_acc(&m1, _inch_per_second, TRUE);
			}
			auto_mode_iteration += 1;
		}
		
		if(auto_mode_iteration >= (target_iteration + 1))
		{
			auto_mode = 0;
			auto_mode_iteration = 0;
			set_motor_speed_inchps_with_acc(&m1, 0, TRUE);
		}
		else
		{
			float _inch_per_second = (float)(AUTO_MODE_START_SPEED + (AUTO_MODE_SPEED_INTERVAL * (auto_mode_iteration + 1))) * (hmi_parameters.inch_per_second >= 0 ? 1 : -1);
			set_motor_speed_inchps_with_acc(&m1, _inch_per_second, FALSE);
		}
	}
}

void Handle_Message(MSG_RX_BUFFER buff)
{
	if(buff.hasReceived)
	{
		switch(buff.buffer[0])
		{
			case 0x01:	//FROM PC
				switch(buff.buffer[2])
				{
					case EVENT_INCREASE_RPM:	// Change the RPM via PC
						hmi_parameters.inch_per_second = buff.buffer[4];
						Write_Parameters_To_HMI(PARAM_INCH_PER_SECOND, hmi_parameters.inch_per_second);
						break;
					case EVENT_TURN_LEFT:			// Change the Rotator degree
						hmi_parameters.degree = buff.buffer[4];
						Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
						break;
					case EVENT_AUTO_HR_INC:
						automode_parameters.hour = buff.buffer[4];
						automode_parameters.minute = buff.buffer[5];
						automode_parameters.second = buff.buffer[6];
						
						WritePage_to_HMI(PAGE_MAIN);
						Write_Parameters_To_HMI(VALUE_AUTOMODE_TIME, 0);
						Write_Parameters_To_HMI(PARAM_INCH_PER_SECOND, hmi_parameters.inch_per_second);
						Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
					
						if((int)automode_parameters.hour | (int)automode_parameters.minute | (int)automode_parameters.second)
						{
							main_loop_task = TASK_AUTO_MODE;
							Write_AutoModeTimeInterval_Background_to_HMI(GREEN_ACTIVE);
						}
						else
						{
							main_loop_task = TASK_IDLE;
							Write_AutoModeTimeInterval_Background_to_HMI(WHITE_DEACTIVE);
						}
						break;
					default:
						break;
				}
				break;
			case 0x02:	//From HMI
				event_id = buff.buffer[2];
				switch(buff.buffer[2])
				{
					case EVENT_INCREASE_RPM:
						upBtnStatus.newBtnStatus = buff.buffer[4];
						upBtnStatus.count = new_tick;
						break;
					case EVENT_DECREASE_RPM:
						downBtnStatus.newBtnStatus = buff.buffer[4];
						downBtnStatus.count = new_tick;
						break;
					case EVENT_TURN_RIGHT:
						downBtnStatus.newBtnStatus = buff.buffer[4];
						downBtnStatus.count = new_tick;
						break;
					case EVENT_TURN_LEFT:
						upBtnStatus.newBtnStatus = buff.buffer[4];
						upBtnStatus.count = new_tick;
						break;
					case EVENT_STOP:
						stopBtnStatus.newBtnStatus = buff.buffer[4];
						stopBtnStatus.count = new_tick;
						//hmi_parameters.inch_per_second = 0;
						//hmi_parameters_buffer[0] = 0;
						/*
						if(m2.motor_motion_flag == MOTOR_IDLE)
						{
							hmi_parameters.degree = 90.0;
							hmi_parameters_buffer[1] = 90.0;
						}*/
						Write_Parameters_To_HMI(PARAM_INCH_PER_SECOND, hmi_parameters.inch_per_second);
						Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
						break;
					case EVENT_START:
						startBtnStatus.newBtnStatus = buff.buffer[4];
						startBtnStatus.count = new_tick;
						break;
					case EVENT_AUTO:
						if(m2.motor_motion_flag == MOTOR_IDLE)
						{
							automode_parameters.tmp_hour = automode_parameters.hour;
							automode_parameters.tmp_minute = automode_parameters.minute;
							automode_parameters.tmp_second = automode_parameters.second;
							WritePage_to_HMI(PAGE_AUTO);
							Write_Parameters_To_HMI(PARAM_AUTOHOUR, automode_parameters.hour);
							Write_Parameters_To_HMI(PARAM_AUTOMIN, automode_parameters.minute);
							Write_Parameters_To_HMI(PARAM_AUTOSEC, automode_parameters.second);
						}
						break;
					case EVENT_AUTO_HR_INC:
						upHourBtnStatus.newBtnStatus = buff.buffer[4];
						upHourBtnStatus.count = new_tick;
						break;
					case EVENT_AUTO_HR_DEC:
						downHourBtnStatus.newBtnStatus = buff.buffer[4];
						downHourBtnStatus.count = new_tick;
						break;
					case EVENT_AUTO_MIN_INC:
						upMinBtnStatus.newBtnStatus = buff.buffer[4];
						upMinBtnStatus.count = new_tick;
						break;
					case EVENT_AUTO_MIN_DEC:
						downMinBtnStatus.newBtnStatus = buff.buffer[4];
						downMinBtnStatus.count = new_tick;
						break;
					case EVENT_AUTO_SEC_INC:
						upSecBtnStatus.newBtnStatus = buff.buffer[4];
						upSecBtnStatus.count = new_tick;
						break;
					case EVENT_AUTO_SEC_DEC:
						downSecBtnStatus.newBtnStatus = buff.buffer[4];
						downSecBtnStatus.count = new_tick;
						break;
					case EVENT_AUTO_CANCEL:
						automode_parameters.hour = automode_parameters.tmp_hour;
						automode_parameters.minute = automode_parameters.tmp_minute;
						automode_parameters.second = automode_parameters.tmp_second;
						automode_parameters_buffer[0] = automode_parameters.tmp_hour;
						automode_parameters_buffer[1] = automode_parameters.tmp_minute;
						automode_parameters_buffer[2] = automode_parameters.tmp_second;
						WritePage_to_HMI(PAGE_MAIN);
						Write_Parameters_To_HMI(VALUE_AUTOMODE_TIME, 0);
						Write_Parameters_To_HMI(PARAM_INCH_PER_SECOND, hmi_parameters.inch_per_second);
						Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
						break;
					case EVENT_AUTO_SET:
						automode_parameters.tmp_hour = automode_parameters.hour;
						automode_parameters.tmp_minute = automode_parameters.minute;
						automode_parameters.tmp_second = automode_parameters.second;
						WritePage_to_HMI(PAGE_MAIN);
						Write_Parameters_To_HMI(VALUE_AUTOMODE_TIME, 0);
						Write_Parameters_To_HMI(PARAM_INCH_PER_SECOND, hmi_parameters.inch_per_second);
						Write_Parameters_To_HMI(PARAM_DEGREE, hmi_parameters.degree);
					
						if((int)automode_parameters.hour | (int)automode_parameters.minute | (int)automode_parameters.second)
						{
							main_loop_task = TASK_AUTO_MODE;
							Write_AutoModeTimeInterval_Background_to_HMI(GREEN_ACTIVE);
						}
						else
						{
							main_loop_task = TASK_IDLE;
							Write_AutoModeTimeInterval_Background_to_HMI(WHITE_DEACTIVE);
						}
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
	}
}
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
