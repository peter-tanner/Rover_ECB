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
#include "stm32f1xx_hal_can.h"
#include <stdint.h>

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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
uint8_t initialized = 0;

uint32_t error_blink_interval = 200;
uint32_t normal_blink_interval = 4000;
uint32_t last_blink = 0;

bool arm_pressed = false;
uint32_t arm_pressed_tick = 0;

uint32_t telemetry_send_rate = 500;
uint32_t last_telemetry_tick = 0; // Added for non-blocking CAN

volatile bool system_tripped = false;

float Curr_val = 0.0f;
float Temp_val = 0.0f;

uint32_t dac_val = 2100; // Resulting in > 1.653V

TIM_HandleTypeDef *htim_can_disconnect = &htim8;
uint32_t tx_mailbox;
struct can_buffer_slot
{
  uint16_t message_priority;
  uint8_t size;
  uint8_t data[8];
};

lwrb_t can_tx_rb;
char can_tx_rb_data[sizeof(struct can_buffer_slot) * 32];
volatile size_t can_tx_rb_current_len;
uint8_t sleep_guard = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void Reset_Latch(void);
void Set_Dac(uint32_t val);
void Generate_Telemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t can_move_to_fifo()
{
  uint32_t primask;
  uint8_t started = 0;
  struct can_buffer_slot slot;

  primask = __get_PRIMASK();
  __disable_irq();
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0 && lwrb_read(&can_tx_rb, &slot, sizeof(slot)) > 0)
  {
    HAL_TIM_Base_Start_IT(htim_can_disconnect);
    __HAL_TIM_SET_COUNTER(htim_can_disconnect, 0);

    HAL_GPIO_WritePin(can_mode_GPIO_Port, can_mode_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(can_led_GPIO_Port, can_led_Pin, GPIO_PIN_SET);

    CAN_TxHeaderTypeDef can_classic_tx_header = {
        .StdId = 0x446,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = slot.size};

    uint8_t delay = 0;
    while (HAL_CAN_AddTxMessage(&hcan, &can_classic_tx_header, slot.data, &tx_mailbox) != HAL_OK)
    {
      HAL_Delay(1);
      delay++;
    }
    if (delay > 0)
    {
#ifdef UART_DBG
      char buf[32];
      snprintf(buf, sizeof(buf), "CAN_OVERFLOW %d\r\n", delay);
#endif
    }
    started = 1;
    __HAL_TIM_SET_COUNTER(htim_can_disconnect, 0);
  }

  __set_PRIMASK(primask);
  return started;
}

void can_transmit(void *data, uint32_t length, uint16_t id_priority__UNUSED)
{
  static struct can_buffer_slot slot_null;
  struct can_buffer_slot slot = {
      .size = length,
      .message_priority = id_priority__UNUSED,
      .data = {}};
  uint8_t fail = 9;

  memcpy(slot.data, data, length);

  /* Write data to transmit buffer */
  while (fail-- && lwrb_write(&can_tx_rb, &slot, sizeof(struct can_buffer_slot)) != sizeof(struct can_buffer_slot))
  {
    // char err_str[] = "can buffer overrun 0\r\n";
    // err_str[sizeof(err_str) - 4] = '0' + fail;
    lwrb_read(&can_tx_rb, &slot_null, sizeof(slot_null));
  }
  can_move_to_fifo();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  lwrb_init(&can_tx_rb, can_tx_rb_data, sizeof(can_tx_rb_data));
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
  MX_CAN_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1); // Start the DAC
  Set_Dac(dac_val);

  // CONFIGURE CAN
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    Error_Handler();
  if (HAL_CAN_Start(&hcan) != HAL_OK)
    Error_Handler();
  HAL_GPIO_WritePin(can_mode_GPIO_Port, can_mode_Pin, GPIO_PIN_SET);
  // END CONFIGURE CAN

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
  {
    system_tripped = true;
  }

  initialized = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (system_tripped && (HAL_GetTick() - last_blink >= error_blink_interval))
    { //blink fault led rapidly if system is tripped
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
      last_blink = HAL_GetTick();
    }
    else if (system_tripped == false && (HAL_GetTick() - last_blink >= normal_blink_interval))
    {                                                        // flash led very briefly if system is not tripped
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //turn led on
      last_blink = HAL_GetTick();

      HAL_TIM_Base_Stop_IT(&htim6);
      __HAL_TIM_SET_AUTORELOAD(&htim6, 50 - 1); // configure timer 6 to trigger interrupt in 50 ms
      __HAL_TIM_SET_COUNTER(&htim6, 0);
      __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
      HAL_TIM_Base_Start_IT(&htim6); // start timer
    }

    //check if the arm pin is being pressed
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET && arm_pressed == false)
    {
      arm_pressed = true;
      arm_pressed_tick = HAL_GetTick();
    }
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
      arm_pressed = false;
    }

    if (system_tripped && arm_pressed && (HAL_GetTick() - arm_pressed_tick >= 1500))
    { // reset the latch if the system is tripped and the arm pin has been held for longer than 1.5s
      system_tripped = false;
      Reset_Latch();
      for (uint32_t i = 0; i < 5; i++)
      { //blink led quicklu a few times
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
        HAL_Delay(100);
      }
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //turn led off
    }

    if (HAL_GetTick() - last_telemetry_tick >= telemetry_send_rate)
    {
      Generate_Telemetry(); // Reads ADC and I2C

      // Prepare CAN Data: [Current, Temp, Trip_Status]
      uint8_t can_data[3];
      can_data[0] = (uint8_t)(Curr_val + 100); // Offset for negative range
      can_data[1] = (uint8_t)Temp_val;
      can_data[2] = (uint8_t)system_tripped;

      can_transmit(can_data, sizeof(can_data), 0);

      last_telemetry_tick = HAL_GetTick();
    }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = CAN_TIMEOUT_PSC_DIV;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = CAN_TIMEOUT_ms;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, latch_reset_Pin | can_led_Pin | can_mode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(fault_led_GPIO_Port, fault_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC1
                           PC2 PC3 PC4 PC5
                           PC6 PC7 PC8 PC9
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 PA5 PA6
                           PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : latch_reset_Pin */
  GPIO_InitStruct.Pin = latch_reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(latch_reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : latch_output_Pin */
  GPIO_InitStruct.Pin = latch_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(latch_output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB12 PB13 PB14 PB3
                           PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : arm_pin_Pin */
  GPIO_InitStruct.Pin = arm_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(arm_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : fault_led_Pin */
  GPIO_InitStruct.Pin = fault_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(fault_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : can_led_Pin can_mode_Pin */
  GPIO_InitStruct.Pin = can_led_Pin | can_mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void try_can_move(CAN_HandleTypeDef *hcan_it)
{
  if (hcan_it == &hcan)
  {
    if (can_move_to_fifo() == 0 && lwrb_get_full(&can_tx_rb) == 0)
    {
      __HAL_TIM_SET_COUNTER(htim_can_disconnect, 0);
      HAL_TIM_Base_Stop_IT(htim_can_disconnect);
      HAL_GPIO_WritePin(can_led_GPIO_Port, can_led_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(can_mode_GPIO_Port, can_mode_Pin, GPIO_PIN_SET);
    }
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  try_can_move(hcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  try_can_move(hcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  try_can_move(hcan);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    //stop the timer so it doesn't repeat
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // turn off led
  }
  else if (htim->Instance == TIM7)
  {
    //stop the timer so it doesn't repeat
    HAL_TIM_Base_Stop_IT(&htim7);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // turn of reset pin
  }
  else if (htim == htim_can_disconnect && initialized)
  {
    // sfx_twotone(5, 50, 10);
    HAL_TIM_Base_Stop_IT(htim_can_disconnect);

    // empty stale
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    lwrb_reset(&can_tx_rb);

    // enter shutdown and standby
    HAL_GPIO_WritePin(can_led_GPIO_Port, can_led_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(can_mode_GPIO_Port, can_mode_Pin, GPIO_PIN_SET);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2 && system_tripped == false) //if the latch gets tripped
  {
    system_tripped = true;
  }
}

void Reset_Latch(void)
{
  //generate a 100ms pulse on the reset pin
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  HAL_TIM_Base_Stop_IT(&htim7);
  __HAL_TIM_SET_AUTORELOAD(&htim7, 100 - 1); // configure timer 6 to trigger interrupt in 100 ms
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim7); // start timer
}

void Set_Dac(uint32_t val)
{
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
}

void Generate_Telemetry(void)
{
  // 1. Current Sense (ADC PC0)
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    float voltage = (raw * 3.3f) / 4095.0f;
    // Sensitivity: 16.5mV/A, Offset: 1.65V
    Curr_val = (voltage - 1.65f) / 0.0165f;
  }
  HAL_ADC_Stop(&hadc1);

  // 2. Temperature Sense (I2C TMP112)
  uint8_t i2c_buf[2];
  // Address 0x48 (7-bit) shifted for HAL
  if (HAL_I2C_Master_Receive(&hi2c1, (0x48 << 1), i2c_buf, 2, 50) == HAL_OK)
  {
    int16_t raw_temp = (i2c_buf[0] << 4) | (i2c_buf[1] >> 4);
    Temp_val = raw_temp * 0.0625f;
  }
}

#if 0
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2 && system_tripped == false) //if the latch gets tripped
    {
        system_tripped = true;
    }

}

void Reset_Latch(void){
	//generate a 100ms pulse on the reset pin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

	  HAL_TIM_Base_Stop_IT(&htim7);
	  __HAL_TIM_SET_AUTORELOAD(&htim7, 100 - 1); // configure timer 6 to trigger interrupt in 100 ms
	  __HAL_TIM_SET_COUNTER(&htim7, 0);
	  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
	  HAL_TIM_Base_Start_IT(&htim7); // start timer
}

void Set_Dac(uint32_t val){
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
}
#endif

#if 0
void Generate_Telemetry(void){

	uint32_t adc_value = 0;

	HAL_ADC_Start(&hadc1);                          // Start conversion
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait until done
	adc_value = HAL_ADC_GetValue(&hadc1);           // Read result (0-4095)
	HAL_ADC_Stop(&hadc1);

#define VREF 3.3f
#define ADC_TO_VOLTAGE(raw) ((float)(raw) / 4095.0f * VREF)

	uint32_t raw = HAL_ADC_GetValue(&hadc1);
	float voltage = ADC_TO_VOLTAGE(raw);

}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __BKPT();
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
