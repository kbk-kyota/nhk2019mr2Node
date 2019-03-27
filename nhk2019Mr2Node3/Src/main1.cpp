/*
 * main1.cpp
 *
 *  Created on: 2019/03/27
 *      Author: kyota kuboki
 */
#include <main1.h>
#include <array>
#include "SolenoidNode.hpp"
#include "CarrierNode.hpp"
#include "can.hpp"
#include "led.h"
#include "solenoid_driver.h"
#include "stm32f1xx_hal.h"

CAN_HandleTypeDef hcan;

#define CAN_MTU 8
TIM_HandleTypeDef htim2;
uint16_t now_pattern = 0b00000000;
uint8_t once = 0;
uint8_t order;
uint8_t endis = 0;
uint8_t sensor_flag = 0;
uint8_t switch_flag = 0;

template<typename T>
static void can_unpack(const uint8_t (&buf)[CAN_MTU], T &data);
template<typename T>
static void can_pack(uint8_t (&buf)[CAN_MTU], const T data);

static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void SystemClock_Config(void);
static void _Error_Handler(void);
static void MX_TIM2_Init(void);

static SolenoidNode * const solenoidNode = new SolenoidNode();

CAN_RxHeaderTypeDef rx_msg;
CAN_TxHeaderTypeDef tx_msg1;
CAN_TxHeaderTypeDef tx_msg2;

uint32_t status;
uint8_t tx_payload1[CAN_MTU];
uint8_t tx_payload2[CAN_MTU];
uint8_t timetosend = 0;
static constexpr uint16_t id_handCmd = 0x102;

uint8_t cmd[CAN_MTU];
int main1(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);
  SystemClock_Config();

  can_init();
  can_set_bitrate(CAN_BITRATE_500K);

  // turn on board_LED

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);

  // blink red LED for test
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);


  can_enable();

  // loop forever

  tx_msg1.StdId = 0x100;//sensor
  tx_msg1.RTR = CAN_RTR_DATA;
  tx_msg1.IDE = CAN_ID_STD;
  tx_msg1.DLC = 1;
  tx_msg2.StdId = 0x101;//limit switch
  tx_msg2.RTR = CAN_RTR_DATA;
  tx_msg2.IDE = CAN_ID_STD;
  tx_msg2.DLC = 1;

  HAL_NVIC_EnableIRQ (TIM2_IRQn);
  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
	  now_pattern = 0b00000000;
	  solenoid_drive(now_pattern);
	  status = can_rx(&rx_msg, cmd);
	  if (status == HAL_OK)
	  {
		  // received can frame
		  if (rx_msg.StdId == id_handCmd)
		  {
			  now_pattern |= 0b00000001;
			  solenoid_drive(now_pattern);
			  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
		  }
	  }

	  if (((timetosend == 1) && (sensor_flag != 0)) || ((timetosend == 1) && (switch_flag != 0)))
	  {
		  can_tx(&tx_msg1, tx_payload1);
		  can_tx(&tx_msg2, tx_payload2);
		  sensor_flag = 0;
		  switch_flag = 0;
		  timetosend = 0;
	  }


  if (endis != 1)
  {
		break;
	}
  }
}

extern "C" void TIM2_IRQHandler(void)
		{
	if (TIM2->SR & TIM_SR_UIF) {
		uint8_t sensor1_status;
		uint8_t sensor2_status;
		if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) != GPIO_PIN_RESET) || (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8) != GPIO_PIN_RESET)) {
			sensor1_status = 0x01;
			sensor_flag = 1;
			can_pack(tx_payload1,sensor1_status);
		}

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET) {
			sensor2_status = 0x00;
			switch_flag = 1;
			can_pack(tx_payload2,sensor2_status);
		}

		timetosend = 1;

		TIM2->SR &= ~TIM_SR_UIF;
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_1|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */





