/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "nfc_conf.h"
#include "rfal_nfca.h"
#include "rfal_platform.h"
#include "st25r200_irq.h"
#include "rfal_nfc.h"
#include "rfal_t2t.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 100% duty cycle
#define MAX_DUTYCYCLE 1000
// Maximum speed, compared to MAX_DUTYCYCLE
#define MAX_SPEED 500
// how big jumps should be while ramping up and down (divisor or MAX_DUTYCYCLE and MAX_SPEED)
#define JUMP_SIZE 50 //unused
// Delay in ms used by speedup and slowdown functions
#define DELAY 250 //unused
#define RUNS_BEFORE_CHARGING 3

// Period of train on track (period / 10000 Hz = period in seconds)
#define PERIOD 100000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t globalCommProtectCnt = 0;
const char * str = "RFID Device Located\r\n";
const char * stop1 = "Stop 1\r\n";
const char * stop2 = "Stop 2\r\n";

static rfalNfcDiscoverParam discParam;
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

enum State {
	INITIAL,
	RUNNING,
	CHARGING
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief      SPI Read and Write byte(s) to device
  * @param[in]  pTxData : Pointer to data buffer to write
  * @param[out] pRxData : Pointer to data buffer for read data
  * @param[in]  Length : number of bytes to write
  * @return     BSP status
  */
int32_t BSP_NFC0XCOMM_SendRecv(const uint8_t * const pTxData, uint8_t * const pRxData, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  int32_t ret = BSP_ERROR_NONE;

  if((pTxData != NULL) && (pRxData != NULL))
  {
    status = HAL_SPI_TransmitReceive(&COMM_HANDLE, (uint8_t *)pTxData, (uint8_t *)pRxData, Length, 2000);
  }
  else if ((pTxData != NULL) && (pRxData == NULL))
  {
    status = HAL_SPI_Transmit(&COMM_HANDLE, (uint8_t *)pTxData, Length, 2000);
  }
  else if ((pTxData == NULL) && (pRxData != NULL))
  {
    status = HAL_SPI_Receive(&COMM_HANDLE, (uint8_t *)pRxData, Length, 2000);
  }
  else
  {
  	ret = BSP_ERROR_WRONG_PARAM;
  }

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    ret = BSP_NFC0XCOMM_Init();
  }

  return ret;
}

__weak void BSP_NFC0XCOMM_IRQ_Callback(void)
{
  /* Prevent unused argument(s) compilation warning */

  /* This function should be implemented by the user application.
   * It is called into this driver when an event from ST25R200 is triggered.
   */
  st25r200Isr();
}

// Check if ID matches the stations ID
bool IDMatch(unsigned char * ID, unsigned char * station, uint8_t length) {
	int idx = 0;
	while (idx < length) {
		if (ID[idx] != station[idx]) {
			return false;
		}
		idx++;
	}
	return true;
}

// Speed up the motor using PWM, starting from 0 and increasing until the DutyCycle is reached
void SpeedUp(){
	/*
	int speed = 0;
	while (speed <= MAX_SPEED)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
		HAL_Delay(DELAY);
		speed += JUMP_SIZE;
	}
	*/
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_SPEED);
}

// Slow down the motor using PWM, starting from DutyCycle (Max speed define) and decreasing until stopped
void SlowDown()
{
	/*
	int speed = MAX_SPEED;
	while (speed >= 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
		HAL_Delay(DELAY);
		speed -= JUMP_SIZE;
	}
	*/
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  rfalNfcDevice * nfcDevice;
  uint8_t stop1A[7] = {0x04, 0x52, 0x80, 0x5A, 0x79, 0x4C, 0x80}, stop1B[7] = {0x04, 0x6A, 0x80, 0x5A, 0x79, 0x4C, 0x80}, stop1C[7] = {0x04, 0x49, 0x7E, 0x5A, 0x79, 0x4C, 0x80};
  uint8_t stop2A[7] = {0x04, 0x11, 0x63, 0xBA, 0xF0, 0x4A, 0x81}, stop2B[7] = {0x04, 0x29, 0x63, 0xBA, 0xF0, 0x4A, 0x81}, stop2C[7] = {0x04, 0x28, 0x63, 0xBA, 0xF0, 0x4A, 0x81};

  int runs = 0;
  enum State state = INITIAL;
  uint8_t nextStop = 1;
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //Initialize NFC Communicaiton
  BSP_NFC0XCOMM_Init();
  USR_INT_LINE.Line = USR_INT_LINE_NUM;
  USR_INT_LINE.PendingCallback = st25r200Isr;
  (void)HAL_EXTI_GetHandle(&USR_INT_LINE, USR_INT_LINE.Line);
  (void)HAL_EXTI_RegisterCallback(&USR_INT_LINE, HAL_EXTI_COMMON_CB_ID, BSP_NFC0XCOMM_IRQ_Callback);
  rfalNfcInitialize();

  rfalNfcDefaultDiscParams(&discParam);
  discParam.devLimit          = 1U;                  // Limit to 1 device
  discParam.totalDuration     = 1000U;               // Discovery duration
  discParam.techs2Find        |= RFAL_NFC_POLL_TECH_A; // Poll for ISO14443A only
  discParam.techs2Bail		  |= RFAL_NFC_POLL_TECH_A;
  discParam.p2pNfcaPrio   	  = true;
  memcpy( &discParam.nfcid3, NFCID3, sizeof(NFCID3) );
  memcpy( &discParam.GB, GB, sizeof(GB) );
  discParam.GBLen         = sizeof(GB);
  rfalNfcDiscover(&discParam);

  // Set direction to forward
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  // Start PWM Timer
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // Initial pulse for momentum
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_SPEED);
	  /*
  HAL_Delay(100);
  // Set to 10% duty cycle
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_DUTYCYCLE/10);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  */
  // backup timer in case the RFID tag is missed
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  rfalNfcWorker();
	  /* Check if tag is detected */
	  if( rfalNfcIsDevActivated( rfalNfcGetState()))
	  {
		  rfalNfcGetActiveDevice(&nfcDevice);
		  if (nfcDevice->type == RFAL_NFC_LISTEN_TYPE_NFCA)
		  {
			  /* Handle the tag here */
			  if (nextStop == 1)
			  {
				  // Stop 1 (near charger)
				  if (IDMatch(nfcDevice->nfcid, stop1A, nfcDevice->nfcidLen) || IDMatch(nfcDevice->nfcid, stop1B, nfcDevice->nfcidLen) || IDMatch(nfcDevice->nfcid, stop1C, nfcDevice->nfcidLen))
				  {
					  if (state == INITIAL)
					  {
						  SpeedUp();
						  state = RUNNING;
					  }
					  else
					  {
						  SlowDown();
						  runs += 1;
						  if (runs >= RUNS_BEFORE_CHARGING)
						  {
							  // Check for charging tag and turn off motor
							  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);
							  state = CHARGING;
							  HAL_Delay(1000);
							  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
							  nextStop = 1;
							  // TODO: implement charger code (wait then start again)
						  }
						  else
						  {
							  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
							  SpeedUp();
						  }
					  }
					  HAL_TIM_Base_Stop(&htim2);
					  __HAL_TIM_SET_COUNTER(&htim2, 0);
					  HAL_TIM_Base_Start(&htim2);
					  nextStop = 2;
				  }
			  }
			  else if (nextStop == 2)
			  {
				  // Stop 2 (far side)
				  if (IDMatch(nfcDevice->nfcid, stop2A, nfcDevice->nfcidLen) || IDMatch(nfcDevice->nfcid, stop2B, nfcDevice->nfcidLen) || IDMatch(nfcDevice->nfcid, stop2C, nfcDevice->nfcidLen))
				  {
					  SlowDown();
					  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
					  SpeedUp();
					  HAL_TIM_Base_Stop(&htim2);
					  __HAL_TIM_SET_COUNTER(&htim2, 0);
					  HAL_TIM_Base_Start(&htim2);
					  nextStop = 1;
				  }
			  }
			  rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
		  }

	  }
	  // If the train runs in one direction for too long
	  else if (__HAL_TIM_GET_COUNTER(&htim2) > PERIOD) {
		  SlowDown();
		  if (nextStop == 1) {
			  nextStop = 2;
			  runs += 1;
			  if (runs >= RUNS_BEFORE_CHARGING)
			  {
				  // turn off motor, wait for reset
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);
				  state = CHARGING;
				  HAL_Delay(1000);
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				  nextStop = 1;
			  }
		  }
		  else {
			  nextStop = 1;
		  }
		  if (state != CHARGING) {
			  HAL_GPIO_TogglePin(DIR_GPIO_Port, DIR_Pin);
			  SpeedUp();
			  HAL_TIM_Base_Stop(&htim2);
			  __HAL_TIM_SET_COUNTER(&htim2, 0);
			  HAL_TIM_Base_Start(&htim2);
		  }
	  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 100;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(GPIOA, LED_1_Pin|LED_2_Pin|LED_4_Pin|RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_3_Pin|LED_5_Pin|LED_6_Pin|NFC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NFC_INT_Pin */
  GPIO_InitStruct.Pin = NFC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NFC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_4_Pin RESET_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_4_Pin|RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_5_Pin LED_6_Pin NFC_CS_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_5_Pin|LED_6_Pin|NFC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
