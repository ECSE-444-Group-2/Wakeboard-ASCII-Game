/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_qspi.h"
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac1);
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct {
	float x;
	float y;
} typedef Pos;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_LENGTH_X 			60
#define DISPLAY_LENGTH_Y 			20

// Scale the gyro sensor angle to movement of the player
#define GYRO_TO_DISP_FACTOR 	0.5

// Smooth gyro signal with leaky integrator
#define LEAKY_LAMBDA 0.95
#define LEAKY_LAMBDA_COMP 0.05

// Take a number of cycles to calibrate the gyro sensor
#define CALIBRATION_CYCLES 100

// Run the game loop (including DSP) at a certain rate (in seconds)
#define GAME_LOOP_PERIOD 0.1

// Player is always at the same Y position
#define PLAYER_Y							(DISPLAY_LENGTH_Y - 1)

// Player cannot go into the last column
#define PLAYER_MAX_X					DISPLAY_LENGTH_X - 2

// ASCII symbol for the player
#define PLAYER_CHAR						'^'

// The position of the obstacle is determined by an index in the 2D
// display array. Each object will have a width (and potentially
// height) associated to them. This parameter specifies how many
// extra indexes the object takes up on either side. For example,
// if the obstacle is 5 indexes wide, OBSTACLE_EXTRA_WIDTH will be
// 2.
#define OBSTACLE_EXTRA_WIDTH	5

// How many indexes the obstacle will move each game loop. Could
// become a variable if we want to increase the difficulty?
#define OBSTACLE_SPEED				0.5

// ASCII symbol for the obstacles
#define OBSTACLE_CHAR					'='

// ITM Port define
#define ITM_Port32(n)					(*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// NOTE: Display is [y][x], NOT [x][y]
uint8_t display[DISPLAY_LENGTH_Y][DISPLAY_LENGTH_X];

// Gyro stuff
float gyroData[3];
uint8_t leakyGyroFlag = 0;
float leakyGyro = 0.0;
float angularDisplacement = 0;
uint8_t calibrationCount = 0;
float calibrationAvg = 0.0;

// Array of obstacles and counter for number of obstacles on-screen
// Array will be sorted from youngest to oldest obstacle
Pos *obstacle;

// Posistion of the player
float playerX = DISPLAY_LENGTH_X >> 1;
// characterPosY is constant, so it's defined as a macro

// Player position

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_QUADSPI_Init(void);
/* USER CODE BEGIN PFP */
void gameOver();
uint8_t collision(Pos *obstacle, int8_t newX, int8_t extraWidth);
void setObstacle(Pos *o, uint8_t newChar, int8_t extraWidth);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int numOfSamples = 18000;
uint8_t sineArray[18000];
int sineWaveNumber = 0;
int nextFreq = 1; // trigger to go to next frequency
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
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */
  // Initialize peripherals
  // Gyro sensor
  BSP_GYRO_Init();
  BSP_GYRO_GetXYZ(gyroData);

  // Set up display
  HAL_UART_Transmit(&huart1, (uint8_t *)"\033[2J", 4, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\n\n", 4, 100);
  uint8_t i;
  uint8_t j;
  for(i = 0; i < DISPLAY_LENGTH_Y; i++){
		for(j = 0; j < DISPLAY_LENGTH_X; j++){
			if (j == DISPLAY_LENGTH_X - 1)
				display[i][j] = '\n';
			else
				display[i][j] = ' ';
		}
	}

  // Initialize obstacle
  obstacle = (Pos *) malloc(sizeof(Pos));
  obstacle->x = -1.0;
  obstacle->y = -1.0;

  // Timers
  HAL_TIM_Base_Start_IT(&htim3);

  BSP_QSPI_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // fetch gyro data
	  // if leaky integrator signal is freshly initialized, begin
	  // otherwise, continue
	  BSP_GYRO_GetXYZ(gyroData);


	  // complete calibration period
	  if (calibrationCount < CALIBRATION_CYCLES) {
		  calibrationAvg += gyroData[0] / CALIBRATION_CYCLES;
		  calibrationCount++;
	  }
	  HAL_Delay(10);

	  if (nextFreq == 1) {
		  // stop the DAC
		  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

		  // start over from the first frequency if we reached the end (8th frequency)
		  sineWaveNumber = sineWaveNumber % 8;

		  // Do block erase so that we have space to store data for next frequency
		  BSP_QSPI_Erase_Block(0);

		  for (int i = 0; i < numOfSamples; i++) {
			  // divide by different values to get different frequencies
			  float32_t freq = (float32_t)i / (24.0 - sineWaveNumber * 2);
			  float32_t sineData = arm_sin_f32(M_PI * freq);
			  // increase the amplitude of the data
			  float32_t amplifiedData = (sineData) * 415;
			  sineArray[i] = (uint8_t) amplifiedData;
		  }

		  // Writes the data to the QSPI memory
		  BSP_QSPI_Write(sineArray, (uint32_t)0, (uint32_t)numOfSamples);

		  // Reads the data from the QSPI memory
		  BSP_QSPI_Read(sineArray, (uint32_t)0, (uint32_t)numOfSamples);

		  // start DAC in DMA mode
		  // DMA will read our array of sine values and write to the DAC for us
		  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sineArray, (uint32_t)numOfSamples, DAC_ALIGN_8B_R);

		  // switch to the next frequency the next time we come in
		  sineWaveNumber++;

		  // reset the trigger
		  nextFreq = 0;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  htim2.Init.Period = 4000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 40000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void gameOver()
{
	//TODO: Go to game over screen or something
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_UART_Transmit(&huart1, (uint8_t *)"\033[2J", 4, 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)"\033[f", 9, 100);
	HAL_UART_Transmit(&huart1, (uint8_t *)"You lose :(", 11, 100);
}

uint8_t collision(Pos *obstacle, int8_t newX, int8_t extraWidth)
{
	int8_t i;

	// If object is not on the last row, no need to check X dimension
	if ((uint8_t)obstacle->y == PLAYER_Y) {
		// Check if character intercepts with width of obstacle
		for(i = -extraWidth; i <= extraWidth; i++) {
			if ((int8_t)obstacle->x + i == newX)
				return 1;
		}
	}
	return 0;
}

void setObstacle(Pos *o, uint8_t newChar, int8_t extraWidth)
{
	int8_t i;
	for(i = -extraWidth; i <= extraWidth; i++)
		display[(uint8_t)o->y][(int8_t)o->x + i] = newChar;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac1)
{
	// set this trigger to 1 so we can change the frequency
	nextFreq = 1;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3 && calibrationCount >= CALIBRATION_CYCLES) {
		//ITM_Port32(31) = 1;
		// Local copy of gyro sensor data
		//float gyroData;
		// it doesn't seem to like handing control back to main, so just poll here
		BSP_GYRO_GetXYZ(gyroData);

		// begin leaky integrator after calibration period
		if (!leakyGyroFlag) {
			leakyGyro = calibrationAvg;
			leakyGyroFlag = 1;
		} else {
			leakyGyro = (LEAKY_LAMBDA * leakyGyro) + (LEAKY_LAMBDA_COMP * gyroData[0]);
		}
		//leakyGyro = (LEAKY_LAMBDA * leakyGyro) + (LEAKY_LAMBDA_COMP * gyroData[0]);

		angularDisplacement += (leakyGyro - calibrationAvg) * GAME_LOOP_PERIOD;
		if (angularDisplacement > 90.0) {
			angularDisplacement = 90.0;
		} else if (angularDisplacement < -90.0) {
			angularDisplacement = -90.0;
		}

		// Erase current player position
		display[PLAYER_Y][(uint8_t)playerX] = ' ';

		// Calculate new player position
		//gyroData = angularDisplacement;
		playerX += (angularDisplacement * GYRO_TO_DISP_FACTOR);
		if (playerX < 0)
			playerX = 0;
		if (playerX > PLAYER_MAX_X + 0.9)
			playerX = PLAYER_MAX_X;

		// Calculate new obstacle position and check for collisions
		if (obstacle->x < 0) {
			obstacle->x = rand() % (PLAYER_MAX_X - OBSTACLE_EXTRA_WIDTH);
			if (obstacle->x < OBSTACLE_EXTRA_WIDTH)
				obstacle->x = OBSTACLE_EXTRA_WIDTH;
			obstacle->y = 0;
		}
		else {
			setObstacle(obstacle, (uint8_t)' ', OBSTACLE_EXTRA_WIDTH);
			obstacle->y += OBSTACLE_SPEED;
			// Check if obstacle has left the display
			if (obstacle->y > DISPLAY_LENGTH_Y - 0.1){
				obstacle->x = -1.0;
				obstacle->y = -1.0;
			}
			// Check for collision
			else if (collision(obstacle, (int8_t)playerX, OBSTACLE_EXTRA_WIDTH)){
				gameOver();
				return;
			}
		}
		//clearBuf(buffer, 50);
		//sprintf(buffer, "Gyroscope: x = %d, y = %d, z = %d", (int)gyroData[0], (int)gyroData[1], (int)gyroData[2]);
		// These two magic strings clear the first line and set the cursor back to the top left corner
		//HAL_UART_Transmit(&huart1, (uint8_t *)"\033[2J", 7, 100);
//		display[playerY][playerX] = ' ';
//		playerX++;
//		if (playerX == DISPLAY_MAX_X - 1){
//			playerX = 0;
//			playerY++;
//			if (playerY == DISPLAY_MAX_Y)
//				playerY = 0;
//		}
		// Move player to new location
		display[PLAYER_Y][(uint8_t)playerX] = PLAYER_CHAR;

		// Update obstacle location
		if (obstacle->x > -0.5)
			setObstacle(obstacle, OBSTACLE_CHAR, OBSTACLE_EXTRA_WIDTH);
		HAL_UART_Transmit(&huart1, (uint8_t *)"\033[f", 9, 100);

		// Print the buffer to UART
		for (uint8_t i = 0; i < DISPLAY_LENGTH_Y; i++){
			HAL_UART_Transmit(&huart1, display[i], DISPLAY_LENGTH_X, 100);
		}
		//ITM_Port32(31) = 2;
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
