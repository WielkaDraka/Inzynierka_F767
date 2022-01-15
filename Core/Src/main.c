/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "UART.h"
#include "LSM6DS33.h"
#include "LIS3MDL.h"
#include "LPS25H.h"
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "Kalman_Filter.h"
#include "AlphaBeta.h"
//#include "alphabetagamma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern LSM_axis accel;
extern LSM_axis gyro;

extern kalman_axis gz;
extern kalman_axis gx;
extern kalman_axis gy;

extern LPS25H alititude;

extern sensor LIS3MDL;
//extern filter kalman;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define rozmiar_int16_t					32768
#define przyspieszenie_graw				9.80665
#define maximum_data_size				200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// acceleration
float az = 0, ax = 0, ay = 0;
//speed
float vz = 0, vx = 0, vy = 0;
//position
float xz = 0, xx = 0, xy = 0;
//angular_speed
float Gyro_Roll = 0, Gyro_Pitch = 0, Gyro_Yaw = 0;
// Roll, Pitch ,Yaw
float Roll = 0, Pitch = 0, Yaw = 0;
// Roll, Pitch, Yaw for Alpha, Beta filter
float AB_Roll = 0, AB_Pitch = 0, AB_Yaw = 0;
// time change
float dt = 0.1;
// absolute height
float AbsoluteHeight = 0;
// temperature
float Temperature = 0;
// Magnemetometer direction
float MdirectionX = 0;
float MdirectionY = 0;
float MdirectionZ = 0;

//data storage
volatile uint8_t data = 0;
float AccelerometerData[3][maximum_data_size];//X, Y, Z.
float GyroscopeData[3][maximum_data_size]; //Roll, Pitch, Yaw
float AltitudeMetersData[maximum_data_size];
float TemperatureData[maximum_data_size];
float MDirectionData[3][maximum_data_size];// X, Y, Z.

//Button
volatile uint8_t Button = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim10)
	{
		  lsm6ds33_readGyro();
		  lsm6ds33_readAccel();
		  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

}
void WHO_AM_I(void);
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
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  lsm6ds33_Init();
  lps25h_Init();
  lis3mdl_Init();

  HAL_TIM_Base_Start_IT(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	 filter_init();
	 set_filter(1.5, 2);
	 AlphaBeta_init();
	 //abg_filter_Init();
	 //set_filter_abg(0.85, 0.05, 0.1);
	 WHO_AM_I();



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 if((HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1 || data > 0) && !Button)
	 {
	  //*******************Gyro****************************//
	  gyro_Roll();
	  gyro_Pitch();

	  //For measure angle - Alpa-beta and no filter
	  //******************************************
	  //Gyro_Roll = (float)(gyro.x * 245) / rozmiar_int16_t;
	  //Roll = Roll + dt * Gyro_Roll;
	  Gyro_Yaw = (float)(gyro.z * 245) / rozmiar_int16_t;
	  Yaw = Yaw + dt * Gyro_Yaw;
	  //Gyro_Pitch = (float)(gyro.y * 245) / rozmiar_int16_t;
	  //Pitch = Pitch + dt * Gyro_Pitch;

	  //AB_Roll = AlphaBeta_gyro_X(Roll, 0.35, 0.1);
	  //AB_Pitch = AlphaBeta_gyro_Y(Pitch, 0.35, 0.1);
	  AB_Yaw = AlphaBeta_gyro_Z(Yaw, 0.35, 0.1);

	  //To save data
	  GyroscopeData[0][data] = gx.x_post[0];
	  GyroscopeData[1][data] = gy.x_post[0];
	  GyroscopeData[2][data] = AB_Yaw;
	  //printf("KALMAN:%0.04f, %0.04f, %0.04f\n",gx.x_post[0],gy.x_post[0], AB_Yaw);
      //printf("Brak filtra:%0.04f, %0.04f, %0.04f \n",AngleZ, AngleX, AngleY);
      //printf("AB         :%0.04f, %0.04f, %0.04f \n",value3, value1, value2);
	  //******************************************
	  //*******************Accelerometer****************************//

	  ax = (accel.x * 2 * przyspieszenie_graw)/rozmiar_int16_t;
	  ay = (accel.y * 2 * przyspieszenie_graw)/rozmiar_int16_t;
	  az = (accel.z * 2 * przyspieszenie_graw)/rozmiar_int16_t - przyspieszenie_graw;

	  //Position for x axis
	  xx = xx + vx * dt + ax * dt * dt / 2;
	  //Speed V fo x asis
	  vx = vx + ax * dt;


	  //Position for y axis
	  xy = xy + vy * dt + ay * dt * dt / 2;
	  //Speed V fo y asis
	  vy = vy + ay * dt;

	  //Assumption axis z is parallel to g
	  //Position for z axis
	  xz = xz + vz * dt + az * dt * dt / 2;
	  //Speed V fo z asis
	  vz = vz + az * dt;

	  //To save data
	  AccelerometerData[0][data] = xx;
	  AccelerometerData[1][data] = xy;
	  AccelerometerData[2][data] = xz;
	  //printf("Position, X, Y, Z: %0.04f, %0.04f, %0.04f\n",xx, xy, xz);
	  //*************************************************************//
	  //************************Barometer****************************//
	  lps25h_pressureToAlitudeMeters();
	  AbsoluteHeight = alititude.altitude_meters;
	  Temperature = alititude.temperature_C;

	  //To save data
	  AltitudeMetersData[data] = AbsoluteHeight;
	  TemperatureData[data] = Temperature;
	  //printf("Barometer: %0.04f, %0.04f\n",AbsoluteHeight, Temperature);
	  //*************************************************************//
	  //*******************Magnetometer******************************//
	  lis3mdl_readDirection();
	  MdirectionX = (float)(LIS3MDL.x * 4 * przyspieszenie_graw)/rozmiar_int16_t;
	  MdirectionY = (float)(LIS3MDL.y * 4 * przyspieszenie_graw)/rozmiar_int16_t;
	  MdirectionZ = (float)(LIS3MDL.z * 4 * przyspieszenie_graw)/rozmiar_int16_t;

	  //To save data
	  MDirectionData[0][data] = MdirectionX;
	  MDirectionData[1][data] = MdirectionY;
	  MDirectionData[2][data] = MdirectionZ;
	  //*************************************************************//
	  //printf("Magnetometer Position, X, Y, Z: %0.04f, %0.04f, %0.04f\n",MdirectionX, MdirectionY, MdirectionZ);
      //printf("%0.04f, %0.04f, %0.04f\n",calculate_X_accel(ax), calculate_X_accel(ay), calculate_X_accel(az));

	  data ++;
	  HAL_Delay(100);
	  if(data == 200)
		  Button = 1;
	 }
	 if(Button == 1 && HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
	 {
		 for(int i = 0; i < maximum_data_size; i++)
		 {
			 printf("GyroscopeData: %0.04f, %0.04f, %0.04f \n", GyroscopeData[0][i],GyroscopeData[1][i],GyroscopeData[2][i]);
		 }
		 for(int i = 0; i < maximum_data_size; i++)
		 {
			 printf("AccelerometerData: %0.04f, %0.04f, %0.04f \n", AccelerometerData[0][i],AccelerometerData[1][i],AccelerometerData[2][i]);
		 }
		 for(int i = 0; i < maximum_data_size; i++)
		 {
			 printf("AltitudeMetersData: %0.04f \n", AltitudeMetersData[i]);
		 }
		 for(int i = 0; i < maximum_data_size; i++)
		 {
			 printf("TemperatureData: %0.04f \n", TemperatureData[i]);
		 }
		 for(int i = 0; i < maximum_data_size; i++)
		 {
			 printf("MDirectionData: %0.04f, %0.04f, %0.04f \n", MDirectionData[0][i],MDirectionData[1][i],MDirectionData[2][i]);
		 }
		 Button = 0;
		 data = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x20404768;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 10799;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void WHO_AM_I(void)
{
    printf("Searching for accelerometer...\n");
  	uint8_t who_am_i = lsm6ds33_read_reg(LSM6DS33_WHO_AM_I);

  	 if (who_am_i == 0x69) {
  		 printf("Found: LSM6DS33\n");
  	 } else {
  		 printf("Error: (0x%02X)\n", who_am_i);
  	 }
  	 printf("Searching for barometer...\n");
  	 who_am_i = lps25h_read_reg(LPS25H_WHO_AM_I);

  	 if (who_am_i == 0xBD) {
  	  printf("Found: LPS25HB\n");
  	 } else {
  	  printf("Error: (0x%02X)\n", who_am_i);
  	 }
  	printf("Searching for magnetometer...\n");
  	who_am_i = lis3mdl_read_reg(LIS3MDL_WHO_AM_I);

  	 if (who_am_i == 0x3D) {
  	  printf("Found: LIS3MDL\n");
  	 } else {
  	  printf("Error: (0x%02X)\n", who_am_i);
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
