/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68 << 1
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define TEMP_OUT_H_REG 0x41
#define GYRO_ZOUT_H_REG 0x47
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t gyro_raw = 0;

float gyro, gyroOffset, gyroMargin = 0;
uint16_t timer_val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;
    char buffer[16];

    //Check WHO_AM_I address
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    sprintf(buffer, "\n\rAddr: %d\n", check);
    HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
    if(check == 104)
    {
        //Write all 0's to wake sensor up
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        //Set data rate of 1KHz
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        //Set Gyroscopic configuration
        Data = 0xE0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
    }
    else
    {
    	//Reset and look for MPU6050 again
        HAL_NVIC_SystemReset();
        HAL_Delay(1200);
    }
}

void MPU6050_Read_Gyro(void)
{
    uint8_t Rec_Data[2];

    //Get raw gyro data
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 1000);
    gyro_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    gyro = ((gyro_raw / 94.4) - gyroOffset);
}

void MPU6050_Calibrate(void)
{
    char buffer[64];
    float count = 512;
    float gyroSum = 0;

    for(int i = 0; i < count; i++)
    {
        MPU6050_Read_Gyro();

        //To get avg values
        gyroSum += gyro;

        //To get max values
        if(fabs(gyro) > gyroMargin)
            gyroMargin = fabs(gyro);
    }
    //Avg values for offset
    gyroOffset = gyroSum / count;

    //Max value for idle margin of error
    gyroMargin = ((gyroMargin - gyroOffset) * 1.1);

    sprintf(buffer, "\rGyro: |%.3f| |%.3f|\n", gyroOffset, gyroMargin);
    HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
}

void checkAngle(int angle)
{
	char buffer[16];
	float value, sum = 0;

	while(1)
	{
		MPU6050_Read_Gyro();
		if(fabs(gyro) > gyroMargin)
			sum += gyro;

		value = sum * 0.02;
		sprintf(buffer, "\rAngle: %.2f\n", value);
		HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
		if(((angle > 0) && (value >= angle)) || ((angle < 0) && (value <= angle)))
			break;
		HAL_Delay(10);
	}
}

int getSensor(void)
{
	int infared = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	if(infared == 0)
		return 1;

	return 0;
}

void forwardControl(int time)
{
	char buffer[16];
	float value, sum = 0;
	uint16_t base_time = __HAL_TIM_GET_COUNTER(&htim16);
	timer_val = __HAL_TIM_GET_COUNTER(&htim16) - base_time;

	while(timer_val < time)
	{
		if(getSensor() == 0)
			return;

		MPU6050_Read_Gyro();
		if(fabs(gyro) > gyroMargin)
			sum += gyro;

		value = sum * 0.001;
		sprintf(buffer, "\rAngle: %.2f\n", value);
		HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
		if(value <= -1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	// Right motor 1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);	// Right motor 2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	// Left motor 1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);	// Left motor 2
		}
		else if(value >= 1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	// Right motor 1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);	// Right motor 2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	// Left motor 1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);	// Left motor 2
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	// Right motor 1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);	// Right motor 2
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	// Left motor 1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);	// Left motor 2
		}
		timer_val = __HAL_TIM_GET_COUNTER(&htim16) - base_time;
	}
	sprintf(buffer, "\r%d\n", timer_val);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
}

void stop(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	// Right motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);	// Right motor 2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	// Left motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);	// Left motor 2
}

void forward(int time)
{
	time = time * 500;
	forwardControl(time);
	stop();
	HAL_Delay(100);
}

void backward(int time)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	// Right motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);	// Right motor 2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	// Left motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);	// Left motor 2
	HAL_Delay(time);
	stop();
	HAL_Delay(100);
}

void left(int angle)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	// Right motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);	// Right motor 2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	// Left motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);	// Left motor 2
	checkAngle(angle);
	stop();
	HAL_Delay(100);
}

void right(int angle)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);	// Right motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);	// Right motor 2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);	// Left motor 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);	// Left motor 2
	checkAngle(-angle);
	stop();
	HAL_Delay(100);
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
  MX_I2C1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	stop();
	MPU6050_Init();
	HAL_Delay(1000);
  	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 0);
  	HAL_Delay(400);
    MPU6050_Calibrate();
    HAL_Delay(200);
    HAL_TIM_Base_Start(&htim16);

    forward(1);
    left(90);
    forward(5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//        while(1)
//        {
//
//        }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
