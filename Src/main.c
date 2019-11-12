/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME_MSG "Welcome to the Nucleo management console\r\n"
#define MAIN_MENU   "Select the option you are interested in:\r\n\t1. Set n_pulse\r\n\t2. Print n_pulse\r\n\t3. Clear screen and print this message "
#define PROMPT "\r\n> "
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const int32_t L = 160 + 5 + 5; // in 170 mm
const int32_t R = 40; // 40 mm
uint32_t input_capture_prev1 = 0;
uint32_t input_capture1 = 0;
uint32_t n_pulse1 = 0;
uint32_t n_pulse_set1 = 0;

uint32_t input_capture_prev2 = 0;
uint32_t input_capture2=0;
uint32_t n_pulse2 = 0;
uint32_t n_pulse_set2 = 0;

int32_t wheel_speed1 = 0;
int32_t wheel_speed2 = 0;

uint32_t wheel_dir1 = 1; // 1 means CW, -1 means CCW
uint32_t wheel_dir2 = 1;

int32_t error1 = 0;
int32_t integrated_error1 = 0;
int32_t error2 = 0;
int32_t integrated_error2 = 0;

uint32_t returned_pwm1 = 0;
uint32_t returned_pwm2 = 0;

uint32_t i = 0;
uint32_t give_command = 0;

struct rover_position
{
	int32_t x;
	int32_t y;
	int32_t phi; // Radian

	int32_t trg_x;
	int32_t trg_y;

	int32_t trg_wheel_spd1;
	int32_t trg_wheel_spd2;

	int32_t angular_vel;
	int32_t linear_vel;
};

struct rover_position my_rover = {0,0,157,0,0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void printWelcomeMessage(void);
uint32_t processUserInput(uint32_t opt);
uint32_t readUserInput(void);
uint32_t pid_controller_wheels(uint32_t current_point, uint32_t desired_point, uint32_t wheel_id);
void calculate_new_speed(void);
void update_my_position(void);
void heading_pid_controller(int32_t error_heading);
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
	uint32_t opt;
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

  HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1);

  // Set motor to be 0 speed
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
  printMessage:

    	printWelcomeMessage();

    	while (1)  {
    		opt = readUserInput();
    		processUserInput(opt);

    		if(opt == 3)
    			goto printMessage;
    	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
	// TIMER2
		// Frequency 1 KHz
		//
  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
	// TIMER 3
		// Frequency: 25 kHz
		// Resolution of voltage output: 3.3/1000
  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 400-1;
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
  sConfigOC.Pulse = 399;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
    // 10 Hz
  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//	uint32_t pw_array_length = PW_ARRAY_LENGTH;
	if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) // measure the speed of wheel 1
	{
		input_capture1++;
	}

	else if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)) // measure the speed of wheel 2
	{
		input_capture2++;
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) && give_command == 1)
	{
		HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

		n_pulse1 = input_capture1 - input_capture_prev1;
		input_capture_prev1 = input_capture1;

		n_pulse2 = input_capture2 - input_capture_prev2;
		input_capture_prev2 = input_capture2;

		 update_my_position();
		 calculate_new_speed();

		returned_pwm1 = pid_controller_wheels(n_pulse1, my_rover.trg_wheel_spd1, 1);
		returned_pwm2 = pid_controller_wheels(n_pulse2, my_rover.trg_wheel_spd2, 2);

		if (wheel_dir1 == 1) // Pin C7
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, returned_pwm1);

		if (wheel_dir2 == 1) // Pin A9
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, returned_pwm2);
	}
	else if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) && give_command == 1)
	{
		n_pulse1 = 0;
		n_pulse2 = 0;
	}
}

void printWelcomeMessage(void) {
	HAL_UART_Transmit(&huart2, (uint8_t*)"\033[0;0H", strlen("\033[0;0H"), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\033[2J", strlen("\033[2J"), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)WELCOME_MSG, strlen(WELCOME_MSG), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)MAIN_MENU, strlen(MAIN_MENU), HAL_MAX_DELAY);
}

uint32_t readUserInput(void) {
	char readBuf[1];

	HAL_UART_Transmit(&huart2, (uint8_t*)PROMPT, strlen(PROMPT), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart2, (uint8_t*)readBuf, 1, HAL_MAX_DELAY);
	return atoi(readBuf);
}


uint32_t processUserInput(uint32_t opt) {
	char msg0[100] = "\r\nYour desired speed is: ";
	char msg1[100];
	char msg2[100];
	char msg3[100];
	char msg4[100];
	char msg5[100];
	char msg6[100];
	char msg7[100];
	char msg8[100];
	char msg9[100];
	char msg10[100];
	char msg11[100];
	char readN_PULSE[3];

	if(!opt || opt > 3)
		return 0;

	sprintf(msg1, "%ld", opt);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg1, strlen(msg1), HAL_MAX_DELAY);

	switch(opt) {
	case 1: // Set n_pulse

		HAL_UART_Transmit(&huart2, (uint8_t*)msg0, strlen(msg0), HAL_MAX_DELAY);
		HAL_UART_Receive(&huart2, (uint8_t*)readN_PULSE, 3, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*)readN_PULSE, 3, HAL_MAX_DELAY);

		my_rover.trg_x = atoi(readN_PULSE)*100;
		my_rover.trg_y = atoi(readN_PULSE)*100;

		// n_pulse_received1 = atoi(readN_PULSE);
		// if (abs(n_pulse_received1) > 30)
		// {
		// 	n_pulse_set1 = 30;
		// 	wheel_dir1 = n_pulse_received1/abs(n_pulse_received1);
		// }
  //   else if (n_pulse_received1 == 0)
  //   {
  //     n_pulse_set1 = 0;
  //     wheel_dir1 = 0;
  //   }
		// else
		// {
		// 	n_pulse_set1 = abs(n_pulse_received1);
		// 	wheel_dir1 = n_pulse_received1/abs(n_pulse_received1);
		// }

		 give_command = 1;
		break;

	case 2: // Print out speed
		// sprintf(msg1, "\r\n input_capture_prev1: %lu", input_capture_prev1);
		// sprintf(msg2, "\r\n n_pulse_set1: %lu", n_pulse_set1);
		// sprintf(msg3, "\r\n n_pulse1: %lu", n_pulse1);
		// sprintf(msg4, "\r\n PWM: %lu", returned_pwm1);
		// sprintf(msg5, "\r\n error1: %ld", error1);
		// sprintf(msg6, "\r\n integrated_error1: %ld", integrated_error1);

		// HAL_UART_Transmit(&huart2, (uint8_t*)msg1, strlen(msg1), HAL_MAX_DELAY);
		// HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
		// HAL_UART_Transmit(&huart2, (uint8_t*)msg3, strlen(msg3), HAL_MAX_DELAY);
		// HAL_UART_Transmit(&huart2, (uint8_t*)msg4, strlen(msg4), HAL_MAX_DELAY);
		// HAL_UART_Transmit(&huart2, (uint8_t*)msg5, strlen(msg5), HAL_MAX_DELAY);
		// HAL_UART_Transmit(&huart2, (uint8_t*)msg6, strlen(msg6), HAL_MAX_DELAY);

    sprintf(msg1, "\r\n X: %ld", my_rover.x);
    sprintf(msg2, "\r\n Y: %ld", my_rover.y);
    sprintf(msg3, "\r\n Phi: %ld", my_rover.phi);
    sprintf(msg4, "\r\n n_pulse1: %ld", n_pulse1);
    sprintf(msg5, "\r\n n_pulse2: %ld", n_pulse2);
    sprintf(msg6, "\r\n n_pulse_set1: %ld", my_rover.trg_wheel_spd1);
    sprintf(msg7, "\r\n n_pulse_set2: %ld", my_rover.trg_wheel_spd2);
    sprintf(msg8, "\r\n PWM 1: %lu", returned_pwm1);
	sprintf(msg9, "\r\n PWM 2: %lu", returned_pwm2);
	sprintf(msg10, "\r\n angular_vel: %ld", my_rover.angular_vel);
	sprintf(msg11, "\r\n linear_vel: %ld", my_rover.linear_vel);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg1, strlen(msg1), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg3, strlen(msg3), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg4, strlen(msg4), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg5, strlen(msg5), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg6, strlen(msg6), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg7, strlen(msg7), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg8, strlen(msg8), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg9, strlen(msg9), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg10, strlen(msg10), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg11, strlen(msg11), HAL_MAX_DELAY);

		break;

	case 3:
		return 2;
	};

	return 1;
}

uint32_t pid_controller_wheels(uint32_t current_point, uint32_t desired_point, uint32_t wheel_id)
{
	int32_t Kp = -17;
	int32_t Ki = -4;

	int32_t output_pwm = 0;
	int32_t return_pwm = 0;


	if (wheel_id == 1)
	{
		error1 = desired_point - current_point;
		output_pwm = (Kp*error1) + (Ki*integrated_error1);

		if ((abs(integrated_error1)<200) || ((error1*integrated_error1)<=0)) // anti winding
		{
			integrated_error1 += error1;
		}

	}
	else if (wheel_id == 2)
	{
		error2 = desired_point - current_point;
		output_pwm = (Kp*error2) + (Ki*integrated_error2);

		if ((abs(integrated_error2)<200) || ((error2*integrated_error2)<=0)) // anti winding
		{
			integrated_error2 += error2;
		}
	}

	if (output_pwm < 0)
	{
		return_pwm = 0;
	}
	else if (output_pwm > 400)
	{
		return_pwm = 400;
	}
	else
	{
		return_pwm = output_pwm;
	}

	return return_pwm;
}

void update_my_position(void)
{
	int32_t s1 = 0;
	int32_t s2 = 0;
	int32_t s_mean = 0;

	s1 = (int32_t) wheel_dir1*(n_pulse1/270.0)*3.14*R*0.05*100;
	s2 = (int32_t) wheel_dir2*(n_pulse2/270.0)*3.14*R*0.05*100;

	s_mean = (s1+s2)/2.0;

	my_rover.x += (int32_t) s_mean*cos(my_rover.phi/100.0); // in mm
	my_rover.y += (int32_t) s_mean*sin(my_rover.phi/100.0); // in mm
	my_rover.phi += (int32_t) (s2-s1)/L; // in 100*rad

	my_rover.phi = (int32_t) atan2(sin((float) (my_rover.phi/100.0)),cos((float) (my_rover.phi/100.0)))*100; // map phi between [-pi,pi]
}

void heading_pid_controller(int32_t error_heading)
{
	int8_t kp = 4;

	my_rover.angular_vel = kp*error_heading;

}

void calculate_new_speed(void)
{
	int32_t dx,dy;
	int32_t error_heading;

	int32_t u1,u2;

	int32_t k;
	int32_t error_distance;

	int32_t desired_heading;

	int32_t left_wheel_cmd, right_wheel_cmd;

	// Calculate error vector
	dx = my_rover.trg_x - my_rover.x;
	dy = my_rover.trg_y - my_rover.y;

	error_distance = (int32_t) sqrt(dx*dx+dy*dy);

	if (error_distance < 5)
	{
		my_rover.trg_wheel_spd1 = 0;
		my_rover.trg_wheel_spd2 = 0;
		return;
	}
	else
	{
		// Calculate control vector u
		k = (int32_t) atan(error_distance/100.0)*100;
		u1 = (int32_t) k*dx/100.0;
		u2 = (int32_t) k*dy/100.0;
	}

	my_rover.linear_vel = (int32_t) sqrt(u1*u1 + u2*u2);

	desired_heading = (int32_t) atan2(u2,u1)*100;
	error_heading = (int32_t) atan2(sin((desired_heading - my_rover.phi)/100.0),cos((desired_heading - my_rover.phi)/100.0))*100;

	heading_pid_controller(error_heading);

	left_wheel_cmd = (int32_t) (2*my_rover.linear_vel - my_rover.angular_vel*L)/(2*R*100);
	right_wheel_cmd = (int32_t) (2*my_rover.linear_vel + my_rover.angular_vel*L)/(2*R*100);

	if (left_wheel_cmd > 0)
	{
		wheel_dir1 = 1;
	}
	else
	{
		wheel_dir1 = -1;
	}

	if (right_wheel_cmd > 0)
	{
		wheel_dir2 = 1;
	}
	else
	{
		wheel_dir2 = -1;
	}

	// Calculate the wheel speed
	my_rover.trg_wheel_spd1 = (int32_t) abs(left_wheel_cmd*(270)/(3.1415*R));
	my_rover.trg_wheel_spd2 = (int32_t) abs(right_wheel_cmd*(270)/(3.1415*R));

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
void assert_failed(uint32_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
