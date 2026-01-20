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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
#define TS 0.002f
#define V_SUPPLY 3.0f
#define PWM_ARR 3999.0f

float CPR = 3840.0f;

float a[3] = { -1.1429805f, 0.4128016f };
float b[3] = { 0.06745527f, 0.13491055f, 0.06745527f };

float param_Kp_speed = 5.0f;
float param_Ti_speed = 0.2f;
float param_Td_speed = 0.00f;
float param_Kp_pos = 15.0f;

float Kp_dig = 0.0f;
float Ki_dig = 0.0f;
float Kd_dig = 0.0f;

float pid_integral = 0.0f;
float pid_prev_meas = 0.0f;

int16_t enc_prev = 0;
volatile int32_t total_pulses = 0;

float x_buf[2] = {0};
float y_buf[2] = {0};

float PULSE_TO_RAD = 0.0f;

////////// Enkoder Setpoint

#define MASTER_CPR 80.0f
float MASTER_PULSE_TO_RAD = 0.0f;

int16_t master_enc_prev = 0;
volatile int32_t master_total_pulses = 0;

///////////////debug
volatile float dbg_target_pos = 0.0f;

volatile float dbg_pos_rad = 0.0f;
volatile float dbg_speed_raw = 0.0f;
volatile float dbg_speed_filt = 0.0f;
volatile float dbg_target_speed = 0.0f;
volatile float dbg_pwm_u = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  Kp_dig = param_Kp_speed;
  Ki_dig = param_Kp_speed * (TS / param_Ti_speed);
  Kd_dig = param_Kp_speed * (param_Td_speed / TS);

  PULSE_TO_RAD = (2.0f * 3.1415927f) / CPR;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);

  MASTER_PULSE_TO_RAD = (2.0f * 3.1415927f) / MASTER_CPR;

  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

			char msg[128];


			sprintf(msg,
			   "POS:%.2f | REF:%.2f | SPD:%.2f | PWM:%.0f\r\n",
			   dbg_pos_rad,
			   dbg_target_pos,
			   dbg_speed_filt,
			   dbg_pwm_u);


			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);


			HAL_Delay(50);


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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
	    //###############################################

	  int16_t master_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
	  int16_t master_diff = master_cnt - master_enc_prev;
	  master_enc_prev = master_cnt;
	  master_total_pulses += master_diff;

	  float target_pos_ref = master_total_pulses * MASTER_PULSE_TO_RAD;



	    //###############################################

    int16_t current_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t diff = current_cnt - enc_prev;
    enc_prev = current_cnt;
    total_pulses += diff;



    //###############################################

    float position_rad = total_pulses * PULSE_TO_RAD;
    float speed_raw = (diff * PULSE_TO_RAD) / TS;

    float speed = (b[0] * speed_raw)
                + (b[1] * x_buf[0])
                + (b[2] * x_buf[1])
                - (a[0] * y_buf[0])
                - (a[1] * y_buf[1]);

    x_buf[1] = x_buf[0];
    x_buf[0] = speed_raw;

    y_buf[1] = y_buf[0];
    y_buf[0] = speed;

    //###############################################

    float error_pos = target_pos_ref - position_rad;
    float target_speed = param_Kp_pos * error_pos;

    if (target_speed > 12.0f)  target_speed = 12.0f;
    if (target_speed < -12.0f) target_speed = -12.0f;

    //###############################################

    float error_speed = target_speed - speed;

    float P = Kp_dig * error_speed;
    pid_integral += Ki_dig * error_speed;
    float D = -Kd_dig * (speed - pid_prev_meas);

    float u_volts = P + pid_integral + D;

    //###############################################

    float u_sat_volts = u_volts;

    if (u_volts > V_SUPPLY) {
        u_sat_volts = V_SUPPLY;
        if (error_speed > 0) pid_integral -= Ki_dig * error_speed;
    }
    else if (u_volts < -V_SUPPLY) {
        u_sat_volts = -V_SUPPLY;
        if (error_speed < 0) pid_integral -= Ki_dig * error_speed;
    }

    pid_prev_meas = speed;

    float u_pwm = (u_sat_volts / V_SUPPLY) * PWM_ARR;

    //###############################################

    if (fabsf(u_pwm) < 100.0f)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    else if (u_pwm > 0.0f)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)u_pwm);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)(-u_pwm));
    }

    //###############################################

    dbg_pos_rad = position_rad;
    dbg_speed_raw = speed_raw;
    dbg_speed_filt = speed;
    dbg_target_speed = target_speed;
    dbg_pwm_u = u_sat_volts;
    dbg_target_pos = target_pos_ref;

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
