/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "Can_receive.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	PID_TypeDef rm3508_speed_pid[4];
	PID_TypeDef rm3508_position_pid[4];

typedef struct _reseive_TypeDef
{
float V_x;
float V_y;
float W;

}_reseive_TypeDef;

_reseive_TypeDef rx_motor;

//PID_TypeDef rm3508_speed_pid[4] =
//{
//	.iterm_cofeA   = 1000,
//  .iterm_cofeB   = 400,
//	.output_lpf_rc = 0.1,
//};
//PID_TypeDef rm3508_position_pid =
//{
//	.iterm_cofeA   = 400,
//  .iterm_cofeB   = 100,
//	.output_lpf_rc = 0.1,
//};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern motor_measure_t motor_chassis[7];

float a=1.4142135;
float b=1.4142135;
float l=400;
float cal_target[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void rx_motor_init()
{
		rx_motor.V_x=100;
		rx_motor.V_y=-100;
		rx_motor.W=0;

}

float V_calculate (i)
{
	float V;
	
	if(i == 0)
	{
		V=-rx_motor.V_x*a+rx_motor.V_y*b+l*rx_motor.W;
	}
	else if(i == 1)
	{
		V=-rx_motor.V_x*a-rx_motor.V_y*b+l*rx_motor.W;
	}
	else if(i == 2)
	{
		V=rx_motor.V_x*a-rx_motor.V_y*b+l*rx_motor.W;
	}
	else if(i ==3)
	{
		 V=rx_motor.V_x*a+rx_motor.V_y*b+l*rx_motor.W;
	}
		
		return V;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();

  HAL_GPIO_WritePin(GPIOH , GPIO_PIN_2 , GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH , GPIO_PIN_3 , GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH , GPIO_PIN_4 , GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH , GPIO_PIN_5 , GPIO_PIN_SET);


  for(int i = 0 ;i<4;i++ )
  	{
		rx_motor_init();
		pid_init(&rm3508_speed_pid[i]);
	rm3508_speed_pid[i].f_param_init(&rm3508_speed_pid[i], PID_Speed, 
								  16000,5000,0,0,800,V_calculate(i),1.48,0.039,0.1,PID_Integral_Limit  | PID_OutputFilter | PID_Derivative_On_Measurement | PID_DerivativeFilter);

		//	pid_init(&rm3508_position_pid[i]);
//	rm3508_position_pid[i].f_param_init(&rm3508_position_pid[i], PID_Position, 
//								  16000,5000,0,0,800,0,0.056,0,0,PID_Integral_Limit);
	}
  
  
  HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//		rm3508_position_pid.f_cal_pid(&rm3508_position_pid, motor_chassis[0].total_angle);
//		rm3508_speed_pid.target = rm3508_position_pid.output;
		
	  for(int i = 0; i<4 ; i ++)
	  {
		  rx_motor_init();
		  rm3508_speed_pid[i].f_cal_pid(&rm3508_speed_pid[i], motor_chassis[i].speed_rpm);
  
	  }
		
		CAN_cmd_chassis(rm3508_speed_pid[0].output,rm3508_speed_pid[1].output,rm3508_speed_pid[2].output,rm3508_speed_pid[3].output);
	
	    HAL_Delay(5);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
