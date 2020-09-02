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
#include "Gpio.h"
#include "Nvic.h"
#include "Syscfg.h"
#include "Exti.h"
#include "Rcc.h"
#include "Timer.h"
#include "TimerBase.h"
#include"TimerMacro.h"
#include "Common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile int sharedAverageAdcValue;
int adcTurn;
extern int usartTurn;
volatile float voltageValue;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void configureTimer4();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  	  enableGpio(PORT_B);
  	  gpioSetMode(gpioB, PIN_6, GPIO_ALT);
  	  gpioSetPinSpeed(gpioB,PIN_6,HIGH_SPEED);
  	  gpioSetAlternateFunction(gpioB, PIN_6 ,AF2); //set PB6 as TIM4_CH1

	  gpioSetMode(gpioB, PIN_8, GPIO_ALT);
	  gpioSetPinSpeed(gpioB, PIN_8,HIGH_SPEED);
	  gpioSetAlternateFunction(gpioB, PIN_8 ,AF2); //set PB8 as TIM4_CH3

	  configureTimer4();


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */
void configureTimer4(){
	  enableTimer4();
	  timerSetControlRegister(timer4,(ARR_ENABLE | TIMER_UP_COUNT |
			  	  	  	  	  	  	  TIMER_ONE_PULSE_DISABLE |TIMER_COUNTER_ENABLE |
									  T1_CH1_SELECT| MASTER_MODE_RESET|OC3_OUT_LOW));
	  //ARR reg is buffered
	  //Up count
	  //one pulse mode disabled
	  //counter enabled
	  //CH1 is connected to T1
	  // Master Mode Reset
	  timerSetSlaveMasterRegister(timer4,SLAVE_MODE| SMS_GATED_M | TRIGGER_FIL_T1);
	  //Trigger was selected TI1FP1
	  //Slave mode selected as gated mode
	  //slave mode was triggered on t1
	  timerSetCompareCaptureModeRegister(timer4,(CC1_INPUT_IC1_MAP_TI1 | IC1_NO_PRESCALE |
			  	  	  	  	  	  	  	  	  	  IC1_NO_FILER| CC3_OUTPUT |OC3_MODE_PWM_M1|OC3_FAST_ENABLE));
	  // CC1 channel is configured as input, IC1 is mapped on TI1
	  // IC1 no prescaler
	  // IC1 no filter
	  // CC3 channel is configured as PWM mode 1 output
	  timerSetCompareCaptureEnableRegister(timer4,(CC1_CAPTURE_ENABLED|CC1_CAP_FALLING_EDGE|
			  	  	  	  	  	  	  	  	  	   OC3_ENABLE|OC3_ACTIVEHIGH));
	  //capture enabled for CH1
	  // trigger on falling edge
	  //output for CH3

	  //to generate PWM with 6khz with 75% duty cycle
	  timerWritePrescaler(timer4,0);
	  timerWriteAutoReloadReg(timer4, 14999);
	  timerWriteCapComReg3(timer4 , 11250);
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



