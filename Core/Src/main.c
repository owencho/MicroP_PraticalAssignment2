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
#include "Usart.h"
#include "Syscfg.h"
#include "Exti.h"
#include "Adc.h"
#include "Irq.h"
#include "Rcc.h"
#include "Serial.h"
#include "Timer.h"
#include "TimerBase.h"
#include"TimerMacro.h"
#include "Common.h"
#include <stdio.h>
#include <stdint.h>
#include <malloc.h>
#include <stdarg.h>
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
volatile int sharedAverageAdcValue;
volatile int adcTurn;
extern volatile int usartTurn;
float voltageValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float calculateADC(int adcValue);
void configureTimer3();
void configureUart5();
void configureAdc1();
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
  disableIRQ();
  enableGpio(PORT_B);
  gpioSetMode(gpioB, PIN_6, GPIO_ALT);
  gpioSetPinSpeed(gpioB,PIN_6,HIGH_SPEED);
  gpioSetAlternateFunction(gpioB, PIN_6 ,AF2); //set PB6 as TIM4_CH1

  gpioSetMode(gpioB, PIN_8, GPIO_ALT);
  gpioSetPinSpeed(gpioB, PIN_8,HIGH_SPEED);
  gpioSetAlternateFunction(gpioB, PIN_8 ,AF2); //set PB8 as TIM4_CH3

  gpioSetMode(gpioB, PIN_0, GPIO_ALT);
  gpioSetPinSpeed(gpioB, PIN_0,HIGH_SPEED);
  gpioSetAlternateFunction(gpioB, PIN_0 ,AF2); //set PB0 as TIM3_CH4

  gpioSetMode(gpioB, PIN_1, GPIO_ALT);
  gpioSetPinSpeed(gpioB, PIN_1,HIGH_SPEED);
  gpioSetAlternateFunction(gpioB, PIN_1 ,AF2); //set PB1 as TIM3_CH4

  // set pin 1 as input of the signal
  enableGpioA();
  gpioSetMode(gpioA, PIN_1, GPIO_ANALOG);
  gpioSetPinSpeed(gpioA,PIN_1,HIGH_SPEED);

  enableGpio(PORT_C);
  gpioSetMode(gpioC, PIN_12, GPIO_ALT);  //set GpioC as alternate mode
  gpioSetPinSpeed(gpioC,PIN_12,HIGH_SPEED);

  enableGpio(PORT_D);
  gpioSetMode(gpioD, PIN_2, GPIO_ALT);  //set GpioC as alternate mode
  gpioSetPinSpeed(gpioD,PIN_2,HIGH_SPEED);


  //set alternate function
  gpioSetAlternateFunction(gpioC ,PIN_12 ,AF8); //set PC12 as USART5_TX
  gpioSetAlternateFunction(gpioD ,PIN_2 ,AF8); //set PD2 as USART5_RX


  configureTimer3();
  configureAdc1();
  configureUart5();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  adcTurn = 0;
  usartTurn =0;
  enableIRQ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(!adcTurn){
		  disableIRQ();
		  voltageValue = calculateADC(sharedAverageAdcValue);
		  if(!usartTurn){
			  serialSend(uart5,"adc value is %d voltage is %d.%d V \r\n",sharedAverageAdcValue,(int)voltageValue,getDecimalPoint(voltageValue));
		  }
		  adcEnableEOCInterrupt(adc1);
		  adcTurn = 1;
		  enableIRQ();
	 	  }
	  //disableIRQ();

	 // enableIRQ();

    /* USER CODE END WHILE */
	  /*
	  if(!adcTurn){
		  disableIRQ();
		  voltageValue = calculateADC(sharedAverageAdcValue);
		  //asprintf(&msgOut, "Voltage is %. 2f", voltageValue);

		  //usartSend(uart5,msgOut);
		  adcTurn = 1;
		  enableIRQ();
	  }
	*/
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

float calculateADC(int adcValue){
	float value;
	value = adcValue * 2827;
	value = value / 3785000;
	return value;
}
void configureTimer3(){
	  enableTimer3();
	  timerSetControlRegister(timer3,(ARR_ENABLE | TIMER_UP_COUNT |
			  	  	  	  	  	  	  TIMER_ONE_PULSE_DISABLE |TIMER_COUNTER_ENABLE |
									  T1_CH1_SELECT| MASTER_MODE_COMP_OC3REF|OC3_OUT_LOW));
	  //ARR disable
	  //ARR reg is buffered
	  //Up count
	  //one pulse mode disabled
	  //counter enabled
	  //CH1 is connected to T1
	  // Master Mode is routed to Output 3
	  timerSetSlaveMasterRegister(timer3,SLAVE_MODE| SMS_DISABLED | TRIGGER_FIL_T1);
	  //slave mode disabled
	  timerSetCompareCaptureModeRegister(timer3,(CC3_OUTPUT |OC3_MODE_TOGGLE));
	  // CC3 channel is configured as toggle mode

	  timerSetCompareCaptureEnableRegister(timer3,(OC3_ENABLE|OC3_ACTIVELOW));


	  //to generate 2khz with 50% duty cycle
	  timerWritePrescaler(timer3,0);
	  timerWriteAutoReloadReg(timer3, 45000);
	  timerWriteCapComReg3(timer3 , 22499);
}

void configureAdc1(){
	  enableAdc1();
	  //enable interrupt
	  //adcEnableEOCInterrupt(adc1);
	  adcSetScanMode(adc1,ENABLE_MODE);
	  nvicEnableInterrupt(18);
	  adcSetADCResolution(adc1,ADC_RES_12_BIT);
	  adcSetRightDataAlignment(adc1);
	  adcSetSingleConvertion(adc1);
	  //adcSetContinousConvertion(adc1);
	  adcSetSamplingTime(adc1,CHANNEL_1,ADC_SAMP_3_CYCLES);
	  adcSetExternalTriggerRegularChannel(adc1,T_DETECTION_RISING);
	  adcSetSingleSequenceRegister(adc1,CHANNEL_1,1);
	  adcSetExternalEventSelectForRegularGroup(adc1,T3_TRGO);
	  adcEnableADCConversion(adc1);
	  //adcSetStartRegularConversion(adc1);
}

void configureUart5(){
	  enableUART5();
	  nvicEnableInterrupt(53);
	  //usartEnableInterrupt(uart5,TRANS_COMPLETE);
	  //usartClearTcFlag(uart5);
	  setUsartOversamplingMode(uart5,OVER_16);
	  usartSetBaudRate(uart5,115200);
	  setUsartWordLength(uart5,DATA_8_BITS);
	  usartEnableParityControl(uart5);
	  setUsartParityMode(uart5,ODD_PARITY);
	  usartSetStopBit(uart5,STOP_BIT_2);
	  usartEnableTransmission(uart5);
	  enableUsart(uart5);
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
