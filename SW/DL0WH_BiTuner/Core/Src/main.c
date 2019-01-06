
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"

#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include "task_USB.h"
#include "device_adc.h"
#include "bus_i2c.h"
#include "bus_spi.h"
#include "task_Controller.h"


#define  PERIOD_VALUE       (uint32_t)(16000UL - 1)                                             /* Period Value = 1ms */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern EventGroupHandle_t             adcEventGroupHandle;
extern EventGroupHandle_t             extiEventGroupHandle;
extern EventGroupHandle_t             globalEventGroupHandle;
extern EventGroupHandle_t             spiEventGroupHandle;
extern EventGroupHandle_t             usbToHostEventGroupHandle;

extern TIM_HandleTypeDef              htim2;
extern __IO uint32_t                  uwTick;

extern uint8_t                        i2c1TxBuffer[I2C_TXBUFSIZE];
extern uint8_t                        i2c1RxBuffer[I2C_RXBUFSIZE];

SYSCLK_CONFIG_t                       g_main_SYSCLK_CONFIG    = SYSCLK_CONFIG_48MHz_MSI;

/* Typical values for 48 MHz MSI configuration */
uint32_t                              g_main_HSE_VALUE        =   20000000UL;
uint32_t                              g_main_HSE_START_MS     =        100UL;
uint32_t                              g_main_MSI_VALUE        =   48000000UL;
uint32_t                              g_main_HSI_VALUE        =   16000000UL;
uint32_t                              g_main_LSI_VALUE        =      32000UL;
uint32_t                              g_main_LSE_VALUE        =      32768UL;
uint32_t                              g_main_LSE_START_MS     =       5000UL;
uint32_t                              g_main_i2c1_timing      = 0x00000000UL;
uint32_t                              g_main_adc1_clkpresclr  = ADC_CLOCK_ASYNC_DIV1;
uint32_t                              g_main_adc2_clkpresclr  = ADC_CLOCK_ASYNC_DIV1;
uint32_t                              g_main_adc3_clkpresclr  = ADC_CLOCK_ASYNC_DIV1;
uint8_t                               g_main_PCLK1_Prescaler  = 1U;


/* Counter Prescaler value */
uint32_t                              uhPrescalerValue        = 0;

volatile uint32_t                     g_rtc_ssr_last          = 0UL;

static uint64_t                       s_timerLast_us          = 0ULL;
static uint64_t                       s_timerStart_us         = 0ULL;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t crcCalc(const uint32_t* ptr, uint32_t len)
{
  return HAL_CRC_Calculate(&hcrc, (uint32_t*) ptr, len);
}


uint8_t sel_u8_from_u32(uint32_t in_u32, uint8_t sel)
{
  return 0xff & (in_u32 >> (sel << 3));
}

void mainCalcFloat2IntFrac(float val, uint8_t fracCnt, int32_t* outInt, uint32_t* outFrac)
{
  const uint8_t isNeg = val >= 0 ?  0U : 1U;

  if (!outInt || !outFrac) {
    return;
  }

  *outInt = (int32_t) val;
  val -= *outInt;

  if (isNeg) {
    val = -val;
  }
  val *= pow(10, fracCnt);
  *outFrac = (uint32_t) (val + 0.5f);
}


void mainPowerSwitchDo(POWERSWITCH_ENUM_t sw, uint8_t enable)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __asm volatile( "NOP" );

  switch (sw) {
  case POWERSWITCH__SLOWER_24MHZ:
    /* SMPS handling */
    if (enable)
    {
      /* Scale2: 1.0V up to 24MHz */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

    } else {
      /* Scale1: 1.2V up to 80MHz */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    }
    break;

  case POWERSWITCH__BAT_SW:
    if (enable) {
      HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_5);

    } else {
      HAL_PWREx_DisableBatteryCharging();
    }
    break;

  case POWERSWITCH__BAT_HICUR:
    if (enable) {
      HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_1_5);

    } else {
      HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_5);
    }
    break;

  default:
    { }
  }

  __HAL_RCC_GPIOC_CLK_DISABLE();
}

void mainPowerSwitchInit(void)
{
  /* Vbat charger of MCU enabled with 1.5 kOhm */
  mainPowerSwitchDo(POWERSWITCH__BAT_HICUR, 1U);

  /* MCU core frequency = 16 MHz */
  mainPowerSwitchDo(POWERSWITCH__SLOWER_24MHZ, 1U);
}


void SystemResetbyARMcore(void)
{
  /* Set SW reset bit */
  SCB->AIRCR = 0x05FA0000UL | SCB_AIRCR_SYSRESETREQ_Msk;
}

/* Used by the run-time stats */
void configureTimerForRunTimeStats(void)
{
  getRunTimeCounterValue();

  /* Interrupt disabled block */
  {
    __disable_irq();

    /* Called only once */
    s_timerStart_us = s_timerLast_us;

    __enable_irq();
  }
}

/* Used by the run-time stats */
unsigned long getRunTimeCounterValue(void)
{
  uint64_t l_timerStart_us = 0ULL;
  uint64_t l_timer_us = HAL_GetTick() & 0x003fffffUL;                                                   // avoid overflows

  /* Add microseconds */
  l_timer_us *= 1000ULL;
  l_timer_us += TIM2->CNT % 1000UL;                                                                     // TIM2 counts microseconds

  /* Interrupt disabled block */
  {
    __disable_irq();

    s_timerLast_us  = l_timer_us;
    l_timerStart_us = s_timerStart_us;

    __enable_irq();
  }

  uint64_t l_timerDiff64 = (l_timer_us >= l_timerStart_us) ?  (l_timer_us - l_timerStart_us) : l_timer_us;
  uint32_t l_timerDiff32 = (uint32_t) (l_timerDiff64 & 0xffffffffULL);
  return l_timerDiff32;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* Check if ARM core is already in reset state */
  if (!(RCC->CSR & 0xff000000UL)) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __asm volatile( "NOP" );

    /* Turn off battery charger of Vbat */
    HAL_PWREx_DisableBatteryCharging();

    /* ARM software reset to be done */
    SystemResetbyARMcore();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  //#define I2C_BUS1_SCAN

  #ifdef I2C_BUS1_SCAN
  i2cBusAddrScan(&hi2c1, i2c1MutexHandle);
  #endif

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_MSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

void HFT_SystemClock_Config(SYSCLK_CONFIG_t sel)
{
#if 0
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_CRSInitTypeDef RCC_CRSInitStruct;

  /* Set global variable */
  g_main_SYSCLK_CONFIG = sel;

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);


  switch (sel) {
  case SYSCLK_CONFIG_16MHz_MSI:
    {

    }

  default: { }
  }

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
#endif
}


#if 1
void  vApplicationIdleHook(void)
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
  task. It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()). If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
  /* TODO:
   * 1) Reduce 80 MHz  to  2 MHz
   * 2)  go to LPRun  (SMPS 2 High (-->  MR range 1) --> MR range 2 --> LPR
   * 3)  Go to LPSleep
   *
   * WAKEUP
   * 1)  In LPRun go to 80 MHz (LPR --> MR range 2 (--> MR range 1) --> SMPS 2 High)
   * 2)  Increase 2 MHz to 80 MHz
   */

  /* Enter sleep mode */
  __asm volatile( "WFI" );

  /* Increase clock frequency to 80 MHz */
  // TODO: TBD
}
#endif

#if 0
void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
#if 0
  HAL_SuspendTick();
#endif
  g_rtc_ssr_last = RTC->SSR;
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
#if 0
  volatile uint32_t l_rtc_ssr_now = RTC->SSR;
  volatile uint32_t l_rtc_sub1024 = (l_rtc_ssr_now >= g_rtc_ssr_last) ?  (l_rtc_ssr_now - g_rtc_ssr_last) : (1024UL - (g_rtc_ssr_last - l_rtc_ssr_now));
  volatile uint32_t l_millis = (l_rtc_sub1024 * 1000UL) / 1024UL;

  if (l_millis <= *ulExpectedIdleTime) {
    uwTick += l_millis;
  }
  HAL_ResumeTick();
#endif
}
#endif

void vApplicationMallocFailedHook(void)
{
  int  dbgLen;
  char dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "***ERROR: Out of memory  vApplicationMallocFailedHook(): file %s on line %d\r\n", __FILE__, __LINE__);
  usbLogLen(dbgBuf, dbgLen);

  configASSERT(0);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  int  dbgLen;
  char dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "***ERROR: ERROR-HANDLER  Wrong parameters value  _Error_Handler(): file %s on line %d\r\n", file, line);
  usbLogLen(dbgBuf, dbgLen);

  configASSERT(0);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  int  dbgLen;
  char dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "***ERROR: ERROR-HANDLER  Wrong parameters value  assert_failed(): file %s on line %ld\r\n", file, line);
  usbLogLen(dbgBuf, dbgLen);

  configASSERT(0);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
