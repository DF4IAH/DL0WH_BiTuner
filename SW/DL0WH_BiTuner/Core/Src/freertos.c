/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
// Core, USB_DEVICE
#include "main.h"
#include "usb_device.h"

// App
#include "bus_spi.h"
#include "device_adc.h"
#include "task_Controller.h"
#include "task_Interpreter.h"
#include "task_USB.h"
#include "task_UART.h"
#include "task_CAT.h"

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
/* USER CODE BEGIN Variables */
extern volatile uint8_t               spi1TxBuffer[SPI1_BUFFERSIZE];
extern volatile uint8_t               spi1RxBuffer[SPI1_BUFFERSIZE];

extern SPI_HandleTypeDef              hspi1;

extern float                          g_adc_refint_val;
extern float                          g_adc_vref_mv;
extern float                          g_adc_bat_mv;
extern float                          g_adc_temp_deg;
extern float                          g_adc_fwd_mv;
extern float                          g_adc_rev_mv;
extern float                          g_adc_vdiode_mv;
extern float                          g_adc_swr;

EventGroupHandle_t                    adcEventGroupHandle;
EventGroupHandle_t                    extiEventGroupHandle;
EventGroupHandle_t                    globalEventGroupHandle;
EventGroupHandle_t                    spiEventGroupHandle;
EventGroupHandle_t                    usbToHostEventGroupHandle;
EventGroupHandle_t                    uartEventGroupHandle;
EventGroupHandle_t                    catEventGroupHandle;

static uint8_t                        s_rtos_DefaultTask_adc_enable     = 0U;
static uint32_t                       s_rtos_DefaultTaskStartTime       = 0UL;

#ifdef SPI_DRIVER_CHECK
static uint32_t                       s_rtos_lastErrorBuf[16]           = { 0UL };
static uint8_t                        s_rtos_lastErrorCnt               = 0U;
#endif

static uint32_t                       s_rtos_Matcher                    = 0UL;



/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId controllerTaskHandle;
osThreadId usbToHostTaskHandle;
osThreadId usbFromHostTaskHandle;
osThreadId interpreterTaskHandle;
osThreadId uartTxTaskHandle;
osThreadId uartRxTaskHandle;
osThreadId catTxTaskHandle;
osThreadId catRxTaskHandle;
osMessageQId usbToHostQueueHandle;
osMessageQId usbFromHostQueueHandle;
osMessageQId controllerInQueueHandle;
osMessageQId controllerOutQueueHandle;
osMessageQId uartTxQueueHandle;
osMessageQId uartRxQueueHandle;
osMessageQId catTxQueueHandle;
osMessageQId catRxQueueHandle;
osTimerId defaultTimerHandle;
osTimerId controllerTimerHandle;
osSemaphoreId c2default_BSemHandle;
osSemaphoreId i2c1_BSemHandle;
osSemaphoreId spi1_BSemHandle;
osSemaphoreId cQin_BSemHandle;
osSemaphoreId cQout_BSemHandle;
osSemaphoreId c2usbToHost_BSemHandle;
osSemaphoreId c2usbFromHost_BSemHandle;
osSemaphoreId usb_BSemHandle;
osSemaphoreId c2interpreter_BSemHandle;
osSemaphoreId usbFromHost_BSemHandle;
osSemaphoreId uart_BSemHandle;
osSemaphoreId cat_BSemHandle;
osSemaphoreId c2uartTx_BSemHandle;
osSemaphoreId c2uartRx_BSemHandle;
osSemaphoreId c2catTx_BSemHandle;
osSemaphoreId c2catRx_BSemHandle;
osSemaphoreId uartRx_BSemHandle;
osSemaphoreId catRx_BSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControllerTask(void const * argument);
void StartUsbToHostTask(void const * argument);
void StartUsbFromHostTask(void const * argument);
void StartInterpreterTask(void const * argument);
void StartUartTxTask(void const * argument);
void StartUartRxTask(void const * argument);
void StartCatTxTask(void const * argument);
void StartCatRxTask(void const * argument);
void rtosDefaultTimerCallback(void const * argument);
void rtosControllerTimerCallback(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
  /* Overwritten by main.c */
}

__weak unsigned long getRunTimeCounterValue(void)
{
  /* Overwritten by main.c */
  return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
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

  /* Overwritten by main.c */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */

  /* Overwritten by main.c */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */

  /* Overwritten by main.c */
}


/* Local functions */

/* Store driver error information */
#ifdef SPI_DRIVER_CHECK
static void rtosDefaultCheckSpiDrvError(uint8_t variant)
{
  if (spi1RxBuffer[0]) {
    s_rtos_lastErrorBuf[s_rtos_lastErrorCnt++] = \
        (((uint32_t) variant         ) << 24) |
        (((uint32_t)(spi1RxBuffer[0])) << 16) |
        (((uint32_t)(spi1RxBuffer[1])) <<  8) |
        (((uint32_t)(spi1RxBuffer[2]))      );

    s_rtos_lastErrorCnt &= 0x0f;
  }
}
#endif

/* Set and reset relays accordingly */
static void rtosDefaultSpiRelays(uint64_t relaySettings)
{
  const uint16_t relayC   = 0xffffU &  relaySettings;
  const uint16_t relayL   = 0xffffU & (relaySettings >> 16);
  const uint16_t relayExt = 0xffffU & (relaySettings >> 32);
  static _Bool isInit     = false;

  if (!isInit) {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Disable the PWM signal in case it is still on */
    HAL_GPIO_WritePin(GPIO_SPI_PWM_GPIO_Port, GPIO_SPI_PWM_Pin, GPIO_PIN_RESET);

    /* Release reset signal of all SPI drivers */
    HAL_GPIO_WritePin(GPIO_SPI_RST_GPIO_Port, GPIO_SPI_RST_Pin, GPIO_PIN_SET);
    osDelay(1);
  }

  /* Preparations */
  const uint8_t spiMsgOpenLoadCurrentEnable[]               = { 0x04U, 0x00U, 0x00U };
  const uint8_t spiMsgRetryOnOvervolNoRetryOverTemp[]       = { 0x09U, 0xffU, 0xffU };
  const uint8_t spiMsgSFPDdisabled[]                        = { 0x0cU, 0x00U, 0x00U };
  const uint8_t spiMsgPWMenabled[]                          = { 0x10U, 0xffU, 0xffU };
  const uint8_t spiMsgPWMand[]                              = { 0x14U, 0x00U, 0x00U };

  /* Relays for C */
  {
    if (!isInit) {
      /* Open Load Current Enable */
      spiProcessSpi1MsgTemplate(SPI1_C,   sizeof(spiMsgOpenLoadCurrentEnable), spiMsgOpenLoadCurrentEnable);

      /* Retry on over-voltage, no restart on over temp */
      spiProcessSpi1MsgTemplate(SPI1_C,   sizeof(spiMsgRetryOnOvervolNoRetryOverTemp), spiMsgRetryOnOvervolNoRetryOverTemp);

      /* No SFPD */
      spiProcessSpi1MsgTemplate(SPI1_C,   sizeof(spiMsgSFPDdisabled), spiMsgSFPDdisabled);

      /* PWM on all outputs */
      spiProcessSpi1MsgTemplate(SPI1_C,   sizeof(spiMsgPWMenabled), spiMsgPWMenabled);

      /* PWM signal is AND'ed */
      spiProcessSpi1MsgTemplate(SPI1_C,   sizeof(spiMsgPWMand), spiMsgPWMand);
    }

    /* Relay outputs to be driven ON/OFF */
    uint8_t spiMsgOnOff[3];
    uint8_t msgLen = 0U;
    spiMsgOnOff[msgLen++] = 0x00U;
    spiMsgOnOff[msgLen++] = (uint8_t) (0x00ffU & (relayC >> 8));
    spiMsgOnOff[msgLen++] = (uint8_t) (0x00ffU &  relayC      );
    spiProcessSpi1MsgTemplate(SPI1_C, sizeof(spiMsgOnOff), spiMsgOnOff);
  }

  /* Relays for L */
  {
    if (!isInit) {
      /* Open Load Current Enable */
      spiProcessSpi1MsgTemplate(SPI1_L,   sizeof(spiMsgOpenLoadCurrentEnable), spiMsgOpenLoadCurrentEnable);

      /* Retry on over-voltage, no restart on over temp */
      spiProcessSpi1MsgTemplate(SPI1_L,   sizeof(spiMsgRetryOnOvervolNoRetryOverTemp), spiMsgRetryOnOvervolNoRetryOverTemp);

      /* No SFPD */
      spiProcessSpi1MsgTemplate(SPI1_L,   sizeof(spiMsgSFPDdisabled), spiMsgSFPDdisabled);

      /* PWM on all outputs */
      spiProcessSpi1MsgTemplate(SPI1_L,   sizeof(spiMsgPWMenabled), spiMsgPWMenabled);

      /* PWM signal is AND'ed */
      spiProcessSpi1MsgTemplate(SPI1_L,   sizeof(spiMsgPWMand), spiMsgPWMand);
    }

    /* Relay outputs to be driven ON/OFF */
    uint8_t spiMsgOnOff[3];
    uint8_t msgLen = 0U;
    spiMsgOnOff[msgLen++] = 0x00U;
    spiMsgOnOff[msgLen++] = (uint8_t) (0x00ffU & (relayL >> 8));
    spiMsgOnOff[msgLen++] = (uint8_t) (0x00ffU &  relayL      );
    spiProcessSpi1MsgTemplate(SPI1_L, sizeof(spiMsgOnOff), spiMsgOnOff);
  }

  /* Relays for Ext */
  {
    if (!isInit) {
      /* Open Load Current Enable */
      spiProcessSpi1MsgTemplate(SPI1_EXT, sizeof(spiMsgOpenLoadCurrentEnable), spiMsgOpenLoadCurrentEnable);

      /* Retry on over-voltage, no restart on over temp */
      spiProcessSpi1MsgTemplate(SPI1_EXT, sizeof(spiMsgRetryOnOvervolNoRetryOverTemp), spiMsgRetryOnOvervolNoRetryOverTemp);

      /* No SFPD */
      spiProcessSpi1MsgTemplate(SPI1_EXT, sizeof(spiMsgSFPDdisabled), spiMsgSFPDdisabled);

      /* PWM on all outputs */
      spiProcessSpi1MsgTemplate(SPI1_EXT, sizeof(spiMsgPWMenabled), spiMsgPWMenabled);

      /* PWM signal is AND'ed */
      spiProcessSpi1MsgTemplate(SPI1_EXT, sizeof(spiMsgPWMand), spiMsgPWMand);
    }

    /* Relay outputs to be driven ON/OFF */
    uint8_t spiMsgOnOff[3];
    uint8_t msgLen = 0U;
    spiMsgOnOff[msgLen++] = 0x00U;
    spiMsgOnOff[msgLen++] = (uint8_t) (0x00ffU & (relayExt >> 8));
    spiMsgOnOff[msgLen++] = (uint8_t) (0x00ffU &  relayExt      );
    spiProcessSpi1MsgTemplate(SPI1_EXT, sizeof(spiMsgOnOff), spiMsgOnOff);
  }

  /* Enable PWM for 30 ms and thus energizes the relay coils for that time */
  {
    HAL_GPIO_WritePin(GPIO_SPI_PWM_GPIO_Port, GPIO_SPI_PWM_Pin, GPIO_PIN_SET);
    osDelay(30UL);
    HAL_GPIO_WritePin(GPIO_SPI_PWM_GPIO_Port, GPIO_SPI_PWM_Pin, GPIO_PIN_RESET);
  }
  isInit = true;

#ifdef SPI_DRIVER_CHECK
  /* Check output states for any driver errors */
  {
    const uint8_t spiMsgAllOff[] = { 0x00U, 0x00U, 0x00U };

    spiProcessSpi1MsgTemplate(SPI1_C,   sizeof(spiMsgAllOff), spiMsgAllOff);
    rtosDefaultCheckSpiDrvError('C');

    spiProcessSpi1MsgTemplate(SPI1_L,   sizeof(spiMsgAllOff), spiMsgAllOff);
    rtosDefaultCheckSpiDrvError('L');

    spiProcessSpi1MsgTemplate(SPI1_EXT, sizeof(spiMsgAllOff), spiMsgAllOff);
    rtosDefaultCheckSpiDrvError('X');
  }
#endif
}

static void rtosDefaultUpdateRelays(void)
{
  /*
   * Relay idx as follows:
   *  0 ..  7 <--> C1 .. C8
   *  8 .. 15 <--> L1 .. L8
   * 16       <--> CV
   * 17       <--> CH
   * 18 .. 23 spare output lines
   */

  static uint32_t matcherCurrent  = 0UL;

  const uint32_t  newDiff         = matcherCurrent ^ s_rtos_Matcher;
  const uint32_t  newSet          = newDiff & s_rtos_Matcher;
  const uint32_t  newClr          = newDiff & matcherCurrent;
  uint64_t        newRelays       = 0ULL;

  /* Concatenate all interleaved relay coils for SET and RESET
   *         Bit .. .. 3        2       1        0
   * Result:  .. .. .. C1.RESET C1.SET  C0.RESET C0.SET .
   */
  for (int8_t idx = 23; idx >= 0; idx--) {
    /* RESET coil */
    newRelays <<= 1;
    newRelays |= ((newClr >> idx) & 0x01ULL);

    /* SET coil */
    newRelays <<= 1;
    newRelays |= ((newSet >> idx) & 0x01ULL);
  }

  /* Send new relay settings via the SPI bus */
  rtosDefaultSpiRelays(newRelays);

  /* Store current state */
  matcherCurrent = s_rtos_Matcher;
}


static void rtosDefaultInit(void)
{
  /* Power switch settings */
  mainPowerSwitchInit();

  /* Enable ADC access */
  s_rtos_DefaultTask_adc_enable = 1U;

  /* Set relays to default setting
   * L's are shorted when SET - inverted logic
   */
  s_rtos_Matcher = 0x00ff00UL;
  rtosDefaultUpdateRelays();
}


static void rtosDefaultCyclicTimerEvent()
{
  /* Called every 30ms to start the ADCs */
  EventBits_t eb = xEventGroupGetBits(adcEventGroupHandle);
  if (!(eb & (EG_ADC1__CONV_RUNNING | EG_ADC2__CONV_RUNNING | EG_ADC3__CONV_RUNNING))) {
    /* No ADC conversions are active */

    adcStartConv(ADC_ADC1_REFINT_VAL);
  }
}

static void rtosDefaultCyclicStart(uint32_t period_ms)
{
  osTimerStart(defaultTimerHandle, period_ms);
}

static void rtosDefaultCyclicStop(void)
{
  osTimerStop(defaultTimerHandle);
}


static void rtosDefaultMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                    msgIdx  = 0UL;
  const uint32_t              hdr     = msgAry[msgIdx++];
  const RtosMsgDefaultCmds_t  cmd     = (RtosMsgDefaultCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgDefault__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_rtos_DefaultTaskStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      rtosDefaultInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Rtos_Default, 0U, MsgDefault__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;


  case MsgDefault__CallFunc01_CyclicTimerEvent:
    rtosDefaultCyclicTimerEvent();
    break;

  case MsgDefault__CallFunc02_CyclicTimerStart:
  {
    const uint32_t period = msgAry[1];
    rtosDefaultCyclicStart(period);
  }
    break;

  case MsgDefault__CallFunc03_CyclicTimerStop:
    rtosDefaultCyclicStop();
    break;


  /* MCU GPIO/Alternate-Functions set-up */
  case MsgDefault__SetVar01_IOs:
    {
      // TODO: implementation
    }
    break;

  /* Set and reset relays */
  case MsgDefault__SetVar03_C_L_CV_CH:
    {
      /* L's are shorted when SET - inverted logic */
      s_rtos_Matcher = 0x03ffffUL & (msgAry[1] ^ 0x00ff00UL);
      rtosDefaultUpdateRelays();
    }
    break;

  /* MCU clocking set-up */
  case MsgDefault__SetVar02_Clocking:
    {
      const DefaultMcuClocking_t mcuClocking = (DefaultMcuClocking_t) (0xffUL & (msgAry[msgIdx++] >> 24U));

      switch (mcuClocking) {
      case DefaultMcuClocking_16MHz_MSI:
        {
          Again_SystemClock_Config(SYSCLK_CONFIG_16MHz_MSI);
        }
        break;

      default: { }
      }
    }
    break;

    default: { }
    }  // switch (cmd)
  }
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of c2default_BSem */
  osSemaphoreDef(c2default_BSem);
  c2default_BSemHandle = osSemaphoreCreate(osSemaphore(c2default_BSem), 1);

  /* definition and creation of i2c1_BSem */
  osSemaphoreDef(i2c1_BSem);
  i2c1_BSemHandle = osSemaphoreCreate(osSemaphore(i2c1_BSem), 1);

  /* definition and creation of spi1_BSem */
  osSemaphoreDef(spi1_BSem);
  spi1_BSemHandle = osSemaphoreCreate(osSemaphore(spi1_BSem), 1);

  /* definition and creation of cQin_BSem */
  osSemaphoreDef(cQin_BSem);
  cQin_BSemHandle = osSemaphoreCreate(osSemaphore(cQin_BSem), 1);

  /* definition and creation of cQout_BSem */
  osSemaphoreDef(cQout_BSem);
  cQout_BSemHandle = osSemaphoreCreate(osSemaphore(cQout_BSem), 1);

  /* definition and creation of c2usbToHost_BSem */
  osSemaphoreDef(c2usbToHost_BSem);
  c2usbToHost_BSemHandle = osSemaphoreCreate(osSemaphore(c2usbToHost_BSem), 1);

  /* definition and creation of c2usbFromHost_BSem */
  osSemaphoreDef(c2usbFromHost_BSem);
  c2usbFromHost_BSemHandle = osSemaphoreCreate(osSemaphore(c2usbFromHost_BSem), 1);

  /* definition and creation of usb_BSem */
  osSemaphoreDef(usb_BSem);
  usb_BSemHandle = osSemaphoreCreate(osSemaphore(usb_BSem), 1);

  /* definition and creation of c2interpreter_BSem */
  osSemaphoreDef(c2interpreter_BSem);
  c2interpreter_BSemHandle = osSemaphoreCreate(osSemaphore(c2interpreter_BSem), 1);

  /* definition and creation of usbFromHost_BSem */
  osSemaphoreDef(usbFromHost_BSem);
  usbFromHost_BSemHandle = osSemaphoreCreate(osSemaphore(usbFromHost_BSem), 1);

  /* definition and creation of uart_BSem */
  osSemaphoreDef(uart_BSem);
  uart_BSemHandle = osSemaphoreCreate(osSemaphore(uart_BSem), 1);

  /* definition and creation of cat_BSem */
  osSemaphoreDef(cat_BSem);
  cat_BSemHandle = osSemaphoreCreate(osSemaphore(cat_BSem), 1);

  /* definition and creation of c2uartTx_BSem */
  osSemaphoreDef(c2uartTx_BSem);
  c2uartTx_BSemHandle = osSemaphoreCreate(osSemaphore(c2uartTx_BSem), 1);

  /* definition and creation of c2uartRx_BSem */
  osSemaphoreDef(c2uartRx_BSem);
  c2uartRx_BSemHandle = osSemaphoreCreate(osSemaphore(c2uartRx_BSem), 1);

  /* definition and creation of c2catTx_BSem */
  osSemaphoreDef(c2catTx_BSem);
  c2catTx_BSemHandle = osSemaphoreCreate(osSemaphore(c2catTx_BSem), 1);

  /* definition and creation of c2catRx_BSem */
  osSemaphoreDef(c2catRx_BSem);
  c2catRx_BSemHandle = osSemaphoreCreate(osSemaphore(c2catRx_BSem), 1);

  /* definition and creation of uartRx_BSem */
  osSemaphoreDef(uartRx_BSem);
  uartRx_BSemHandle = osSemaphoreCreate(osSemaphore(uartRx_BSem), 1);

  /* definition and creation of catRx_BSem */
  osSemaphoreDef(catRx_BSem);
  catRx_BSemHandle = osSemaphoreCreate(osSemaphore(catRx_BSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of defaultTimer */
  osTimerDef(defaultTimer, rtosDefaultTimerCallback);
  defaultTimerHandle = osTimerCreate(osTimer(defaultTimer), osTimerPeriodic, NULL);

  /* definition and creation of controllerTimer */
  osTimerDef(controllerTimer, rtosControllerTimerCallback);
  controllerTimerHandle = osTimerCreate(osTimer(controllerTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of controllerTask */
  osThreadDef(controllerTask, StartControllerTask, osPriorityHigh, 0, 256);
  controllerTaskHandle = osThreadCreate(osThread(controllerTask), NULL);

  /* definition and creation of usbToHostTask */
  osThreadDef(usbToHostTask, StartUsbToHostTask, osPriorityNormal, 0, 256);
  usbToHostTaskHandle = osThreadCreate(osThread(usbToHostTask), NULL);

  /* definition and creation of usbFromHostTask */
  osThreadDef(usbFromHostTask, StartUsbFromHostTask, osPriorityAboveNormal, 0, 256);
  usbFromHostTaskHandle = osThreadCreate(osThread(usbFromHostTask), NULL);

  /* definition and creation of interpreterTask */
  osThreadDef(interpreterTask, StartInterpreterTask, osPriorityNormal, 0, 512);
  interpreterTaskHandle = osThreadCreate(osThread(interpreterTask), NULL);

  /* definition and creation of uartTxTask */
  osThreadDef(uartTxTask, StartUartTxTask, osPriorityNormal, 0, 256);
  uartTxTaskHandle = osThreadCreate(osThread(uartTxTask), NULL);

  /* definition and creation of uartRxTask */
  osThreadDef(uartRxTask, StartUartRxTask, osPriorityAboveNormal, 0, 256);
  uartRxTaskHandle = osThreadCreate(osThread(uartRxTask), NULL);

  /* definition and creation of catTxTask */
  osThreadDef(catTxTask, StartCatTxTask, osPriorityNormal, 0, 256);
  catTxTaskHandle = osThreadCreate(osThread(catTxTask), NULL);

  /* definition and creation of catRxTask */
  osThreadDef(catRxTask, StartCatRxTask, osPriorityAboveNormal, 0, 256);
  catRxTaskHandle = osThreadCreate(osThread(catRxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of usbToHostQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(usbToHostQueue, 256, uint8_t);
  usbToHostQueueHandle = osMessageCreate(osMessageQ(usbToHostQueue), NULL);

  /* definition and creation of usbFromHostQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(usbFromHostQueue, 32, uint8_t);
  usbFromHostQueueHandle = osMessageCreate(osMessageQ(usbFromHostQueue), NULL);

  /* definition and creation of controllerInQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(controllerInQueue, 64, uint32_t);
  controllerInQueueHandle = osMessageCreate(osMessageQ(controllerInQueue), NULL);

  /* definition and creation of controllerOutQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(controllerOutQueue, 32, uint32_t);
  controllerOutQueueHandle = osMessageCreate(osMessageQ(controllerOutQueue), NULL);

  /* definition and creation of uartTxQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(uartTxQueue, 256, uint8_t);
  uartTxQueueHandle = osMessageCreate(osMessageQ(uartTxQueue), NULL);

  /* definition and creation of uartRxQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(uartRxQueue, 32, uint8_t);
  uartRxQueueHandle = osMessageCreate(osMessageQ(uartRxQueue), NULL);

  /* definition and creation of catTxQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(catTxQueue, 256, uint8_t);
  catTxQueueHandle = osMessageCreate(osMessageQ(catTxQueue), NULL);

  /* definition and creation of catRxQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(catRxQueue, 32, uint8_t);
  catRxQueueHandle = osMessageCreate(osMessageQ(catRxQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* add event groups */
  adcEventGroupHandle = xEventGroupCreate();
  extiEventGroupHandle = xEventGroupCreate();
  globalEventGroupHandle = xEventGroupCreate();
  spiEventGroupHandle = xEventGroupCreate();
  usbToHostEventGroupHandle = xEventGroupCreate();
  uartEventGroupHandle = xEventGroupCreate();
  catEventGroupHandle = xEventGroupCreate();

  /* add to registry */
  vQueueAddToRegistry(controllerInQueueHandle,    "Q ctrlIn");
  vQueueAddToRegistry(controllerOutQueueHandle,   "Q ctrlOut");
  vQueueAddToRegistry(usbToHostQueueHandle,       "Q uToH");
  vQueueAddToRegistry(usbFromHostQueueHandle,     "Q uFrH");
  vQueueAddToRegistry(uartTxQueueHandle,          "Q uarTx");
  vQueueAddToRegistry(uartRxQueueHandle,          "Q uarRx");
  vQueueAddToRegistry(catTxQueueHandle,           "Q catTx");
  vQueueAddToRegistry(catRxQueueHandle,           "Q catRx");

  vQueueAddToRegistry(cQin_BSemHandle,            "Rs cQin");
  vQueueAddToRegistry(cQout_BSemHandle,           "Rs cQout");
  vQueueAddToRegistry(spi1_BSemHandle,            "Rs SPI1");
  vQueueAddToRegistry(usb_BSemHandle,             "Rs USB");
  vQueueAddToRegistry(uart_BSemHandle,            "Rs UART");
  vQueueAddToRegistry(cat_BSemHandle,             "Rs CAT");
  vQueueAddToRegistry(i2c1_BSemHandle,            "Rs I2C1");

  vQueueAddToRegistry(c2default_BSemHandle,       "Wk c2dflt");
  vQueueAddToRegistry(c2interpreter_BSemHandle,   "Wk c2intr");
  vQueueAddToRegistry(c2usbFromHost_BSemHandle,   "Wk c2uFrH");
  vQueueAddToRegistry(c2usbToHost_BSemHandle,     "Wk c2uToH");
  vQueueAddToRegistry(c2uartTx_BSemHandle,        "Wk c2uarT");
  vQueueAddToRegistry(c2uartRx_BSemHandle,        "Wk c2uarR");
  vQueueAddToRegistry(c2catTx_BSemHandle,         "Wk c2catT");
  vQueueAddToRegistry(c2catRx_BSemHandle,         "Wk c2catR");

  vQueueAddToRegistry(usbFromHost_BSemHandle,     "Wk uFrH");
  vQueueAddToRegistry(uartRx_BSemHandle,          "Wk uarRx");
  vQueueAddToRegistry(catRx_BSemHandle,           "Wk catRx");

  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* defaultTaskInit() section */

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_rtos_DefaultTaskStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);


  /* defaultTaskLoop() section */
  do {
    uint32_t msgLen                       = 0UL;
    uint32_t msgAry[CONTROLLER_MSG_Q_LEN];

    /* Wait for door bell and hand-over controller out queue */
    osSemaphoreWait(c2default_BSemHandle, 250UL);

    /* Work the next complete messages */
    while (0UL < (msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Rtos_Default, 1UL))) {         // Special case of callbacks need to limit blocking time) {
      /* Decode and execute the commands when a message exists
       * (in case of callbacks the loop catches its wakeup semaphore
       * before ctrlQout is released results to request on an empty queue) */
      rtosDefaultMsgProcess(msgLen, msgAry);
    }
  } while (1);
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControllerTask */
/**
* @brief Function implementing the controllerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControllerTask */
void StartControllerTask(void const * argument)
{
  /* USER CODE BEGIN StartControllerTask */
  controllerTaskInit();

  /* Infinite loop */
  for(;;)
  {
    controllerTaskLoop();
  }
  /* USER CODE END StartControllerTask */
}

/* USER CODE BEGIN Header_StartUsbToHostTask */
/**
* @brief Function implementing the usbToHostTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbToHostTask */
void StartUsbToHostTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbToHostTask */
  usbUsbToHostTaskInit();

  /* Infinite loop */
  for (;;) {
    usbUsbToHostTaskLoop();
  }
  /* USER CODE END StartUsbToHostTask */
}

/* USER CODE BEGIN Header_StartUsbFromHostTask */
/**
* @brief Function implementing the usbFromHostTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbFromHostTask */
void StartUsbFromHostTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbFromHostTask */
  usbUsbFromHostTaskInit();

  /* Infinite loop */
  for (;;) {
    usbUsbFromHostTaskLoop();
  }
  /* USER CODE END StartUsbFromHostTask */
}

/* USER CODE BEGIN Header_StartInterpreterTask */
/**
* @brief Function implementing the interpreterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInterpreterTask */
void StartInterpreterTask(void const * argument)
{
  /* USER CODE BEGIN StartInterpreterTask */
  interpreterTaskInit();

  /* Infinite loop */
  for (;;) {
    interpreterTaskLoop();
  }
  /* USER CODE END StartInterpreterTask */
}

/* USER CODE BEGIN Header_StartUartTxTask */
/**
* @brief Function implementing the uartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTxTask */
void StartUartTxTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTxTask */
  uartTxTaskInit();

  /* Infinite loop */
  for (;;) {
    uartTxTaskLoop();
  }
  /* USER CODE END StartUartTxTask */
}

/* USER CODE BEGIN Header_StartUartRxTask */
/**
* @brief Function implementing the uartRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartRxTask */
void StartUartRxTask(void const * argument)
{
  /* USER CODE BEGIN StartUartRxTask */
  uartRxTaskInit();

  /* Infinite loop */
  for (;;) {
    uartRxTaskLoop();
  }
  /* USER CODE END StartUartRxTask */
}

/* USER CODE BEGIN Header_StartCatTxTask */
/**
* @brief Function implementing the catTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCatTxTask */
void StartCatTxTask(void const * argument)
{
  /* USER CODE BEGIN StartCatTxTask */
  catTxTaskInit();

  /* Infinite loop */
  for (;;) {
    catTxTaskLoop();
  }
  /* USER CODE END StartCatTxTask */
}

/* USER CODE BEGIN Header_StartCatRxTask */
/**
* @brief Function implementing the catRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCatRxTask */
void StartCatRxTask(void const * argument)
{
  /* USER CODE BEGIN StartCatRxTask */
  catRxTaskInit();

  /* Infinite loop */
  for (;;) {
    catRxTaskLoop();
  }
  /* USER CODE END StartCatRxTask */
}


/* rtosDefaultTimerCallback function */
void rtosDefaultTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosDefaultTimerCallback */

  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Rtos_Default, 0U, MsgDefault__CallFunc01_CyclicTimerEvent);
  controllerMsgPushToInQueue(msgLen, msgAry, 1UL);

  /* USER CODE END rtosDefaultTimerCallback */
}

/* rtosControllerTimerCallback function */
void rtosControllerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosControllerTimerCallback */
  controllerTimerCallback(argument);
  /* USER CODE END rtosControllerTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
