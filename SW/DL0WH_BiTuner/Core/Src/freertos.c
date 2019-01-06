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
#include "device_adc.h"
#include "task_Controller.h"
#include "task_USB.h"

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
extern float                          g_adc_refint_val;
extern float                          g_adc_vref_mv;
extern float                          g_adc_bat_mv;
extern float                          g_adc_temp_deg;
extern float                          g_adc_fwd_mv;
extern float                          g_adc_rev_mv;
extern float                          g_adc_vdiode_mv;
extern float                          g_swr;

EventGroupHandle_t                    adcEventGroupHandle;
EventGroupHandle_t                    extiEventGroupHandle;
EventGroupHandle_t                    globalEventGroupHandle;
EventGroupHandle_t                    spiEventGroupHandle;
EventGroupHandle_t                    usbToHostEventGroupHandle;

static uint8_t                        s_rtos_DefaultTask_adc_enable;
static uint32_t                       s_rtos_DefaultTaskStartTime;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId controllerTaskHandle;
osThreadId usbToHostTaskHandle;
osThreadId usbFromHostTaskHandle;
osMessageQId usbToHostQueueHandle;
osMessageQId usbFromHostQueueHandle;
osMessageQId controllerInQueueHandle;
osMessageQId controllerOutQueueHandle;
osTimerId defaultTimerHandle;
osTimerId controllerTimerHandle;
osSemaphoreId c2Default_BSemHandle;
osSemaphoreId i2c1_BSemHandle;
osSemaphoreId spi1_BSemHandle;
osSemaphoreId cQin_BSemHandle;
osSemaphoreId cQout_BSemHandle;
osSemaphoreId c2usbToHost_BSemHandle;
osSemaphoreId c2usbFromHost_BSemHandle;
osSemaphoreId usb_BSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControllerTask(void const * argument);
void StartUsbToHostTask(void const * argument);
void StartUsbFromHostTask(void const * argument);
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

}

__weak unsigned long getRunTimeCounterValue(void)
{
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
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
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
}


/* Local functions */

static void rtosDefaultInit(void)
{
  /* Power switch settings */
  mainPowerSwitchInit();

  /* Enable ADC access */
  s_rtos_DefaultTask_adc_enable = 1U;
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

  /* MCU GPIO/Alternate-Functions set-up */
  case MsgDefault__SetVar01_IOs:
    {
      // TODO: implementation
    }
    break;

  /* MCU clocking set-up */
  case MsgDefault__SetVar02_Clocking:
    {
      const DefaultMcuClocking_t mcuClocking = (DefaultMcuClocking_t) (0xffUL & (msgAry[msgIdx++] >> 24U));

      switch (mcuClocking) {
      case DefaultMcuClocking_16MHz_MSI:
        {
          HFT_SystemClock_Config(SYSCLK_CONFIG_16MHz_MSI);
        }
        break;

      default: { }
      }
    }
    break;

  /* ADC1 single conversion */
  case MsgDefault__CallFunc01_MCU_ADC1:
    {
      /* Do ADC1 conversion and logging of ADC1 data */
      if (s_rtos_DefaultTask_adc_enable) {
        int   dbgLen;
        char  dbgBuf[128];

        /* Start chain of ADC1 conversions */
        adcStartConv(ADC_ADC1_TEMP_DEG);

        const uint32_t regMask = EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP;
        BaseType_t regBits = xEventGroupWaitBits(adcEventGroupHandle, regMask, regMask, pdTRUE, 100 / portTICK_PERIOD_MS);
        if ((regBits & regMask) == regMask) {
          /* All channels of ADC1 are complete */
          float     l_adc_refint_val    = adcGetVal(ADC_ADC1_REFINT_VAL);
          float     l_adc_vref_mv       = adcGetVal(ADC_ADC1_VREF_MV);
          float     l_adc_bat_mv        = adcGetVal(ADC_ADC1_BAT_MV);
          float     l_adc_temp_deg      = adcGetVal(ADC_ADC1_TEMP_DEG);

          /* Push to global vars */
          {
            __disable_irq();

            g_adc_refint_val  = l_adc_refint_val;
            g_adc_vref_mv     = l_adc_vref_mv;
            g_adc_bat_mv      = l_adc_bat_mv;
            g_adc_temp_deg    = l_adc_temp_deg;

            __enable_irq();
          }

          int32_t   l_adc_temp_deg_i    = 0L;
          uint32_t  l_adc_temp_deg_f100 = 0UL;

          mainCalcFloat2IntFrac(l_adc_temp_deg, 2, &l_adc_temp_deg_i, &l_adc_temp_deg_f100);

          dbgLen = sprintf(dbgBuf, "ADC1: refint_val = %4d, Vref = %4d mV, Bat = %4d mV, Temp = %+3ld.%02luC\r\n",
              (int16_t) (l_adc_refint_val + 0.5f),
              (int16_t) (l_adc_vref_mv    + 0.5f),
              (int16_t) (l_adc_bat_mv     + 0.5f),
              l_adc_temp_deg_i, l_adc_temp_deg_f100);
          usbLogLen(dbgBuf, dbgLen);
        }
      }
    }
    break;

  /* ADC3 Vdiode single conversion */
  case MsgDefault__CallFunc02_MCU_ADC3_VDIODE:
    {
      /* Do ADC3 Vdiode conversion and logging of ADC3 data */
      if (s_rtos_DefaultTask_adc_enable) {
        int   dbgLen;
        char  dbgBuf[128];

        adcStartConv(ADC_ADC3_IN3_VDIODE_MV);

        const uint32_t regMask = EG_ADC3__CONV_AVAIL_VDIODE;
        BaseType_t regBits = xEventGroupWaitBits(adcEventGroupHandle, regMask, regMask, pdTRUE, 100 / portTICK_PERIOD_MS);
        if ((regBits & regMask) == regMask) {
          /* All channels of ADC3 are complete */
          float l_adc_vdiode_mv = adcGetVal(ADC_ADC3_IN3_VDIODE_MV);

          /* Push to global var */
          {
            __disable_irq();
            g_adc_vdiode_mv = l_adc_vdiode_mv;
            __enable_irq();
          }

          dbgLen = sprintf(dbgBuf, "ADC3: Vdiode = %4d mV\r\n", (int16_t) (l_adc_vdiode_mv + 0.5f));
          usbLogLen(dbgBuf, dbgLen);
        }
      }
    }
    break;

  /* ADC2 FWD single conversion */
  case MsgDefault__CallFunc03_MCU_ADC2_FWD:
    {
      /* Do ADC2 FWD conversion and logging of ADC2 data */
      if (s_rtos_DefaultTask_adc_enable) {
        int   dbgLen;
        char  dbgBuf[128];

        /* Switch MUX to FWD input */
        HAL_GPIO_WritePin(GPIO_SWR_SEL_REV_GPIO_Port, GPIO_SWR_SEL_REV_Pin, GPIO_PIN_RESET);
        __DMB();
        HAL_GPIO_WritePin(GPIO_SWR_SEL_FWD_GPIO_Port, GPIO_SWR_SEL_FWD_Pin, GPIO_PIN_SET);

        adcStartConv(ADC_ADC2_IN1_FWDREV_MV);

        const uint32_t regMask = EG_ADC2__CONV_AVAIL_FWDREV;
        BaseType_t regBits = xEventGroupWaitBits(adcEventGroupHandle, regMask, regMask, pdTRUE, 100 / portTICK_PERIOD_MS);
        if ((regBits & regMask) == regMask) {
          float l_adc_vdiode_mv;

          /* Get global var */
          {
            __disable_irq();
            l_adc_vdiode_mv = g_adc_vdiode_mv;
            __enable_irq();
          }

          /* Get the linearized voltage */
          float l_adc_fwd_mv = calc_fwdRev_mv(adcGetVal(ADC_ADC2_IN1_FWDREV_MV), l_adc_vdiode_mv);

          /* Push to global var */
          {
            __disable_irq();
            g_adc_fwd_mv = l_adc_fwd_mv;
            __enable_irq();
          }

          dbgLen = sprintf(dbgBuf, "ADC2: FWD = %5d mV\r\n", (int16_t) (l_adc_fwd_mv + 0.5f));
          usbLogLen(dbgBuf, dbgLen);
        }
      }
    }
    break;

  /* ADC2 REV single conversion */
  case MsgDefault__CallFunc04_MCU_ADC2_REV:
    {
      /* Do ADC2 REV conversion and logging of ADC2 data */
      if (s_rtos_DefaultTask_adc_enable) {
        int   dbgLen;
        char  dbgBuf[128];

        /* Switch MUX to FWD input */
        HAL_GPIO_WritePin(GPIO_SWR_SEL_FWD_GPIO_Port, GPIO_SWR_SEL_FWD_Pin, GPIO_PIN_RESET);
        __DMB();
        HAL_GPIO_WritePin(GPIO_SWR_SEL_REV_GPIO_Port, GPIO_SWR_SEL_REV_Pin, GPIO_PIN_SET);

        adcStartConv(ADC_ADC2_IN1_FWDREV_MV);

        const uint32_t regMask = EG_ADC2__CONV_AVAIL_FWDREV;
        BaseType_t regBits = xEventGroupWaitBits(adcEventGroupHandle, regMask, regMask, pdTRUE, 100 / portTICK_PERIOD_MS);
        if ((regBits & regMask) == regMask) {
          float l_adc_fwd_mv;
          float l_adc_vdiode_mv;

          /* Get global vars */
          {
            __disable_irq();
            l_adc_fwd_mv    = g_adc_fwd_mv;
            l_adc_vdiode_mv = g_adc_vdiode_mv;
            __enable_irq();
          }

          /* Get the linearized voltage and (V)SWR */
          float l_adc_rev_mv = calc_fwdRev_mv(adcGetVal(ADC_ADC2_IN1_FWDREV_MV), l_adc_vdiode_mv);
          float l_swr        = calc_swr(l_adc_fwd_mv, l_adc_rev_mv);

          /* Push to global vars */
          {
            __disable_irq();
            g_adc_rev_mv  = l_adc_rev_mv;
            g_swr         = l_swr;
            __enable_irq();
          }

          int32_t   l_swr_i    = 0L;
          uint32_t  l_swr_f100 = 0UL;

          mainCalcFloat2IntFrac(l_swr, 3, &l_swr_i, &l_swr_f100);

          dbgLen = sprintf(dbgBuf, "ADC2: REV = %5d mV, SWR = %+3ld.%03lu\r\n",
              (int16_t) (l_adc_rev_mv + 0.5f),
              l_swr_i, l_swr_f100);
          usbLogLen(dbgBuf, dbgLen);
        }
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
  /* definition and creation of c2Default_BSem */
  osSemaphoreDef(c2Default_BSem);
  c2Default_BSemHandle = osSemaphoreCreate(osSemaphore(c2Default_BSem), 1);

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
  osThreadDef(controllerTask, StartControllerTask, osPriorityIdle, 0, 256);
  controllerTaskHandle = osThreadCreate(osThread(controllerTask), NULL);

  /* definition and creation of usbToHostTask */
  osThreadDef(usbToHostTask, StartUsbToHostTask, osPriorityIdle, 0, 128);
  usbToHostTaskHandle = osThreadCreate(osThread(usbToHostTask), NULL);

  /* definition and creation of usbFromHostTask */
  osThreadDef(usbFromHostTask, StartUsbFromHostTask, osPriorityIdle, 0, 128);
  usbFromHostTaskHandle = osThreadCreate(osThread(usbFromHostTask), NULL);

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
  osMessageQDef(controllerInQueue, 8, uint32_t);
  controllerInQueueHandle = osMessageCreate(osMessageQ(controllerInQueue), NULL);

  /* definition and creation of controllerOutQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(controllerOutQueue, 32, uint32_t);
  controllerOutQueueHandle = osMessageCreate(osMessageQ(controllerOutQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* add event groups */
  adcEventGroupHandle = xEventGroupCreate();
  extiEventGroupHandle = xEventGroupCreate();
  globalEventGroupHandle = xEventGroupCreate();
  spiEventGroupHandle = xEventGroupCreate();
  usbToHostEventGroupHandle = xEventGroupCreate();

  /* add to registry */
  vQueueAddToRegistry(i2c1_BSemHandle,          "Resc I2C1 BSem");
  vQueueAddToRegistry(spi1_BSemHandle,          "Resc SPI1 BSem");
  vQueueAddToRegistry(cQin_BSemHandle,          "Resc cQin BSem");
  vQueueAddToRegistry(cQout_BSemHandle,         "Resc cQout BSem");
  vQueueAddToRegistry(usb_BSemHandle,           "Resc USB BSem");

  vQueueAddToRegistry(c2Default_BSemHandle,     "Wake c2Default BSem");
  vQueueAddToRegistry(c2usbFromHost_BSemHandle, "Wake c2usbFromHost BSem");
  vQueueAddToRegistry(c2usbToHost_BSemHandle,   "Wake c2usbToHost BSem");
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
  {
    s_rtos_DefaultTask_adc_enable = 0U;
    s_rtos_DefaultTaskStartTime   = 0UL;
  }

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_rtos_DefaultTaskStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);

  do {
    uint32_t msgLen                       = 0UL;
    uint32_t msgAry[CONTROLLER_MSG_Q_LEN];

    /* Wait for door bell and hand-over controller out queue */
    {
      osSemaphoreWait(c2Default_BSemHandle, osWaitForever);
      msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Rtos_Default, 1UL);                // Special case of callbacks need to limit blocking time
    }

    /* Decode and execute the commands when a message exists
     * (in case of callbacks the loop catches its wakeup semaphore
     * before ctrlQout is released results to request on an empty queue) */
    if (msgLen) {
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

/* rtosDefaultTimerCallback function */
void rtosDefaultTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosDefaultTimerCallback */
  // TODO: code here

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
