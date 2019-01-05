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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId controllerTaskHandle;
osThreadId usbToHostTaskHandle;
osThreadId usbFromHostTaskHandle;
osThreadId adcTaskHandle;
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
void StartAdcTask(void const * argument);
void rtosDefaultTimerCallback(void const * argument);
void rtosControllerTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);

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

  /* definition and creation of adcTask */
  osThreadDef(adcTask, StartAdcTask, osPriorityIdle, 0, 256);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

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

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsbFromHostTask */
}

/* USER CODE BEGIN Header_StartAdcTask */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcTask */
void StartAdcTask(void const * argument)
{
  /* USER CODE BEGIN StartAdcTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAdcTask */
}

/* rtosDefaultTimerCallback function */
void rtosDefaultTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosDefaultTimerCallback */
  
  /* USER CODE END rtosDefaultTimerCallback */
}

/* rtosControllerTimerCallback function */
void rtosControllerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosControllerTimerCallback */
  
  /* USER CODE END rtosControllerTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
