/*
 * task_CAT.c
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */
#if 0
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <sys/_stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "task_Controller.h"

#include "task_CAT.h"


/* Variables -----------------------------------------------------------------*/
extern osMessageQId         catTxQueueHandle;
extern osMessageQId         catRxQueueHandle;

extern osSemaphoreId        cat_BSemHandle;
extern osSemaphoreId        c2catTx_BSemHandle;
extern osSemaphoreId        c2catRx_BSemHandle;

extern EventGroupHandle_t   globalEventGroupHandle;
extern EventGroupHandle_t   catEventGroupHandle;


uint8_t                     g_catTxDmaBuf[256]                      = { 0 };
uint8_t                     g_catRxDmaBuf[256]                      = { 0 };

static uint8_t              s_catTx_enable                          = 0U;
static uint32_t             s_catTxStartTime                        = 0UL;
static osThreadId           s_catTxTaskHandle                       = 0;

static uint8_t              s_catRx_enable                          = 0U;
static uint32_t             s_catRxStartTime                        = 0UL;
static osThreadId           s_catRxTaskHandle                       = 0;


static void catInit(void)
{

}

static void catMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const CatCmds_t         cmd     = (CatCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgCat__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_catTxStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Activation flag */
      s_cat_enable = 1U;

      /* Init module */
      catInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Network_USBfromHost, 0U, MsgCat__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

#if 0
  case MsgCat__DeInitDo:
    {
      /* Init module */
      catDeInit();

      /* Deactivate flag */
      s_cat_enable = 0U;
    }
    break;
#endif

  default: { }
  }  // switch (cmd)
}


/* Tasks */

void catTxTaskInit(void)
{
  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_catTxStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void catTxTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2catTx_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Network_CAT, 1UL);                  // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    catTxMsgProcess(msgLen, msgAry);
  }
}


void catRxTaskInit(void)
{
  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_catRxStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void catRxTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2catRx_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Network_CAT, 1UL);                  // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    catRxMsgProcess(msgLen, msgAry);
  }
}
#endif
