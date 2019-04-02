/*
 * task_CAT.c
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <sys/_stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "usart.h"

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


volatile uint8_t            g_catTxDmaBuf[256]                      = { 0U };

volatile uint8_t            g_catRxDmaBuf[16]                       = { 0U };
uint32_t                    g_catRxDmaBufLast                       = 0UL;
uint32_t                    g_catRxDmaBufIdx                        = 0UL;

static uint8_t              s_catTx_enable                          = 0U;
static uint32_t             s_catTxStartTime                        = 0UL;
static osThreadId           s_catTxPutterTaskHandle                 = 0;

static uint8_t              s_catRx_enable                          = 0U;
static uint32_t             s_catRxStartTime                        = 0UL;
static osThreadId           s_catRxGetterTaskHandle                 = 0;



/* HAL callbacks for the CAT */

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_CAT_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* Set flags: DMA complete */
  xEventGroupSetBitsFromISR(  catEventGroupHandle, CAT_EG__DMA_TX_END, &xHigherPriorityTaskWoken);
  xEventGroupClearBitsFromISR(catEventGroupHandle, CAT_EG__DMA_TX_RUN);

  /* Set flag: buffer empty */
  xEventGroupSetBitsFromISR(  catEventGroupHandle, CAT_EG__TX_BUF_EMPTY, &xHigherPriorityTaskWoken);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_CAT_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* Set flags: DMA complete */
  xEventGroupClearBitsFromISR(catEventGroupHandle, CAT_EG__DMA_RX_RUN);
  xEventGroupSetBitsFromISR(  catEventGroupHandle, CAT_EG__DMA_RX_END, &xHigherPriorityTaskWoken);
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_CAT_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  return;

#if 0
  Error_Handler();
#endif
}


/* UART HAL Init */

static void catUartHalInit(void)
{
  _Bool already = 0;

  /* Block when already called */
  {
    taskENTER_CRITICAL();
    if (s_catTx_enable || s_catRx_enable) {
      already = true;
    }
    taskEXIT_CRITICAL();

    if (already) {
      return;
    }
  }

  /* Init UART */
  {
    huart4.Init.BaudRate                = 9600;
    huart4.Init.WordLength              = UART_WORDLENGTH_8B;
    huart4.Init.StopBits                = UART_STOPBITS_1;
    huart4.Init.Parity                  = UART_PARITY_NONE;
    huart4.Init.HwFlowCtl               = UART_HWCONTROL_NONE;
    huart4.Init.Mode                    = UART_MODE_TX_RX;
    huart4.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;

    if(HAL_UART_DeInit(&huart4) != HAL_OK)
    {
      Error_Handler();
    }
    if(HAL_UART_Init(&huart4) != HAL_OK)
    {
      Error_Handler();
    }
  }

}


/* Messaging */

uint32_t catRxPullFromQueue(uint8_t* msgAry, uint32_t maxLen, uint32_t waitMs)
{
  const uint32_t maxLenM1 = maxLen - 1;
  uint32_t len = 0UL;

  /* Sanity checks */
  if (!msgAry || !maxLen) {
    return 0UL;
  }

  /* Get semaphore to queue out */
  osSemaphoreWait(cat_BSemHandle, waitMs);

  do {
    osEvent ev = osMessageGet(catRxQueueHandle, waitMs);
    if (ev.status == osEventMessage) {
      msgAry[len++] = ev.value.v;

    } else {
      break;
    }
  } while (len < maxLenM1);

  while ((!msgAry[len - 1]) && (len > 1UL)) {
    /* Strip off last NULL character(s) */
    --len;
  }

  /* Return semaphore */
  osSemaphoreRelease(cat_BSemHandle);

  return len;
}


/* UART TX */

static void catTxStartDma(const uint8_t* cmdBuf, uint8_t cmdLen)
{
  /* Sanity checks */
  if (!cmdBuf || !cmdLen) {
    return;
  }

  /* When TX is running wait for the end of the previous transfer */
  if (CAT_EG__DMA_TX_RUN & xEventGroupGetBits(catEventGroupHandle)) {
    /* Block until end of transmission - at max. 1 sec */
    xEventGroupWaitBits(catEventGroupHandle,
        CAT_EG__DMA_TX_END,
        CAT_EG__DMA_TX_END,
        0, 1000 / portTICK_PERIOD_MS);
  }

  /* Copy to DMA TX buffer */
  memcpy((uint8_t*)g_catTxDmaBuf, cmdBuf, cmdLen);
  memset((uint8_t*)g_catTxDmaBuf + cmdLen, 0, sizeof(g_catTxDmaBuf) - cmdLen);

  /* Re-set flag for TX */
  xEventGroupSetBits(catEventGroupHandle, CAT_EG__DMA_TX_RUN);

  /* Start transmission */
  if (HAL_OK != HAL_UART_Transmit_DMA(&huart4, (uint8_t*)g_catTxDmaBuf, cmdLen))
  {
    Error_Handler();
  }
}

void catTxPutterTask(void const * argument)
{
  const uint8_t   maxWaitMs   = 25U;
  uint8_t         buf[256];
  const uint32_t  bufLenM1    = sizeof(buf) - 1;

#if 0
  /* Clear queue */
  while (osMessageGet(catTxQueueHandle, 1UL).status == osEventMessage) {
  }
  xEventGroupSetBits(catEventGroupHandle, CAT_EG__TX_BUF_EMPTY);
#endif

  /* TaskLoop */
  for (;;) {
    uint8_t len = 0U;

    uint8_t* bufPtr = buf;
    for (uint32_t idx = 0UL; idx < bufLenM1; idx++) {
      osEvent ev = osMessageGet(catTxQueueHandle, maxWaitMs);
      if (ev.status == osEventMessage) {
        if (ev.value.v) {
          *(bufPtr++) = (uint8_t) ev.value.v;
        }

      } else {
        len = bufPtr - buf;
        break;
      }
    }
    buf[len] = 0U;

    /* Start DMA TX engine */
    if (len) {
      catTxStartDma(buf, len);

    } else {
      /* Delay for the next attempt */
      osDelay(25UL);
    }
  }
}

static void catTxInit(void)
{
  /* Init UART HAL */
  catUartHalInit();

  /* Start putter thread */
  osThreadDef(catTxPutterTask, catTxPutterTask, osPriorityHigh, 0, 256);
  s_catTxPutterTaskHandle = osThreadCreate(osThread(catTxPutterTask), NULL);
}

static void catTxMsgProcess(const uint32_t* msgAry, uint32_t msgLen)
{
  if (msgLen >= 1UL) {
    uint32_t            msgIdx  = 0UL;
    const uint32_t      hdr     = msgAry[msgIdx++];
    const CatTxCmds_t   cmd     = (CatTxCmds_t) (0xffUL & hdr);

    switch (cmd) {
    case MsgCatTx__InitDo:
      if (msgLen == 2UL) {
        /* Start at defined point of time */
        const uint32_t delayMs = msgAry[msgIdx++];
        if (delayMs) {
          uint32_t  previousWakeTime = s_catTxStartTime;
          osDelayUntil(&previousWakeTime, delayMs);
        }

        /* Init module */
        catTxInit();

        /* Activation flag */
        s_catTx_enable = 1U;

        /* Return Init confirmation */
        uint32_t cmdBack[1];
        cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Network_CatTx, 0U, MsgCatTx__InitDone);
        controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, 10UL);
      }
      break;

    default: { }
    }  // switch (cmd)
  }
}

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
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Network_CatTx, 1UL);                 // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    catTxMsgProcess(msgAry, msgLen);
  }
}


/* CAT-UART RX */

static void catRxStartDma(void)
{
  const uint16_t dmaBufSize = sizeof(g_catRxDmaBuf);

  /* Clear DMA buffer */
  memset((char*) g_catRxDmaBuf, 0, dmaBufSize);

  /* Reset working indexes */
  g_catRxDmaBufLast = g_catRxDmaBufIdx = 0UL;

  xEventGroupClearBits(catEventGroupHandle, CAT_EG__DMA_RX_END);

  /* Start RX DMA after aborting the previous one */
  if (HAL_UART_Receive_DMA(&huart4, (uint8_t*)g_catRxDmaBuf, dmaBufSize) != HAL_OK)
  {
    //Error_Handler();
  }

  /* Set RX running flag */
  xEventGroupSetBits(catEventGroupHandle, CAT_EG__DMA_RX_RUN);
}


void catRxGetterTask(void const * argument)
{
  const uint16_t dmaBufSize = sizeof(g_catRxDmaBuf);
  const uint8_t nulBuf[1]   = { 0U };
  const uint8_t maxWaitMs   = 25UL;

  /* TaskLoop */
  for (;;) {
    /* Find last written byte */
    g_catRxDmaBufIdx = g_catRxDmaBufLast + strnlen((char*)g_catRxDmaBuf + g_catRxDmaBufLast, dmaBufSize - g_catRxDmaBufLast);

    /* Send new character in RX buffer to the queue */
    if (g_catRxDmaBufIdx > g_catRxDmaBufLast) {
      /* From UART data into the buffer */
      const uint32_t    l_catRxDmaBufIdx  = g_catRxDmaBufIdx;
      volatile uint8_t* bufPtr            = g_catRxDmaBuf + g_catRxDmaBufLast;

      for (int32_t idx = g_catRxDmaBufLast + 1L; idx <= l_catRxDmaBufIdx; ++idx, ++bufPtr) {
        xQueueSendToBack(catRxQueueHandle, (uint8_t*)bufPtr, maxWaitMs);
      }
      xQueueSendToBack(catRxQueueHandle, nulBuf, maxWaitMs);

      g_catRxDmaBufLast = l_catRxDmaBufIdx;

      /* Restart DMA if transfer has finished */
      if (CAT_EG__DMA_RX_END & xEventGroupGetBits(catEventGroupHandle)) {
        /* Reactivate CAT-UART RX DMA transfer */
        catRxStartDma();
      }

    } else {
      /* Restart DMA if transfer has finished */
      if (CAT_EG__DMA_RX_END & xEventGroupGetBits(catEventGroupHandle)) {
        /* Reactivate UART RX DMA transfer */
        catRxStartDma();
      }

      /* Delay for the next attempt */
      osDelay(25UL);
    }
  }
}

static void catRxInit(void)
{
  /* Init CAT-UART HAL */
  catUartHalInit();

  /* Start getter thread */
  osThreadDef(catRxGetterTask, catRxGetterTask, osPriorityHigh, 0, 128);
  s_catRxGetterTaskHandle = osThreadCreate(osThread(catRxGetterTask), NULL);

  /* Activate CAT-UART RX DMA transfer */
  if (HAL_UART_Receive_DMA(&huart4, (uint8_t*) g_catRxDmaBuf, sizeof(g_catRxDmaBuf) - 1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void catRxMsgProcess(const uint32_t* msgAry, uint32_t msgLen)
{
  if (msgLen >= 1UL) {
    uint32_t            msgIdx  = 0UL;
    const uint32_t      hdr     = msgAry[msgIdx++];
    const CatRxCmds_t   cmd     = (CatRxCmds_t) (0xffUL & hdr);

    switch (cmd) {
    case MsgCatRx__InitDo:
      if (msgLen == 2UL) {
        /* Start at defined point of time */
        const uint32_t delayMs = msgAry[msgIdx++];
        if (delayMs) {
          uint32_t  previousWakeTime = s_catRxStartTime;
          osDelayUntil(&previousWakeTime, delayMs);
        }

        /* Init module */
        catRxInit();

        /* Activation flag */
        s_catRx_enable = 1U;

        /* Return Init confirmation */
        uint32_t cmdBack[1];
        cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Network_CatRx, 0U, MsgCatRx__InitDone);
        controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, 10UL);
      }
      break;

    default: { }
    }  // switch (cmd)
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
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Network_CatRx, 1UL);                 // Special case of callbacks needed to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    catRxMsgProcess(msgAry, msgLen);
  }
}
