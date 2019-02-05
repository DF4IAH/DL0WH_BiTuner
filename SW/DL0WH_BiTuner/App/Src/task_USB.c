/*
 * task_USB.c
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

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
#include "usb_device.h"
#include <usbd_cdc_if.h>
#include "task_Controller.h"

#include "task_USB.h"


/* Variables -----------------------------------------------------------------*/
extern osMessageQId         usbFromHostQueueHandle;
extern osMessageQId         usbToHostQueueHandle;

extern osSemaphoreId        usb_BSemHandle;
extern osSemaphoreId        c2usbToHost_BSemHandle;
extern osSemaphoreId        c2usbFromHost_BSemHandle;
extern osSemaphoreId        usbFromHost_BSemHandle;

extern EventGroupHandle_t   globalEventGroupHandle;
extern EventGroupHandle_t   usbToHostEventGroupHandle;


/* UsbToHost */

static uint8_t              s_usbUsbToHost_enable                   = 0U;
static uint32_t             s_usbUsbToHostStartTime                 = 0UL;
static osThreadId           s_usbUsbToHostTaskHandle                = 0;


/* UsbFromHost */

static uint8_t              s_usbUsbFromHost_enable                 = 0U;
static uint32_t             s_usbUsbFromHostStartTime               = 0UL;
static osThreadId           s_usbUsbFromHostTaskHandle              = 0;

volatile uint8_t            v_usbUsbFromHostISRBuf[64]              = { 0U };
volatile uint32_t           v_usbUsbFromHostISRBufLen               = 0UL;



/* IRQ */

void usbFromHostFromIRQ(const uint8_t* buf, uint32_t len)
{
  if (buf && len && !v_usbUsbFromHostISRBufLen) {
    BaseType_t  lMaxLen = sizeof(v_usbUsbFromHostISRBuf) - 1;
    BaseType_t  lLen = len;

    if (lLen > lMaxLen) {
      lLen = lMaxLen;
    }
    memcpy((void*)v_usbUsbFromHostISRBuf, (const void*)buf, lLen);
    v_usbUsbFromHostISRBuf[lLen] = 0U;
    __ISB();
    v_usbUsbFromHostISRBufLen = lLen;
  }
}


const uint8_t usbToHost_MaxWaitQueueMs = 100U;
static void usbToHost(const uint8_t* buf, uint32_t len)
{
	if (buf && len) {
		while (len--) {
			osMessagePut(usbToHostQueueHandle, (uint32_t) *(buf++), usbToHost_MaxWaitQueueMs);
		}
		osMessagePut(usbToHostQueueHandle, 0UL, usbToHost_MaxWaitQueueMs);
	}
}

static void usbToHostWait(const uint8_t* buf, uint32_t len)
{
  /* Sanity checks */
  if (!buf || !len) {
    return;
  }

  EventBits_t eb = xEventGroupWaitBits(usbToHostEventGroupHandle,
      USB_TO_HOST_EG__BUF_EMPTY,
      USB_TO_HOST_EG__BUF_EMPTY,
      0, portMAX_DELAY);
  if (eb & USB_TO_HOST_EG__BUF_EMPTY) {
    usbToHost(buf, len);
  }
}

void usbLogLen(const char* str, int len)
{
  /* Sanity checks */
  if (!str || !len) {
    return;
  }

  if (s_usbUsbToHost_enable) {
    osSemaphoreWait(usb_BSemHandle, 0UL);
    usbToHostWait((uint8_t*)str, len);
    osSemaphoreRelease(usb_BSemHandle);
  }
}

inline
void usbLog(const char* str)
{
  /* Sanity check */
  if (!str) {
    return;
  }

  usbLogLen(str, strlen(str));
}


const char usbClrScrBuf[4] = { 0x0c, 0x0d, 0x0a, 0 };

/* USB-to-Host */

void usbStartUsbToHostTask(void const * argument)
{
  static uint8_t  buf[48] = { 0U };
  static uint32_t bufCtr  = 0UL;
  uint8_t         inChr   = 0U;

  /* Clear queue */
  while (xQueueReceive(usbToHostQueueHandle, &inChr, 0) == pdPASS) {
  }
  xEventGroupSetBits(usbToHostEventGroupHandle, USB_TO_HOST_EG__BUF_EMPTY);

  /* Init connection with dummy data */
  const uint8_t MaxCnt = 30U;
  for (uint8_t cnt = MaxCnt; cnt; cnt--) {
    uint8_t res = CDC_Transmit_FS((uint8_t*) usbClrScrBuf, 3);
    if (res == USBD_BUSY) {
      cnt = MaxCnt;
      osDelay(500UL);

    } else {
      osDelay(10UL);
    }
  }
  osDelay(250UL);


  /* TaskLoop */
  for (;;) {
    BaseType_t xStatus;

    do {
      /* Take next character from the queue - at least update each 100ms */
      inChr = 0U;
      xStatus = xQueueReceive(usbToHostQueueHandle, &inChr, 25UL / portTICK_PERIOD_MS);
      if ((pdPASS == xStatus) && inChr) {
        /* Group-Bit for empty queue cleared */
        xEventGroupClearBits(usbToHostEventGroupHandle, USB_TO_HOST_EG__BUF_EMPTY);

        buf[bufCtr++] = inChr;
      }

      /* Flush when 0 or when buffer is full */
      if (!inChr || (bufCtr >= (sizeof(buf) - 1))) {
        uint32_t retryCnt = 0UL;

        /* Do not send empty buffer */
        if (!bufCtr) {
          break;
        }

        buf[bufCtr] = 0U;
        for (retryCnt = 25UL; retryCnt; --retryCnt) {
          /* Transmit to USB host */
          uint8_t ucRetVal = CDC_Transmit_FS(buf, bufCtr);
          if (USBD_BUSY != ucRetVal) {
            /* Let the backend transport the DCD data */
            osDelay(5UL);

            /* Data accepted for transmission */
            bufCtr = 0UL;
            buf[0] = 0U;

            /* Group-Bit for empty queue set */
            xEventGroupSetBits(usbToHostEventGroupHandle, USB_TO_HOST_EG__BUF_EMPTY);

            break;

          } else {
            /* USB EP busy - try again */
            /* Delay for next USB packet to come and go */
            osDelay(2UL);
          }
        }

        if (!retryCnt) {
          /* USB EP still busy - drop whole buffer content */
          bufCtr = 0UL;
        }
      }
    } while (!xQueueIsQueueEmptyFromISR(usbToHostQueueHandle));
  }
}


static void usbUsbToHostInit(void)
{
#if 0
  /* Activate USB communication */
  HFTcore_USB_DEVICE_Init();
#endif

  osThreadDef(usbUsbToHostTask, usbStartUsbToHostTask, osPriorityHigh, 0, 128);
  s_usbUsbToHostTaskHandle = osThreadCreate(osThread(usbUsbToHostTask), NULL);
}

static void usbUsbToHostDeInit(void)
{
  osThreadTerminate(s_usbUsbToHostTaskHandle);
  s_usbUsbToHostTaskHandle = 0;
}


static void usbUsbToHostMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const usbMsgUsbCmds_t   cmd     = (usbMsgUsbCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgUsb__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_usbUsbToHostStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Activation flag */
      s_usbUsbToHost_enable = 1U;

      /* Init module */
      usbUsbToHostInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Network_USBtoHost, 0U, MsgUsb__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, 10UL);
    }
    break;

  case MsgUsb__DeInitDo:
    {
      /* DeInit module */
      usbUsbToHostDeInit();

      /* Deactivate flag */
      s_usbUsbToHost_enable = 0U;
    }
    break;

  default: { }
  }  // switch (cmd)
}


void usbUsbToHostTaskInit(void)
{
  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_usbUsbToHostStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void usbUsbToHostTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2usbToHost_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Network_USBtoHost, 1UL);           // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    usbUsbToHostMsgProcess(msgLen, msgAry);
  }
}


/* USB-from-Host */

uint32_t usbPullFromOutQueue(uint8_t* msgAry, uint32_t waitMs)
{
  uint32_t len = 0UL;

  /* Get semaphore to queue out */
  osSemaphoreWait(usbFromHost_BSemHandle, waitMs);

  do {
    osEvent ev = osMessageGet(usbFromHostQueueHandle, waitMs);
    if (ev.status == osEventMessage) {
      msgAry[len++] = ev.value.v;

    } else if (len) {
      if (!msgAry[len - 1]) {
        /* Strip off last NULL character */
        --len;
      }
      break;

    } else {
      break;
    }
  } while (1);

  /* Return semaphore */
  osSemaphoreRelease(usbFromHost_BSemHandle);

  return len;
}


void usbStartUsbFromHostTask(void const * argument)
{
  const uint8_t nulBuf[1]   = { 0U };

  /* TaskLoop */
  for (;;) {
    if (v_usbUsbFromHostISRBufLen) {
      BaseType_t higherPriorityTaskWoken = pdFALSE;

      /* Interrupt disabled section */
      {
        taskDISABLE_INTERRUPTS();

        /* USB OUT EP from host put data into the buffer */
        volatile uint8_t* bufPtr = v_usbUsbFromHostISRBuf;
        for (uint32_t idx = 0UL; idx < v_usbUsbFromHostISRBufLen; ++idx, ++bufPtr) {
          xQueueSendToBackFromISR(usbFromHostQueueHandle, (uint8_t*)bufPtr, &higherPriorityTaskWoken);
        }
        xQueueSendToBackFromISR(usbFromHostQueueHandle, nulBuf, &higherPriorityTaskWoken);

        v_usbUsbFromHostISRBufLen = 0UL;
        __DMB();
        memset((char*) v_usbUsbFromHostISRBuf, 0, sizeof(v_usbUsbFromHostISRBuf));

        taskENABLE_INTERRUPTS();
      }

    } else {
      /* Delay for the next attempt */
      osDelay(25UL);
    }
  }
}


static void usbUsbFromHostInit(void)
{
#if 0
  /* Activate USB communication */
  HFTcore_USB_DEVICE_Init();
#endif

  osThreadDef(usbUsbFromHostTask, usbStartUsbFromHostTask, osPriorityHigh, 0, 128);
  s_usbUsbFromHostTaskHandle = osThreadCreate(osThread(usbUsbFromHostTask), NULL);
}

static void usbUsbFromHostDeInit(void)
{
  osThreadTerminate(s_usbUsbFromHostTaskHandle);
  s_usbUsbFromHostTaskHandle = 0;
}


static void usbUsbFromHostMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const usbMsgUsbCmds_t   cmd     = (usbMsgUsbCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgUsb__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_usbUsbFromHostStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Activation flag */
      s_usbUsbFromHost_enable = 1U;

      /* Init module */
      usbUsbFromHostInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Network_USBfromHost, 0U, MsgUsb__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, 10UL);
    }
    break;

  case MsgUsb__DeInitDo:
    {
      /* Init module */
      usbUsbFromHostDeInit();

      /* Deactivate flag */
      s_usbUsbFromHost_enable = 0U;
    }
    break;

  default: { }
  }  // switch (cmd)
}


void usbUsbFromHostTaskInit(void)
{
  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_usbUsbFromHostStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void usbUsbFromHostTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2usbFromHost_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Network_USBfromHost, 1UL);           // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    usbUsbFromHostMsgProcess(msgLen, msgAry);
  }
}
