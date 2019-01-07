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
extern osSemaphoreId        c2usbToHost_BSemHandle;
extern osSemaphoreId        c2usbFromHost_BSemHandle;
extern osSemaphoreId        usb_BSemHandle;

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



const uint8_t usbToHost_MaxWaitQueueMs = 100U;
void usbToHost(const uint8_t* buf, uint32_t len)
{
	if (buf && len) {
		while (len--) {
			osMessagePut(usbToHostQueueHandle, *(buf++), usbToHost_MaxWaitQueueMs);
		}
		osMessagePut(usbToHostQueueHandle, 0, usbToHost_MaxWaitQueueMs);
	}
}

const uint16_t usbToHostWait_MaxWaitSemMs = 500U;
void usbToHostWait(const uint8_t* buf, uint32_t len)
{
  EventBits_t eb = xEventGroupWaitBits(usbToHostEventGroupHandle, USB_TO_HOST_EG__BUF_EMPTY, 0UL, 0, usbToHostWait_MaxWaitSemMs);
  if (eb & USB_TO_HOST_EG__BUF_EMPTY) {
    usbToHost(buf, len);
  }
}

void usbLogLen(const char* str, int len)
{
  if (s_usbUsbToHost_enable) {
    osSemaphoreWait(usb_BSemHandle, 0UL);
    usbToHostWait((uint8_t*)str, len);
    osSemaphoreRelease(usb_BSemHandle);
  }
}

inline
void usbLog(const char* str)
{
  usbLogLen(str, strlen(str));
}


void usbFromHostFromIRQ(const uint8_t* buf, uint32_t len)
{
	if (buf && len && !v_usbUsbFromHostISRBufLen) {
		BaseType_t	lMaxLen = sizeof(v_usbUsbFromHostISRBuf) - 1;
		BaseType_t	lLen = len;

		if (lLen > lMaxLen) {
			lLen = lMaxLen;
		}
		memcpy((void*)v_usbUsbFromHostISRBuf, (const void*)buf, lLen);
		v_usbUsbFromHostISRBuf[lLen] = 0U;
		__ISB();
		v_usbUsbFromHostISRBufLen = lLen;
	}
}


const char usbClrScrBuf[4] = { 0x0c, 0x0d, 0x0a, 0 };


/* USB-to-Host */

void usbStartUsbToHostTask(void const * argument)
{
  static uint8_t buf[32]  = { 0U };
  static uint8_t bufCtr   = 0U;
  uint8_t inChr = 0U;

  /* Clear queue */
  while (xQueueReceive(usbToHostQueueHandle, &inChr, 0) == pdPASS) {
  }
  xEventGroupSetBits(usbToHostEventGroupHandle, USB_TO_HOST_EG__BUF_EMPTY);

  osDelay(3500UL);

  /* Init connection with dummy data */
  for (uint8_t cnt = 30U; cnt; cnt--) {
    CDC_Transmit_FS((uint8_t*) usbClrScrBuf, 3);
    osDelay(10UL);
  }
  osDelay(250UL);

  /* To be moved to the controller.c module */
  {
    const char usb_Greeting_T01[]   = "+===============+\r\n";
    const char usb_Greeting_Name[]  = "* DL0WH BiTuner *\r\n";
    const char usb_Greeting_CRLF[]  = "\r\n";

    CDC_Transmit_FS((uint8_t*) usb_Greeting_T01, strlen(usb_Greeting_T01));
    osDelay(10UL);
    CDC_Transmit_FS((uint8_t*) usb_Greeting_Name, strlen(usb_Greeting_Name));
    osDelay(10UL);
    CDC_Transmit_FS((uint8_t*) usb_Greeting_T01, strlen(usb_Greeting_T01));
    osDelay(10UL);
    CDC_Transmit_FS((uint8_t*) usb_Greeting_CRLF, 2);
    osDelay(500UL);
  }

  /* TaskLoop */
  for (;;) {
    BaseType_t xStatus;

    do {
      /* Take next character from the queue - at least update each 100ms */
      inChr = 0;
      xStatus = xQueueReceive(usbToHostQueueHandle, &inChr, 100 / portTICK_PERIOD_MS);
      if ((pdPASS == xStatus) && inChr) {
        /* Group-Bit for empty queue cleared */
        xEventGroupClearBits(usbToHostEventGroupHandle, USB_TO_HOST_EG__BUF_EMPTY);

        buf[bufCtr++] = inChr;

      } else {
        if (pdPASS != xStatus) {
          /* Group-Bit for empty queue set */
          xEventGroupSetBits(usbToHostEventGroupHandle, USB_TO_HOST_EG__BUF_EMPTY);
        }
      }

      /* Flush when 0 or when buffer is full */
      if (!inChr || (bufCtr >= (sizeof(buf) - 1))) {
        uint32_t retryCnt;

        /* Do not send empty buffer */
        if (!bufCtr) {
          break;
        }

        buf[bufCtr] = 0;
        for (retryCnt = 25; retryCnt; --retryCnt) {
          /* Transmit to USB host */
          uint8_t ucRetVal = CDC_Transmit_FS(buf, bufCtr);
          if (USBD_BUSY != ucRetVal) {
            /* Data accepted for transmission */
            bufCtr = 0;
            buf[0] = 0;

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
          bufCtr = 0;
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
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
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

void usbStartUsbFromHostTask(void const * argument)
{
  const uint8_t nulBuf[1]   = { 0U };
  const uint8_t maxWaitMs   = 25U;

  /* TaskLoop */
  for (;;) {
    if (v_usbUsbFromHostISRBufLen) {
      /* USB OUT EP from host put data into the buffer */
      volatile uint8_t* bufPtr = v_usbUsbFromHostISRBuf;
      for (BaseType_t idx = 0; idx < v_usbUsbFromHostISRBufLen; ++idx, ++bufPtr) {
        xQueueSendToBack(usbFromHostQueueHandle, (uint8_t*)bufPtr, maxWaitMs);
      }
      xQueueSendToBack(usbFromHostQueueHandle, nulBuf, maxWaitMs);

      memset((char*) v_usbUsbFromHostISRBufLen, 0, sizeof(v_usbUsbFromHostISRBufLen));
      __DMB();
      v_usbUsbFromHostISRBufLen = 0UL;

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
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
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
