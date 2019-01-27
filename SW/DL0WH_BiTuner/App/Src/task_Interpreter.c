/*
 * task_Interpreter.c
 *
 *  Created on: 22.01.2019
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <sys/_stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include <cmsis_os.h>
#include "FreeRTOSConfig.h"
#include "stm32l476xx.h"

#include "main.h"
#include "task_USB.h"
#include "task_Controller.h"

#include "task_Interpreter.h"


/* Variables -----------------------------------------------------------------*/
extern osMessageQId         usbFromHostQueueHandle;
extern osMessageQId         interOutQueueHandle;

extern osSemaphoreId        usbToHostBinarySemHandle;
extern osSemaphoreId        c2interpreter_BSemHandle;

extern EventGroupHandle_t   globalEventGroupHandle;
extern EventGroupHandle_t   controllerEventGroupHandle;


//extern ENABLE_MASK_t        g_enableMsk;
//extern MON_MASK_t           g_monMsk;

extern const char           usbClrScrBuf[4];


/* Private variables ---------------------------------------------------------*/
const uint16_t              Interpreter_MaxWaitMs             = 100;

static uint8_t              s_interpreter_enable              = 0U;
static uint32_t             s_interpreterStartTime            = 0UL;

static uint8_t              s_interpreterLineBuf[256]         = { 0 };
static uint32_t             s_interpreterLineBufLen           = 0UL;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void interpreterUnknownCommand(void)
{
  usbLog("\r\n?? unknown command - please try 'help' ??\r\n\r\n");
}

#ifdef OLD
static void interpreterPushToInterOutQueue(const uint8_t* cmdAry, uint16_t cmdLen)
{
  /* Sanity check */
  if (cmdLen > 255U) {
    return;
  }

  for (uint8_t idx = 0; idx < cmdLen; ++idx) {
    xQueueSendToBack(interOutQueueHandle, cmdAry + idx, 1);
  }
  xEventGroupSetBits(controllerEventGroupHandle, Controller_EGW__INTER_QUEUE_OUT);
}
#endif

#if 0
static uint8_t interpreterDoInterprete__HexString_HexNibbleParser(char c)
{
  if ('0' <= c && c <= '9') {
    return c - '0';
  }

  c = tolower(c);
  if ('a' <= c && c <= 'f') {
    return (c - 'a') + 10;
  }

  /* Bad hex */
  return 0;
}
#endif

#if 0
static void interpreterDoInterprete__HexString(const char* buf, uint16_t len)
{
  uint8_t hexAry[256] = { 0 };
  uint8_t idxIn = 0U, idxOut = 0U;

  /* Sanity check: 0xAB is minimal valid length */
  if (!len) {
    return;
  }

  while ((idxIn  < len) &&
         (idxOut < (sizeof(hexAry) - 1))) {
    uint8_t c = buf[idxIn++];
    uint8_t nibLo = 0U, nibHi = 0U;

    /* Parse first nibble */
    {
      /* Skip any blanks */
      while (isspace(c) &&
            (idxIn < len)) {
        c = buf[idxIn++];
      }

      /* Skip HEX prefix */
      if ((c == '0') && (tolower(buf[idxIn]) == 'x')) {
        idxIn++;
        c = buf[idxIn++];
      }

      /* Length check: at least two bytes are needed here */
      if (idxIn > len) {
        return interpreterSendLoRaBare((char*) hexAry, idxOut);
      }

      /* Scan first hex nibble */
      nibHi = interpreterDoInterprete__HexString_HexNibbleParser(c);
    }

    /* Parse second nibble */
    {
      c = buf[idxIn++];

      /* one nibble only */
      if ((idxIn > len) || isspace(c)) {
        --idxIn;
        nibLo = nibHi;
        nibHi = 0U;

      } else {
        nibLo = interpreterDoInterprete__HexString_HexNibbleParser(c);
      }
    }

    /* Write out */
    const uint8_t byte = (uint8_t) (0xff & ((nibHi << 4) | (nibLo)));
    hexAry[idxOut++] = byte;
  }

  interpreterSendLoRaBare((char*) hexAry, idxOut);
}
#endif

static void interpreterDoInterprete(const uint8_t* buf, uint32_t len)
{
  const char *cb = (const char*) s_interpreterLineBuf;

  for (uint32_t idx = 0; idx < len; idx++) {
    if (idx + s_interpreterLineBufLen < 254) {
      s_interpreterLineBuf[s_interpreterLineBufLen++] = buf[idx];
    }
  }

  if (false) {

#if 0
  } else if (!strncmp("adr ", cb, 4) && (4 < len)) {
    const long    adrEnable   = strtol(cb + 4, NULL, 10);
    const uint8_t pushAry[3]  = { 2, InterOutQueueCmds__ADRset, (adrEnable ?  1U : 0U) };

  } else if (!strncmp("c", cb, 1) && (1 == len)) {
    interpreterClearScreen();

  } else if (!strncmp("conf ", cb, 5) && (5 < len)) {
    const long    confEnable  = strtol(cb + 5, NULL, 10);
    const uint8_t pushAry[3]  = { 2, InterOutQueueCmds__ConfirmedPackets, (confEnable ?  1U : 0U) };

  } else if (!strncmp("dr ", cb, 3) && (3 < len)) {
    const long    val         = strtol(cb + 3, NULL, 10);
    const uint8_t drSet = (uint8_t) val;
    const uint8_t pushAry[3]  = { 2, InterOutQueueCmds__DRset, drSet };

  } else if (!strncmp("f ", cb, 2) && (2 < len)) {
    /* LoRaBare frequency setting [f]=Hz */
    const long    f            = strtol(cb + 2, NULL, 10);
    const uint8_t frequencyHz0 = (uint8_t) (f       ) & 0xffUL;
    const uint8_t frequencyHz1 = (uint8_t) (f >>  8U) & 0xffUL;
    const uint8_t frequencyHz2 = (uint8_t) (f >> 16U) & 0xffUL;
    const uint8_t frequencyHz3 = (uint8_t) (f >> 24U) & 0xffUL;
    const uint8_t pushAry[6]   = { 5, InterOutQueueCmds__LoRaBareFrequency, frequencyHz0, frequencyHz1, frequencyHz2, frequencyHz3 };

#endif
  } else if (!strncmp("help", cb, 4) && (4 == len)) {
    interpreterPrintHelp();

  } else if (!strncmp("C", cb, 1) && (3 == len)) {
    /* Set capacitance */
    const uint32_t valLsw      = *(cb + 1) > '0' ?  *(cb + 1) - '0' : 0;
    const uint32_t valLenable  = *(cb + 2) == '1' ?  1UL : 0UL;
    if (valLsw < 8) {
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 1U, MsgInterpreter__SetVar02_C);
      cmd[1] = (valLsw      << 24U) |
               (valLenable  << 16U) ;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, osWaitForever);
    }

  } else if (!strncmp("L", cb, 1) && (3 == len)) {
    /* Set inductance */
    const uint8_t valLsw      = *(cb + 1) > '0' ?  *(cb + 1) - '0' : 0;
    const uint8_t valLenable  = *(cb + 2) == '1' ?  1U : 0U;
    if (valLsw < 8) {
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 1U, MsgInterpreter__SetVar01_L);
      cmd[1] = (valLsw      << 24U) |
               (valLenable  << 16U) ;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, osWaitForever);
    }

#if 0
  } else if (!strncmp("push", cb, 4) && (4 == len)) {
    /* Set flag for sending and upload data */
    const uint8_t pushAry[2]  = { 1, InterOutQueueCmds__DoSendDataUp };

  } else if (!strncmp("mon ", cb, 4) && (4 < len)) {
    const long    val         = strtol(cb + 4, NULL, 10);
    g_monMsk                  = (uint32_t) val;

  } else if (!strncmp("pwrred ", cb, 7) && (7 < len)) {
    /* LoRaWAN power reduction setting [pwrred]=dB */
    const long    val         = strtol(cb + 7, NULL, 10);
    const uint8_t pwrRed      = (uint8_t) val;
    const uint8_t pushAry[3]  = { 2, InterOutQueueCmds__PwrRedDb, pwrRed };

  } else if (!strncmp("reqcheck", cb, 8) && (8 == len)) {
    /* LoRaWAN link check message */
    const uint8_t pushAry[2]  = { 1, InterOutQueueCmds__LinkCheckReq };

  } else if (!strncmp("reqtime", cb, 7) && (7 == len)) {
    /* LoRaWAN device time message */
    const uint8_t pushAry[2]  = { 1, InterOutQueueCmds__DeviceTimeReq };

#endif
  } else if (!strncmp("restart", cb, 7) && (7 == len)) {
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 1U, MsgInterpreter__CallFunc01_Restart);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, osWaitForever);

#if 0
  } else if (!strncmp("rx ", cb, 3) && (3 < len)) {
    /* LoRaBare frequency setting [f]=Hz */
    const long    rxEnable     = strtol(cb + 3, NULL, 10);
    const uint8_t pushAry[3]   = { 2, InterOutQueueCmds__LoRaBareRxEnable, (rxEnable ?  1U : 0U) };

  } else if (!strncmp("timer ", cb, 6) && (6 < len)) {
    /* LoRaWAN timer setting [timer]=s */
    const long    val         = strtol(cb + 6, NULL, 10);
    const uint8_t repeatTimer0 = (uint8_t) (val     ) & 0xffUL;
    const uint8_t repeatTimer1 = (uint8_t) (val >> 8) & 0xffUL;
    const uint8_t pushAry[4]  = { 3, InterOutQueueCmds__Timer, repeatTimer0, repeatTimer1 };
#endif

  } else if (!strncmp("?", cb, 1) && (1 == len)) {
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 1U, MsgInterpreter__CallFunc02_PrintLC);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, osWaitForever);

  } else {
    interpreterUnknownCommand();
  }

  /* Slice out until next line follows */
  uint32_t newIdx = 0UL;
  for (uint32_t idx = 0; idx < s_interpreterLineBufLen; idx++) {
    const uint8_t c0 = s_interpreterLineBuf[idx];
    const uint8_t c1 = s_interpreterLineBuf[idx];

    if (c0 == 0x0a || c0 == 0x0d) {
      newIdx = (c1 == 0x0a || c1 == 0x0d) ?  (idx + 1) : idx;
      break;
    }
  }

  /* Truncate current line */
  if (newIdx) {
    /* Line termination found: cue to next line */
    memmove(s_interpreterLineBuf, s_interpreterLineBuf + newIdx, s_interpreterLineBufLen - newIdx + 1UL);
    s_interpreterLineBufLen -= newIdx;

  } else {
    /* Old single line clear */
    s_interpreterLineBufLen = 0UL;
  }
  memset(s_interpreterLineBuf + (sizeof(s_interpreterLineBuf) - s_interpreterLineBufLen), 0, sizeof(s_interpreterLineBuf) - s_interpreterLineBufLen);
}


/* Global functions ----------------------------------------------------------*/

const uint8_t               interpreterHelpMsg001[]            = "\r\n";
const uint8_t               interpreterHelpMsg002[]            = "\tHELP - list of commands:\r\n";
const uint8_t               interpreterHelpMsg003[]            = "\t========================\r\n";
//const uint8_t             interpreterHelpMsg011[]            = "\t\tCommand\t\tRemarks\r\n";
//const uint8_t             interpreterHelpMsg012[]            = "\t\t-------\t\t-------\r\n";

const uint8_t               interpreterHelpMsg111[]            =     "\t\t> Main commands\r\n";
const uint8_t               interpreterHelpMsg112[]            =     "\t\t--------------------------------------------------------------------------\r\n";
const uint8_t               interpreterHelpMsg121[]            = "\t\tc\t\tClear screen.\r\n";
const uint8_t               interpreterHelpMsg131[]            = "\t\thelp\t\tPrint this list of commands.\r\n";
const uint8_t               interpreterHelpMsg141[]            = "\t\tmon <n>\t\tMonitor bitmask:\r\n";
const uint8_t               interpreterHelpMsg151[]            = "\t\trestart\t\tRestart this device.\r\n\r\n";

void interpreterPrintHelp(void)
{
  osSemaphoreWait(usbToHostBinarySemHandle, 0);

  usbToHostWait(interpreterHelpMsg001, strlen((char*) interpreterHelpMsg001));
  usbToHostWait(interpreterHelpMsg002, strlen((char*) interpreterHelpMsg002));
  usbToHostWait(interpreterHelpMsg003, strlen((char*) interpreterHelpMsg003));
  usbToHostWait(interpreterHelpMsg001, strlen((char*) interpreterHelpMsg001));

//usbToHostWait(interpreterHelpMsg011, strlen((char*) interpreterHelpMsg011));
//usbToHostWait(interpreterHelpMsg012, strlen((char*) interpreterHelpMsg012));
//usbToHostWait(interpreterHelpMsg001, strlen((char*) interpreterHelpMsg001));

  usbToHostWait(interpreterHelpMsg111, strlen((char*) interpreterHelpMsg111));
  usbToHostWait(interpreterHelpMsg112, strlen((char*) interpreterHelpMsg112));
  usbToHostWait(interpreterHelpMsg121, strlen((char*) interpreterHelpMsg121));
  usbToHostWait(interpreterHelpMsg131, strlen((char*) interpreterHelpMsg131));
  usbToHostWait(interpreterHelpMsg141, strlen((char*) interpreterHelpMsg141));
  usbToHostWait(interpreterHelpMsg151, strlen((char*) interpreterHelpMsg151));

  osSemaphoreRelease(usbToHostBinarySemHandle);
}

void interpreterShowCursor(void)
{
  usbLog("> ");
}

void interpreterClearScreen(void)
{
  usbLog(usbClrScrBuf);
}


static void interpreterInit(void)
{
  interpreterClearScreen();
  interpreterPrintHelp();
}

static void interpreterMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const interpreterCmds_t cmd     = (interpreterCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgInterpreter__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_interpreterStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Activation flag */
      s_interpreter_enable = 1U;

      /* Init module */
      interpreterInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgInterpreter__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  default: { }
  }  // switch (cmd)
}


/* Tasks */

void interpreterTaskInit(void)
{
  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_interpreterStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void interpreterTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];
  uint8_t   usbAry[32];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2interpreter_BSemHandle, 100UL);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Interpreter, 1UL);                   // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    interpreterMsgProcess(msgLen, msgAry);
  }

  /* Check if data from the USB host is available */
  //  ev = osMessageGet(usbFromHostQueueHandle, 1UL);
  uint32_t len = usbPullFromOutQueue(usbAry, 1UL);
  if (len) {
    interpreterDoInterprete(usbAry, len);
  }
}
