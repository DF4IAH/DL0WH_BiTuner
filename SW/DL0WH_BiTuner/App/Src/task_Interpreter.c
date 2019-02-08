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
#include "task_UART.h"
#include "task_Controller.h"

#include "task_Interpreter.h"


/* Variables -----------------------------------------------------------------*/
extern osMessageQId         usbFromHostQueueHandle;
extern osMessageQId         uartRxQueueHandle;
extern osMessageQId         interOutQueueHandle;

extern osSemaphoreId        c2interpreter_BSemHandle;

extern EventGroupHandle_t   globalEventGroupHandle;
extern EventGroupHandle_t   controllerEventGroupHandle;

static osThreadId           s_interpreterGetterTaskHandle     = 0;

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
static uint32_t interpreterCalcLineLen(const uint8_t* buf, uint32_t len)
{
  const uint8_t* bufPtr = buf;

  for (uint32_t idx = 0UL; idx < len; idx++, bufPtr++) {
    if (*bufPtr == 0x0aU || *bufPtr == 0x0dU) {
      return idx;
    }
  }
  return len;
}

static void interpreterUnknownCommand(void)
{
  const char* unknownStr = "\r\n?? unknown command - please try 'HELP' ??\r\n\r\n";
  interpreterConsolePush(unknownStr, strlen(unknownStr));
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
  const char*     cb          = (const char*) s_interpreterLineBuf;
  uint8_t*        bufOutPtr   = s_interpreterLineBuf + s_interpreterLineBufLen;
  const uint8_t*  bufInPtr    = buf;
  uint8_t         c;

  for (uint32_t idx = 0UL; idx < len; idx++) {
    if ((idx + s_interpreterLineBufLen) < 254UL) {
      *(bufOutPtr++) = c = *(bufInPtr++);
      ++s_interpreterLineBufLen;
    }
  }
  if (c != 0x0aU && c != 0x0dU) {
    /* Line not complete */
    return;
  }

  /* Count length w/o CR LF */
  len = interpreterCalcLineLen(s_interpreterLineBuf, s_interpreterLineBufLen);
  if (!len) {
    s_interpreterLineBufLen = 0UL;
    memset(s_interpreterLineBuf, 0, sizeof(s_interpreterLineBuf));
    return;
  }

  /* To upper for non-binaries only */
  if (toupper(s_interpreterLineBuf[0]) != 'K') {
    /* Do upper case */
    calcStrToUpper((char*)s_interpreterLineBuf, s_interpreterLineBufLen);

  } else {
    /* Uppercase the 'K', only */
    s_interpreterLineBuf[0] = 'K';
  }

  /* List of string patterns to compare */
  if (!strncmp("C", cb, 1) && (1 == len)) {
    interpreterClearScreen();

#if 0
  } else if (!strncmp("CONF ", cb, 5) && (5 < len)) {
    const long    confEnable  = strtol(cb + 5, NULL, 10);
    const uint8_t pushAry[3]  = { 2, InterOutQueueCmds__ConfirmedPackets, (confEnable ?  1U : 0U) };

#endif
  } else if (!strncmp("HELP", cb, 4) && (4 == len)) {
    interpreterClearScreen();
    interpreterPrintHelp();

  } else if (!strncmp("C", cb, 1) && (3 == len)) {
    /* Set capacitance */
    uint32_t valCsw = *(cb + 1) > '0' ?  *(cb + 1) - '0' : 0;
    const uint32_t valLenable  = *(cb + 2) == '1' ?  1UL : 0UL;
    if (0 < valCsw && valCsw <= 8) {
      --valCsw;
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 2U, MsgController__SetVar02_C);
      cmd[1] = (valCsw      << 24U) |
               (valLenable  << 16U) ;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
      controllerMsgPushToInQueue(1, cmd, 10UL);
    }

  } else if (!strncmp("L", cb, 1) && (3 == len)) {
    /* Set inductance */
    uint8_t valLsw = *(cb + 1) > '0' ?  *(cb + 1) - '0' : 0;
    const uint8_t valLenable  = *(cb + 2) == '1' ?  1U : 0U;
    if (0 < valLsw && valLsw <= 8) {
      --valLsw;
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 2U, MsgController__SetVar01_L);
      cmd[1] = (valLsw      << 24U) |
               (valLenable  << 16U) ;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
      controllerMsgPushToInQueue(1, cmd, 10UL);
    }

  } else if ((!strncmp("CL", cb, 2) || !strncmp("V1", cb, 2) || !strncmp("H0", cb, 2)) && (2 == len)) {
    /* Set configuration to Gamma (CV) */
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__SetVar03_CL);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

  } else if ((!strncmp("LC", cb, 2) || !strncmp("H1", cb, 2) || !strncmp("V0", cb, 2)) && (2 == len)) {
    /* Set configuration to reverted Gamma (CH) */
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__SetVar04_LC);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

  } else if (!strncmp("K", cb, 1) && (4 == len)) {
    /* Set relays */
    const uint8_t relayC    = *(cb + 1);
    const uint8_t relayL    = *(cb + 2);
    uint8_t       relayExt  = *(cb + 3) & 0x01U;
    relayExt               |= (relayExt ^ 0x01U) << 1U;
    const uint32_t relays   = ((uint32_t)relayExt << 16) | ((uint32_t)relayL << 8) | (uint32_t)relayC;

    uint32_t cmd[2];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 4U, MsgController__SetVar05_K);
    cmd[1] = relays;
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

#if 0
  } else if (!strncmp("MON ", cb, 4) && (4 < len)) {
    const long    val         = strtol(cb + 4, NULL, 10);
    g_monMsk                  = (uint32_t) val;
#endif
  } else if (!strncmp("RESTART", cb, 7) && (7 == len)) {
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc04_Restart);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

  } else if (!strncmp("?", cb, 1) && (1 == len)) {
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

  } else {
    interpreterUnknownCommand();
  }

  /* Find next line */
  uint32_t newIdx = 0UL;
  uint8_t* bufPtr = s_interpreterLineBuf;
  for (uint32_t idx = 0UL; idx < s_interpreterLineBufLen; idx++) {
    const uint8_t c0 = *(bufPtr++);
    const uint8_t c1 = *bufPtr;

    if (c0 == 0x0aU || c0 == 0x0dU) {
      newIdx = (c1 == 0x0aU || c1 == 0x0dU) ?  (idx + 1) : idx;

      /* Point to first character of next line */
      ++newIdx;
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
  memset(s_interpreterLineBuf + s_interpreterLineBufLen, 0, sizeof(s_interpreterLineBuf) - s_interpreterLineBufLen);
}


/* Global functions ----------------------------------------------------------*/

const char                  interpreterHelpMsg001[]            = "\r\n";
const char                  interpreterHelpMsg002[]            = "\tHELP - list of commands:\r\n";
const char                  interpreterHelpMsg003[]            = "\t=======================\r\n";

const char                  interpreterHelpMsg111[]            =     "\t\t> Main commands\r\n";
const char                  interpreterHelpMsg112[]            =     "\t\t--------------------------------------------------------------------------\r\n";

const char                  interpreterHelpMsg121[]            = "\t\tCxy\t\tC relay x: 1..8, y: 1=SET 0=RESET.\r\n";
const char                  interpreterHelpMsg122[]            = "\t\tLxy\t\tL relay x: 1..8, y: 1=SET 0=RESET.\r\n";
const char                  interpreterHelpMsg123[]            = "\t\tCL\t\tSet C at the TRX-side and the L to the antenna side (Gamma).\r\n";
const char                  interpreterHelpMsg124[]            = "\t\tLC\t\tSet L at the TRX-side and the C to the antenna side (reverted Gamma).\r\n";
const char                  interpreterHelpMsg125[]            = "\t\tHx\t\t1: LC mode, 0: CL mode.\r\n";
const char                  interpreterHelpMsg126[]            = "\t\tVx\t\t1: CL mode, 0: LC mode.\r\n";
const char                  interpreterHelpMsg127[]            = "\t\tKxyz\t\tShort form for setting the C, L, CV and CH relays.\r\n";
const char                  interpreterHelpMsg128[]            = "\t\t?\t\tShow current relay settings and electric values.\r\n";

const char                  interpreterHelpMsg131[]            = "\t\tC\t\tClear screen.\r\n";
const char                  interpreterHelpMsg132[]            = "\t\tHELP\t\tPrint this list of commands.\r\n";
const char                  interpreterHelpMsg133[]            = "\t\tRESTART\t\tRestart this device.\r\n";


void interpreterConsolePush(const char* buf, int bufLen)
{
  /* Send to USB */
  if (true) {
    usbLogLen(buf, bufLen);
  }

  /* Send to UART */
  if (true) {
    uartLogLen(buf, bufLen);
  }
}

void interpreterPrintHelp(void)
{
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));
  interpreterConsolePush(interpreterHelpMsg002, strlen(interpreterHelpMsg002));
  interpreterConsolePush(interpreterHelpMsg003, strlen(interpreterHelpMsg003));
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));

  interpreterConsolePush(interpreterHelpMsg111, strlen(interpreterHelpMsg111));
  interpreterConsolePush(interpreterHelpMsg112, strlen(interpreterHelpMsg112));
  interpreterConsolePush(interpreterHelpMsg121, strlen(interpreterHelpMsg121));
  interpreterConsolePush(interpreterHelpMsg122, strlen(interpreterHelpMsg122));
  interpreterConsolePush(interpreterHelpMsg123, strlen(interpreterHelpMsg123));
  interpreterConsolePush(interpreterHelpMsg124, strlen(interpreterHelpMsg124));
  interpreterConsolePush(interpreterHelpMsg125, strlen(interpreterHelpMsg125));
  interpreterConsolePush(interpreterHelpMsg126, strlen(interpreterHelpMsg126));
  interpreterConsolePush(interpreterHelpMsg127, strlen(interpreterHelpMsg127));
  interpreterConsolePush(interpreterHelpMsg128, strlen(interpreterHelpMsg128));
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));

  interpreterConsolePush(interpreterHelpMsg131, strlen(interpreterHelpMsg131));
  interpreterConsolePush(interpreterHelpMsg132, strlen(interpreterHelpMsg132));
  interpreterConsolePush(interpreterHelpMsg133, strlen(interpreterHelpMsg133));
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));
}

void interpreterShowCursor(void)
{
  interpreterConsolePush("> ", 2UL);
}

void interpreterClearScreen(void)
{
  interpreterConsolePush(usbClrScrBuf, strlen(usbClrScrBuf));
}


void interpreterGetterTask(void const * argument)
{
  /* Handle serial console on input streams USB and UART */
  for (;;) {
    uint8_t inBuf[64]  = { 0U };
    uint32_t inBufLen;

    /* Transfer USB input*/
    inBufLen = usbPullFromOutQueue(inBuf, 1UL);

    /* Transfer UART input*/
    if (!inBufLen) {
      inBufLen = uartRxPullFromQueue(inBuf, 1UL);
    }

    if (inBufLen) {
      /* Echo */
      if (true) {
        interpreterConsolePush((char*)inBuf, inBufLen);
      }

      /* Lets do the interpreter */
      interpreterDoInterprete(inBuf, inBufLen);

    } else {
      osDelay(25UL);
    }
  }
}

static void interpreterInit(void)
{
  /* Wait until init message is done */
  osDelay(5000UL);

  /* Prepare console output */
  interpreterPrintHelp();

  /* Start console input thread */
  osThreadDef(interpreterGetterTask, interpreterGetterTask, osPriorityNormal, 0, 128);
  s_interpreterGetterTaskHandle = osThreadCreate(osThread(interpreterGetterTask), NULL);
}

static void interpreterMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const InterpreterCmds_t cmd     = (InterpreterCmds_t) (0xffUL & hdr);

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
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, 10UL);
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

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2interpreter_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Interpreter, 1UL);                   // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    interpreterMsgProcess(msgLen, msgAry);
  }
}
