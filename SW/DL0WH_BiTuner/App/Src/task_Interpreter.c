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
/* Global functions ----------------------------------------------------------*/

const char                  interpreterHelpMsg001[]            = "\r\n";
const char                  interpreterHelpMsg002[]            = "\tHELP - list of commands:\r\n";
const char                  interpreterHelpMsg003[]            = "\t=======================\r\n";

const char                  interpreterHelpMsg111[]            =     "\t\t> main commands\r\n";
const char                  interpreterHelpMsg112[]            =     "\t\t--------------------------------------------------------------------------\r\n";

const char                  interpreterHelpMsg121[]            = "\t\tCxy\t\tC relay x: 1..8, y: 1=SET 0=RESET.\r\n";
const char                  interpreterHelpMsg122[]            = "\t\tLxy\t\tL relay x: 1..8, y: 1=SET 0=RESET.\r\n";
const char                  interpreterHelpMsg123[]            = "\t\tCL\t\tSet C at the TRX-side and the L to the antenna side (Gamma).\r\n";
const char                  interpreterHelpMsg124[]            = "\t\tLC\t\tSet L at the TRX-side and the C to the antenna side (reverted Gamma).\r\n";
const char                  interpreterHelpMsg125[]            = "\t\tHxxyyzz\t\tHexadecimal entry of the 18 bits of x:[CH,CV], y:[L8-1], z:[C8-1] relays.\r\n";
const char                  interpreterHelpMsg126[]            = "\t\t?\t\tShow current relay settings and electric values.\r\n";

const char                  interpreterHelpMsg131[]            = "\t\tMG\t\tMeasuremnet OpAmp gain:   0..256\r\n";
const char                  interpreterHelpMsg132[]            = "\t\tMO\t\tMeasuremnet OpAmp offset: 0..256\r\n";

const char                  interpreterHelpMsg141[]            = "\t\tC\t\tClear screen.\r\n";
const char                  interpreterHelpMsg142[]            = "\t\tHELP\t\tPrint this list of commands.\r\n";
const char                  interpreterHelpMsg143[]            = "\t\tRESTART\t\tRestart this device.\r\n";

const char                  interpreterHelpMsg151[]            =     "\t\t> additional DJ0ABR compatible commands\r\n";
const char                  interpreterHelpMsg152[]            =     "\t\t--------------------------------------------------------------------------\r\n";

const char                  interpreterHelpMsg161[]            = "\t\tKxyz\t\tShort form for setting the C, L, CV and CH relays.\r\n";
const char                  interpreterHelpMsg162[]            = "\t\tHx\t\t1: LC mode, 0: CL mode.\r\n";
const char                  interpreterHelpMsg163[]            = "\t\tVx\t\t1: CL mode, 0: LC mode.\r\n";
const char                  interpreterHelpMsg164[]            = "\t\tCH\t\t   LC mode.\r\n";
const char                  interpreterHelpMsg165[]            = "\t\tCV\t\t   CL mode.\r\n";


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
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));

  interpreterConsolePush(interpreterHelpMsg131, strlen(interpreterHelpMsg131));
  interpreterConsolePush(interpreterHelpMsg132, strlen(interpreterHelpMsg132));
  interpreterConsolePush(interpreterHelpMsg112, strlen(interpreterHelpMsg112));
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));

  interpreterConsolePush(interpreterHelpMsg141, strlen(interpreterHelpMsg141));
  interpreterConsolePush(interpreterHelpMsg142, strlen(interpreterHelpMsg142));
  interpreterConsolePush(interpreterHelpMsg143, strlen(interpreterHelpMsg143));
  interpreterConsolePush(interpreterHelpMsg112, strlen(interpreterHelpMsg112));
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));


  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));
  interpreterConsolePush(interpreterHelpMsg151, strlen(interpreterHelpMsg151));
  interpreterConsolePush(interpreterHelpMsg152, strlen(interpreterHelpMsg152));

  interpreterConsolePush(interpreterHelpMsg161, strlen(interpreterHelpMsg161));
  interpreterConsolePush(interpreterHelpMsg162, strlen(interpreterHelpMsg162));
  interpreterConsolePush(interpreterHelpMsg163, strlen(interpreterHelpMsg163));
  interpreterConsolePush(interpreterHelpMsg164, strlen(interpreterHelpMsg164));
  interpreterConsolePush(interpreterHelpMsg165, strlen(interpreterHelpMsg165));
  interpreterConsolePush(interpreterHelpMsg112, strlen(interpreterHelpMsg112));
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));
}

void interpreterShowCursor(void)
{
  interpreterConsolePush("> ", 2UL);
}

static void interpreterShowCrLfCursor(void)
{
  interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));
  interpreterShowCursor();
}

void interpreterClearScreen(void)
{
  interpreterConsolePush(usbClrScrBuf, strlen(usbClrScrBuf));
}

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


static uint8_t interpreterDoInterprete__HexString_HexNibbleParser(char c)
{
  if ('0' <= c && c <= '9') {
    return c - '0';
  }

  c = toupper(c);
  if ('A' <= c && c <= 'F') {
    return (c - 'A') + 10;
  }

  /* Bad hex */
  return 0;
}

static uint32_t interpreterDoInterprete__HexString(const char* buf, uint8_t len)
{
  uint32_t  retVal        = 0UL;
  uint8_t   idxIn         = 0U;

  /* Sanity check: 0xAB is minimal valid length */
  if (!buf || !len || (len > 8U)) {
    return 0UL;
  }

  while (idxIn < len) {
    uint8_t c     = buf[idxIn++];
    uint8_t nibLo = 0U;
    uint8_t nibHi = 0U;

    /* Parse first nibble */
    {
#if 0
      /* Skip any blanks */
      while (isspace(c) && (idxIn < len)) {
        c = buf[idxIn++];
      }

      /* Skip HEX prefix */
      if ((c == '0') && (tolower(buf[idxIn]) == 'x')) {
        idxIn++;
        c = buf[idxIn++];
      }
#endif

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
    const uint8_t byte = (uint8_t) (0xffU & ((nibHi << 4) | (nibLo)));
    retVal <<= 8;
    retVal  |= byte;
  }
  return retVal;
}


static void interpreterDoInterprete(const uint8_t* buf, uint32_t len)
{
  const char*     cb          = (const char*) s_interpreterLineBuf;
  uint8_t*        bufOutPtr   = s_interpreterLineBuf + s_interpreterLineBufLen;
  const uint8_t*  bufInPtr    = buf;
  uint8_t         c           = 0U;

  /* Strip ending NUL char */
  if (!buf[len - 1]) {
    --len;
  }

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

    interpreterShowCursor();
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
  if (!strncmp("C", cb, 1) && (1UL == len)) {
    interpreterClearScreen();
    interpreterShowCursor();

  } else if (!strncmp("HELP", cb, 4) && (4UL == len)) {
    interpreterConsolePush(interpreterHelpMsg001, strlen(interpreterHelpMsg001));
    interpreterPrintHelp();
    interpreterShowCursor();

  } else if (!strncmp("C", cb, 1) && (3UL == len)) {
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
      controllerMsgPushToInQueue(1, cmd, 10UL);    }

  } else if (!strncmp("L", cb, 1) && (3UL == len)) {
    /* Set inductance */
    uint8_t valLsw = *(cb + 1) > '0' ?  *(cb + 1) - '0' : 0;
    const uint8_t valLenable  = *(cb + 2) != '0' ?  1U : 0U;
    if (0 < valLsw && valLsw <= 8) {
      --valLsw;
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 2U, MsgController__SetVar01_L);
      cmd[1] = (valLsw      << 24U) |
               (valLenable  << 16U) ;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

      cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
      controllerMsgPushToInQueue(1, cmd, 10UL);
    } else {
      interpreterUnknownCommand();
    }

  } else if ((!strncmp("CL", cb, 2) || !strncmp("V1", cb, 2) || !strncmp("H0", cb, 2) || !strncmp("CV", cb, 2)) && (2UL == len)) {
    /* Set configuration to Gamma (CV) */
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__SetVar03_CL);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(1, cmd, 10UL);

  } else if ((!strncmp("LC", cb, 2) || !strncmp("H1", cb, 2) || !strncmp("V0", cb, 2) || !strncmp("CH", cb, 2)) && (2UL == len)) {
    /* Set configuration to reverted Gamma (CH) */
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__SetVar04_LC);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(1, cmd, 10UL);

  } else if (!strncmp("H", cb, 1) && (7UL == len)) {
    /* Set relays */
    const uint32_t bitmask  = interpreterDoInterprete__HexString(cb + 1, 6);
    uint32_t       relayExt = bitmask & 0x010000UL;
    relayExt               |= (relayExt ^ 0x010000UL) << 1;
    const uint32_t relays   =  relayExt | (bitmask & 0x00ffffUL);

    uint32_t cmd[2];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 4U, MsgController__SetVar05_K);
    cmd[1] = relays;
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(1, cmd, 10UL);

  } else if (!strncmp("K", cb, 1) && (4UL == len)) {
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

    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(1, cmd, 10UL);

  } else if (!strncmp("MG", cb, 2) && ((3UL <= len) && (len <= 5UL))) {
    const long val = strtol(cb + 2, NULL, 10);

    if (0 <= val && val <= 256) {
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Interpreter, 4U, MsgDefault__CallFunc04_DigPot_SetGain);
      cmd[1] = (uint32_t) val;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);
    } else {
      interpreterUnknownCommand();
    }
    interpreterShowCrLfCursor();

  } else if (!strncmp("MO", cb, 2) && ((3UL <= len) && (len <= 5UL))) {
    const long val = strtol(cb + 2, NULL, 10);

    if (0 <= val && val <= 256) {
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Interpreter, 4U, MsgDefault__CallFunc05_DigPot_SetOffset);
      cmd[1] = (uint32_t) val;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);
    } else {
      interpreterUnknownCommand();
    }
    interpreterShowCrLfCursor();

  } else if (!strncmp("RESTART", cb, 7) && (7UL == len)) {
    const char infoStr[] = "*** Restarting, please wait...\r\n";
    interpreterConsolePush(infoStr, strlen(infoStr));
    osDelay(500UL);

    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc04_Restart);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

  } else if (!strncmp("?", cb, 1) && (1UL == len)) {
    uint32_t cmd[1];
    cmd[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Interpreter, 0U, MsgController__CallFunc05_PrintLC);
    controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

  } else {
    interpreterUnknownCommand();
    interpreterShowCursor();
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

  /* Start console input thread */
  osThreadDef(interpreterGetterTask, interpreterGetterTask, osPriorityNormal, 0, 128);
  s_interpreterGetterTaskHandle = osThreadCreate(osThread(interpreterGetterTask), NULL);

  /* Prepare console output */
  interpreterPrintHelp();
  interpreterShowCursor();
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
