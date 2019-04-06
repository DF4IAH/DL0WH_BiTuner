/*
 * task_Controller.c
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include "task_Controller.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l476xx.h"

//#include "stm32l4xx_hal_gpio.h"
#include "bus_spi.h"
#include "device_adc.h"
#include "task_USB.h"
#include "task_CAT.h"
#include "task_Interpreter.h"


#define I2C1_BUS_ADDR_SCAN

#ifdef I2C1_BUS_ADDR_SCAN
# include "bus_i2c.h"
extern I2C_HandleTypeDef    hi2c1;
extern osSemaphoreId        i2c1_BSemHandle;
#endif


extern osMessageQId         controllerInQueueHandle;
extern osMessageQId         controllerOutQueueHandle;

extern osTimerId            controllerTimerHandle;

extern osSemaphoreId        usb_BSemHandle;
extern osSemaphoreId        uart_BSemHandle;
extern osSemaphoreId        cat_BSemHandle;

extern osSemaphoreId        cQin_BSemHandle;
extern osSemaphoreId        cQout_BSemHandle;

extern osSemaphoreId        c2default_BSemHandle;
extern osSemaphoreId        c2interpreter_BSemHandle;
extern osSemaphoreId        c2usbToHost_BSemHandle;
extern osSemaphoreId        c2usbFromHost_BSemHandle;
extern osSemaphoreId        c2uartTx_BSemHandle;
extern osSemaphoreId        c2uartRx_BSemHandle;
extern osSemaphoreId        c2catTx_BSemHandle;
extern osSemaphoreId        c2catRx_BSemHandle;

extern EventGroupHandle_t   globalEventGroupHandle;

extern float                g_adc_refint_val;
extern float                g_adc_vref_mv;
extern float                g_adc_bat_mv;
extern float                g_adc_temp_deg;
extern float                g_adc_fwd_mv;
extern float                g_adc_rev_mv;
extern float                g_adc_vdiode_mv;
extern float                g_adc_swr;


#define Controller_L0_nH                                      50.0f
volatile const float Controller_Ls_nH[8]                      = {
    187.5f,
    375.0f,
    750.0f,
    1.5e+3f,
    3.0e+3f,
    6.0e+3f,
    12.0e+3f,
    24.0e+3f
};

#define Controller_C0_pF                                      25.0f
volatile const float Controller_Cp_pF[8]                      = {
    25.0f,
    //33.0f,
    50.0f,
    89.0f,
    165.0f,
    340.0f,
    600.0f,
    1137.0f,
    1939.0f
};

static const float          Controller_AutoSWR_P_mW_Min       = 5.0f;
static const float          Controller_AutoSWR_P_mW_Max       = 15.0f;
static const float          Controller_AutoSWR_SWR_Init       = 999.9f;
static const float          Controller_AutoSWR_SWR_Min        = 1.1f;
static const float          Controller_AutoSWR_SWR_Max        = 10.0f;
static const uint8_t        Controller_AutoSWR_SWR_Max_Cnt    = 30U;
static const float          Controller_AutoSWR_Time_ms_Max    = 750.0f;
static const uint8_t        Controller_AutoSWR_CVHpong_Max    = 1U;
static const uint8_t        Controller_AutoSWR_LCpong_Max     = 6U;

static uint32_t             s_controller_swr_tmr              = 0UL;
static uint32_t             s_controller_30ms_cnt             = 0UL;

static ControllerMsg2Proc_t s_msg_in                          = { 0 };
static ControllerMods_t     s_mod_start                       = { 0 };
static ControllerMods_t     s_mod_rdy                         = { 0 };

static ControllerFsm_t        s_controller_FSM_state            = ControllerFsm__NOP;
static ControllerOptiCVH_t    s_controller_FSM_optiCVH          = ControllerOptiCVH__CV;
static ControllerOptiLC_t     s_controller_FSM_optiLC           = ControllerOptiLC__L;
static ControllerOptiStrat_t  s_controller_FSM_optiStrat        = ControllerOptiStrat__Double;
static ControllerOptiUpDn_t   s_controller_FSM_optiUpDn         = ControllerOptiUpDn__Up;
static uint8_t                s_controller_opti_CVHpongCtr      = 0U;
static uint8_t                s_controller_opti_LCpongCtr       = 0U;
static uint8_t                s_controller_opti_L_relays        = 0U;
static uint8_t                s_controller_opti_C_relays        = 0U;
static float                  s_controller_opti_L               = Controller_L0_nH;
static float                  s_controller_opti_C               = Controller_C0_pF;
static float                  s_controller_opti_swr_1st         = 0.0f;
static float                  s_controller_opti_swr_1st_L       = 0.0f;
static float                  s_controller_opti_swr_1st_C       = 0.0f;
static float                  s_controller_opti_swr_2nd         = 0.0f;
static float                  s_controller_opti_swr_2nd_L       = 0.0f;
static float                  s_controller_opti_swr_2nd_C       = 0.0f;
static uint8_t                s_controller_bad_swr_ctr          = 0U;
static float                  s_controller_best_swr             = 0.0f;
static float                  s_controller_best_swr_L           = 0.0f;
static float                  s_controller_best_swr_C           = 0.0f;
static ControllerOptiCVH_t    s_controller_best_swr_CVH         = ControllerOptiCVH__CV;
#if 0
static vector<float>          s_controller_xnull_L;
static vector<float>          s_controller_xnull_C;
static float                  s_controller_xnull_LC_ratio       = 0.0f;
#endif

static float                  s_controller_adc_bat_mv           = 0.0f;
static float                  s_controller_adc_temp_deg         = 0.0f;
static float                  s_controller_adc_fwd_mv           = 0.0f;
static float                  s_controller_adc_rev_mv           = 0.0f;
static float                  s_controller_adc_vdiode_mv        = 0.0f;
static float                  s_controller_adc_swr              = 0.0f;
static float                  s_controller_adc_fwd_mw           = 0.0f;
static float                  s_controller_adc_rev_mw           = 0.0f;

static _Bool                  s_controller_doCycle              = false;
static _Bool                  s_controller_doAdc                = false;

static DefaultMcuClocking_t s_controller_McuClocking          = DefaultMcuClocking__4MHz_MSI;



/* Controller's welcome */

const char controllerGreetMsg01[] = "\r\n";
const char controllerGreetMsg02[] = "+=======================================================+\r\n";
const char controllerGreetMsg03[] = "*                                                       *\r\n";
const char controllerGreetMsg04[] = "*  DL0WH BiTuner - ARM Cortex M4  powered by STM32L476  *\r\n";

const char controllerGreetMsg11[] =
    "\tDL0WH BiTuner version:\r\n"
    "\t=====================\r\n"
    "\r\n"
    "\t\tFirmware date\t%08lu\r\n"
    "\t\tDeveloped by\tDF4IAH\r\n";

static void controllerGreet(void)
{
  char verBuf[128];

  interpreterClearScreen();

  snprintf(verBuf, sizeof(verBuf) - 1, controllerGreetMsg11, BITUNER_CTRL_VERSION);

  /* usbToHost block */
  {
    interpreterConsolePush(controllerGreetMsg02, strlen(controllerGreetMsg02));
    interpreterConsolePush(controllerGreetMsg03, strlen(controllerGreetMsg03));
    interpreterConsolePush(controllerGreetMsg04, strlen(controllerGreetMsg04));
    interpreterConsolePush(controllerGreetMsg03, strlen(controllerGreetMsg03));
    interpreterConsolePush(controllerGreetMsg02, strlen(controllerGreetMsg02));
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01));
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01));

    interpreterConsolePush(verBuf, strlen(verBuf));
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01));
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01));
  }
}

const char controllerPackages0x00[] = "LQFP64";
const char controllerPackages0x02[] = "LQFP100";
const char controllerPackages0x03[] = "UFBGA132";
const char controllerPackages0x04[] = "LQFP144, WLCSP81 or WLCSP72";
const char controllerPackages0x10[] = "UFBGA169";
const char controllerPackages0x11[] = "WLCSP100";
const char controllerPackages0xXX[] = "(reserved)";
static void controllerPrintMCU(void)
{
  char lotBuf[8];
  char buf[220] = { 0 };
  const char *packagePtr = NULL;

  uint32_t uidPosX    = (*((uint32_t*)  UID_BASE     )      ) & 0X0000ffffUL;
  uint32_t uidPosY    = (*((uint32_t*)  UID_BASE     ) >> 16) & 0X0000ffffUL;
  uint32_t uidWaf     = (*((uint32_t*) (UID_BASE + 4))      ) & 0X000000ffUL;
  char* uidLot        = ((char*)       (UID_BASE + 5));
  memcpy((void*)lotBuf, (const void*)uidLot, 7);
  lotBuf[7] = 0;

  uint32_t package    = (*((uint32_t*)  PACKAGE_BASE))        & 0X0000001fUL;
  switch(package) {
  case 0x00:
    packagePtr = controllerPackages0x00;
    break;

  case 0x02:
    packagePtr = controllerPackages0x02;
    break;

  case 0x03:
    packagePtr = controllerPackages0x03;
    break;

  case 0x04:
    packagePtr = controllerPackages0x04;
    break;

  case 0x10:
    packagePtr = controllerPackages0x10;
    break;

  case 0x11:
    packagePtr = controllerPackages0x11;
    break;

  default:
    packagePtr = controllerPackages0xXX;
  }

  uint16_t flashSize      = (uint16_t) ((*((uint32_t*) FLASHSIZE_BASE)) & 0x0000ffffUL);

  const int len = snprintf(buf, sizeof(buf) - 1,
      "\r\n" \
      "\tMCU Info:\r\n" \
      "\t========\r\n" \
      "\r\n" \
      "\t\tLot-ID\t\t%s\r\n" \
      "\t\tWafer\t\t%lu\r\n" \
      "\t\tPos. X/Y\t%2lu/%2lu\r\n" \
      "\t\tPackage(s)\t%s\r\n" \
      "\t\tFlash size\t%4u kB\r\n\r\n\r\n",
      lotBuf, uidWaf, uidPosX, uidPosY, packagePtr, flashSize);
  interpreterConsolePush(buf, len);
}

static void controllerInitAfterGreet(void)
{
  /* Print MCU infos */
  controllerPrintMCU();
}


/* Calculation function */

uint32_t controllerCalcMsgHdr(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd)
{
  return ((uint32_t)dst << 24U) | ((uint32_t)src << 16U) | ((uint32_t)lengthBytes << 8U) | ((uint32_t)cmd);
}

uint32_t controllerCalcMsgInit(uint32_t* ary, ControllerMsgDestinations_t dst, uint32_t startDelayMs)
{
  ary[0] = controllerCalcMsgHdr(dst, Destinations__Controller, sizeof(uint32_t), MsgController__InitDo);
  ary[1] = startDelayMs;
  return 2UL;
}

float controllerCalcMatcherL2nH(uint8_t Lval)
{
  float sum = Controller_L0_nH;

  for (uint8_t idx = 0; idx < 8; idx++) {
    if (Lval & (1U << idx)) {
      sum += Controller_Ls_nH[idx];
    }
  }
  return sum;
}

float controllerCalcMatcherC2pF(uint8_t Cval)
{
  float sum = Controller_C0_pF;

  for (uint8_t idx = 0; idx < 8; idx++) {
    if (Cval & (1U << idx)) {
      sum += Controller_Cp_pF[idx];
    }
  }
  return sum;
}

uint8_t controllerCalcMatcherNH2L(float nH)
{
  float   absMin  = 1e+3f;
  uint8_t valThis = 0U;

  for (uint16_t val = 0U; val < 256U; val++) {
    const float valL    = controllerCalcMatcherL2nH(val);
    const float absThis = fabs(valL - nH);

    if (absMin > absThis) {
      absMin = absThis;
      valThis = val;
    }
  }
  return valThis;
}

uint8_t controllerCalcMatcherPF2C(float pF)
{
  float   absMin  = 1e+3f;
  uint8_t valThis = 0U;

  for (uint16_t val = 0U; val < 256U; val++) {
    const float valC    = controllerCalcMatcherC2pF(val);
    const float absThis = fabs(valC - pF);

    if (absMin > absThis) {
      absMin  = absThis;
      valThis = val;
    }
  }
  return valThis;
}

#if 0
static void controllerCalcMatcherLcRatio(void)
{
  if (!s_controller_xnull_L.empty() && !s_controller_xnull_LC_ratio) {
    const int   vCnt    = s_controller_xnull_L.size();
    const float deltaL  = s_controller_xnull_L[vCnt - 1] - s_controller_xnull_L[0];
    const float deltaC  = s_controller_xnull_C[vCnt - 1] - s_controller_xnull_C[0];
    char buf[64];

    s_controller_xnull_LC_ratio = (deltaC != 0.0f) ?  deltaL / deltaC : 0.0f;

    const int bufLen = snprintf(buf, sizeof(buf) - 1,
        "controllerCalcMatcherLcRatio: new xnull_LC_ratio= %f, deltaL= %f, deltaC= %f.\r\n",
        s_controller_xnull_LC_ratio,
        deltaL, deltaC);
    interpreterConsolePush(buf, bufLen);
  }
}
#endif


/* Messaging functions */

uint32_t controllerMsgPushToInQueue(uint32_t msgLen, uint32_t* msgAry, uint32_t waitMs)
{
  uint32_t idx = 0UL;

  /* Sanity checks */
  if (!msgLen || (msgLen > CONTROLLER_MSG_Q_LEN) || !msgAry) {
    return 0UL;
  }

  if (!(EG_GLOBAL__Controller_CTRL_IS_RUNNING & xEventGroupGetBits(globalEventGroupHandle))) {
    /* Do not enter as long as Controller is not up */
    return 0UL;
  }

  /* Get semaphore to queue in */
  if (osOK == osSemaphoreWait(cQin_BSemHandle, waitMs)) {
    /* Push to the queue */
    while (idx < msgLen) {
      osMessagePut(controllerInQueueHandle, msgAry[idx++], 1UL);
    }

    /* Return semaphore */
    osSemaphoreRelease(cQin_BSemHandle);

    /* Door bell */
    xEventGroupSetBits(globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_IN);
  }

  return idx;
}

void controllerMsgPushToOutQueue(uint32_t msgLen, uint32_t* msgAry, uint32_t waitMs)
{
  osSemaphoreId semId = 0;

  /* Get semaphore to queue out */
  if(osOK != osSemaphoreWait(cQout_BSemHandle, waitMs)) {
    return;
  }

  /* Find active semaphores */
  switch ((ControllerMsgDestinations_t) (msgAry[0] >> 24)) {
  case Destinations__Rtos_Default:
    if (s_mod_start.rtos_Default) {
      semId = c2default_BSemHandle;
    }
    break;

  case Destinations__Interpreter:
    if (s_mod_start.Interpreter) {
      semId = c2interpreter_BSemHandle;
    }
    break;

  case Destinations__Network_USBtoHost:
    if (s_mod_start.network_USBtoHost) {
      semId = c2usbToHost_BSemHandle;
    }
    break;

  case Destinations__Network_USBfromHost:
    if (s_mod_start.network_USBfromHost) {
      semId = c2usbFromHost_BSemHandle;
    }
    break;

  case Destinations__Network_UartTx:
    if (s_mod_start.network_UartTx) {
      semId = c2uartTx_BSemHandle;
    }
    break;

  case Destinations__Network_UartRx:
    if (s_mod_start.network_UartRx) {
      semId = c2uartRx_BSemHandle;
    }
    break;

  case Destinations__Network_CatTx:
    if (s_mod_start.network_CatTx) {
      semId = c2catTx_BSemHandle;
    }
    break;

  case Destinations__Network_CatRx:
    if (s_mod_start.network_CatRx) {
      semId = c2catRx_BSemHandle;
    }
    break;

  default:
    semId = 0;
  }  // switch ()

  /* Message only for active modules/semaphores */
  if (semId) {
    /* Push to the queue */
    uint8_t idx = 0U;
    while (idx < msgLen) {
      osMessagePut(controllerOutQueueHandle, msgAry[idx++], osWaitForever);
    }

    /* Wakeup of module */
    /* Grant blocked module to request the queue */
    osSemaphoreRelease(semId);                                                                        // Give semaphore token to launch module

    /* Big Ben for the public */
    xEventGroupSetBits(  globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_OUT);
    osDelay(5UL);
    xEventGroupClearBits(globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_OUT);

    /* Hand over ctrlQout */
    osSemaphoreRelease(cQout_BSemHandle);
  }
}

static uint32_t controllerMsgPullFromInQueue(void)
{
  /* Prepare all fields */
  memset(&s_msg_in, 0, sizeof(s_msg_in));

  /* Process each message token */
  do {
    const osEvent evt = osMessageGet(controllerInQueueHandle, 1UL);
    if (osEventMessage == evt.status) {
      const uint32_t token = evt.value.v;

      /* Starting new message or go ahead with option bytes */
      if (!s_msg_in.bRemain) {
        if (!token) {
          return osErrorResource;
        }

        /* Store raw 32bit tokens */
        s_msg_in.rawLen = 0U;
        s_msg_in.rawAry[s_msg_in.rawLen++] = token;

        /* Message type */
        s_msg_in.msgDst   = (ControllerMsgDestinations_t)   (0xffUL & (token >> 24U));
        s_msg_in.msgSrc   = (ControllerMsgDestinations_t)   (0xffUL & (token >> 16U));
        s_msg_in.msgLen   =                                  0xffUL & (token >>  8U) ;
        s_msg_in.msgCmd   = (ControllerCmds_t) (0xffUL &  token        );

        /* Init option fields */
        s_msg_in.bRemain  = s_msg_in.msgLen;
        s_msg_in.optLen   = 0U;
        memset(s_msg_in.optAry, 0, sizeof(s_msg_in.optAry));

      } else {
        /* Option fields */
        const uint8_t cnt = min(4U, s_msg_in.bRemain);

        /* Store raw 32bit tokens */
        s_msg_in.rawAry[s_msg_in.rawLen++] = token;

        for (uint8_t idx = 0; idx < cnt; ++idx) {
          const uint8_t byte = (uint8_t) (0xffUL & (token >> ((3U - idx) << 3U)));
          s_msg_in.optAry[s_msg_in.optLen++] = byte;
          s_msg_in.bRemain--;
        }
      }

    } else {
      return osErrorResource;
    }
  } while (s_msg_in.bRemain);

  return osOK;
}

uint32_t controllerMsgPullFromOutQueue(uint32_t* msgAry, ControllerMsgDestinations_t dst, uint32_t waitMs)
{
  uint32_t len = 0UL;

  /* Get semaphore to queue out */
  osSemaphoreWait(cQout_BSemHandle, waitMs);

  do {
    osEvent ev = osMessagePeek(controllerOutQueueHandle, 1UL);
    if (ev.status == osEventMessage) {
      const uint32_t  hdr       = ev.value.v;
      uint32_t        lenBytes  = 0xffUL & (hdr >> 8U);

      if (dst == (0xffUL & (hdr >> 24))) {
        (void) osMessageGet(controllerOutQueueHandle, 1UL);
        msgAry[len++] = hdr;

        while (lenBytes) {
          /* Push token into array */
          ev = osMessageGet(controllerOutQueueHandle, 1UL);
          const uint32_t opt = ev.value.v;
          msgAry[len++] = opt;

          /* Count off bytes transferred */
          if (lenBytes > 4) {
            lenBytes -= 4U;
          } else {
            lenBytes = 0U;
          }
        }
        break;

      } else {
        /* Strip off unused message */
        const uint8_t cnt = 1U + ((lenBytes + 3U) / 4U);

        for (uint8_t idx = 0; idx < cnt; idx++) {
          (void) osMessageGet(controllerOutQueueHandle, 1UL);
        }
      }

    } else {
      /* No more messages available */
      break;
    }
  } while (true);

  /* Return semaphore */
  osSemaphoreRelease(cQout_BSemHandle);

  return len;
}


/* FSM functions */

static void controllerFSM_LogAutoFinished(void)
{
  /* SWR: we have got it */

  s_controller_FSM_state = ControllerFsm__done;
  s_controller_doAdc     = true;

  /* Logging */
  {
    char buf[128];

    const int len = snprintf(buf, sizeof(buf) - 1,
        "Controller FSM: ControllerFsm__findImagZeroL - SWR good enough - tuner has finished.\r\n");
    interpreterConsolePush(buf, len);
  }
}

static void controllerFSM_LogState(void)
{
  /* Show current state of optimization */
  char      buf[512];

  int32_t   s_controller_opti_swr_1st_i;
  uint32_t  s_controller_opti_swr_1st_f;

  int32_t   s_controller_opti_swr_1st_L_i;
  uint32_t  s_controller_opti_swr_1st_L_f;

  int32_t   s_controller_opti_swr_1st_C_i;
  uint32_t  s_controller_opti_swr_1st_C_f;

  int32_t   s_controller_opti_swr_2nd_i;
  uint32_t  s_controller_opti_swr_2nd_f;

  int32_t   s_controller_opti_swr_2nd_L_i;
  uint32_t  s_controller_opti_swr_2nd_L_f;

  int32_t   s_controller_opti_swr_2nd_C_i;
  uint32_t  s_controller_opti_swr_2nd_C_f;

  int       len;

  s_controller_30ms_cnt += 30UL;

  len = snprintf(buf, sizeof(buf) - 1,
                "\r\nController FSM:\tcontrollerFSM_LogState: time= %5lu ms (iteration= %03lu)\r\n" \
                "\ta)\t\t FSM_state= %u, optiLC= %c, optiStrat= %u, optiUpDn= %s, optiCVH= %s:\r\n" \
                "\tb)\t\t opti_L= %5lu nH (%03u), opti_C= %5lu pF (%03u),\r\n" \
                "\tc)\t\t opti_CVHpongCtr= %u, opti_LCpongCtr= %u, bad_swr_ctr= %02u,\r\n",
                s_controller_30ms_cnt,
                (s_controller_30ms_cnt / 30),
                s_controller_FSM_state,
                (s_controller_FSM_optiLC   == ControllerOptiLC__L ?  'L' : 'C'),
                s_controller_FSM_optiStrat,
                (s_controller_FSM_optiUpDn == ControllerOptiUpDn__Up ?  "Up" : "Dn"),
                (s_controller_FSM_optiCVH  == ControllerOptiCVH__CV  ?  "CV" : "CH"),
                (uint32_t)s_controller_opti_L, s_controller_opti_L_relays,
                (uint32_t)s_controller_opti_C, s_controller_opti_C_relays,
                s_controller_opti_CVHpongCtr, s_controller_opti_LCpongCtr, s_controller_bad_swr_ctr);
  interpreterConsolePush(buf, len);

  mainCalcFloat2IntFrac(s_controller_opti_swr_1st,    3, &s_controller_opti_swr_1st_i,    &s_controller_opti_swr_1st_f  );
  mainCalcFloat2IntFrac(s_controller_opti_swr_1st_L,  1, &s_controller_opti_swr_1st_L_i,  &s_controller_opti_swr_1st_L_f);
  mainCalcFloat2IntFrac(s_controller_opti_swr_1st_C,  1, &s_controller_opti_swr_1st_C_i,  &s_controller_opti_swr_1st_C_f);
  mainCalcFloat2IntFrac(s_controller_opti_swr_2nd,    3, &s_controller_opti_swr_2nd_i,    &s_controller_opti_swr_2nd_f  );
  mainCalcFloat2IntFrac(s_controller_opti_swr_2nd_L,  1, &s_controller_opti_swr_2nd_L_i,  &s_controller_opti_swr_2nd_L_f);
  mainCalcFloat2IntFrac(s_controller_opti_swr_2nd_C,  1, &s_controller_opti_swr_2nd_C_i,  &s_controller_opti_swr_2nd_C_f);
  len = snprintf(buf, sizeof(buf) - 1,
                "\td)\t\t swr_1st= %2ld.%03lu, L= %5ld.%01lu nH, C= %5ld.%01lu pF # swr_2nd= %2ld.%03lu, L= %5ld.%01lu nH, C= %5ld.%01lu pF,\r\n",
                s_controller_opti_swr_1st_i,        s_controller_opti_swr_1st_f,
                s_controller_opti_swr_1st_L_i,      s_controller_opti_swr_1st_L_f,
                s_controller_opti_swr_1st_C_i,      s_controller_opti_swr_1st_C_f,
                s_controller_opti_swr_2nd_i,        s_controller_opti_swr_2nd_f,
                s_controller_opti_swr_2nd_L_i,      s_controller_opti_swr_2nd_L_f,
                s_controller_opti_swr_2nd_C_i,      s_controller_opti_swr_2nd_C_f
               );
  interpreterConsolePush(buf, len);

  int32_t   swr_i, best_swr_i;
  uint32_t  swr_f, best_swr_f;

  mainCalcFloat2IntFrac(s_controller_adc_swr,   3, &swr_i,      &swr_f);
  mainCalcFloat2IntFrac(s_controller_best_swr,  3, &best_swr_i, &best_swr_f);
  len = snprintf(buf, sizeof(buf) - 1,
                "\te)\t\t fwd_mv=%5lu mV, fwd_mw=%5lu mW,\r\n" \
                "\tf)\t\t swr= %2ld.%03lu, best_swr= %2ld.%03lu @ CVH= %u: L= %5lu nH, C= %5lu pF.\r\n\r\n",
                (uint32_t)s_controller_adc_fwd_mv, (uint32_t)s_controller_adc_fwd_mw,
                swr_i, swr_f,  best_swr_i, best_swr_f,  s_controller_best_swr_CVH, (uint32_t)s_controller_best_swr_L, (uint32_t)s_controller_best_swr_C);
  interpreterConsolePush(buf, len);
}

static void controllerFSM_GetGlobalVars(void)
{
  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    s_controller_adc_bat_mv     = g_adc_bat_mv;
    s_controller_adc_temp_deg   = g_adc_temp_deg;
    s_controller_adc_fwd_mv     = g_adc_fwd_mv;
    s_controller_adc_rev_mv     = g_adc_rev_mv;
    s_controller_adc_vdiode_mv  = g_adc_vdiode_mv;
    s_controller_adc_swr        = g_adc_swr;

    if (s_controller_adc_swr < Controller_AutoSWR_SWR_Init) {
      /* Add current data to the maps */
      if (s_controller_adc_swr < s_controller_opti_swr_1st) {
        /* Move back */
        s_controller_opti_swr_2nd   = s_controller_opti_swr_1st;
        s_controller_opti_swr_2nd_L = s_controller_opti_swr_1st_L;
        s_controller_opti_swr_2nd_C = s_controller_opti_swr_1st_C;

        /* Enter */
        s_controller_opti_swr_1st   = s_controller_adc_swr;
        s_controller_opti_swr_1st_L = s_controller_opti_L;
        s_controller_opti_swr_1st_C = s_controller_opti_C;

      } else if (s_controller_adc_swr < s_controller_opti_swr_2nd) {
        /* Enter */
        s_controller_opti_swr_2nd   = s_controller_adc_swr;
        s_controller_opti_swr_2nd_L = s_controller_opti_L;
        s_controller_opti_swr_2nd_C = s_controller_opti_C;
      }
    }

    taskENABLE_INTERRUPTS();

    /* FWD power calculation */
    {
      const float fwdMv = mainCalc_mV_to_mW(s_controller_adc_fwd_mv);
      const float revMv = mainCalc_mV_to_mW(s_controller_adc_rev_mv);

      /* Disabled IRQ section */
      taskDISABLE_INTERRUPTS();

      s_controller_adc_fwd_mw = fwdMv;
      s_controller_adc_rev_mw = revMv;

      taskENABLE_INTERRUPTS();
    }


#if 0
    /* Logging */
    {
      int32_t   l_adc_temp_deg_i    = 0L;
      uint32_t  l_adc_temp_deg_f100 = 0UL;
      char      dbgBuf[128];
      float     s_controller_adc_revint_val;
      float     s_controller_adc_vref_mv;

      /* Disabled IRQ section */
      taskDISABLE_INTERRUPTS();

      s_controller_adc_revint_val = g_adc_refint_val;
      s_controller_adc_vref_mv    = g_adc_vref_mv;

      taskENABLE_INTERRUPTS();


      mainCalcFloat2IntFrac(s_controller_adc_temp_deg, 2, &l_adc_temp_deg_i, &l_adc_temp_deg_f100);

      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC1: refint_val = %4d, Vref = %4d mV, Bat = %4d mV, Temp = %+3ld.%02luC\r\n",
          (int16_t) (s_controller_adc_revint_val + 0.5f),
          (int16_t) (s_controller_adc_vref_mv    + 0.5f),
          (int16_t) (s_controller_adc_bat_mv     + 0.5f),
          l_adc_temp_deg_i, l_adc_temp_deg_f100);
      interpreterConsolePush(dbgBuf, dbgLen);
    }
#endif

#if 1
    /* Logging */
    {
      char  dbgBuf[128];

      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC2: FWD = %5d mV\r\n",
          (int16_t) (s_controller_adc_fwd_mv + 0.5f));
      interpreterConsolePush(dbgBuf, dbgLen);
    }
#endif

#if 1
    /* Logging */
    {
      char      dbgBuf[128];
      int32_t   l_swr_i;
      uint32_t  l_swr_f100;

      mainCalcFloat2IntFrac(s_controller_adc_swr, 2, &l_swr_i, &l_swr_f100);

      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC2: REV = %5d mV, SWR = %+3ld.%03lu\r\n",
          (int16_t) (s_controller_adc_rev_mv + 0.5f),
          l_swr_i, l_swr_f100);
      interpreterConsolePush(dbgBuf, dbgLen);
    }
#endif

#if 0
    /* Logging */
    {
      char  dbgBuf[128];

      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC3: Vdiode = %4d mV\r\n",
          (int16_t) (s_controller_adc_vdiode_mv + 0.5f));
      interpreterConsolePush(dbgBuf, dbgLen);
    }
#endif
  }
}

static void controllerFSM_PushOptiVars(void)
{
  uint8_t controller_opti_CV;
  uint8_t controller_opti_CH;
  uint32_t msgAry[2];

  /* Calculate current L and C counter settings */
  /* Disabled IRQ section */
  /* { */
    taskDISABLE_INTERRUPTS();
    const float               valL      = s_controller_opti_L;
    const float               valC      = s_controller_opti_C;
    const ControllerOptiCVH_t configLC  = s_controller_FSM_optiCVH;
    taskENABLE_INTERRUPTS();
  /* } */

  const uint8_t relL = controllerCalcMatcherNH2L(valL);
  const uint8_t relC = controllerCalcMatcherPF2C(valC);

  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();
    s_controller_opti_L_relays = relL;
    s_controller_opti_C_relays = relC;
    taskENABLE_INTERRUPTS();
  }

  /* Update CV/CH state */
  if (configLC == ControllerOptiCVH__CH) {
    controller_opti_CV  = 0U;
    controller_opti_CH  = 1U;

  } else {
    controller_opti_CV  = 1U;
    controller_opti_CH  = 0U;
  }

  /* Three extra bytes to take over */
  msgAry[0] = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 3, MsgDefault__SetVar03_C_L_CV_CH);

  /* Compose relay bitmap */
  msgAry[1] = ((uint32_t)controller_opti_CH << 17) |
              ((uint32_t)controller_opti_CV << 16) |
              ((uint32_t)relL               <<  8) |
              ((uint32_t)relC                    ) ;

  controllerMsgPushToOutQueue(sizeof(msgAry) / sizeof(uint32_t), msgAry, osWaitForever);
}

static void controllerFSM_StartAdc(void)
{
  s_controller_doAdc = true;
}

static _Bool controllerFSM_CheckPower(void)
{
  if ((Controller_AutoSWR_P_mW_Min > s_controller_adc_fwd_mw) || (s_controller_adc_fwd_mw > Controller_AutoSWR_P_mW_Max)) {
#if 0
    /* Logging */
    {
      char      buf[128];
      int32_t   pwr_i;
      uint32_t  pwr_f;

      mainCalcFloat2IntFrac(s_controller_adc_fwd_mw, 3, &pwr_i, &pwr_f);
      const int len = snprintf(buf, sizeof(buf) - 1,
                              "Controller FSM: power= %5ld.%03lu out of [%u .. %u] Watts - stop auto tuner.\r\n",
                              pwr_i, pwr_f,
                              (uint16_t)Controller_AutoSWR_P_mW_Min, (uint16_t)Controller_AutoSWR_P_mW_Max);
      interpreterConsolePush(buf, len);
    }
#endif

    /* Reset SWR start timer */
    s_controller_swr_tmr = osKernelSysTick();

    /* Overdrive or to low energy */
    s_controller_FSM_state = ControllerFsm__doAdc;

    return true;
  }
  return false;
}

static _Bool controllerFSM_CheckSwrTime(void)
{
  if (s_controller_adc_swr < Controller_AutoSWR_SWR_Min) {
    /* Logging */
    {
      char buf[128];
      int32_t   swr_i;
      uint32_t  swr_f;

      mainCalcFloat2IntFrac(s_controller_adc_swr, 3, &swr_i, &swr_f);
      const int len = snprintf(buf, sizeof(buf) - 1,
                              "Controller FSM: VSWR= %2ld.%03lu is good enough - stop auto tuner.\r\n",
                              swr_i, swr_f);
      interpreterConsolePush(buf, len);
    }

    /* No need to start */
    s_controller_FSM_state = ControllerFsm__doAdc;

    return true;

  } else if (Controller_AutoSWR_Time_ms_Max > (osKernelSysTick() - s_controller_swr_tmr)) {
    /* Timer has not yet elapsed */
    s_controller_FSM_state = ControllerFsm__doAdc;

    return true;
  }
  return false;
}

static void controllerFSM_SwitchOverCVH(void)
{
  /* Check if another CVH switch over is allowed */
  if (++s_controller_opti_CVHpongCtr <= Controller_AutoSWR_CVHpong_Max) {
    s_controller_opti_LCpongCtr = 0U;
    s_controller_bad_swr_ctr    = 0U;

    /* Logging */
    {
      char      buf[128];
      const int len = snprintf(buf, sizeof(buf) - 1,
                              "Controller FSM: switch over the constellation to %d (0: CV, 1: CH), CVH pingpong ctr= %u.\r\n",
                              !s_controller_FSM_optiCVH, s_controller_opti_CVHpongCtr);
      interpreterConsolePush(buf, len);
    }

    /* Switch over to opposite CVH constellation */
    s_controller_FSM_optiCVH      = (s_controller_FSM_optiCVH == ControllerOptiCVH__CV) ?  ControllerOptiCVH__CH : ControllerOptiCVH__CV;
    if (s_controller_FSM_optiCVH == ControllerOptiCVH__CH) {
      s_controller_FSM_optiLC     = ControllerOptiLC__C;
      s_controller_opti_C         = Controller_C0_pF + Controller_Cp_pF[0];

    } else {
      s_controller_FSM_optiLC     = ControllerOptiLC__L;
      s_controller_opti_L         = Controller_L0_nH + Controller_Ls_nH[0];
    }

    s_controller_FSM_optiStrat    = ControllerOptiStrat__Double;
    s_controller_FSM_optiUpDn     = ControllerOptiUpDn__Up;
    s_controller_FSM_state        = ControllerFsm__findImagZero;

    /* Erase array for new L/C combinations */
    s_controller_opti_swr_1st     = Controller_AutoSWR_SWR_Init;
    s_controller_opti_swr_2nd     = Controller_AutoSWR_SWR_Init;

  } else {
    /* Exhausted - start over again */
    s_controller_FSM_state        = ControllerFsm__done;
    s_controller_doAdc            = true;
  }
}

static void controllerFSM_DoubleStrategy(void)
{
  if (s_controller_FSM_optiLC == ControllerOptiLC__L) {
    //const float delta_L = s_controller_opti_L;

    /* Double L */
    s_controller_opti_L += s_controller_opti_L;

    #if 0
    /* Parallel strategy */
    if (s_controller_xnull_LC_ratio != 0.0f) {
      s_controller_opti_C += 0.1f * (delta_L / s_controller_xnull_LC_ratio);
    }
    #endif

  } else {
    //const float delta_C = s_controller_opti_C;

    /* Double C */
    s_controller_opti_C += s_controller_opti_C;

    #if 0
    /* Parallel strategy */
    if (s_controller_xnull_LC_ratio != 0.0f) {
      s_controller_opti_C += 0.1f * (delta_C * s_controller_xnull_LC_ratio);
    }
    #endif
  }
}

static void controllerFSM_HalfStrategy(void)
{
  if (s_controller_FSM_optiLC == ControllerOptiLC__L) {
    //const float lastL = s_controller_opti_L;

    /* Mean L */
    s_controller_opti_L = (s_controller_opti_swr_1st_L + s_controller_opti_swr_2nd_L) / 2.0f;

    #if 0
    /* Parallel strategy */
    if (s_controller_xnull_LC_ratio != 0.0f) {
      const float deltaL = s_controller_opti_L - lastL;

      s_controller_opti_C *= deltaL / s_controller_xnull_LC_ratio;
    }
    #endif

  } else {
    //const float lastC = s_controller_opti_C;

    /* Mean C */
    s_controller_opti_C = (s_controller_opti_swr_1st_C + s_controller_opti_swr_2nd_C) / 2.0f;

    #if 0
    /* Parallel strategy */
    if (s_controller_xnull_LC_ratio != 0.0f) {
      const float deltaC = s_controller_opti_C - lastC;

      s_controller_opti_C *= deltaC * s_controller_xnull_LC_ratio;
    }
    #endif
  }
}

static void controllerFSM_ZeroXHalfStrategy(void)
{
  if (s_controller_FSM_optiStrat != ControllerOptiStrat__Half) {
    return;
  }

  /* Work out the half strategy */
  controllerFSM_HalfStrategy();

  if (s_controller_FSM_optiLC == ControllerOptiLC__L) {
    const float L_diff_abs = fabs(s_controller_opti_swr_1st_L - s_controller_opti_swr_2nd_L);

    //printf("controllerFSM_ZeroXHalfStrategy 1: abs(swr_1st_L - swr_2nd_L)=%f\r\n", L_diff_abs);
    if (L_diff_abs <= Controller_Ls_nH[0]) {
      #if 0
      /* First zero X found */
      s_controller_xnull_L.push_back(s_controller_opti_L);
      s_controller_xnull_C.push_back(s_controller_opti_C);
      #endif

      /* Check if optimum found by increase of L */
      if (s_controller_opti_swr_1st_L > Controller_Ls_nH[0]) {
        /* Advance L, at least by one step, limiting */
        const float advanced = s_controller_opti_swr_1st_L * 1.4f + Controller_Ls_nH[0];
        if (advanced > 2.0f * Controller_Ls_nH[7]) {
          s_controller_opti_L = controllerCalcMatcherNH2L( controllerCalcMatcherL2nH(advanced) );

        } else {
          s_controller_opti_L = advanced;
        }

        /* Next optimize C */
        s_controller_opti_C         = Controller_C0_pF + Controller_Cp_pF[0];
        s_controller_opti_LCpongCtr = 0U;
        s_controller_FSM_optiLC     = ControllerOptiLC__C;
        s_controller_FSM_optiStrat  = ControllerOptiStrat__Double;
        s_controller_FSM_optiUpDn   = ControllerOptiUpDn__Up;
        s_controller_FSM_state      = ControllerFsm__findMinSwr;

        /* Forget this minimum due to C advance */
        s_controller_adc_swr        = Controller_AutoSWR_SWR_Init;

        /* Erase array for new L/C combinations */
        s_controller_opti_swr_1st   = Controller_AutoSWR_SWR_Init;
        s_controller_opti_swr_2nd   = Controller_AutoSWR_SWR_Init;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_SwitchOverCVH();
      }

    }

  } else {
    const float C_diff_abs = fabs(s_controller_opti_swr_1st_C - s_controller_opti_swr_2nd_C);

    //printf("controllerFSM_ZeroXHalfStrategy 2: abs(swr_1st_C - swr_2nd_C)=%f\r\n", C_diff_abs);
    if (C_diff_abs <= Controller_Cp_pF[0]) {
      #if 0
      /* First zero X found */
      s_controller_xnull_C.push_back(s_controller_opti_C);
      s_controller_xnull_L.push_back(s_controller_opti_L);
      #endif

      /* Check if optimum found by increase of C */
      if (s_controller_opti_swr_1st_C > Controller_Cp_pF[0]) {
        /* Advance C, at least by one step, limiting */
        const float advanced = s_controller_opti_swr_1st_C * 1.4f + Controller_Cp_pF[0];
        if (advanced > 2.0f * Controller_Cp_pF[7]) {
          s_controller_opti_C = controllerCalcMatcherPF2C( controllerCalcMatcherC2pF(advanced) );
        } else {
          s_controller_opti_C = advanced;
        }

        /* Next optimize L */
        s_controller_opti_L         = Controller_L0_nH + Controller_Ls_nH[0];
        s_controller_opti_LCpongCtr = 0U;
        s_controller_FSM_optiLC     = ControllerOptiLC__L;
        s_controller_FSM_optiStrat  = ControllerOptiStrat__Double;
        s_controller_FSM_optiUpDn   = ControllerOptiUpDn__Up;
        s_controller_FSM_state      = ControllerFsm__findMinSwr;

        /* Forget this minimum due to L advance */
        s_controller_adc_swr        = Controller_AutoSWR_SWR_Init;

        /* Erase array for new L/C combinations */
        s_controller_opti_swr_1st   = Controller_AutoSWR_SWR_Init;
        s_controller_opti_swr_2nd   = Controller_AutoSWR_SWR_Init;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_SwitchOverCVH();
      }
    }
  }
}

static void controllerFSM_OptiHalfStrategy(void)
{
  if (s_controller_FSM_optiStrat != ControllerOptiStrat__Half) {
    return;
  }

  /* Work out the half strategy */
  controllerFSM_HalfStrategy();

  if (s_controller_FSM_optiLC == ControllerOptiLC__L) {
    const float L_diff_abs = fabs(s_controller_opti_swr_1st_L - s_controller_opti_swr_2nd_L);

    //printf("controllerFSM_OptiHalfStrategy 1: abs(swr_1st_L - swr_2nd_L)=%f\r\n", L_diff_abs);
    if (L_diff_abs <= Controller_Ls_nH[0]) {
      #if 0
      /* Again, zero X found */
      s_controller_xnull_L.push_back(s_controller_opti_L);
      s_controller_xnull_C.push_back(s_controller_opti_C);
      controllerCalcMatcherLcRatio();
      #endif

      if (++s_controller_opti_LCpongCtr < Controller_AutoSWR_LCpong_Max) {
        /* Optimize C, again */
        s_controller_opti_C         = Controller_C0_pF + Controller_Cp_pF[0];
        s_controller_FSM_optiLC     = ControllerOptiLC__C;
        s_controller_FSM_optiStrat  = ControllerOptiStrat__Double;
        s_controller_FSM_optiUpDn   = ControllerOptiUpDn__Up;
        s_controller_FSM_state      = ControllerFsm__findMinSwr;

        /* Erase array for new L/C combinations */
        s_controller_opti_swr_1st   = Controller_AutoSWR_SWR_Init;
        s_controller_opti_swr_2nd   = Controller_AutoSWR_SWR_Init;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_SwitchOverCVH();
      }
    }

  } else {
    const float C_diff_abs = fabs(s_controller_opti_swr_1st_C - s_controller_opti_swr_2nd_C);

    //printf("controllerFSM_OptiHalfStrategy 2: abs(swr_1st_C - swr_2nd_C)=%f\r\n", C_diff_abs);
    if (C_diff_abs <= Controller_Cp_pF[0]) {
      #if 0
      /* Again, zero X found */
      s_controller_xnull_L.push_back(s_controller_opti_L);
      s_controller_xnull_C.push_back(s_controller_opti_C);
      controllerCalcMatcherLcRatio();
      #endif

      if (++s_controller_opti_LCpongCtr < Controller_AutoSWR_LCpong_Max) {
        /* Optimize L, again */
        s_controller_opti_L         = Controller_L0_nH + Controller_Ls_nH[0];
        s_controller_FSM_optiLC     = ControllerOptiLC__L;
        s_controller_FSM_optiStrat  = ControllerOptiStrat__Double;
        s_controller_FSM_optiUpDn   = ControllerOptiUpDn__Up;
        s_controller_FSM_state      = ControllerFsm__findMinSwr;

        /* Erase array for new L/C combinations */
        s_controller_opti_swr_1st   = Controller_AutoSWR_SWR_Init;
        s_controller_opti_swr_2nd   = Controller_AutoSWR_SWR_Init;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_SwitchOverCVH();
      }
    }
  }
}

static void controllerFSM(void)
{
  switch (s_controller_FSM_state)
  {

  case ControllerFsm__NOP:
  case ControllerFsm__doAdc:
  {
    /* Init SWR compare value */
    s_controller_adc_swr = Controller_AutoSWR_SWR_Init;

    /* Erase array for new L/C combinations */
    s_controller_opti_swr_1st   = Controller_AutoSWR_SWR_Init;
    s_controller_opti_swr_2nd   = Controller_AutoSWR_SWR_Init;

    #if 0
    /* Erase Xnull vector */
    s_controller_xnull_L.clear();
    s_controller_xnull_C.clear();
    s_controller_xnull_LC_ratio = 0.0f;
    #endif

    controllerFSM_StartAdc();

    s_controller_FSM_state = ControllerFsm__startAuto;
  }
    break;

  case ControllerFsm__startAuto:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    break;  // TODO: remove me!

    /* Check for security */
    if (controllerFSM_CheckPower())
      break;

    /* Check if auto tuner should start */
    if (controllerFSM_CheckSwrTime())
      break;

    /* Run (V)SWR optimization */
    s_controller_FSM_optiCVH      = ControllerOptiCVH__CV;
    s_controller_FSM_optiLC       = ControllerOptiLC__L;
    s_controller_FSM_optiStrat    = ControllerOptiStrat__Double;
    s_controller_FSM_optiUpDn     = ControllerOptiUpDn__Up;
    s_controller_FSM_state        = ControllerFsm__findImagZero;
    s_controller_opti_CVHpongCtr  = s_controller_opti_LCpongCtr = s_controller_bad_swr_ctr = 0U;
    s_controller_best_swr         = s_controller_adc_swr = Controller_AutoSWR_SWR_Init;
    s_controller_opti_L           = Controller_L0_nH + Controller_Ls_nH[0];
    s_controller_opti_C           = Controller_C0_pF;

    /* Logging */
    {
      char buf[128];

      const int len = snprintf(buf, sizeof(buf) - 1,
          "Controller FSM: ControllerFsm__startAuto - start auto tuner.\r\n");
      interpreterConsolePush(buf, len);
    }

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();
    controllerFSM_StartAdc();
  }
    break;

  case ControllerFsm__findImagZero:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Check for security */
    if (controllerFSM_CheckPower())
      break;

    /* Show current state of optimization */
    controllerFSM_LogState();

    /* Check if SWR is not usable in that constellation */
    if (s_controller_adc_swr > Controller_AutoSWR_SWR_Max) {
      if (s_controller_bad_swr_ctr++ >= Controller_AutoSWR_SWR_Max_Cnt) {
        /* Switch over to opposite CVH constellation and restart of growing L and C */
        s_controller_opti_L           = Controller_L0_nH + Controller_Ls_nH[0];
        s_controller_opti_C           = Controller_C0_pF + Controller_Cp_pF[0];
        controllerFSM_SwitchOverCVH();
        break;
      }

    } else {
      if (s_controller_bad_swr_ctr > 0UL) {
        s_controller_bad_swr_ctr--;
      }
    }

    /* Check for SWR */
    if (s_controller_adc_swr == s_controller_opti_swr_1st) {
      /* SWR got better */

      if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Min) {
        /* SWR: we have got it */
        controllerFSM_LogAutoFinished();
        break;
      }

    } else {
      /* SWR got worse */

      if (s_controller_FSM_optiStrat == ControllerOptiStrat__Double) {
        /* Change strategy */
        s_controller_FSM_optiStrat = ControllerOptiStrat__Half;
        s_controller_FSM_optiUpDn  = ControllerOptiUpDn__Dn;
      }
    }

    /* Execute the strategy */
    if (s_controller_FSM_optiStrat == ControllerOptiStrat__Double) {
      /* Double L/C for quick access */
      controllerFSM_DoubleStrategy();

    } else if (s_controller_FSM_optiStrat == ControllerOptiStrat__Half) {
      /* Half strategy for zero X configuration */
      controllerFSM_ZeroXHalfStrategy();
    }

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();
    controllerFSM_StartAdc();
  }
    break;

  case ControllerFsm__findMinSwr:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      break;
    }

    /* Show current state of optimization */
    controllerFSM_LogState();

    /* Check if SWR is not usable in that constellation */
    if (s_controller_adc_swr > Controller_AutoSWR_SWR_Max) {
      if (s_controller_bad_swr_ctr++ >= Controller_AutoSWR_SWR_Max_Cnt) {
        /* Switch over to opposite CVH constellation and restart of growing L and C */
        s_controller_opti_L           = Controller_L0_nH + Controller_Ls_nH[0];
        s_controller_opti_C           = Controller_C0_pF + Controller_Cp_pF[0];
        controllerFSM_SwitchOverCVH();
        break;
      }

    } else {
      /* Good SWR drops one bad */
      if (s_controller_bad_swr_ctr > 0UL) {
        s_controller_bad_swr_ctr--;
      }
    }

    /* Check for SWR */
    if (s_controller_adc_swr == s_controller_opti_swr_1st) {
      /* SWR got better */

      if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Min) {
        /* SWR: we have got it */
        controllerFSM_LogAutoFinished();
        break;
      }

    } else {
      /* SWR got worse */

      if (s_controller_FSM_optiStrat == ControllerOptiStrat__Double) {
        /* Change strategy */
        s_controller_FSM_optiStrat = ControllerOptiStrat__Half;
        s_controller_FSM_optiUpDn  = ControllerOptiUpDn__Dn;
      }
    }

    /* Execute the strategy */
    if (s_controller_FSM_optiStrat == ControllerOptiStrat__Double) {
      /* Double L/C for quick access */
      controllerFSM_DoubleStrategy();

    } else if (s_controller_FSM_optiStrat == ControllerOptiStrat__Half) {
      /* Half strategy */
      controllerFSM_OptiHalfStrategy();
    }

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();
    controllerFSM_StartAdc();
  }
    break;

  case ControllerFsm__done:
  {
    /* Take the best result */
    s_controller_FSM_optiCVH  = s_controller_best_swr_CVH;
    s_controller_opti_L       = s_controller_best_swr_L;
    s_controller_opti_C       = s_controller_best_swr_C;

    /* Show current state of optimization */
    controllerFSM_LogState();

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Check if current VSWR is good enough */
    if (s_controller_adc_swr > Controller_AutoSWR_SWR_Min) {
      /* Done - wait for new start */
      s_controller_FSM_state = ControllerFsm__doAdc;
    }

    controllerFSM_StartAdc();
  }
    break;

  default:
  {
    s_controller_FSM_state = ControllerFsm__NOP;
  }

  }  // switch ()
}


/* Model interaction */

static void controllerSetL(uint8_t relLnum, uint8_t relEnable)
{
  float valL;
  float fwd_mw;

  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    valL    = s_controller_opti_L;
    fwd_mw  = s_controller_adc_fwd_mw;

    taskENABLE_INTERRUPTS();
  }

  if (fwd_mw <= Controller_AutoSWR_P_mW_Max) {
    uint8_t relay = controllerCalcMatcherNH2L(s_controller_opti_L);
    if (relEnable) {
      relay |=   1UL << relLnum;

    } else {
      relay &= ~(1UL << relLnum);
    }
    valL = controllerCalcMatcherL2nH(relay);

    /* Disabled IRQ section */
    {
      taskDISABLE_INTERRUPTS();

      s_controller_opti_L = valL;

      taskENABLE_INTERRUPTS();
    }

    controllerFSM_PushOptiVars();

  } else {
    const char errMsg[] = "*** Power to high to set L ***\r\n\r\n";
    interpreterConsolePush(errMsg, strlen(errMsg));
  }
}

static void controllerSetC(uint8_t relLnum, uint8_t relEnable)
{
  float valC;
  float fwd_mw;

  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    valC = s_controller_opti_C;
    fwd_mw  = s_controller_adc_fwd_mw;

    taskENABLE_INTERRUPTS();
  }

  if (fwd_mw <= Controller_AutoSWR_P_mW_Max) {
    uint8_t relay = controllerCalcMatcherPF2C(valC);
    if (relEnable) {
      relay |=   1UL << relLnum;

    } else {
      relay &= ~(1UL << relLnum);
    }
    valC = controllerCalcMatcherC2pF(relay);

    /* Disabled IRQ section */
    {
      taskDISABLE_INTERRUPTS();

      s_controller_opti_C = valC;

      taskENABLE_INTERRUPTS();
    }

    controllerFSM_PushOptiVars();

  } else {
    const char errMsg[] = "*** Power to high to set C ***\r\n\r\n";
    interpreterConsolePush(errMsg, strlen(errMsg));
  }
}

static void controllerSetConfigLC(bool isLC)
{
  float fwd_mw;

  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    fwd_mw  = s_controller_adc_fwd_mw;

    taskENABLE_INTERRUPTS();
  }

  if (fwd_mw <= Controller_AutoSWR_P_mW_Max) {
    /* Disabled IRQ section */
    taskDISABLE_INTERRUPTS();

    s_controller_FSM_optiCVH = isLC ?  ControllerOptiCVH__CH : ControllerOptiCVH__CV;

    taskENABLE_INTERRUPTS();

    controllerFSM_PushOptiVars();

  } else {
    const char errMsg[] = "*** Power to high to set L/C mode ***\r\n\r\n";
    interpreterConsolePush(errMsg, strlen(errMsg));
  }
}

static void controllerSetCLExt(uint32_t relays)
{
  const uint8_t l_C_relays        = ( relays        &    0xffUL);
  const uint8_t l_L_relays        = ((relays >> 8U) &    0xffUL);
  const ControllerOptiCVH_t l_CVH = ( relays        & 0x10000UL) ?  ControllerOptiCVH__CV : ControllerOptiCVH__CH;

  /* Calculate corresponding C and L values */
  const float valC = controllerCalcMatcherC2pF(l_C_relays);
  const float valL = controllerCalcMatcherL2nH(l_L_relays);
  float fwd_mw;

  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    fwd_mw  = s_controller_adc_fwd_mw;

    taskENABLE_INTERRUPTS();
  }

  if (fwd_mw <= Controller_AutoSWR_P_mW_Max) {
    /* Disabled IRQ section */
    {
      taskDISABLE_INTERRUPTS();

      s_controller_opti_L_relays  = l_L_relays;
      s_controller_opti_C_relays  = l_C_relays;
      s_controller_FSM_optiCVH    = l_CVH;
      s_controller_opti_L         = valL;
      s_controller_opti_C         = valC;

      taskENABLE_INTERRUPTS();
    }

    controllerFSM_PushOptiVars();

  } else {
    const char errMsg[] = "*** Power to high to set C, L or C/L mode ***\r\n\r\n";
    interpreterConsolePush(errMsg, strlen(errMsg));
  }
}

static void controllerPrintLC(void)
{
  /* Disabled IRQ section */
  /* { */
    taskDISABLE_INTERRUPTS();

    const float valL                          = s_controller_opti_L;
    const float valC                          = s_controller_opti_C;
    const       ControllerOptiCVH_t configLC  = s_controller_FSM_optiCVH;

    taskENABLE_INTERRUPTS();
  /* } */

  /* Print relay settings */
  {
    const uint8_t relL  = controllerCalcMatcherNH2L(valL);
    const uint8_t relC  = controllerCalcMatcherPF2C(valC);
    char  buf[4]        = { ' ', ' ', '?' };
    char  strbuf[128]   = { 0 };
    uint32_t relays;

    relays  = ((uint32_t)relC <<  0U);
    relays |= ((uint32_t)relL <<  8U);
    relays |= configLC == ControllerOptiCVH__CV ?  0x10000UL : 0x20000UL;

    /* 1st line: header */
    const char* bufStr = "\r\n\r\n## CH CV  L8 L7 L6 L5 L4 L3 L2 L1  C8 C7 C6 C5 C4 C3 C2 C1\r\n##";
    interpreterConsolePush(bufStr, strlen(bufStr));

    /* 2nd line: bit field */
    for (int8_t idx = 17; idx >= 0; idx--) {
      if (idx == 15 || idx == 7) {
        interpreterConsolePush(" ", 1);
      }
      buf[2] = (relays & (1UL << idx)) != 0UL ?  '1' : '0';
      interpreterConsolePush(buf, 3);
    }

    /* 3rd line: hex number */
    const int len = snprintf(strbuf, (sizeof(strbuf) - 1), "\r\n## (Relay short settings: H%06lx)\r\n", (relays & 0x03ffffUL));
    interpreterConsolePush(strbuf, len);
  }

  /* 4th line: print L/C configuration and L, C values */
  {
    const char*    sConfig      = configLC == ControllerOptiCVH__CV ?  "C-L   normal Gamma" : "L-C reverted Gamma";
    char           strbuf[128]  = { 0 };

    const int len = snprintf(strbuf, (sizeof(strbuf) - 1), "## Config: %s\t L = %ld nH\t C = %ld pF\r\n\r\n", sConfig, (uint32_t)valL, (uint32_t)valC);
    interpreterConsolePush(strbuf, len);
  }

  interpreterShowCursor();
}


/* Timer functions */

static void controllerCyclicTimerEvent(void)
{
  /* Cyclic jobs to do */

  /* FSM logic */
  controllerFSM();

  /* Handle serial CAT interface packets */
  {
    //uint8_t inBuf[256]  = { 0U };
    //uint8_t inBufLen    = 0U;

    // TODO: coding here
  }
}

static void controllerCyclicStart(uint32_t period_ms)
{
  osTimerStart(controllerTimerHandle, period_ms);
}

static void controllerCyclicStop(void)
{
  osTimerStop(controllerTimerHandle);
}

void controllerTimerCallback(void const *argument)
{
  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Controller, Destinations__Controller, 0U, MsgController__CallFunc01_CyclicTimerEvent);
  controllerMsgPushToInQueue(msgLen, msgAry, 1UL);
}


/* Messaging */

static void controllerMsgProcessor(void)
{
  uint32_t msgAry[CONTROLLER_MSG_Q_LEN] = { 0 };

  if (!s_msg_in.rawLen) {
    return;
  }

  if (s_msg_in.msgDst > Destinations__Controller) {
    /* Forward message to the destination via the ctrlQout */
    const uint8_t cnt                     = s_msg_in.rawLen;
    uint8_t msgLen                        = 0U;

    /* Copy message header and option entries to the target */
    for (uint8_t idx = 0; idx < cnt; ++idx) {
      msgAry[msgLen++] = s_msg_in.rawAry[idx];
    }

    /* Push message out */
    controllerMsgPushToOutQueue(msgLen, msgAry, 10UL);

  } else {
    /* Message is for us */

    /* Register all ready modules */
    switch ((ControllerCmds_t) s_msg_in.msgCmd) {
    case MsgController__InitDone:
      {
        switch (s_msg_in.msgSrc) {
        case Destinations__Rtos_Default:
          s_mod_rdy.rtos_Default = 1U;

          /* Send MCU clocking set-up request */
          {
            uint8_t msgLen    = 0U;

            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 1U, MsgDefault__SetVar02_Clocking);
            msgAry[msgLen++]  = (s_controller_McuClocking << 24U);

            /* Start the RtosDefault cyclic timer each 30 ms */
            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 1U, MsgDefault__CallFunc02_CyclicTimerStart);
            msgAry[msgLen++]  = 30UL;

            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 1U, MsgDefault__CallFunc04_DigPot_SetGain);
            msgAry[msgLen++]  = 64;

            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 1U, MsgDefault__CallFunc05_DigPot_SetOffset);
            msgAry[msgLen++]  = 138;

            controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
          }
          break;

        case Destinations__Interpreter:
          s_mod_rdy.Interpreter = 1U;
          break;

        case Destinations__Network_USBtoHost:
          s_mod_rdy.network_USBtoHost = 1U;
          break;

        case Destinations__Network_USBfromHost:
          s_mod_rdy.network_USBfromHost = 1U;
          break;

        case Destinations__Network_UartTx:
          s_mod_rdy.network_UartTx = 1U;
          break;

        case Destinations__Network_UartRx:
          s_mod_rdy.network_UartRx = 1U;
          break;

        case Destinations__Network_CatTx:
          s_mod_rdy.network_CatTx = 1U;
          break;

        case Destinations__Network_CatRx:
          s_mod_rdy.network_CatRx = 1U;
          break;

        default:
          Error_Handler();
        }  // switch (s_msg_in.msgSrc)
      }  // case MsgController__InitDone { }
    break;

    /* Process own command */
    case MsgController__CallFunc01_CyclicTimerEvent:
      controllerCyclicTimerEvent();
      break;

    case MsgController__CallFunc04_Restart:
      SystemResetbyARMcore();
      break;

    case MsgController__CallFunc05_PrintLC:
      controllerPrintLC();
      break;

    case MsgController__SetVar01_L:
    {
      const uint8_t relNum    = (uint8_t) (0xffUL & (s_msg_in.rawAry[1] >> 24U));
      const uint8_t relEnable = (uint8_t) (0xffUL & (s_msg_in.rawAry[1] >> 16U));
      controllerSetL(relNum, relEnable);
    }
      break;

    case MsgController__SetVar02_C:
    {
      const uint8_t relNum    = (uint8_t) (0xffUL & (s_msg_in.rawAry[1] >> 24U));
      const uint8_t relEnable = (uint8_t) (0xffUL & (s_msg_in.rawAry[1] >> 16U));
      controllerSetC(relNum, relEnable);
    }
      break;

    case MsgController__SetVar03_CL:
    {
      controllerSetConfigLC(false);
    }
      break;

    case MsgController__SetVar04_LC:
    {
      controllerSetConfigLC(true);
    }
      break;

    case MsgController__SetVar05_K:
    {
      const uint32_t relays = s_msg_in.rawAry[1] & 0x03ffffUL;
      controllerSetCLExt(relays);
    }
      break;

    default:
    {
      Error_Handler();
    }  // default:
    }  // switch ((controllerCmds_t) s_msg_in.msgCmd)
  }  // else

  /* Discard message */
  memset(&s_msg_in, 0, sizeof(s_msg_in));
}


static void controllerInit(void)
{
  /* Load configuration */

  /* At once switch ADC MUX to any valid input port */
  adcMuxSelect(1);

  /* Prepare all semaphores */
  {
    osSemaphoreWait(c2default_BSemHandle,     osWaitForever);
    osSemaphoreWait(c2interpreter_BSemHandle, osWaitForever);
    osSemaphoreWait(c2usbToHost_BSemHandle,   osWaitForever);
    osSemaphoreWait(c2usbFromHost_BSemHandle, osWaitForever);
    osSemaphoreWait(c2uartTx_BSemHandle,      osWaitForever);
    osSemaphoreWait(c2uartRx_BSemHandle,      osWaitForever);
    osSemaphoreWait(c2catTx_BSemHandle,       osWaitForever);
    osSemaphoreWait(c2catRx_BSemHandle,       osWaitForever);
  }

  /* Read FLASH data */
  {

  }

  /* Set-up runtime environment */
  {
    memset(&s_msg_in,   0, sizeof(s_msg_in));
    memset(&s_mod_rdy,  0, sizeof(s_mod_rdy));

    s_controller_McuClocking                                  = DefaultMcuClocking_16MHz_MSI;

    s_controller_doCycle                                      = 1U;

    s_mod_start.rtos_Default                                  = 1U;
    s_mod_start.Interpreter                                   = 1U;
    s_mod_start.network_USBtoHost                             = 0U;
    s_mod_start.network_USBfromHost                           = 0U;
    s_mod_start.network_UartTx                                = 1U;
    s_mod_start.network_UartRx                                = 1U;
    s_mod_start.network_CatTx                                 = 0U;
    s_mod_start.network_CatRx                                 = 0U;
  }

  /* Signaling controller is up and running */
  xEventGroupSetBits(globalEventGroupHandle, EG_GLOBAL__Controller_CTRL_IS_RUNNING);
  osDelay(100UL);

  /* Send INIT message for modules that should be active */
  {
    uint32_t msgAry[2];

    /* main_default */
    if (s_mod_start.rtos_Default) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Rtos_Default,
          0UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* Interpreter */
    if (s_mod_start.Interpreter) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Interpreter,
          25UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_USBtoHost */
    if (s_mod_start.network_USBtoHost) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_USBtoHost,
          50UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_USBfromHost */
    if (s_mod_start.network_USBfromHost) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_USBfromHost,
          75UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_UartTx */
    if (s_mod_start.network_UartTx) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_UartTx,
          100UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_UartRx */
    if (s_mod_start.network_UartRx) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_UartRx,
          125UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_CatTx */
    if (s_mod_start.network_CatTx) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_CatTx,
          150UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_CatRx */
    if (s_mod_start.network_CatRx) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_CatRx,
          175UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }
  }

  /* Init messages */
  {
    osDelay(3500UL);

    /* Greetings to the interfaces */
    controllerGreet();

    /* Inits to be done after USB/DCD connection is established */
    controllerInitAfterGreet();

    osDelay(3000UL);
  }

  #ifdef I2C1_BUS_ADDR_SCAN
  i2cBusAddrScan(&hi2c1, i2c1_BSemHandle);
  #endif

  /* Set relay state */
  controllerFSM_PushOptiVars();

  /* Reset SWR start timer */
  s_controller_swr_tmr = osKernelSysTick();


  /* Enable service cycle */
  if (s_controller_doCycle) {
    controllerCyclicStart(5000UL);  // TODO: change to 30ms

  } else {
    controllerCyclicStop();
  }
}


/* Tasks */

void controllerTaskInit(void)
{
  controllerInit();
}

void controllerTaskLoop(void)
{
  (void) xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_QUEUE_IN,
      EG_GLOBAL__Controller_QUEUE_IN,
      0, 250UL);

  /* Work the next complete messages */
  while (osOK == controllerMsgPullFromInQueue()) {
    /* Process message */
    controllerMsgProcessor();
  }
}
