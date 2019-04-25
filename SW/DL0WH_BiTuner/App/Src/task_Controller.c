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


//#define I2C1_BUS_ADDR_SCAN

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
extern EventGroupHandle_t   adcEventGroupHandle;

extern float                g_adc_refint_val;
extern float                g_adc_vref_mv;
extern float                g_adc_bat_mv;
extern float                g_adc_temp_deg;
extern float                g_adc_logamp_ofsMeas_mv;
extern float                g_adc_fwd_raw_mv;
extern float                g_adc_fwd_mv_log;
extern float                g_adc_fwd_mv;
extern float                g_adc_rev_raw_mv;
extern float                g_adc_rev_mv_log;
extern float                g_adc_rev_mv;
extern float                g_adc_vdiode_mv;
extern float                g_adc_swr;


#define Controller_L0_nH                                      50.0f
const float Controller_Ls_nH[8]                      = {
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
const float Controller_Cp_pF[8]                      = {
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



static ControllerMsg2Proc_t s_msg_in                          = { 0 };
static ControllerMods_t     s_mod_start                       = { 0 };
static ControllerMods_t     s_mod_rdy                         = { 0 };


static const float          Controller_AutoSWR_P_mW_Min       =  5000.0f  / 1000.0f;  // -30 dB coupling
static const float          Controller_AutoSWR_P_mW_Max       = 15000.0f  / 1000.0f;  // -30 dB coupling
static const float          Controller_AutoSWR_SWR_Init       = SWR_MAX;
static const float          Controller_AutoSWR_SWR_Course_Min = 1.3f;
static const float          Controller_AutoSWR_SWR_Fine_Min   = 1.1f;
static const uint8_t        Controller_AutoSWR_Fine_Ctr_Max   = 100U;
static const float          Controller_AutoSWR_WaitBefore_ms  = 750.0f;
static const uint8_t        Controller_AutoSWR_CVHpong_Max    = 1U;
static const uint8_t        Controller_AutoSWR_LCpong_Max     = 3U;

static uint32_t             s_controller_swr_tmr              = 0UL;
static uint8_t              s_controller_fine_ctr             = 0U;

static uint8_t              s_controller_RelVal_L_cur         = 0U;
static uint8_t              s_controller_RelVal_C_cur         = 0U;

static Meas_Data_t          s_controller_L_Meas               = { 0 };
static Meas_Data_t          s_controller_C_Meas               = { 0 };

static ControllerFsm_t        s_controller_FSM_state            = ControllerFsm__NOP;
static ControllerOptiCVH_t    s_controller_FSM_optiCVH          = ControllerOptiCVH__CV;
static uint8_t                s_controller_opti_CVHpongCtr      = 0U;
static uint8_t                s_controller_opti_LCpongCtr       = 0U;

static float                  s_controller_best_swr             = 0.0f;
static uint8_t                s_controller_best_swr_L           = 0U;
static uint8_t                s_controller_best_swr_C           = 0U;
static ControllerOptiCVH_t    s_controller_best_swr_CVH         = ControllerOptiCVH__CV;

static float                  s_controller_adc_bat_mv           = 0.0f;
static float                  s_controller_adc_temp_deg         = 0.0f;
static float                  s_controller_adc_fwd_mv_log       = 0.0f;
static float                  s_controller_adc_fwd_mv           = 0.0f;
static float                  s_controller_adc_rev_mv_log       = 0.0f;
static float                  s_controller_adc_rev_mv           = 0.0f;
static float                  s_controller_adc_vdiode_mv        = 0.0f;
static float                  s_controller_adc_swr              = 0.0f;
static float                  s_controller_adc_swr_previous     = 0.0f;
static float                  s_controller_adc_fwd_mw           = 0.0f;
static float                  s_controller_adc_rev_mw           = 0.0f;

static float                  s_controller_cmad_deg             = 0.0f;
static uint8_t                s_controller_logamp_pot_val       = 0.0f;

static _Bool                  s_controller_doAdc                = 0;
static _Bool                  s_controller_doAutoMatching       = 0;
static ControllerMonitor_BF_t s_controller_verbose_bf           = 0UL;
static uint32_t               s_controller_doCycle              = 0UL;

static DefaultMcuClocking_t   s_controller_McuClocking          = DefaultMcuClocking_80MHz_MSI16_PLL;



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
    interpreterConsolePush(controllerGreetMsg02, strlen(controllerGreetMsg02), 0);
    interpreterConsolePush(controllerGreetMsg03, strlen(controllerGreetMsg03), 0);
    interpreterConsolePush(controllerGreetMsg04, strlen(controllerGreetMsg04), 0);
    interpreterConsolePush(controllerGreetMsg03, strlen(controllerGreetMsg03), 0);
    interpreterConsolePush(controllerGreetMsg02, strlen(controllerGreetMsg02), 0);
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01), 0);
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01), 0);

    interpreterConsolePush(verBuf, strlen(verBuf), 0);
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01), 0);
    interpreterConsolePush(controllerGreetMsg01, strlen(controllerGreetMsg01), 1);
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
  interpreterConsolePush(buf, len, 1);
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

static uint8_t controllerGetMinIdxL(void)
{
  uint8_t minIdx  = P000;
  float minSwr    = s_controller_L_Meas.swr[P000];

  for (uint8_t idx = 1U; idx < 5U; idx++) {
    if (minSwr > s_controller_L_Meas.swr[idx]) {
      minSwr = s_controller_L_Meas.swr[idx];
      minIdx = idx;
    }
  }
  return minIdx;
}

static uint8_t controllerGetMinIdxC(void)
{
  uint8_t minIdx  = P000;
  float minSwr    = s_controller_C_Meas.swr[P000];

  for (uint8_t idx = 1U; idx < 5U; idx++) {
    if (minSwr > s_controller_C_Meas.swr[idx]) {
      minSwr = s_controller_C_Meas.swr[idx];
      minIdx = idx;
    }
  }
  return minIdx;
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
    osEvent ev = osMessagePeek(controllerOutQueueHandle, 2UL);
    if (ev.status == osEventMessage) {
      const uint32_t  hdr       = ev.value.v;
      uint32_t        lenBytes  = 0xffUL & (hdr >> 8U);

      if (dst == (0xffUL & (hdr >> 24))) {
        (void) osMessageGet(controllerOutQueueHandle, 2UL);
        msgAry[len++] = hdr;

        while (lenBytes) {
          /* Push token into array */
          ev = osMessageGet(controllerOutQueueHandle, 2UL);
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
          (void) osMessageGet(controllerOutQueueHandle, 2UL);
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
#if 1
  /* Final state of the auto tuner */
  const uint32_t runtime = osKernelSysTick() - s_controller_swr_tmr;

  /* Logging */
  {
    char buf[128];
    int32_t   swr_i;
    uint32_t  swr_f;

    mainCalcFloat2IntFrac(s_controller_adc_swr, 3, &swr_i, &swr_f);
    const int len = snprintf(buf, sizeof(buf) - 1,
                            "Auto STOP: VSWR= %2ld.%03lu, time= %lu ms, iterations= %lu\r\n",
                            swr_i, swr_f,
                            runtime,
                            runtime / RELAY_STILL_TIME);
    interpreterConsolePush(buf, len, 0);
  }
#endif
}

static void controllerFSM_LogState(void)
{
#if 0
  /* Show current state of optimization */
  const _Bool     doWait = 0;
  char            buf[512];
#endif

#if 0
  {
    const int len = snprintf(buf, sizeof(buf) - 1,
                      "FSM_state= 0x%02X\r\n",
                      s_controller_FSM_state);
        interpreterConsolePush(buf, len, doWait);
  }
#endif

#if 0
  {
    const uint32_t  l_controller_timer = osKernelSysTick() - s_controller_swr_tmr;
    const int len = snprintf(buf, sizeof(buf) - 1,
                  "\r\nController FSM:\tcontrollerFSM_LogState: systick= %5lu ms, time= %5lu ms, iteration= %03lu\r\n" \
                  "\ta)\t\t FSM_state= %u, optiCVH= %s:\r\n",
                  osKernelSysTick(), l_controller_timer, (l_controller_timer / RELAY_STILL_TIME),
                  s_controller_FSM_state, (s_controller_FSM_optiCVH  == ControllerOptiCVH__CV  ?  "CV" : "CH")
                  );
    interpreterConsolePush(buf, len, doWait);
  }
#endif

#if 0
  {
    int32_t   swr_i, best_swr_i;
    uint32_t  swr_f, best_swr_f;

    mainCalcFloat2IntFrac(s_controller_adc_swr,   3, &swr_i,      &swr_f);
    mainCalcFloat2IntFrac(s_controller_best_swr,  3, &best_swr_i, &best_swr_f);
    const int len = snprintf(buf, sizeof(buf) - 1,
                  "\tb)\t\t swr= %2ld.%03lu, best_swr= %2ld.%03lu @ CVH= %u: L= %03u RelVal, C= %03u RelVal.\r\n",
                  swr_i, swr_f,
                  best_swr_i, best_swr_f,
                  s_controller_best_swr_CVH, s_controller_best_swr_L, s_controller_best_swr_C
                  );
    interpreterConsolePush(buf, len, doWait);
  }
#endif

#if 0
  {
    mainCalcFloat2IntFrac(s_controller_opti_swr_1st,    3, &s_controller_opti_swr_1st_i,    &s_controller_opti_swr_1st_f  );
    mainCalcFloat2IntFrac(s_controller_opti_swr_1st_L,  1, &s_controller_opti_swr_1st_L_i,  &s_controller_opti_swr_1st_L_f);
    mainCalcFloat2IntFrac(s_controller_opti_swr_1st_C,  1, &s_controller_opti_swr_1st_C_i,  &s_controller_opti_swr_1st_C_f);
    mainCalcFloat2IntFrac(s_controller_opti_swr_2nd,    3, &s_controller_opti_swr_2nd_i,    &s_controller_opti_swr_2nd_f  );
    mainCalcFloat2IntFrac(s_controller_opti_swr_2nd_L,  1, &s_controller_opti_swr_2nd_L_i,  &s_controller_opti_swr_2nd_L_f);
    mainCalcFloat2IntFrac(s_controller_opti_swr_2nd_C,  1, &s_controller_opti_swr_2nd_C_i,  &s_controller_opti_swr_2nd_C_f);
    const len = snprintf(buf, sizeof(buf) - 1,
                  "\tc)\t\t swr_1st= %2ld.%03lu, L= %5ld.%01lu nH, C= %5ld.%01lu pF # swr_2nd= %2ld.%03lu, L= %5ld.%01lu nH, C= %5ld.%01lu pF,\r\n",
                  s_controller_opti_swr_1st_i,        s_controller_opti_swr_1st_f,
                  s_controller_opti_swr_1st_L_i,      s_controller_opti_swr_1st_L_f,
                  s_controller_opti_swr_1st_C_i,      s_controller_opti_swr_1st_C_f,
                  s_controller_opti_swr_2nd_i,        s_controller_opti_swr_2nd_f,
                  s_controller_opti_swr_2nd_L_i,      s_controller_opti_swr_2nd_L_f,
                  s_controller_opti_swr_2nd_C_i,      s_controller_opti_swr_2nd_C_f
                 );
    interpreterConsolePush(buf, len, doWait);
  }
#endif

#if 0
  interpreterShowCrLf();
#endif
}

static void controllerFSM_GetGlobalVars(void)
{
  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    s_controller_adc_bat_mv     = g_adc_bat_mv;
    s_controller_adc_temp_deg   = g_adc_temp_deg;
    s_controller_adc_fwd_mv_log = g_adc_fwd_mv_log;
    s_controller_adc_fwd_mv     = g_adc_fwd_mv;
    s_controller_adc_rev_mv_log = g_adc_rev_mv_log;
    s_controller_adc_rev_mv     = g_adc_rev_mv;
    s_controller_adc_vdiode_mv  = g_adc_vdiode_mv;
    s_controller_adc_swr        = g_adc_swr;

    taskENABLE_INTERRUPTS();
  }

  /* FWD power calculation */
  {
    const float fwdMv = mainCalc_mV_to_mW(s_controller_adc_fwd_mv);
    const float revMv = mainCalc_mV_to_mW(s_controller_adc_rev_mv);

    {
      /* Disabled IRQ section */
      taskDISABLE_INTERRUPTS();

      s_controller_adc_fwd_mw = fwdMv;
      s_controller_adc_rev_mw = revMv;

      taskENABLE_INTERRUPTS();
    }
  }

  /* Logging every 1 sec */
  if (s_controller_verbose_bf & ControllerMon__ShowAdcs) {
    static uint32_t s_timeLast = 0UL;
    uint32_t l_timeNow = osKernelSysTick();
    char  dbgBuf[128];

    if (s_timeLast + 1000UL > l_timeNow) {
      return;
    }
    s_timeLast = l_timeNow;

    {
      int32_t   l_adc_temp_deg_i    = 0L;
      uint32_t  l_adc_temp_deg_f100 = 0UL;
      float     s_controller_adc_revint_val;
      float     s_controller_adc_vref_mv;

      {
        /* Disabled IRQ section */
        taskDISABLE_INTERRUPTS();

        s_controller_adc_revint_val = g_adc_refint_val;
        s_controller_adc_vref_mv    = g_adc_vref_mv;

        taskENABLE_INTERRUPTS();
      }

      mainCalcFloat2IntFrac(s_controller_adc_temp_deg, 2, &l_adc_temp_deg_i, &l_adc_temp_deg_f100);

      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC1: refint_val = %4d, Vref = %4d mV, Bat = %4d mV, Temp = %+3ld.%02luC\r\n",
          (int16_t) (s_controller_adc_revint_val + 0.5f),
          (int16_t) (s_controller_adc_vref_mv    + 0.5f),
          (int16_t) (s_controller_adc_bat_mv     + 0.5f),
          l_adc_temp_deg_i, l_adc_temp_deg_f100);
      interpreterConsolePush(dbgBuf, dbgLen, 1);
    }

    {
      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC2: FWD = %5d mV  (ADC mV = %5d mV)\r\n",
          (int16_t) (s_controller_adc_fwd_mv      + 0.5f),
          (int16_t) (s_controller_adc_fwd_mv_log  + 0.5f));
      interpreterConsolePush(dbgBuf, dbgLen, 0);
    }

    {
      int32_t   l_swr_i;
      uint32_t  l_swr_f100;

      mainCalcFloat2IntFrac(s_controller_adc_swr, 2, &l_swr_i, &l_swr_f100);

      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC2: REV = %5d mV  (ADC mV = %5d mV), SWR = %+3ld.%03lu\r\n",
          (int16_t) (s_controller_adc_rev_mv      + 0.5f),
          (int16_t) (s_controller_adc_rev_mv_log  + 0.5f),
          l_swr_i, l_swr_f100);
      interpreterConsolePush(dbgBuf, dbgLen, 0);
    }

    {
      const int dbgLen = snprintf(dbgBuf, sizeof(dbgBuf) - 1,
          "ADC3: Vdiode = %4d mV\r\n",
          (int16_t) (s_controller_adc_vdiode_mv + 0.5f));
      interpreterConsolePush(dbgBuf, dbgLen, 0);
    }

    interpreterShowCrLf();
  }
}

static void controllerFSM_PushOptiVars(void)
{
  uint8_t   rel_CV_cur;
  uint8_t   rel_CH_cur;
  uint32_t  msgAry[2];

  /* Disabled IRQ section */
  /* { */
    taskDISABLE_INTERRUPTS();
    const uint8_t             rel_L_cur = s_controller_RelVal_L_cur;
    const uint8_t             rel_C_cur = s_controller_RelVal_C_cur;
    const ControllerOptiCVH_t configLC  = s_controller_FSM_optiCVH;
    taskENABLE_INTERRUPTS();
    /* } */

  /* Update CV/CH state */
  if (configLC == ControllerOptiCVH__CH) {
    rel_CV_cur  = 0U;
    rel_CH_cur  = 1U;

  } else {
    rel_CV_cur  = 1U;
    rel_CH_cur  = 0U;
  }

  /* Three extra bytes to take over */
  msgAry[0] = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 3, MsgDefault__SetVar03_C_L_CV_CH);

  /* Compose relay bitmap */
  msgAry[1] = ((uint32_t)rel_CH_cur << 17) |
              ((uint32_t)rel_CV_cur << 16) |
              ((uint32_t)rel_L_cur  <<  8) |
              ((uint32_t)rel_C_cur       ) ;

  controllerMsgPushToOutQueue(sizeof(msgAry) / sizeof(uint32_t), msgAry, osWaitForever);
}

static _Bool controllerFSM_CheckPower(void)
{
  if ((Controller_AutoSWR_P_mW_Min > s_controller_adc_fwd_mw) || (s_controller_adc_fwd_mw > Controller_AutoSWR_P_mW_Max)) {
#if 1
    /* Logging */
    if (s_controller_FSM_state > ControllerFsm__StartAuto) {
      char      buf[128];
      int32_t   pwr_i;
      uint32_t  pwr_f;

      mainCalcFloat2IntFrac(s_controller_adc_fwd_mw, 3, &pwr_i, &pwr_f);
      const int len = snprintf(buf, sizeof(buf) - 1,
                              "Controller FSM: power= %5ld.%03lu out of [%u .. %u] mW - stop auto tuner.\r\n",
                              pwr_i, pwr_f,
                              (uint16_t)Controller_AutoSWR_P_mW_Min, (uint16_t)Controller_AutoSWR_P_mW_Max);
      interpreterConsolePush(buf, len, 0);
    }
#endif

    /* Overdrive or to low energy */
    s_controller_FSM_state = ControllerFsm__Init;

    return true;
  }

  /* Valid range */

  /* Best SWR */
  if (s_controller_best_swr > s_controller_adc_swr) {
    s_controller_best_swr     = s_controller_adc_swr;
    s_controller_best_swr_CVH = s_controller_FSM_optiCVH;
    s_controller_best_swr_L   = s_controller_RelVal_L_cur;
    s_controller_best_swr_C   = s_controller_RelVal_C_cur;
  }
  return false;
}

static _Bool controllerFSM_CheckSwrCourseStopVal(void)
{
  if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Course_Min) {
    return true;
  }
  return false;
}

static _Bool controllerFSM_CheckSwrFineStopVal(void)
{
  if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Fine_Min) {
    return true;

  } else if (++s_controller_fine_ctr > Controller_AutoSWR_Fine_Ctr_Max) {
    return true;
  }

  return false;
}

static _Bool controllerFSM_CheckSwrTime(void)
{
  const uint32_t timeNow  = osKernelSysTick();
  const int32_t  timeDiff = timeNow - s_controller_swr_tmr;

  if (Controller_AutoSWR_WaitBefore_ms > timeDiff) {
    /* Timer has not yet elapsed */
    return true;
  }

  return controllerFSM_CheckSwrFineStopVal();
}

static void controllerFSM(void)
{
  switch (s_controller_FSM_state)
  {

  // XXX: 0x00 and 0x01
  case ControllerFsm__NOP:
  case ControllerFsm__Init:
  {
    s_controller_doAdc = 1;

    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    if (!s_controller_doAutoMatching) {
      break;
    }

    /* Init SWR compare values */
    s_controller_swr_tmr = osKernelSysTick();

    s_controller_C_Meas.relayVal[P000]  = s_controller_L_Meas.relayVal[P000] = 0x00U;
    s_controller_C_Meas.relayVal[P025]  = s_controller_L_Meas.relayVal[P025] = 0x3fU;
    s_controller_C_Meas.relayVal[P050]  = s_controller_L_Meas.relayVal[P050] = 0x7fU;
    s_controller_C_Meas.relayVal[P075]  = s_controller_L_Meas.relayVal[P075] = 0xbfU;
    s_controller_C_Meas.relayVal[P100]  = s_controller_L_Meas.relayVal[P100] = 0xffU;

    for (uint8_t idx = 0U; idx < 5U; idx++) {
      s_controller_C_Meas.swr[idx] = s_controller_L_Meas.swr[idx] = Controller_AutoSWR_SWR_Init;
    }

    s_controller_best_swr     = Controller_AutoSWR_SWR_Init;
    s_controller_best_swr_CVH = ControllerOptiCVH__CV;
    s_controller_best_swr_L   = 0U;
    s_controller_best_swr_C   = 0U;

    s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P000];
    s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P000];

    s_controller_FSM_optiCVH  = ControllerOptiCVH__CV;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();

    s_controller_FSM_state = ControllerFsm__StartAuto;
  }
    break;

  // XXX: 0x02
  case ControllerFsm__StartAuto:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    s_controller_doAdc = 1;

    if (!s_controller_doAutoMatching) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      break;
    }
    /* Check if auto tuner should start */
    if (controllerFSM_CheckSwrTime()) {
      break;
    }

    /* Run (V)SWR optimization */
    s_controller_fine_ctr         = 0U;

    s_controller_opti_LCpongCtr   = 0U;
    s_controller_opti_CVHpongCtr  = 0U;

    s_controller_adc_swr_previous = Controller_AutoSWR_SWR_Init;

    s_controller_RelVal_C_cur     = s_controller_RelVal_L_cur     = 0U;

    s_controller_FSM_state        = ControllerFsm__L_Meas_P000;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Logging */
    {
      char buf[] = "Start ATU\r\n";
      interpreterConsolePush(buf, strlen(buf), 0);
    }

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x11
  case ControllerFsm__L_Meas_P000:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_L_Meas.swr[P000] = s_controller_adc_swr;

    /* Prepare next measurement */
    if (s_controller_L_Meas.swr[P100] == Controller_AutoSWR_SWR_Init) {
      s_controller_FSM_state = ControllerFsm__L_Meas_P100;
      s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P100];

    } else {
      s_controller_FSM_state = ControllerFsm__L_Meas_P050;
      s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P050];
    }

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x12
  case ControllerFsm__L_Meas_P100:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_L_Meas.swr[P100] = s_controller_adc_swr;

    /* Prepare next measurement */
    s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P050];
    s_controller_FSM_state = ControllerFsm__L_Meas_P050;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x13
  case ControllerFsm__L_Meas_P050:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_L_Meas.swr[P050] = s_controller_adc_swr;

    /* Calculate P025 and P075 relay values */
    {
      const uint8_t delta = s_controller_L_Meas.relayVal[P100] - s_controller_L_Meas.relayVal[P000];
      s_controller_L_Meas.relayVal[P025] = s_controller_L_Meas.relayVal[P000] + (delta >> 2U);
      s_controller_L_Meas.relayVal[P075] = s_controller_L_Meas.relayVal[P100] - (delta >> 2U);
      s_controller_L_Meas.swr[P025] = Controller_AutoSWR_SWR_Init;
      s_controller_L_Meas.swr[P075] = Controller_AutoSWR_SWR_Init;
    }

    /* Prepare next measurement */
    s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P025];
    s_controller_FSM_state    = ControllerFsm__L_Meas_P025;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x14
  case ControllerFsm__L_Meas_P025:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_L_Meas.swr[P025] = s_controller_adc_swr;

    /* Prepare next measurement */
    s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P075];
    s_controller_FSM_state = ControllerFsm__L_Meas_P075;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x15
  case ControllerFsm__L_Meas_P075:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_L_Meas.swr[P075] = s_controller_adc_swr;

    /* Prepare next measurement */
    s_controller_FSM_state = ControllerFsm__L_Select;

    /* Iterate to next FSM state */
    s_controller_doAdc = 0;

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x16
  case ControllerFsm__L_Select:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    /* Check for end of ATU optimization */
    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Check for L value delta */
    if (s_controller_L_Meas.relayVal[P100] - s_controller_L_Meas.relayVal[P000] <= 2) {
      /* Delta value too small to process further on L */

      /* Check if another switch to optimizing C is allowed */
      if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
        /* Continue with optimizing C */
        s_controller_FSM_state = ControllerFsm__C_Meas_P000;

        /* Log */
        {
          const char buf[] = "!L --> C!\r\n";
          interpreterConsolePush(buf, strlen(buf), 0);
        }

      } else {
        s_controller_opti_LCpongCtr = 0U;

        /* Check if another Gamma switch is allowed */
        if (++s_controller_opti_CVHpongCtr <= Controller_AutoSWR_CVHpong_Max) {
          /* Do a Gamma switch */
          s_controller_FSM_optiCVH = (s_controller_FSM_optiCVH == ControllerOptiCVH__CV) ?  ControllerOptiCVH__CH : ControllerOptiCVH__CV;

          /* Log */
          {
            char buf[128];
            const int len = snprintf(buf, sizeof(buf) - 1, "!Gamma revert! next: %s\r\n",
                (s_controller_FSM_optiCVH == ControllerOptiCVH__CV) ?  "CV" : "CH");
            interpreterConsolePush(buf, len, 0);
          }

        } else {
          /* All tries exhausted, take best result of all */
          /* Log */
          {
            const char buf[] = "!Exhausted!\r\n";
            interpreterConsolePush(buf, strlen(buf), 0);
          }

          s_controller_FSM_state = ControllerFsm__done;
        }
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      /* Show current state of optimization */
      controllerFSM_LogState();
      break;
    }

    /* Find minimum SWR */
    const uint8_t bestIdx = controllerGetMinIdxL();
    int16_t val[5];
    float   swr[5] = {
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init };

    switch (bestIdx) {
    case P000:
    {
      val[P000] = s_controller_L_Meas.relayVal[P000] - (s_controller_L_Meas.relayVal[P025] - s_controller_L_Meas.relayVal[P000]);

      val[P100] = s_controller_L_Meas.relayVal[P025];
      swr[P100] = s_controller_L_Meas.swr[P025];
    }
      break;

    case P025:
    {
      val[P000] = s_controller_L_Meas.relayVal[P000];
      swr[P000] = s_controller_L_Meas.swr[P000];

      val[P100] = s_controller_L_Meas.relayVal[P050];
      swr[P100] = s_controller_L_Meas.swr[P050];
    }
      break;

    case P050:
    {
      val[P000] = s_controller_L_Meas.relayVal[P025];
      swr[P000] = s_controller_L_Meas.swr[P025];

      val[P100] = s_controller_L_Meas.relayVal[P075];
      swr[P100] = s_controller_L_Meas.swr[P075];
    }
      break;

    case P075:
    {
      val[P000] = s_controller_L_Meas.relayVal[P050];
      swr[P000] = s_controller_L_Meas.swr[P050];

      val[P100] = s_controller_L_Meas.relayVal[P100];
      swr[P100] = s_controller_L_Meas.swr[P100];
    }
      break;

    case P100:
    {
      val[P000] = s_controller_L_Meas.relayVal[P075];
      swr[P000] = s_controller_L_Meas.swr[P075];

      val[P100] = s_controller_L_Meas.relayVal[P100] + (s_controller_L_Meas.relayVal[P100] - s_controller_L_Meas.relayVal[P075]);
    }
      break;

    default: { }
    }


    if (val[P000] < 0) {
      val[P000] = 0;
    }

    if (val[P100] > 255) {
      val[P100] = 255;
    }

    /* Calculate new P025, P050 and P075 values */
    val[P050] = (val[P000] + val[P100]) / 2;
    val[P025] = (val[P000] + val[P050]) / 2;
    val[P075] = (val[P050] + val[P100]) / 2;

    /* Store values */
    for (uint8_t idx = 0U; idx < 5U; idx++) {
      s_controller_L_Meas.relayVal[idx] = (uint8_t) val[idx];
      s_controller_L_Meas.swr[idx]      = swr[idx];
    }

    /* Prepare next measurement */
    if (s_controller_L_Meas.swr[P000] == Controller_AutoSWR_SWR_Init) {
      s_controller_FSM_state = ControllerFsm__L_Meas_P000;
      s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P000];

    } else if (s_controller_L_Meas.swr[P100] == Controller_AutoSWR_SWR_Init) {
      s_controller_FSM_state = ControllerFsm__L_Meas_P100;
      s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P100];

    } else {
      s_controller_FSM_state = ControllerFsm__L_Meas_P050;
      s_controller_RelVal_L_cur = s_controller_L_Meas.relayVal[P050];
    }

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;



  // XXX: 0x21
  case ControllerFsm__C_Meas_P000:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_C_Meas.swr[P000] = s_controller_adc_swr;

    /* Prepare next measurement */
    if (s_controller_C_Meas.swr[P100] == Controller_AutoSWR_SWR_Init) {
      s_controller_FSM_state = ControllerFsm__C_Meas_P100;
      s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P100];

    } else {
      s_controller_FSM_state = ControllerFsm__C_Meas_P050;
      s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P050];
    }

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x22
  case ControllerFsm__C_Meas_P100:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_C_Meas.swr[P100] = s_controller_adc_swr;

    /* Prepare next measurement */
    s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P050];
    s_controller_FSM_state = ControllerFsm__C_Meas_P050;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x23
  case ControllerFsm__C_Meas_P050:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_C_Meas.swr[P050] = s_controller_adc_swr;

    /* Calculate P025 and P075 relay values */
    {
      const uint8_t delta = s_controller_C_Meas.relayVal[P100] - s_controller_C_Meas.relayVal[P000];
      s_controller_C_Meas.relayVal[P025] = s_controller_C_Meas.relayVal[P000] + (delta >> 2U);
      s_controller_C_Meas.relayVal[P075] = s_controller_C_Meas.relayVal[P100] - (delta >> 2U);
      s_controller_C_Meas.swr[P025] = Controller_AutoSWR_SWR_Init;
      s_controller_C_Meas.swr[P075] = Controller_AutoSWR_SWR_Init;
    }

    /* Prepare next measurement */
    s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P025];
    s_controller_FSM_state    = ControllerFsm__C_Meas_P025;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x24
  case ControllerFsm__C_Meas_P025:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_C_Meas.swr[P025] = s_controller_adc_swr;

    /* Prepare next measurement */
    s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P075];
    s_controller_FSM_state = ControllerFsm__C_Meas_P075;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x25
  case ControllerFsm__C_Meas_P075:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Store measured (V)SWR */
    s_controller_C_Meas.swr[P075] = s_controller_adc_swr;

    /* Prepare next measurement */
    s_controller_FSM_state = ControllerFsm__C_Select;

    /* Iterate to next FSM state */
    s_controller_doAdc = 0;

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x26
  case ControllerFsm__C_Select:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }
    if (controllerFSM_CheckSwrCourseStopVal()) {
      s_controller_adc_swr_previous = s_controller_adc_swr;
      s_controller_FSM_state        = ControllerFsm__fine_L_minus;

      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      break;
    }

    /* Check for C value delta */
    if (s_controller_C_Meas.relayVal[P100] - s_controller_C_Meas.relayVal[P000] <= 2) {
      /* Delta value too small to process further on C */

      /* Check if another switch to optimizing L is allowed */
      if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
        /* Continue with optimizing L */
        s_controller_FSM_state = ControllerFsm__L_Meas_P000;

        /* Log */
        {
          const char buf[] = "!C --> L!\r\n";
          interpreterConsolePush(buf, strlen(buf), 0);
        }

      } else {
        s_controller_opti_LCpongCtr = 0U;

        /* Check if another Gamma switch is allowed */
        if (++s_controller_opti_CVHpongCtr <= Controller_AutoSWR_CVHpong_Max) {
          /* Do a Gamma switch */
          s_controller_FSM_optiCVH = (s_controller_FSM_optiCVH == ControllerOptiCVH__CV) ?  ControllerOptiCVH__CH : ControllerOptiCVH__CV;

          /* Log */
          {
            char buf[128];
            const int len = snprintf(buf, sizeof(buf) - 1, "!Gamma revert! next: %s\r\n",
                (s_controller_FSM_optiCVH == ControllerOptiCVH__CV) ?  "CV" : "CH");
            interpreterConsolePush(buf, len, 0);
          }

        } else {
          /* All tries exhausted, take best result of all */
          s_controller_FSM_state = ControllerFsm__done;
        }
      }

      /* Push opti data to relays */
      controllerFSM_PushOptiVars();

      /* Show current state of optimization */
      controllerFSM_LogState();
      break;
    }

    /* Find minimum SWR */
    const uint8_t bestIdx = controllerGetMinIdxC();
    int16_t val[5];
    float   swr[5] = {
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init,
        Controller_AutoSWR_SWR_Init };

    switch (bestIdx) {
    case P000:
    {
      val[P000] = s_controller_C_Meas.relayVal[P000] - (s_controller_C_Meas.relayVal[P025] - s_controller_C_Meas.relayVal[P000]);

      val[P100] = s_controller_C_Meas.relayVal[P025];
      swr[P100] = s_controller_C_Meas.swr[P025];
    }
      break;

    case P025:
    {
      val[P000] = s_controller_C_Meas.relayVal[P000];
      swr[P000] = s_controller_C_Meas.swr[P000];

      val[P100] = s_controller_C_Meas.relayVal[P050];
      swr[P100] = s_controller_C_Meas.swr[P050];
    }
      break;

    case P050:
    {
      val[P000] = s_controller_C_Meas.relayVal[P025];
      swr[P000] = s_controller_C_Meas.swr[P025];

      val[P100] = s_controller_C_Meas.relayVal[P075];
      swr[P100] = s_controller_C_Meas.swr[P075];
    }
      break;

    case P075:
    {
      val[P000] = s_controller_C_Meas.relayVal[P050];
      swr[P000] = s_controller_C_Meas.swr[P050];

      val[P100] = s_controller_C_Meas.relayVal[P100];
      swr[P100] = s_controller_C_Meas.swr[P100];
    }
      break;

    case P100:
    {
      val[P000] = s_controller_C_Meas.relayVal[P075];
      swr[P000] = s_controller_C_Meas.swr[P075];

      val[P100] = s_controller_C_Meas.relayVal[P100] + (s_controller_C_Meas.relayVal[P100] - s_controller_C_Meas.relayVal[P075]);
    }
      break;

    default: { }
    }


    if (val[P000] < 0) {
      val[P000] = 0;
    }

    if (val[P100] > 255) {
      val[P100] = 255;
    }

    /* Calculate new P025, P050 and P075 values */
    val[P050] = (val[P000] + val[P100]) / 2;
    val[P025] = (val[P000] + val[P050]) / 2;
    val[P075] = (val[P050] + val[P100]) / 2;

    /* Store values */
    for (uint8_t idx = 0U; idx < 5U; idx++) {
      s_controller_C_Meas.relayVal[idx] = (uint8_t) val[idx];
      s_controller_C_Meas.swr[idx]      = swr[idx];
    }

    /* Prepare next measurement */
    if (s_controller_C_Meas.swr[P000] == Controller_AutoSWR_SWR_Init) {
      s_controller_FSM_state = ControllerFsm__C_Meas_P000;
      s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P000];

    } else if (s_controller_C_Meas.swr[P100] == Controller_AutoSWR_SWR_Init) {
      s_controller_FSM_state = ControllerFsm__C_Meas_P100;
      s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P100];

    } else {
      s_controller_FSM_state = ControllerFsm__C_Meas_P050;
      s_controller_RelVal_C_cur = s_controller_C_Meas.relayVal[P050];
    }

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;


  // XXX: 0x31
  case ControllerFsm__fine_L_minus:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }

    /* Check if new value makes result worse */
    if ((s_controller_adc_swr_previous <= s_controller_adc_swr) ||
        !s_controller_RelVal_L_cur) {
      /* Revert to previous value */
      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur++;
      }

      /* Switch to next quadrature adjustments */
      s_controller_FSM_state = ControllerFsm__fine_C_minus;

      /* Prepare next quadrature setting */
      if (s_controller_RelVal_C_cur) {
        s_controller_RelVal_C_cur--;
      }

    } else {
      /* Try next count value */
      s_controller_RelVal_L_cur--;
    }

    /* Update previous value */
    s_controller_adc_swr_previous = s_controller_adc_swr;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x32
  case ControllerFsm__fine_C_minus:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }

    /* Check if new value makes result worse */
    if ((s_controller_adc_swr_previous <= s_controller_adc_swr) ||
        !s_controller_RelVal_C_cur) {
      /* Revert to previous value */
      if (s_controller_RelVal_C_cur) {
        s_controller_RelVal_C_cur++;
      }

      /* Switch to next quadrature adjustments */
      s_controller_FSM_state = ControllerFsm__fine_L_plus;

      /* Prepare next quadrature setting */
      if (s_controller_RelVal_L_cur < 255U) {
        s_controller_RelVal_L_cur++;
      }

    } else {
      /* Try next count value */
      s_controller_RelVal_C_cur--;
    }

    /* Update previous value */
    s_controller_adc_swr_previous = s_controller_adc_swr;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x33
  case ControllerFsm__fine_L_plus:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }

    /* Check if new value makes result worse */
    if ((s_controller_adc_swr_previous <= s_controller_adc_swr) ||
        (s_controller_RelVal_L_cur >= 255U)) {
      /* Revert to previous value */
      s_controller_RelVal_L_cur--;

      /* Switch to next quadrature adjustments */
      s_controller_FSM_state = ControllerFsm__fine_C_plus;

      /* Prepare next quadrature setting */
      if (s_controller_RelVal_C_cur < 255U) {
        s_controller_RelVal_C_cur++;
      }

    } else {
      /* Try next count value */
      s_controller_RelVal_L_cur++;
    }

    /* Update previous value */
    s_controller_adc_swr_previous = s_controller_adc_swr;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;

  // XXX: 0x34
  case ControllerFsm__fine_C_plus:
  {
    /* Pull global vars */
    controllerFSM_GetGlobalVars();

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Check for security */
    if (controllerFSM_CheckPower()) {
      s_controller_FSM_state = ControllerFsm__Init;
      break;
    }

    if (controllerFSM_CheckSwrFineStopVal()) {
      s_controller_FSM_state = ControllerFsm__done;
      break;
    }

    /* Check if new value makes result worse */
    if ((s_controller_adc_swr_previous <= s_controller_adc_swr) ||
        (s_controller_RelVal_C_cur >= 255U)) {
      /* Revert to previous value */
      s_controller_RelVal_C_cur--;

      /* Switch to next quadrature adjustments */
      s_controller_FSM_state = ControllerFsm__fine_L_minus;

      /* Prepare next quadrature setting */
      if (s_controller_RelVal_L_cur) {
        s_controller_RelVal_L_cur--;
      }

    } else {
      /* Try next count value */
      s_controller_RelVal_C_cur++;
    }

    /* Update previous value */
    s_controller_adc_swr_previous = s_controller_adc_swr;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
  }
    break;


  // XXX: 0xf0
  case ControllerFsm__done:
  {
    /* Take the best result */
    s_controller_FSM_optiCVH  = s_controller_best_swr_CVH;
    s_controller_RelVal_L_cur = s_controller_best_swr_L;
    s_controller_RelVal_C_cur = s_controller_best_swr_C;

    /* Do not iterate FSM again */
    s_controller_doAdc = 1;

    /* Push opti data to relays */
    controllerFSM_PushOptiVars();

    /* Show current state of optimization */
    controllerFSM_LogState();
    controllerFSM_LogAutoFinished();

    s_controller_FSM_state = ControllerFsm__Init;
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
  uint8_t relValL;
  float   fwd_mw;

  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    relValL = s_controller_RelVal_L_cur;
    fwd_mw  = s_controller_adc_fwd_mw;

    taskENABLE_INTERRUPTS();
  }

  if (fwd_mw <= Controller_AutoSWR_P_mW_Max) {
    if (relEnable) {
      relValL |=   1UL << relLnum;

    } else {
      relValL &= ~(1UL << relLnum);
    }
    //float valL_nH = controllerCalcMatcherL2nH(relValL);

    /* Disabled IRQ section */
    {
      taskDISABLE_INTERRUPTS();

      s_controller_RelVal_L_cur = relValL;

      taskENABLE_INTERRUPTS();
    }

    controllerFSM_PushOptiVars();

  } else {
    const char errMsg[] = "*** Power to high to set L ***\r\n\r\n";
    interpreterConsolePush(errMsg, strlen(errMsg), 0);
  }
}

static void controllerSetC(uint8_t relLnum, uint8_t relEnable)
{
  uint8_t relValC;
  float   fwd_mw;

  /* Disabled IRQ section */
  {
    taskDISABLE_INTERRUPTS();

    relValC = s_controller_RelVal_C_cur;
    fwd_mw  = s_controller_adc_fwd_mw;

    taskENABLE_INTERRUPTS();
  }

  if (fwd_mw <= Controller_AutoSWR_P_mW_Max) {
    if (relEnable) {
      relValC |=   1UL << relLnum;

    } else {
      relValC &= ~(1UL << relLnum);
    }
    //float valC_pF = controllerCalcMatcherC2pF(relValC);

    /* Disabled IRQ section */
    {
      taskDISABLE_INTERRUPTS();

      s_controller_RelVal_C_cur = relValC;

      taskENABLE_INTERRUPTS();
    }

    controllerFSM_PushOptiVars();

  } else {
    const char errMsg[] = "*** Power to high to set C ***\r\n\r\n";
    interpreterConsolePush(errMsg, strlen(errMsg), 0);
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
    interpreterConsolePush(errMsg, strlen(errMsg), 0);
  }
}

static void controllerSetCLExt(uint32_t relays)
{
  const uint8_t l_C_relays        = ( relays        &    0xffUL);
  const uint8_t l_L_relays        = ((relays >> 8U) &    0xffUL);
  const ControllerOptiCVH_t l_CVH = ( relays        & 0x10000UL) ?  ControllerOptiCVH__CV : ControllerOptiCVH__CH;
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

      s_controller_RelVal_L_cur = l_L_relays;
      s_controller_RelVal_C_cur = l_C_relays;
      s_controller_FSM_optiCVH  = l_CVH;

      taskENABLE_INTERRUPTS();
    }

    controllerFSM_PushOptiVars();

  } else {
    const char errMsg[] = "*** Power to high to set C, L or C/L mode ***\r\n\r\n";
    interpreterConsolePush(errMsg, strlen(errMsg), 0);
  }
}

static void controllerSetAuto(uint8_t autoEnable)
{
  char strbuf[128];

  s_controller_doAutoMatching = autoEnable ?  1 : 0;

  const int len = snprintf(strbuf, (sizeof(strbuf) - 1), "\r\n*** Auto tuner: %s ***\r\n\r\n", autoEnable ?  "enabled" : "disabled");
  interpreterConsolePush(strbuf, len, 0);
  interpreterShowCursor();
}

static void controllerSetVerboseMode(uint8_t verboseMode)
{
  s_controller_verbose_bf = verboseMode;
}

static void controllerPrintLC(void)
{
  /* Disabled IRQ section */
  /* { */
    taskDISABLE_INTERRUPTS();

    const uint8_t relValL               = s_controller_RelVal_L_cur;
    const uint8_t relValC               = s_controller_RelVal_C_cur;
    const ControllerOptiCVH_t configLC  = s_controller_FSM_optiCVH;

    taskENABLE_INTERRUPTS();
  /* } */

  const float valL  = controllerCalcMatcherL2nH(relValL);
  const float valC  = controllerCalcMatcherC2pF(relValC);

  /* Print relay settings */
  {
    char  buf[4]        = { ' ', ' ', '?' };
    char  strbuf[128]   = { 0 };
    uint32_t relays;

    relays  = ((uint32_t)relValC <<  0U);
    relays |= ((uint32_t)relValL <<  8U);
    relays |= configLC == ControllerOptiCVH__CV ?  0x10000UL : 0x20000UL;

    /* 1st line: header */
    const char* bufStr = "\r\n\r\n## CH CV  L8 L7 L6 L5 L4 L3 L2 L1  C8 C7 C6 C5 C4 C3 C2 C1\r\n##";
    interpreterConsolePush(bufStr, strlen(bufStr), 1);

    /* 2nd line: bit field */
    for (int8_t idx = 17; idx >= 0; idx--) {
      if (idx == 15 || idx == 7) {
        interpreterConsolePush(" ", 1, 0);
      }
      buf[2] = (relays & (1UL << idx)) != 0UL ?  '1' : '0';
      interpreterConsolePush(buf, 3, 0);
    }

    /* 3rd line: hex number */
    const int len = snprintf(strbuf, (sizeof(strbuf) - 1), "\r\n## (Relay short settings: H%06lx)\r\n", (relays & 0x03ffffUL));
    interpreterConsolePush(strbuf, len, 0);
  }

  /* 4th line: print L/C configuration and L, C values */
  {
    const char*    sConfig      = configLC == ControllerOptiCVH__CV ?  "C-L   normal Gamma" : "L-C reverted Gamma";
    char           strbuf[128]  = { 0 };

    const int len = snprintf(strbuf, (sizeof(strbuf) - 1), "## Config: %s\t L = %ld nH\t C = %ld pF\r\n\r\n", sConfig, (uint32_t)valL, (uint32_t)valC);
    interpreterConsolePush(strbuf, len, 0);
  }

  interpreterShowCursor();
}


/* Timer functions */

static void controllerCyclicTimerEvent(void)
{
  /* Cyclic jobs to do */
  //const uint32_t  timeNow = osKernelSysTick();
  const uint32_t  timeNow   = portGET_RUN_TIME_COUNTER_VALUE();
  static uint32_t timeLast  = 0UL;

  /* Called every 30ms to start the ADCs */
  EventBits_t eb = xEventGroupGetBits(adcEventGroupHandle);
  if (!(eb & (EG_ADC1__CONV_RUNNING | EG_ADC2__CONV_RUNNING | EG_ADC3__CONV_RUNNING))) {
    /* No ADC conversions are active */

    adcStartConv(ADC_ADC1_REFINT_VAL);
    eb = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_VREF, 0UL, pdFALSE, 5UL / portTICK_PERIOD_MS);
    if (!(eb & EG_ADC1__CONV_AVAIL_VREF)) {
      return;
    }

    adcStartConv(ADC_ADC1_BAT_MV);
    adcStartConv(ADC_ADC2_IN1_FWD_MV);
    eb = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_BAT | EG_ADC2__CONV_AVAIL_FWD, 0UL, pdTRUE, 5UL / portTICK_PERIOD_MS);
    if ((eb & (EG_ADC1__CONV_AVAIL_BAT | EG_ADC2__CONV_AVAIL_FWD)) != (EG_ADC1__CONV_AVAIL_BAT | EG_ADC2__CONV_AVAIL_FWD)) {
      return;
    }

    adcStartConv(ADC_ADC1_TEMP_DEG);
    adcStartConv(ADC_ADC2_IN1_REV_MV);
    eb = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_TEMP | EG_ADC2__CONV_AVAIL_REV, 0UL, pdTRUE, 5UL / portTICK_PERIOD_MS);
    if ((eb & (EG_ADC1__CONV_AVAIL_TEMP | EG_ADC2__CONV_AVAIL_REV)) != (EG_ADC1__CONV_AVAIL_TEMP | EG_ADC2__CONV_AVAIL_REV)) {
      return;
    }

    adcStartConv(ADC_ADC3_IN3_VDIODE_MV);
  }

  /* Wait for end of convert */
  eb = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_VDIODE, 0UL, pdFALSE, 30UL / portTICK_PERIOD_MS);
  if (!(eb & EG_ADC3__CONV_AVAIL_VDIODE)) {
    return;
  }


  /* FSM logic */
  do {
    s_controller_doAdc = 0;
    controllerFSM();
  } while (!s_controller_doAdc);

  /* Fast path */
  if (!s_controller_doAdc) {
    return;
  }


  /* Handle serial CAT interface packets */
  {
    // TODO: coding here
  }

  /* Temperature compensation */
  if ((((timeNow - timeLast) > 1000000UL) || (timeNow < timeLast)) &&
        (s_controller_FSM_state <= ControllerFsm__StartAuto)) {
    /* Each second adjust for temperature changes */

    /* Calculate current temperature at the CMAD6001 diodes */
    s_controller_cmad_deg = 30.0f + (s_controller_adc_vdiode_mv - CMAD_30DEG_ADC_MV) / CMAD_ADC_MVpDEG;

    /* Adjust DigPoti setting */
    const float   l_controller_adcMin_mv          = min(g_adc_fwd_raw_mv, g_adc_rev_raw_mv);
    const uint8_t l_controller_logamp_potLast_val = s_controller_logamp_pot_val;

    /* Check against limits */
    if (l_controller_adcMin_mv < (-LOGAMP_OFS_MVpPOTVAL)) {
      /* Step down */
      --s_controller_logamp_pot_val;
      g_adc_logamp_ofsMeas_mv += -LOGAMP_OFS_MVpPOTVAL;

    } else if (l_controller_adcMin_mv > (3.0f * -LOGAMP_OFS_MVpPOTVAL)) {
      /* Step up */
      ++s_controller_logamp_pot_val;
      g_adc_logamp_ofsMeas_mv -= -LOGAMP_OFS_MVpPOTVAL;
    }

    /* Switch DigPoti to new setting */
    if (s_controller_logamp_pot_val != l_controller_logamp_potLast_val) {
      uint32_t cmd[2];
      cmd[0] = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 4U, MsgDefault__CallFunc05_DigPot_SetOffset);
      cmd[1] = (uint32_t) s_controller_logamp_pot_val;
      controllerMsgPushToInQueue(sizeof(cmd) / sizeof(int32_t), cmd, 10UL);

    } else {
      /* When no transmission takes place both have nearly same values */
      if (fabs(g_adc_fwd_raw_mv - g_adc_rev_raw_mv) < 25.0f) {
        g_adc_logamp_ofsMeas_mv = l_controller_adcMin_mv;
      }
    }

    /* Update timestamp */
    timeLast = timeNow;
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

#if 0
            /* Start the RtosDefault cyclic timer  */
            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 1U, MsgDefault__CallFunc02_CyclicTimerStart);
            msgAry[msgLen++]  = 1000UL;
#endif

            /* DigPot. - measurement set gain: 0..256 */
            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 1U, MsgDefault__CallFunc04_DigPot_SetGain);
            msgAry[msgLen++]  = LOGAMP_MUL_POT_VAL;

            /* DigPot. - measurement set offset: 0..256 */
            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 1U, MsgDefault__CallFunc05_DigPot_SetOffset);
            msgAry[msgLen++]  = s_controller_logamp_pot_val = LOGAMP_OFS_POT_VAL;

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

    case MsgController__SetVar06_A:
    {
      const uint8_t autoEnable = (s_msg_in.rawAry[1] & 0x01000000UL) >> 24U;
      controllerSetAuto(autoEnable);
    }
      break;

    case MsgController__SetVar07_V:
    {
      const uint8_t verboseMode = (s_msg_in.rawAry[1] & 0x01000000UL) >> 24U;
      controllerSetVerboseMode(verboseMode);
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

    s_controller_McuClocking                                  = DefaultMcuClocking_80MHz_MSI16_PLL;

    s_controller_doCycle                                      = RELAY_STILL_TIME;  // Each 30ms the relays are ready for a new setting
    s_controller_doAutoMatching                               = 1;
    s_controller_verbose_bf                                   = 0UL;

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

  /* Set relay state - inverted, then normal */
  {
    s_controller_RelVal_L_cur = 0xffU;
    s_controller_RelVal_C_cur = 0xffU;
    s_controller_FSM_optiCVH  = ControllerOptiCVH__CH;
    controllerFSM_PushOptiVars();

    osDelay(RELAY_STILL_TIME);

    s_controller_RelVal_L_cur = 0x00U;
    s_controller_RelVal_C_cur = 0x00U;
    s_controller_FSM_optiCVH  = ControllerOptiCVH__CV;
    controllerFSM_PushOptiVars();
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

  /* Reset SWR start timer */
  s_controller_swr_tmr = osKernelSysTick();


  /* Enable service cycle */
  if (s_controller_doCycle) {
    controllerCyclicStart(s_controller_doCycle);

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
