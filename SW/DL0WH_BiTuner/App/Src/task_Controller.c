/*
 * task_Controller.c
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <task_USB.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l476xx.h"

//#include "stm32l4xx_hal_gpio.h"
#include "bus_spi.h"

#include "task_Controller.h"


extern osMessageQId         controllerInQueueHandle;
extern osMessageQId         controllerOutQueueHandle;
extern osTimerId            controllerTimerHandle;
extern osSemaphoreId        c2Default_BSemHandle;
extern osSemaphoreId        cQin_BSemHandle;
extern osSemaphoreId        cQout_BSemHandle;
extern osSemaphoreId        c2usbToHost_BSemHandle;
extern osSemaphoreId        c2usbFromHost_BSemHandle;
extern osSemaphoreId        usb_BSemHandle;

extern EventGroupHandle_t   globalEventGroupHandle;

extern float                g_adc_fwd_mv;
extern float                g_adc_swr;


static const float Controller_L0_nH                           = 50.0f;
static const float Controller_Ls_nH[8]                        = {
    187.5f,
    375.0f,
    750.0f,
    1.5e+3f,
    3.0e+3f,
    6.0e+3f,
    12.0e+3f,
    24.0e+3f
};

static const float Controller_C0_pf                           = 25.0f;
static float Controller_Cs_pF[8]                              = {
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
static const float          Controller_AutoSWR_SWR_Max        = 1.1f;
static const float          Controller_AutoSWR_Time_ms_Max    = 750.0f;
static const uint8_t        Controller_AutoSWR_CVHpong_Max    = 4U;
static const uint8_t        Controller_AutoSWR_LCpong_Max     = 6U;
static uint32_t             s_controller_swr_tmr              = 0UL;

static ControllerMsg2Proc_t s_msg_in                          = { 0 };

static ControllerMods_t     s_mod_start                       = { 0 };
static ControllerMods_t     s_mod_rdy                         = { 0 };
static ControllerFsm_t      s_controller_FSM_state            = ControllerFsm__NOP;
static ControllerOptiCVH_t  s_controller_FSM_optiCVH          = ControllerOptiCVH__CV;
static ControllerOptiLC_t   s_controller_FSM_optiLC           = ControllerOptiLC__L_double;
static uint8_t              s_controller_opti_CVHpongCtr      = 0U;
static uint8_t              s_controller_opti_LCpongCtr       = 0U;
static uint8_t              s_controller_opti_L               = 0U;
static float                s_controller_opti_L_val           = 0.0f;
static float                s_controller_opti_L_min_val       = 0.0f;
static float                s_controller_opti_L_max_val       = 0.0f;
static uint8_t              s_controller_opti_C               = 0U;
static float                s_controller_opti_C_val           = 0.0f;
static float                s_controller_opti_C_min_val       = 0.0f;
static float                s_controller_opti_C_max_val       = 0.0f;

static float                s_controller_adc_fwd_mv           = 0.0f;
static float                s_controller_adc_swr              = 0.0f;
static float                s_controller_adc_fwd_mw           = 0.0f;
static float                s_controller_last_swr             = 0.0f;

static _Bool                s_controller_doCycle              = false;
static _Bool                s_controller_doAdc                = false;
static DefaultMcuClocking_t s_controller_McuClocking          = DefaultMcuClocking__4MHz_MSI;


/* Calculation function */

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
  float sum = Controller_C0_pf;

  for (uint8_t idx = 0; idx < 8; idx++) {
    if (Cval & (1U << idx)) {
      sum += Controller_Cs_pF[idx];
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

uint32_t controllerCalcMsgHdr(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd)
{
  return ((uint32_t)dst << 24U) | ((uint32_t)src << 16U) | ((uint32_t)lengthBytes << 8U) | ((uint32_t)cmd);
}

static uint32_t controllerCalcMsgInit(uint32_t* ary, ControllerMsgDestinations_t dst, uint32_t startDelayMs)
{
  ary[0] = controllerCalcMsgHdr(dst, Destinations__Controller, sizeof(uint32_t), MsgController__InitDo);
  ary[1] = startDelayMs;
  return 2UL;
}


/* Messaging functions */

uint32_t controllerMsgPushToInQueue(uint8_t msgLen, uint32_t* msgAry, uint32_t waitMs)
{
  uint8_t idx = 0U;

  /* Sanity checks */
  if (!msgLen || (msgLen > CONTROLLER_MSG_Q_LEN) || !msgAry) {
    return 0UL;
  }

  /* Get semaphore to queue in */
  osSemaphoreWait(cQin_BSemHandle, waitMs);

  /* Push to the queue */
  while (idx < msgLen) {
    osMessagePut(controllerInQueueHandle, msgAry[idx++], osWaitForever);
  }

  /* Return semaphore */
  osSemaphoreRelease(cQin_BSemHandle);

  /* Door bell */
  xEventGroupSetBits(globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_IN);

  return idx;
}

void controllerMsgPushToOutQueue(uint8_t msgLen, uint32_t* msgAry, uint32_t waitMs)
{
  osSemaphoreId semId = 0;

  /* Get semaphore to queue out */
  if(osOK != osSemaphoreWait(cQout_BSemHandle, waitMs)) {
    return;
  }

  /* Push to the queue */
  uint8_t idx = 0U;
  while (idx < msgLen) {
    osMessagePut(controllerOutQueueHandle, msgAry[idx++], osWaitForever);
  }

  /* Ring bell at the destination */
  switch ((ControllerMsgDestinations_t) (msgAry[0] >> 24)) {
  case Destinations__Rtos_Default:
    semId = c2Default_BSemHandle;
    break;

  case Destinations__Network_USBtoHost:
    semId = c2usbToHost_BSemHandle;
    break;

  case Destinations__Network_USBfromHost:
    semId = c2usbFromHost_BSemHandle;
    break;

  default:
    semId = 0;
  }  // switch ()

  /* Wakeup of module */
  if (semId) {
    /* Grant blocked module to request the queue */
    osSemaphoreRelease(semId);                                                                        // Give semaphore token to launch module
  }

  /* Big Ben for the public */
  xEventGroupSetBits(  globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_OUT);
  xEventGroupClearBits(globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_OUT);

  /* Hand over ctrlQout */
  osSemaphoreRelease(cQout_BSemHandle);
}

static uint32_t controllerMsgPullFromInQueue(void)
{
  /* Prepare all fields */
  memset(&s_msg_in, 0, sizeof(s_msg_in));

  /* Process each message token */
  do {
    const osEvent evt = osMessageGet(controllerInQueueHandle, osWaitForever);
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
        s_msg_in.msgCmd   = (ControllerMsgControllerCmds_t) (0xffUL &  token        );

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
    }
  } while (1);

  /* Return semaphore */
  osSemaphoreRelease(cQout_BSemHandle);

  return len;
}


/* FSM functions */

static void controllerFSM_getGlobalVars(void)
{
  __disable_irq();
  s_controller_adc_fwd_mv     = g_adc_fwd_mv;
  s_controller_adc_swr        = g_adc_swr;
  __enable_irq();

  s_controller_adc_fwd_mw     = mainCalc_mV_to_mW(s_controller_adc_fwd_mv);
}

static _Bool controllerFSM_checkPower(void)
{
  if ((Controller_AutoSWR_P_mW_Min > s_controller_adc_fwd_mw) || (s_controller_adc_fwd_mw > Controller_AutoSWR_P_mW_Max)) {
    /* Logging */
    {
      char      buf[128];
      int32_t   pwr_i;
      uint32_t  pwr_f;

      mainCalcFloat2IntFrac(s_controller_adc_fwd_mw, 3, &pwr_i, &pwr_f);
      const int len = sprintf(buf, "Controller FSM: power=%5ld.%03lu out of [%u .. %u] Watts - stop auto tuner.\r\n",
                              pwr_i, pwr_f,
                              (uint16_t)Controller_AutoSWR_P_mW_Min, (uint16_t)Controller_AutoSWR_P_mW_Max);
      usbLogLen(buf, len);
    }

    /* Reset SWR start timer */
    s_controller_swr_tmr = osKernelSysTick();

    /* Overdrive or to low energy */
    s_controller_FSM_state = ControllerFsm__doAdc;

    return true;
  }
  return false;
}

static _Bool controllerFSM_checkSwrTime(void)
{
  if (Controller_AutoSWR_SWR_Max > s_controller_adc_swr) {
    /* Logging */
    {
      char buf[128];
      int32_t   swr_i;
      uint32_t  swr_f;

      mainCalcFloat2IntFrac(s_controller_adc_swr, 3, &swr_i, &swr_f);
      const int len = sprintf(buf, "Controller FSM: SWR=%2ld.%03lu is good enough - stop auto tuner.\r\n",
                              swr_i, swr_f);
      usbLogLen(buf, len);
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

static void controllerFSM_switchOverCVH(void)
{
  /* Check if another CVH switch over is allowed */
  if (++s_controller_opti_CVHpongCtr <= Controller_AutoSWR_CVHpong_Max) {
    s_controller_opti_LCpongCtr = 0U;

    /* Logging */
    {
      char      buf[128];
      const int len = sprintf(buf, "Controller FSM: switch over the constellation to %d (0: CV, 1: CH), CVH pingpong ctr=%u.\r\n",
                              !s_controller_FSM_optiCVH, s_controller_opti_CVHpongCtr);
      usbLogLen(buf, len);
    }

    /* Switch over to opposite CVH constellation */
    s_controller_FSM_optiCVH = !s_controller_FSM_optiCVH;
    if (s_controller_FSM_optiCVH == ControllerOptiCVH__CH) {
      s_controller_FSM_optiLC = ControllerOptiLC__C_double;
      s_controller_opti_L_val = Controller_L0_nH;
      s_controller_opti_C_val = Controller_C0_pf + Controller_Cs_pF[0];
      s_controller_FSM_state  = ControllerFsm__findImagZeroC;

    } else {
      s_controller_FSM_optiLC = ControllerOptiLC__L_double;
      s_controller_opti_L_val = Controller_L0_nH + Controller_Ls_nH[0];
      s_controller_opti_C_val = Controller_C0_pf;
      s_controller_FSM_state  = ControllerFsm__findImagZeroL;
    }

  } else {
    /* Exhausted - start over again */
    s_controller_FSM_state = ControllerFsm__doAdc;
  }
}

static void controllerFSM_pushOptiVars(void)
{
  uint8_t controller_opti_CV;
  uint8_t controller_opti_CH;
  uint32_t msgAry[2];

  /* Calculate current L and C counter setings */
  s_controller_opti_L = controllerCalcMatcherNH2L(s_controller_opti_L_val);
  s_controller_opti_C = controllerCalcMatcherPF2C(s_controller_opti_C_val);

  /* Update CV/CH state */
  if (s_controller_FSM_optiCVH == ControllerOptiCVH__CH) {
    controller_opti_CV  = 0U;
    controller_opti_CH  = 1U;

  } else {
    controller_opti_CV  = 1U;
    controller_opti_CH  = 0U;
  }

  /* Compose relay bitmap */
  msgAry[1] = ((uint32_t)  controller_opti_CH << 17) |
              ((uint32_t)  controller_opti_CV << 16) |
              ((uint32_t)s_controller_opti_L  <<  8) |
              ((uint32_t)s_controller_opti_C       ) ;

  /* Three extra bytes to take over */
  msgAry[0] = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 3, MsgDefault__SetVar03_C_L_CV_CH);
  controllerMsgPushToOutQueue(sizeof(msgAry) / sizeof(uint32_t), msgAry, osWaitForever);
}

static void controllerFSM_startAdc(void)
{
  s_controller_last_swr = s_controller_adc_swr;
  s_controller_doAdc    = true;
}

static void controllerFSM_zeroXHalfStrategy(void)
{
  if (s_controller_FSM_optiLC == ControllerOptiLC__L_half) {
    /* Find intermediate L */
    s_controller_opti_L_val = (s_controller_opti_L_min_val + s_controller_opti_L_max_val) / 2.0f;

    /* Check if minimum of SWR is found */
    if (Controller_Ls_nH[0] >= (s_controller_opti_L_max_val - s_controller_opti_L_min_val)) {
      /* Check if optimum found by increase of L */
      if (s_controller_opti_L_val >= Controller_Ls_nH[0]) {
        /* Advance L by 25%, at least by one step, limiting */
        float advanced = s_controller_opti_L_val * 1.25f + Controller_Ls_nH[0];
        if (advanced > 2.0f * Controller_Ls_nH[7]) {
          s_controller_opti_L_val = controllerCalcMatcherNH2L( controllerCalcMatcherL2nH(advanced) );
        }

        /* Next optimize C */
        s_controller_opti_C_min_val = Controller_C0_pf;
        s_controller_opti_C_max_val = s_controller_opti_C_val = Controller_C0_pf + Controller_Cs_pF[0];
        s_controller_opti_LCpongCtr = 0U;
        s_controller_FSM_optiLC     = ControllerOptiLC__C_double;
        s_controller_FSM_state      = ControllerFsm__findMinSwrC;

        /* Forget this minimum due to C advance */
        s_controller_last_swr = 99.9f;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_switchOverCVH();
      }
    }

  } else if (s_controller_FSM_optiLC == ControllerOptiLC__C_half) {
    /* Find intermediate C */
    s_controller_opti_C_val = (s_controller_opti_C_min_val + s_controller_opti_C_max_val) / 2.0f;

    /* Check if minimum of SWR is found */
    if (Controller_Cs_pF[0] >= (s_controller_opti_C_max_val - s_controller_opti_C_min_val)) {
      /* Check if optimum will be found by increase of C */
      if (s_controller_opti_C_val > Controller_Cs_pF[1]) {
        /* Advance C by 25%, at least by one step */
        float advanced = s_controller_opti_C_val * 1.25f + Controller_Cs_pF[0];
        if (advanced > 2.0f * Controller_Cs_pF[7]) {
          s_controller_opti_C_val = controllerCalcMatcherPF2C( controllerCalcMatcherC2pF(advanced) );
        }

        /* Next optimize L */
        s_controller_opti_L_min_val = Controller_L0_nH;
        s_controller_opti_L_max_val = s_controller_opti_L_val = Controller_L0_nH + Controller_Ls_nH[0];
        s_controller_opti_LCpongCtr = 0U;
        s_controller_FSM_optiLC     = ControllerOptiLC__L_double;
        s_controller_FSM_state      = ControllerFsm__findMinSwrL;

        /* Forget this minimum due to L advance */
        s_controller_last_swr = 99.9f;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_switchOverCVH();
      }
    }
  }
}

static void controllerFSM_optiHalfStrategy(void)
{
  if (s_controller_FSM_optiLC == ControllerOptiLC__L_half) {
    /* Find intermediate L */
    s_controller_opti_L_val = (s_controller_opti_L_min_val + s_controller_opti_L_max_val) / 2.0f;

    if (Controller_Ls_nH[0] >= (s_controller_opti_L_max_val - s_controller_opti_L_min_val)) {
      if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
        /* Optimize C, again */
        s_controller_FSM_optiLC = ControllerOptiLC__C_cntUp;
        s_controller_FSM_state  = ControllerFsm__findMinSwrC;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_switchOverCVH();
      }
    }

  } else if (s_controller_FSM_optiLC == ControllerOptiLC__C_half) {
    /* Find intermediate C */
    s_controller_opti_C_val = (s_controller_opti_C_min_val + s_controller_opti_C_max_val) / 2.0f;

    if (Controller_Cs_pF[0] >= (s_controller_opti_C_max_val - s_controller_opti_C_min_val)) {
      if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
        /* Optimize L, again */
        s_controller_FSM_optiLC = ControllerOptiLC__L_cntUp;
        s_controller_FSM_state  = ControllerFsm__findMinSwrL;

      } else {
        /* Switch over to opposite CVH constellation */
        controllerFSM_switchOverCVH();
      }
    }
  }
}

static void controllerFSM_logState(void)
{
  /* Show current state of optimization */
  char      buf[512];
  int32_t   s_controller_opti_L_val_i, s_controller_opti_L_min_val_i, s_controller_opti_L_max_val_i, s_controller_opti_C_val_i, s_controller_opti_C_min_val_i, s_controller_opti_C_max_val_i;
  uint32_t  s_controller_opti_L_val_f, s_controller_opti_L_min_val_f, s_controller_opti_L_max_val_f, s_controller_opti_C_val_f, s_controller_opti_C_min_val_f, s_controller_opti_C_max_val_f;
  int len;

  mainCalcFloat2IntFrac(s_controller_opti_L_val,      3, &s_controller_opti_L_val_i,     &s_controller_opti_L_val_f);
  mainCalcFloat2IntFrac(s_controller_opti_L_min_val,  3, &s_controller_opti_L_min_val_i, &s_controller_opti_L_min_val_f);
  mainCalcFloat2IntFrac(s_controller_opti_L_max_val,  3, &s_controller_opti_L_max_val_i, &s_controller_opti_L_max_val_f);
  mainCalcFloat2IntFrac(s_controller_opti_C_val,      3, &s_controller_opti_C_val_i,     &s_controller_opti_C_val_f);
  mainCalcFloat2IntFrac(s_controller_opti_C_min_val,  3, &s_controller_opti_C_min_val_i, &s_controller_opti_C_min_val_f);
  mainCalcFloat2IntFrac(s_controller_opti_C_max_val,  3, &s_controller_opti_C_max_val_i, &s_controller_opti_C_max_val_f);
  len = sprintf(buf, "Controller FSM:\tcontrollerFSM_logState - FSM_state=%u, FSM_optiLC=%u, FSM_optiVCH=%u, FSM_opti_L=%03u, FSM_opti_C=%03u,\r\n" \
                "\t\t\topti_CVHpongCtr=%03u, opti_LCpongCtr=%03u,\r\n" \
                "\t\t\tL_val=%5ld.%03lu nH, L_min=%5ld.%03lu nH, L_max=%5ld.%03lu nH\t\tC_val=%5ld.%03lu pF, C_min=%5ld.%03lu pF, C_max=%5ld.%03lu pF,\r\n",
                s_controller_FSM_state, s_controller_FSM_optiLC, s_controller_FSM_optiCVH, s_controller_opti_L, s_controller_opti_C,
                s_controller_opti_CVHpongCtr, s_controller_opti_LCpongCtr,
                s_controller_opti_L_val_i, s_controller_opti_L_val_f,  s_controller_opti_L_min_val_i, s_controller_opti_L_min_val_f,  s_controller_opti_L_max_val_i, s_controller_opti_L_max_val_f,
                s_controller_opti_C_val_i, s_controller_opti_C_val_f,  s_controller_opti_C_min_val_i, s_controller_opti_C_min_val_f,  s_controller_opti_C_max_val_i, s_controller_opti_C_max_val_f);
  usbLogLen(buf, len);


  int32_t   swr_i, last_swr_i;
  uint32_t  swr_f, last_swr_f;

  mainCalcFloat2IntFrac(s_controller_adc_swr,  3, &swr_i,      &swr_f);
  mainCalcFloat2IntFrac(s_controller_last_swr, 3, &last_swr_i, &last_swr_f);
  len = sprintf(buf, "\t\t\tSum L=%5lu nH, C=%5lu pF,\r\n" \
                "\t\t\tfwd_mv=%5lu mV, fwd_mw=%5lu mW,\r\n" \
                "\t\t\tswr=%5ld.%03lu, last_swr=%5ld.%03lu.\r\n\r\n",
                (uint32_t)controllerCalcMatcherL2nH(s_controller_opti_L_val), (uint32_t)controllerCalcMatcherC2pF(s_controller_opti_C_val),
                (uint32_t)s_controller_adc_fwd_mv, (uint32_t)s_controller_adc_fwd_mw,
                swr_i, swr_f, last_swr_i, last_swr_f);
  usbLogLen(buf, len);
}

static void controllerFSM(void)
{
  switch (s_controller_FSM_state)
  {

  case ControllerFsm__NOP:
  case ControllerFsm__doAdc:
  {
    /* Init SWR compare value */
    s_controller_last_swr = 99.9f;

    controllerFSM_startAdc();

    s_controller_FSM_state = ControllerFsm__startAuto;
  }
    break;

  case ControllerFsm__startAuto:
  {
    /* Pull global vars */
    controllerFSM_getGlobalVars();

    /* Check for security */
    if (controllerFSM_checkPower())
      break;

    /* Check if auto tuner should start */
    if (controllerFSM_checkSwrTime())
      break;

    /* Run (V)SWR optimization */
    s_controller_FSM_optiCVH      = ControllerOptiCVH__CV;
    s_controller_FSM_optiLC       = ControllerOptiLC__L_double;
    s_controller_FSM_state        = ControllerFsm__findImagZeroL;
    s_controller_opti_CVHpongCtr  = 0U;
    s_controller_opti_LCpongCtr   = 0U;
    s_controller_opti_L_min_val   = Controller_L0_nH;
    s_controller_opti_L_max_val   =                               s_controller_opti_L_val = Controller_L0_nH + Controller_Ls_nH[0];
    s_controller_opti_C_max_val   = s_controller_opti_C_min_val = s_controller_opti_C_val = Controller_C0_pf;

    /* Push opti data to relays */
    controllerFSM_pushOptiVars();
    controllerFSM_startAdc();

    /* Logging */
    {
      char buf[128];

      const int len = sprintf(buf, "Controller FSM: ControllerFsm__startAuto - start auto tuner.\r\n");
      usbLogLen(buf, len);
    }
  }
    break;

  case ControllerFsm__findImagZeroL:
  {
    /* Pull global vars */
    controllerFSM_getGlobalVars();

    /* Check for security */
    if (controllerFSM_checkPower())
      break;

    /* Check for SWR */
    if (s_controller_adc_swr < s_controller_last_swr) {
      /* SWR got better */

      if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Max) {
        /* SWR: we have got it */
        s_controller_FSM_state = ControllerFsm__done;
        s_controller_doAdc     = true;

        /* Logging */
        {
          char buf[128];

          const int len = sprintf(buf, "Controller FSM: ControllerFsm__findImagZeroL - SWR good enough - tuner has finished.\r\n");
          usbLogLen(buf, len);
        }
        break;
      }

      /* Inductance at least this value */
      s_controller_opti_L_min_val = s_controller_opti_L_val;

      if (s_controller_FSM_optiLC == ControllerOptiLC__L_double) {
        /* Double L for quick access */
        s_controller_opti_L_val *= 2.0f;
      }

    } else {
      /* SWR got worse */

      /* Inductance at most this value */
      s_controller_opti_L_max_val = s_controller_opti_L_val;

      if (s_controller_FSM_optiLC == ControllerOptiLC__L_double) {
        /* Overshoot of inductance - change strategy */
        s_controller_FSM_optiLC = ControllerOptiLC__L_half;
      }
    }

    /* Half strategy for zero X (when active) */
    controllerFSM_zeroXHalfStrategy();

    /* Push opti data to relays */
    controllerFSM_pushOptiVars();
    controllerFSM_startAdc();

    /* Show current state of optimization */
    controllerFSM_logState();
  }
    break;

  case ControllerFsm__findImagZeroC:
  {
    /* Pull global vars */
    controllerFSM_getGlobalVars();

    /* Check for security */
    if (controllerFSM_checkPower())
      break;

    /* Check for SWR */
    if (s_controller_adc_swr < s_controller_last_swr) {
      /* SWR got better */

      if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Max) {
        /* SWR: we have got it */
        s_controller_FSM_state = ControllerFsm__done;
        s_controller_doAdc     = true;

        /* Logging */
        {
          char buf[128];

          const int len = sprintf(buf, "Controller FSM: ControllerFsm__findImagZeroC - SWR good enough - tuner has finished.\r\n");
          usbLogLen(buf, len);
        }
        break;
      }

      /* Capacitance at least this value */
      s_controller_opti_C_min_val = s_controller_opti_C_val;

      if (s_controller_FSM_optiLC == ControllerOptiLC__C_double) {
        /* Double C for quick access */
        s_controller_opti_C_val *= 2.0f;
      }

    } else {
      /* SWR got worse */

      /* Capacitance at most this value */
      s_controller_opti_C_max_val = s_controller_opti_C_val;

      if (s_controller_FSM_optiLC == ControllerOptiLC__C_double) {
        /* Overshoot of capacitance - change strategy */
        s_controller_FSM_optiLC = ControllerOptiLC__C_half;
      }
    }

    /* Half strategy for zero X (when active) */
    controllerFSM_zeroXHalfStrategy();

    /* Push opti data to relays */
    controllerFSM_pushOptiVars();
    controllerFSM_startAdc();

    /* Show current state of optimization */
    controllerFSM_logState();
  }
    break;

  case ControllerFsm__findMinSwrC:
  {
    /* Pull global vars */
    controllerFSM_getGlobalVars();

    /* Check for security */
    if (controllerFSM_checkPower())
      break;

    /* Check for SWR */
    if (s_controller_adc_swr < s_controller_last_swr) {
      /* SWR got better */

      if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Max) {
        /* SWR: we have got it */
        s_controller_FSM_state = ControllerFsm__done;
        s_controller_doAdc     = true;

        /* Logging */
        {
          char buf[128];

          const int len = sprintf(buf, "Controller FSM: ControllerFsm__findMinSwrC - SWR good enough - tuner has finished.\r\n");
          usbLogLen(buf, len);
        }
        break;
      }

      /* New limit */
      if (s_controller_FSM_optiLC == ControllerOptiLC__C_cntDwn) {
        /* Decreasing */
        s_controller_opti_C_max_val = s_controller_opti_C_val;

      } else {
        /* Increasing */
        s_controller_opti_C_min_val = s_controller_opti_C_val;
      }

      /* Search strategy */
      if (s_controller_FSM_optiLC == ControllerOptiLC__C_double) {
        /* Double C for quick access */
        s_controller_opti_C_val *= 2.0f;

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__C_cntUp) {
        /* Single step increase */
        s_controller_opti_C_val += Controller_Cs_pF[0];
        if (s_controller_opti_C_val >= 2.0f * Controller_Cs_pF[7]) {
          s_controller_opti_C_val = controllerCalcMatcherC2pF( controllerCalcMatcherPF2C(s_controller_opti_C_val) );

          /* Banging the limit - try opposite CVH setup */
          controllerFSM_switchOverCVH();
        }

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__C_cntDwn) {
        /* Single step decrease */
        s_controller_opti_C_val -= Controller_Cs_pF[0];
        if (s_controller_opti_C_val < Controller_C0_pf) {
          s_controller_opti_C_val = Controller_C0_pf;

          if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
            /* Change over to L trim */
            s_controller_FSM_optiLC = ControllerOptiLC__L_cntUp;
            s_controller_FSM_state  = ControllerFsm__findMinSwrL;

          } else {
            /* Exhausted - try opposite CVH setup */
            controllerFSM_switchOverCVH();
          }
        }
      }

    } else {
      /* SWR got worse */

      /* New limit */
      if (s_controller_FSM_optiLC == ControllerOptiLC__L_cntDwn) {
        /* Decreasing */
        s_controller_opti_C_min_val = s_controller_opti_C_val;

      } else {
        /* Increasing */
        s_controller_opti_C_max_val = s_controller_opti_C_val;
      }

      /* Search strategy */
      if (s_controller_FSM_optiLC == ControllerOptiLC__C_double) {
        /* Change strategy */
        s_controller_FSM_optiLC = ControllerOptiLC__C_half;

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__C_cntUp) {
        /* Change strategy */
        s_controller_FSM_optiLC = ControllerOptiLC__C_cntDwn;

        /* Single step decrease */
        s_controller_opti_C_val -= Controller_Cs_pF[0];
        if (s_controller_opti_C_val < Controller_C0_pf) {
          s_controller_opti_C_val = Controller_C0_pf;

          if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
            /* Change over to L trim */
            s_controller_FSM_optiLC = ControllerOptiLC__L_cntUp;
            s_controller_FSM_state  = ControllerFsm__findMinSwrL;

          } else {
            /* Switch over to opposite CVH constellation */
            controllerFSM_switchOverCVH();
          }
        }

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__C_cntDwn) {
        if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
          /* Change over to L trim */
          s_controller_FSM_optiLC = ControllerOptiLC__L_cntUp;
          s_controller_FSM_state  = ControllerFsm__findMinSwrL;

        } else {
          /* Switch over to opposite CVH constellation */
          controllerFSM_switchOverCVH();
        }
      }
    }

    /* Half strategy (when active) */
    controllerFSM_optiHalfStrategy();

    /* Push opti data to relays */
    controllerFSM_pushOptiVars();
    controllerFSM_startAdc();

    /* Show current state of optimization */
    controllerFSM_logState();
  }
    break;

  case ControllerFsm__findMinSwrL:
  {
    /* Pull global vars */
    controllerFSM_getGlobalVars();

    /* Check for security */
    if (controllerFSM_checkPower())
      break;

    /* Check for SWR */
    if (s_controller_adc_swr < s_controller_last_swr) {
      /* SWR got better */

      if (s_controller_adc_swr <= Controller_AutoSWR_SWR_Max) {
        /* SWR: we have got it */
        s_controller_FSM_state = ControllerFsm__done;
        s_controller_doAdc     = true;

        /* Logging */
        {
          char buf[128];

          const int len = sprintf(buf, "Controller FSM: ControllerFsm__findMinSwrL - SWR good enough - tuner has finished.\r\n");
          usbLogLen(buf, len);
        }
        break;
      }

      /* New limit */
      if (s_controller_FSM_optiLC == ControllerOptiLC__L_cntDwn) {
        /* Decreasing */
        s_controller_opti_L_max_val = s_controller_opti_L_val;

      } else {
        /* Increasing */
        s_controller_opti_L_min_val = s_controller_opti_L_val;
      }

      /* Search strategy */
      if (s_controller_FSM_optiLC == ControllerOptiLC__L_double) {
        /* Double L for quick access */
        s_controller_opti_L_val *= 2.0f;

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__L_cntUp) {
        /* Single step increase */
        s_controller_opti_L_val += Controller_Ls_nH[0];
        if (s_controller_opti_L_val >= 2.0f * Controller_Ls_nH[7]) {
          s_controller_opti_L_val = controllerCalcMatcherL2nH( controllerCalcMatcherNH2L(s_controller_opti_L_val) );

          /* Banging the limit - try opposite CVH setup */
          controllerFSM_switchOverCVH();
        }

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__L_cntDwn) {
        /* Single step decrease */
        s_controller_opti_L_val -= Controller_Ls_nH[0];
        if (s_controller_opti_L_val < Controller_L0_nH) {
          s_controller_opti_L_val = Controller_L0_nH;

          if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
            /* Change over to C trim */
            s_controller_FSM_optiLC = ControllerOptiLC__C_cntUp;
            s_controller_FSM_state  = ControllerFsm__findMinSwrC;

          } else {
            /* Switch over to opposite CVH constellation */
            controllerFSM_switchOverCVH();
          }
        }
      }

    } else {
      /* SWR got worse */

      /* New limit */
      if (s_controller_FSM_optiLC == ControllerOptiLC__L_cntDwn) {
        /* Decreasing */
        s_controller_opti_L_min_val = s_controller_opti_L_val;

      } else {
        /* Increasing */
        s_controller_opti_L_max_val = s_controller_opti_L_val;
      }

      /* Search strategy */
      if (s_controller_FSM_optiLC == ControllerOptiLC__L_double) {
        /* Change strategy */
        s_controller_FSM_optiLC = ControllerOptiLC__L_half;

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__L_cntUp) {
        /* Change strategy */
        s_controller_FSM_optiLC = ControllerOptiLC__L_cntDwn;

        /* Single step decrease */
        s_controller_opti_L_val -= Controller_Ls_nH[0];
        if (s_controller_opti_L_val < Controller_L0_nH) {
          s_controller_opti_L_val = Controller_L0_nH;

          if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
            /* Change over to C trim */
            s_controller_FSM_optiLC = ControllerOptiLC__C_cntUp;
            s_controller_FSM_state  = ControllerFsm__findMinSwrC;

          } else {
            /* Exhausted - start over again */
            s_controller_FSM_state = ControllerFsm__doAdc;
          }
        }

      } else if (s_controller_FSM_optiLC == ControllerOptiLC__L_cntDwn) {
        if (++s_controller_opti_LCpongCtr <= Controller_AutoSWR_LCpong_Max) {
          /* Change over to C trim */
          s_controller_FSM_optiLC = ControllerOptiLC__C_cntUp;
          s_controller_FSM_state  = ControllerFsm__findMinSwrC;

        } else {
          /* Switch over to opposite CVH constellation */
          controllerFSM_switchOverCVH();
        }
      }
    }

    /* Half strategy (when active) */
    controllerFSM_optiHalfStrategy();

    /* Push opti data to relays */
    controllerFSM_pushOptiVars();
    controllerFSM_startAdc();

    /* Show current state of optimization */
    controllerFSM_logState();
 }
    break;

  case ControllerFsm__done:
  {
    /* Done - wait for new start */
    s_controller_FSM_state = ControllerFsm__doAdc;
    controllerFSM_startAdc();
  }
    break;

  default:
  {
    s_controller_FSM_state = ControllerFsm__NOP;
  }

  }  // switch ()
}


/* Timer functions */

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

/* Cyclic job to do */
static void controllerCyclicTimerEvent(void)
{
  uint32_t  msgAry[16];
  uint8_t   msgLen = 0U;

  /* FSM logic */
  controllerFSM();

  if (s_controller_doAdc) {
    /* Get ADC channels */
    msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 0U, MsgDefault__CallFunc01_MCU_ADC1);
    msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 0U, MsgDefault__CallFunc02_MCU_ADC3_VDIODE);
    msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 0U, MsgDefault__CallFunc03_MCU_ADC2_FWD);
    msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Rtos_Default, Destinations__Controller, 0U, MsgDefault__CallFunc04_MCU_ADC2_REV);
  }

  /* Push to queue */
  controllerMsgPushToInQueue(msgLen, msgAry, 1UL);
}


static void controllerMsgProcessor(void)
{
  uint32_t msgAry[CONTROLLER_MSG_Q_LEN] = { 0 };

  if (s_msg_in.msgDst > Destinations__Controller) {
    /* Forward message to the destination via the ctrlQout */
    const uint8_t cnt                     = s_msg_in.rawLen;
    uint8_t msgLen                        = 0U;

    /* Copy message header and option entries to the target */
    for (uint8_t idx = 0; idx < cnt; ++idx) {
      msgAry[msgLen++] = s_msg_in.rawAry[idx];
    }

    /* Push message out */
    controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);

  } else {
    /* Message is for us */

    /* Register all ready modules */
    switch (s_msg_in.msgCmd) {
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
            controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
          }
          break;

        case Destinations__Network_USBtoHost:
          s_mod_rdy.network_USBtoHost = 1U;
          break;

        case Destinations__Network_USBfromHost:
          s_mod_rdy.network_USBfromHost = 1U;
          break;

        default:
          Error_Handler();
        }  // switch (s_msg_in.msgSrc)
      }  // case MsgController__InitDone { }
    break;

    /* Process own command */
    case MsgController__CallFunc01_CyclicTimerEvent:
    {
      controllerCyclicTimerEvent();
    }
    break;

    default:
      Error_Handler();
    }  // switch (s_msg_in.msgCmd)
  }  // else

  /* Discard message */
  memset(&s_msg_in, 0, sizeof(s_msg_in));
}


static void controllerInit(void)
{
  /* Load configuration */

  /* Prepare all semaphores */
  {
    osSemaphoreWait(c2Default_BSemHandle,     osWaitForever);
    osSemaphoreWait(c2usbToHost_BSemHandle,   osWaitForever);
    osSemaphoreWait(c2usbFromHost_BSemHandle, osWaitForever);
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
    s_mod_start.network_USBtoHost                             = 1U;
    s_mod_start.network_USBfromHost                           = 1U;
  }

  /* Signaling controller is up and running */
  xEventGroupSetBits(globalEventGroupHandle, EG_GLOBAL__Controller_CTRL_IS_RUNNING);
  osDelay(10UL);

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
  }

  /* Set relay state */
  controllerFSM_pushOptiVars();

  /* Enable service cycle */
  if (s_controller_doCycle) {
    /* Latching relays do need 30ms to change state */
    controllerCyclicStart(30UL);

  } else {
    controllerCyclicStop();
  }

  /* Reset SWR start timer */
  s_controller_swr_tmr = osKernelSysTick();

  //#define I2C1_BUS_ADDR_SCAN
  #ifdef I2C1_BUS_ADDR_SCAN
  i2cBusAddrScan(&hi2c1, i2c1MutexHandle);
  #endif
}


/* Tasks */

void controllerTaskInit(void)
{
  controllerInit();
}

void controllerTaskLoop(void)
{
  const EventBits_t eb = xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_QUEUE_IN,
      EG_GLOBAL__Controller_QUEUE_IN,
      0, HAL_MAX_DELAY);

  if (EG_GLOBAL__Controller_QUEUE_IN & eb) {
    /* Get next complete messages */
    if (osOK != controllerMsgPullFromInQueue()) {
      /* Message Queue corrupt */
      Error_Handler();
    }

    /* Process message */
    controllerMsgProcessor();
  }
}
