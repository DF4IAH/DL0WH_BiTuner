/*
 * task_Controller.h
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include "stm32l4xx_hal.h"
#include "main.h"


#define CONTROLLER_MSG_Q_LEN                                  8


typedef enum ControllerMsgDestinations_ENUM {

  Destinations__Unspec                                        = 0,
  Destinations__Controller,
  Destinations__Interpreter,
  Destinations__Rtos_Default,

  Destinations__Network_USBtoHost,
  Destinations__Network_USBfromHost,

  Destinations__Network_UartTx,
  Destinations__Network_UartRx,

  Destinations__Network_CatTx,
  Destinations__Network_CatRx,

} ControllerMsgDestinations_t;

typedef enum ControllerCmds_ENUM {

  MsgController__InitDo                                       = 0x01U,
  MsgController__InitDone,

  MsgController__DeInitDo                                     = 0x05U,

  MsgController__SetVar01_L                                   = 0x41U,
  MsgController__SetVar02_C,
  MsgController__SetVar03_CL,
  MsgController__SetVar04_LC,
  MsgController__SetVar05_K,
  MsgController__SetVar06_A,
  MsgController__SetVar07_V,

//MsgController__GetVar01                                     = 0x81U,

  MsgController__CallFunc01_CyclicTimerEvent                  = 0xc1U,
  MsgController__CallFunc02_CyclicTimerStart,
  MsgController__CallFunc03_CyclicTimerStop,
  MsgController__CallFunc04_Restart,
  MsgController__CallFunc05_PrintLC,

} ControllerCmds_t;



typedef enum Meas_Percent {
  P000                                                        = 0U,
  P025,
  P050,
  P075,
  P100
} Meas_Percent_t;

typedef struct Meas_Data {
  uint8_t relayVal[5];
  float   swr[5];
} Meas_Data_t;



typedef enum ControllerFsm_ENUM {

  ControllerFsm__NOP                                          = 0x00U,
  ControllerFsm__Init,
  ControllerFsm__StartAuto,

  ControllerFsm__L_Meas_P000                                  = 0x11U,
  ControllerFsm__L_Meas_P100,
  ControllerFsm__L_Meas_P050,
  ControllerFsm__L_Meas_P025,
  ControllerFsm__L_Meas_P075,
  ControllerFsm__L_Select,

  ControllerFsm__C_Meas_P000                                  = 0x21U,
  ControllerFsm__C_Meas_P100,
  ControllerFsm__C_Meas_P050,
  ControllerFsm__C_Meas_P025,
  ControllerFsm__C_Meas_P075,
  ControllerFsm__C_Select,

  ControllerFsm__done                                         = 0x30U,

} ControllerFsm_t;

typedef enum ControllerOptiCVH_ENUM {

  ControllerOptiCVH__CV                                       = 0U,
  ControllerOptiCVH__CH,

} ControllerOptiCVH_t;

typedef enum ControllerMonitor_BF {

  ControllerMon__ShowAdcs                                     = 0x0001UL,

} ControllerMonitor_BF_t;



typedef struct ControllerMods {

  uint8_t                             rtos_Default;
  uint8_t                             Interpreter;
  uint8_t                             network_USBtoHost;
  uint8_t                             network_USBfromHost;
  uint8_t                             network_UartTx;
  uint8_t                             network_UartRx;
  uint8_t                             network_CatTx;
  uint8_t                             network_CatRx;
  uint8_t                             ADC;

} ControllerMods_t;

typedef struct ControllerMsg2Proc {

  uint32_t                            rawAry[CONTROLLER_MSG_Q_LEN];
  uint8_t                             rawLen;

  ControllerMsgDestinations_t         msgSrc;
  ControllerMsgDestinations_t         msgDst;
  ControllerCmds_t                    msgCmd;
  uint8_t                             msgLen;

  uint8_t                             bRemain;

  uint8_t                             optLen;
  uint8_t                             optAry[CONTROLLER_MSG_Q_LEN << 2];

} ControllerMsg2Proc_t;



uint32_t controllerCalcMsgHdr(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd);
uint32_t controllerCalcMsgInit(uint32_t* ary, ControllerMsgDestinations_t dst, uint32_t startDelayMs);

float controllerCalcMatcherL2nH(uint8_t Lval);
float controllerCalcMatcherC2pF(uint8_t Cval);
uint8_t controllerCalcMatcherNH2L(float nH);
uint8_t controllerCalcMatcherPF2C(float pF);

uint32_t controllerMsgPushToInQueue(uint32_t msgLen, uint32_t* msgAry, uint32_t waitMs);
void controllerMsgPushToOutQueue(uint32_t msgLen, uint32_t* msgAry, uint32_t waitMs);
uint32_t controllerMsgPullFromOutQueue(uint32_t* msgAry, ControllerMsgDestinations_t dst, uint32_t waitMs);

void controllerTimerCallback(void const *argument);


/* Task */

void controllerTaskInit(void);
void controllerTaskLoop(void);

#endif /* TASK_CONTROLLER_H_ */
