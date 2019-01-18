#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include "main.hpp"


typedef enum ControllerMsgDestinations_ENUM {

  Destinations__Unspec                                        = 0,
  Destinations__Controller,
  Destinations__Rtos_Default,

  Destinations__Network_USBtoHost,
  Destinations__Network_USBfromHost,

} ControllerMsgDestinations_t;


typedef enum ControllerFsm_ENUM {

  ControllerFsm__NOP                                          = 0U,
  ControllerFsm__doAdc,
  ControllerFsm__startAuto,
  ControllerFsm__findImagZero,
  ControllerFsm__findMinSwr,
  ControllerFsm__done,

} ControllerFsm_t;

typedef enum ControllerOptiCVH_ENUM {

  ControllerOptiCVH__CV                                       = 0U,
  ControllerOptiCVH__CH,

} ControllerOptiCVH_t;

typedef enum ControllerOptiLC_ENUM {

  ControllerOptiLC__L                                         = 0U,
  ControllerOptiLC__C,

} ControllerOptiLC_t;

typedef enum ControllerOptiStrat_ENUM {

  ControllerOptiStrat__Double                                 = 0U,
  ControllerOptiStrat__Half,
  ControllerOptiStrat__SingleCnt,

} ControllerOptiStrat_t;

typedef enum ControllerOptiUpDn_ENUM {

  ControllerOptiUpDn__Up                                      = 0U,
  ControllerOptiUpDn__Dn,

} ControllerOptiUpDn_t;



uint32_t controllerCalcMsgHdr(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd);
uint32_t controllerCalcMsgInit(uint32_t* ary, ControllerMsgDestinations_t dst, uint32_t startDelayMs);

float controllerCalcMatcherL2nH(uint8_t Lval);
float controllerCalcMatcherC2pF(uint8_t Cval);
uint8_t controllerCalcMatcherNH2L(float nH);
uint8_t controllerCalcMatcherPF2C(float pF);
float controllerCalcVSWR_Simu(float antOhmR, float antOhmI, float Z0R, float Z0I, float frequencyHz);

void controllerCyclicTimerEvent(void);
void controllerInit(void);


#endif
