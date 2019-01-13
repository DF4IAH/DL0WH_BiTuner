#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include "main.h"


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
  ControllerFsm__findImagZeroL,
  ControllerFsm__findImagZeroC,
  ControllerFsm__findMinSwrC,
  ControllerFsm__findMinSwrL,

} ControllerFsm_t;

typedef enum ControllerOptiCVH_ENUM {

  ControllerOptiCVH__CV                                       = 0U,
  ControllerOptiCVH__CH,

} ControllerOptiCVH_t;

typedef enum ControllerOptiLC_ENUM {

  ControllerOptiLC__L_double                                  = 0U,
  ControllerOptiLC__L_half,
  ControllerOptiLC__L_cntUp,
  ControllerOptiLC__L_cntDwn,
  ControllerOptiLC__C_double,
  ControllerOptiLC__C_half,
  ControllerOptiLC__C_cntUp,
  ControllerOptiLC__C_cntDwn,

} ControllerOptiLC_t;



uint32_t controllerCalcMsgHdr(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd);
uint32_t controllerCalcMsgInit(uint32_t* ary, ControllerMsgDestinations_t dst, uint32_t startDelayMs);

float controllerCalcMatcherC2pF(uint8_t Cval);
float controllerCalcMatcherL2nH(uint8_t Lval);
uint32_t controllerCalcMsgHdr(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd);

void controllerCyclicTimerEvent(void);
void controllerInit(void);


#endif
