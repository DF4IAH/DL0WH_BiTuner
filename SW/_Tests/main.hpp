#ifndef _MAIN_H
#define _MAIN_H

#include <stdint.h>


#ifndef true
# define true   1U
#endif

#ifndef false
# define false  0U
#endif


typedef enum RtosMsgDefaultCmds_ENUM {

  MsgDefault__InitDo                                          = 0x01U,
  MsgDefault__InitDone,

  MsgDefault__SetVar01_IOs                                    = 0x41U,
  MsgDefault__SetVar02_Clocking,
  MsgDefault__SetVar03_C_L_CV_CH,

//MsgDefault__GetVar01_x                                      = 0x81U,

  MsgDefault__CallFunc01_MCU_ADC1                             = 0xc1U,
  MsgDefault__CallFunc02_MCU_ADC3_VDIODE,
  MsgDefault__CallFunc03_MCU_ADC2_FWD,
  MsgDefault__CallFunc04_MCU_ADC2_REV,

} RtosMsgDefaultCmds_t;


void __disable_irq(void);
void __enable_irq(void);

void usbLog(char* buf);
void usbLogLen(char* buf, int lenusbLog);

void mainCalcFloat2IntFrac(float val, uint8_t fracCnt, int32_t* outInt, uint32_t* outFrac);
float mainCalc_fwdRev_mV(float adc_mv, float vdiode_mv);
float mainCalc_VSWR(float fwd, float rev);
float mainCalc_mV_to_mW(float mV);

uint32_t osKernelSysTick(void);

int main(int argc, char* argv[]);


#endif
