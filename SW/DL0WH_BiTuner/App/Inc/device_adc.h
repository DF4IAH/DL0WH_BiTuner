/*
 * device_adc.h
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

#ifndef DEVICE_ADC_H_
#define DEVICE_ADC_H_


#define ADC1_DMA_CHANNELS             3
#define ADC2_DMA_CHANNELS             1

#define ADC_V_OFFS_VREF_mV           -58.0f
#define ADC_V_OFFS_BAT_mV            -62.0f
#define ADC_V_OFFS_FWDREV_mV         -60.0f
#define ADC_V_OFFS_VDIODE_mV          52.0f

#define ADC_MUL_BAT                   4.3196f
#define ADC_MUL_TEMP                  1.1680f


typedef enum ADC_ENUM {

  ADC_ADC1_REFINT_VAL               = 0x00,
  ADC_ADC1_VREF_MV,
  ADC_ADC1_BAT_MV,
  ADC_ADC1_TEMP_DEG,

  ADC_ADC2_IN1_FWD_MV,
  ADC_ADC2_IN1_REV_MV,
  ADC_ADC3_IN3_VDIODE_MV,

} ADC_ENUM_t;


/* Event Group ADC */
typedef enum EG_ADC_ENUM {

  EG_ADC1__CONV_AVAIL_REFINT        = 0x0001U,
  EG_ADC1__CONV_AVAIL_BAT           = 0x0002U,
  EG_ADC1__CONV_AVAIL_TEMP          = 0x0004U,

  EG_ADC2__CONV_AVAIL_FWD           = 0x0010U,
  EG_ADC2__CONV_AVAIL_REV,

  EG_ADC3__CONV_AVAIL_VDIODE        = 0x0100U,

} EG_ADC_ENUM_t;


void adcStartConv(ADC_ENUM_t adc);
void adcStopConv(ADC_ENUM_t adc);

#endif /* DEVICE_ADC_H_ */
