/*
 * device_adc.h
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

#ifndef DEVICE_ADC_H_
#define DEVICE_ADC_H_


#define ADC1_DMA_CHANNELS             5

#define ADC_V_OFFS_VREF_mV           -58.0f
#define ADC_V_OFFS_BAT_mV            -64.0f
#define ADC_V_OFFS_FWDREV_mV         -60.0f
#define ADC_V_OFFS_VDIODE_mV          52.0f

#define ADC_MUL_BAT                   4.3196f
#define ADC_MUL_TEMP                  1.1630f


typedef enum ADC_ENUM {

  ADC_REFINT_VAL                      = 0x00,
  ADC_VREF_MV,
  ADC_BAT_MV,
  ADC_TEMP_DEG,

  ADC_FWD_MV,
  ADC_REV_MV,
  ADC_VDIODE_MV,

} ADC_ENUM_t;


/* Event Group ADC */
typedef enum EG_ADC_ENUM {

  EG_ADC__CONV_AVAIL_REFINT           = 0x0001U,
  EG_ADC__CONV_AVAIL_BAT              = 0x0002U,
  EG_ADC__CONV_AVAIL_TEMP             = 0x0004U,

  EG_ADC__CONV_AVAIL_FWD              = 0x0010U,
  EG_ADC__CONV_AVAIL_REV,

  EG_ADC__CONV_AVAIL_VDIODE           = 0x0100U,

} EG_ADC_ENUM_t;


void adcStartConv(ADC_ENUM_t adc);
void adcStopConv(ADC_ENUM_t adc);

#endif /* DEVICE_ADC_H_ */
