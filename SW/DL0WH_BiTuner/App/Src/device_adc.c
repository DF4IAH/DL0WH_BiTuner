/*
 * device_adc.c
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "device_adc.h"


extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc2;
extern ADC_HandleTypeDef    hadc3;

extern EventGroupHandle_t   adcEventGroupHandle;


static uint16_t             s_adc1_dma_buf[ADC1_CHANNELS]     = { 0U };
static uint16_t             s_adc2_dma_buf[ADC2_CHANNELS]     = { 0U };
static uint16_t             s_adc3_dma_buf[ADC3_CHANNELS]     = { 0U };

static float                s_adc1_refint_val                 = 0.f;
static float                s_adc1_vref_mv                    = 0.f;
static float                s_adc1_bat_mv                     = 0.f;
static float                s_adc1_temp_deg                   = 0.f;
static float                s_adc2_fwdrev_mv                  = 0.f;
static float                s_adc3_vdiode_mv                  = 0.f;


float adcGetVal(ADC_ENUM_t channel)
{
  switch (channel) {
  case ADC_ADC1_REFINT_VAL:
    return s_adc1_refint_val;

  case ADC_ADC1_VREF_MV:
    return s_adc1_vref_mv;

  case ADC_ADC1_BAT_MV:
    return s_adc1_bat_mv;

  case ADC_ADC1_TEMP_DEG:
    return s_adc1_temp_deg;

  case ADC_ADC2_IN1_FWDREV_MV:
    return s_adc2_fwdrev_mv;

  case ADC_ADC3_IN3_VDIODE_MV:
    return s_adc3_vdiode_mv;

  default:
    return 0.f;
  }
}


void adcStartConv(ADC_ENUM_t adc)
{
  switch (adc) {
  case ADC_ADC1_REFINT_VAL:
  case ADC_ADC1_BAT_MV:
  case ADC_ADC1_TEMP_DEG:
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) s_adc1_dma_buf, (sizeof(s_adc1_dma_buf) / sizeof(uint16_t)));
    break;

  case ADC_ADC2_IN1_FWDREV_MV:
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC2__CONV_AVAIL_FWDREV);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*) s_adc2_dma_buf, (sizeof(s_adc2_dma_buf) / sizeof(uint16_t)));
    break;

  case ADC_ADC3_IN3_VDIODE_MV:
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_VDIODE);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*) s_adc3_dma_buf, (sizeof(s_adc3_dma_buf) / sizeof(uint16_t)));
    break;

  default:
    { }
  }
}

void adcStopConv(ADC_ENUM_t adc)
{
  switch (adc) {
  case ADC_ADC1_REFINT_VAL:
  case ADC_ADC1_BAT_MV:
  case ADC_ADC1_TEMP_DEG:
    HAL_ADC_Stop_DMA(&hadc1);
    break;

  case ADC_ADC2_IN1_FWDREV_MV:
    HAL_ADC_Stop_DMA(&hadc2);
    break;

  case ADC_ADC3_IN3_VDIODE_MV:
    HAL_ADC_Stop_DMA(&hadc3);
    break;

  default:
    { }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

  if (hadc == &hadc1) {
    const uint16_t* TS_CAL1_ADDR = (const uint16_t*) 0x1FFF75A8;
    const uint16_t* TS_CAL2_ADDR = (const uint16_t*) 0x1FFF75CA;

    s_adc1_refint_val = s_adc1_dma_buf[0] / 16.0f;
    s_adc1_vref_mv   = (((float)VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR)) / s_adc1_refint_val) - ADC_V_OFFS_VREF_mV;
    s_adc1_bat_mv    = ADC_MUL_BAT * ((s_adc1_dma_buf[2] * s_adc1_vref_mv / 65535.0f) + ADC_V_OFFS_BAT_mV);
    s_adc1_temp_deg   = ((float)(110 - 30) / (*TS_CAL2_ADDR - *TS_CAL1_ADDR)) * ((s_adc1_dma_buf[3] *ADC_MUL_TEMP / 16.f) - *TS_CAL1_ADDR) + 30.f;

    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP, &pxHigherPriorityTaskWoken);

  } else if (hadc == &hadc2) {
    s_adc2_fwdrev_mv = (s_adc2_dma_buf[0] * s_adc1_vref_mv / 65535.0f) + ADC_V_OFFS_FWDREV_mV;
    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC2__CONV_AVAIL_FWDREV, &pxHigherPriorityTaskWoken);

  } else if (hadc == &hadc3) {
    s_adc3_vdiode_mv = (s_adc3_dma_buf[0] * s_adc1_vref_mv / 65535.0f) + ADC_V_OFFS_VDIODE_mV;
    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_VDIODE, &pxHigherPriorityTaskWoken);
  }
}
