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


extern float                g_adc_refint_val;
extern float                g_adc_vref_mv;
extern float                g_adc_bat_mv;
extern float                g_adc_temp_deg;
extern float                g_adc_fwd_mv;
extern float                g_adc_rev_mv;
extern float                g_adc_vdiode_mv;
extern _Bool                g_adc_select_rev;

static uint16_t             s_adc1_dma_buf[ADC1_DMA_CHANNELS] = { 0U };
static uint16_t             s_adc2_dma_buf[ADC2_DMA_CHANNELS] = { 0U };


void adcStartConv(ADC_ENUM_t adc)
{
  switch (adc) {
  case ADC_ADC1_REFINT_VAL:
  case ADC_ADC1_BAT_MV:
  case ADC_ADC1_TEMP_DEG:
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);

    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_VREF | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) s_adc1_dma_buf, (sizeof(s_adc1_dma_buf) / sizeof(uint16_t)));
    break;


  case ADC_ADC2_IN1_FWD_MV:
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING);

    xEventGroupClearBits(adcEventGroupHandle, EG_ADC2__CONV_AVAIL_FWD);
    g_adc_select_rev = 0;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*) s_adc2_dma_buf, (sizeof(s_adc2_dma_buf) / sizeof(uint16_t)));
    break;

  case ADC_ADC2_IN1_REV_MV:
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING);

    xEventGroupClearBits(adcEventGroupHandle, EG_ADC2__CONV_AVAIL_REV);
    g_adc_select_rev = 1;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*) s_adc2_dma_buf, (sizeof(s_adc2_dma_buf) / sizeof(uint16_t)));
    break;


  case ADC_ADC3_IN3_VDIODE_MV:
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC3__CONV_RUNNING);

    xEventGroupClearBits(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_VDIODE);
    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOSMP);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOSMP);
    HAL_ADC_Start_IT(&hadc3);
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
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);
    break;

  case ADC_ADC2_IN1_FWD_MV:
  case ADC_ADC2_IN1_REV_MV:
    HAL_ADC_Stop_DMA(&hadc2);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING);
    break;

  case ADC_ADC3_IN3_VDIODE_MV:
    HAL_ADC_Stop_IT(&hadc3);
    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOSMP);
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC3__CONV_RUNNING);
    break;

  default:
    { }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* ISR context */

  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

  if (hadc == &hadc1) {
    const uint16_t* TS_CAL1_ADDR = (const uint16_t*) 0x1FFF75A8;
    const uint16_t* TS_CAL2_ADDR = (const uint16_t*) 0x1FFF75CA;

    HAL_ADC_Stop_DMA(&hadc1);

    g_adc_refint_val = s_adc1_dma_buf[0] / 16.0f;
    g_adc_vref_mv    = (((float)VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR)) / g_adc_refint_val) + ADC_V_OFFS_VREF_mV;
    g_adc_bat_mv     = ((ADC_MUL_BAT * s_adc1_dma_buf[2] * g_adc_vref_mv) / 65535.0f)       + ADC_V_OFFS_BAT_mV;
    g_adc_temp_deg   = ((float)(110 - 30) / (*TS_CAL2_ADDR - *TS_CAL1_ADDR)) * ((s_adc1_dma_buf[3] *ADC_MUL_TEMP / 16.f) - *TS_CAL1_ADDR) + 30.f;

    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);
    xEventGroupSetBitsFromISR(adcEventGroupHandle,
        EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_VREF | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP,
        &pxHigherPriorityTaskWoken);

  } else if (hadc == &hadc2) {
    HAL_ADC_Stop_DMA(&hadc2);

    const float l_adc_fwdrev_mv = (s_adc2_dma_buf[0] * g_adc_vref_mv / 65535.0f)            + ADC_V_OFFS_FWDREV_mV;

    xEventGroupClearBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING);

    if (g_adc_select_rev) {
      g_adc_rev_mv = l_adc_fwdrev_mv;
      xEventGroupSetBitsFromISR(adcEventGroupHandle,
          EG_ADC2__CONV_AVAIL_FWD,
          &pxHigherPriorityTaskWoken);

    } else {
      g_adc_fwd_mv = l_adc_fwdrev_mv;
      xEventGroupSetBitsFromISR(adcEventGroupHandle,
          EG_ADC2__CONV_AVAIL_REV,
          &pxHigherPriorityTaskWoken);
    }


  } else if (hadc == &hadc3) {
    HAL_ADC_Stop_IT(&hadc3);
    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOSMP);

    const uint32_t val = HAL_ADC_GetValue(&hadc3);
    g_adc_vdiode_mv = (val * g_adc_vref_mv / 65535.0f)                                      + ADC_V_OFFS_VDIODE_mV;
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC3__CONV_RUNNING);
    xEventGroupSetBitsFromISR(adcEventGroupHandle,
        EG_ADC3__CONV_AVAIL_VDIODE,
        &pxHigherPriorityTaskWoken);
  }
}
