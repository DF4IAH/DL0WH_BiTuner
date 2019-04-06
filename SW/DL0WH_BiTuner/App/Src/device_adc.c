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

#include "main.h"
#include "device_adc.h"


extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc2;
extern ADC_HandleTypeDef    hadc3;

extern EventGroupHandle_t   adcEventGroupHandle;


extern ADC1_RUNNING_VAR_t   g_adc1_running_var;

extern float                g_adc_refint_val;
extern float                g_adc_vref_mv;
extern float                g_adc_bat_mv;
extern float                g_adc_temp_deg;
extern float                g_adc_fwd_mv;
extern float                g_adc_rev_mv;
extern float                g_adc_vdiode_mv;
extern float                g_adc_swr;
extern _Bool                g_adc_select_rev;

static uint16_t             s_adc1_dma_buf[ADC1_DMA_CHANNELS] = { 0U };
static uint16_t             s_adc2_dma_buf[ADC2_DMA_CHANNELS] = { 0U };

static float                s_adc1_val_mean_refint[16]        = { 0.f };
static float                s_adc1_val_mean_vbat[16]          = { 0.f };
static float                s_adc1_val_mean_temp[16]          = { 0.f };
static float                s_adc3_val_mean_diode[16]         = { 0.f };


static float adcCalcMean(float* fa, uint32_t cnt, float newVal)
{
  float sum = 0.0f;

  for (int i = 0; i < cnt; i++) {
    sum += fa[i];
  }

  if (!sum) {
    /* Init array for quick mean value */
    for (int j = 0; j < cnt; j++) {
      fa[j] = newVal;
    }
    sum = newVal * cnt;
  }

  /* Correct sum for old and ne value */
  sum -= fa[0];
  sum += newVal;

  /* Move array */
  for (int k = 1; k < cnt; k++) {
    fa[k - 1] = fa[k];
  }

  /* Enter new value at the end */
  fa[cnt - 1] = newVal;

  return sum / cnt;
}

static void adcMuxSelect(uint8_t mux)
{
  switch (mux) {
  case 1:
    HAL_GPIO_WritePin(GPIO_SWR_SEL_REV_GPIO_Port, GPIO_SWR_SEL_REV_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_SWR_SEL_FWD_GPIO_Port, GPIO_SWR_SEL_FWD_Pin, GPIO_PIN_SET);
    break;

  default:
  case 0:
  case 2:
    HAL_GPIO_WritePin(GPIO_SWR_SEL_FWD_GPIO_Port, GPIO_SWR_SEL_FWD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_SWR_SEL_REV_GPIO_Port, GPIO_SWR_SEL_REV_Pin, GPIO_PIN_SET);
    break;
  }

#if 0
  /* Delay some time = 2661 systicks = 33.3 us */
  for (uint8_t i = 255; i; --i)
    ;
#endif
}

void adcStartConv(ADC_ENUM_t adc)
{
  switch (adc) {
  case ADC_ADC1_REFINT_VAL:
  case ADC_ADC1_VREF_MV:
  {
    ADC_ChannelConfTypeDef sConfig;

    HAL_ADC_Stop_DMA(&hadc1);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING | EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_VREF | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP);

    sConfig.Channel       = ADC_CHANNEL_VREFINT;
    sConfig.Rank          = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime  = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff    = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber  = ADC_OFFSET_NONE;
    sConfig.Offset        = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    g_adc1_running_var = ADC1__RUNNING_VREFINT;
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &(s_adc1_dma_buf[0]), 1);
  }
    break;

  case ADC_ADC1_BAT_MV:
  {
    ADC_ChannelConfTypeDef sConfig;

    HAL_ADC_Stop_DMA(&hadc1);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING | EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_VREF | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP);

    sConfig.Channel       = ADC_CHANNEL_VBAT;
    sConfig.Rank          = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime  = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff    = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber  = ADC_OFFSET_NONE;
    sConfig.Offset        = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    g_adc1_running_var = ADC1__RUNNING_VBAT;
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &(s_adc1_dma_buf[1]), 1);
  }
    break;

  case ADC_ADC1_TEMP_DEG:
  {
    ADC_ChannelConfTypeDef sConfig;

    HAL_ADC_Stop_DMA(&hadc1);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING | EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_VREF | EG_ADC1__CONV_AVAIL_BAT | EG_ADC1__CONV_AVAIL_TEMP);

    sConfig.Channel       = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank          = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime  = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff    = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber  = ADC_OFFSET_NONE;
    sConfig.Offset        = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    g_adc1_running_var = ADC1__RUNNING_TEMP;
    xEventGroupSetBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &(s_adc1_dma_buf[2]), 1);
  }
    break;


  case ADC_ADC2_IN1_FWD_MV:
    adcMuxSelect(1);
    HAL_ADC_Stop_DMA(&hadc2);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING | EG_ADC2__CONV_AVAIL_FWD | EG_ADC2__CONV_AVAIL_REV);

    xEventGroupSetBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING);
    g_adc_select_rev = 0;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*) s_adc2_dma_buf, (sizeof(s_adc2_dma_buf) / sizeof(uint16_t)));
    break;

  case ADC_ADC2_IN1_REV_MV:
    adcMuxSelect(2);
    HAL_ADC_Stop_DMA(&hadc2);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING | EG_ADC2__CONV_AVAIL_REV);

    xEventGroupSetBits(adcEventGroupHandle, EG_ADC2__CONV_RUNNING);
    g_adc_select_rev = 1;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*) s_adc2_dma_buf, (sizeof(s_adc2_dma_buf) / sizeof(uint16_t)));
    break;


  case ADC_ADC3_IN3_VDIODE_MV:
    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOSMP);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC3__CONV_RUNNING | EG_ADC3__CONV_AVAIL_VDIODE);

    xEventGroupSetBits(adcEventGroupHandle, EG_ADC3__CONV_RUNNING);
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
  case ADC_ADC1_VREF_MV:
  case ADC_ADC1_BAT_MV:
  case ADC_ADC1_TEMP_DEG:
    HAL_ADC_Stop_DMA(&hadc1);
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);
    g_adc1_running_var = ADC1__NOT_RUNNING;
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
    HAL_ADC_Stop_DMA(&hadc1);
    xEventGroupClearBitsFromISR(adcEventGroupHandle, EG_ADC1__CONV_RUNNING);

    switch (g_adc1_running_var) {
    case ADC1__RUNNING_VREFINT:
    {
      const float vref_cal_value = (float) *VREFINT_CAL_ADDR;

      const float mean = adcCalcMean(s_adc1_val_mean_refint, sizeof(s_adc1_val_mean_refint) / sizeof(float), s_adc1_dma_buf[0] / 16.0f);
      g_adc_refint_val = mean;
      g_adc_vref_mv    = (((float)VREFINT_CAL_VREF * vref_cal_value) / g_adc_refint_val) + ADC_V_OFFS_VREF_mV;

      xEventGroupSetBitsFromISR(adcEventGroupHandle, (EG_ADC1__CONV_AVAIL_REFINT | EG_ADC1__CONV_AVAIL_VREF), &pxHigherPriorityTaskWoken);
    }
      break;

    case ADC1__RUNNING_VBAT:
    {
      const float mean = adcCalcMean(s_adc1_val_mean_vbat, sizeof(s_adc1_val_mean_vbat) / sizeof(float), s_adc1_dma_buf[1] / 16.0f);
      g_adc_bat_mv     = ((ADC_MUL_BAT * mean * g_adc_vref_mv) / 4095.0f) + ADC_V_OFFS_BAT_mV;

      xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_BAT, &pxHigherPriorityTaskWoken);
    }
      break;

    case ADC1__RUNNING_TEMP:
    {
      const uint16_t* TS_CAL1_ADDR = (const uint16_t*) 0x1FFF75A8;
      const uint16_t* TS_CAL2_ADDR = (const uint16_t*) 0x1FFF75CA;
      const float ts_cal1 = (float) *TS_CAL1_ADDR;
      const float ts_cal2 = (float) *TS_CAL2_ADDR;

      const float mean = adcCalcMean(s_adc1_val_mean_temp, sizeof(s_adc1_val_mean_temp) / sizeof(float), s_adc1_dma_buf[2] / 16.0f);
      g_adc_temp_deg   = ((float)(110 - 30) / (ts_cal2 - ts_cal1)) * ((mean * ADC_MUL_TEMP) - ts_cal1) + 30.f;

      xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_TEMP, &pxHigherPriorityTaskWoken);
    }
      break;

    default: { }
    }

    g_adc1_running_var = ADC1__NOT_RUNNING;

  } else if (hadc == &hadc2) {
    HAL_ADC_Stop_DMA(&hadc2);
    xEventGroupClearBitsFromISR(adcEventGroupHandle, EG_ADC2__CONV_RUNNING);

    const float l_adc_fwdrev_mv = (s_adc2_dma_buf[0] * g_adc_vref_mv / 65535.0f) + ADC_V_OFFS_FWDREV_mV;

    /* Variant */
    if (g_adc_select_rev) {
      g_adc_rev_mv  = l_adc_fwdrev_mv;
      g_adc_swr     = mainCalc_VSWR(g_adc_fwd_mv, l_adc_fwdrev_mv);

      /* Turn MUX to the FWD side to give enough time */
      adcMuxSelect(1);

      xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC2__CONV_AVAIL_REV, &pxHigherPriorityTaskWoken);

    } else {
      g_adc_fwd_mv = l_adc_fwdrev_mv;

      /* Turn MUX to the REV side to give enough time */
      adcMuxSelect(2);

      xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC2__CONV_AVAIL_FWD, &pxHigherPriorityTaskWoken);
    }

  } else if (hadc == &hadc3) {
    HAL_ADC_Stop_IT(&hadc3);
    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOSMP);
    xEventGroupClearBitsFromISR(adcEventGroupHandle, EG_ADC3__CONV_RUNNING);

    const uint32_t val = HAL_ADC_GetValue(&hadc3);
    const float mean = adcCalcMean(s_adc3_val_mean_diode, sizeof(s_adc3_val_mean_diode) / sizeof(float), val / 16.0f);
    g_adc_vdiode_mv = (mean * g_adc_vref_mv / 4095.0f) + ADC_V_OFFS_VDIODE_mV;
    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_VDIODE, &pxHigherPriorityTaskWoken);
  }
}
