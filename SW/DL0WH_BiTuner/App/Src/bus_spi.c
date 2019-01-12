/*
 * bus_spi.c
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include "bus_spi.h"

#include <string.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal_gpio.h"

#include "spi.h"
#include "task_USB.h"



extern EventGroupHandle_t   extiEventGroupHandle;
extern osSemaphoreId        usbToHost_BSemHandle;
extern osSemaphoreId        spi1_BSemHandle;
extern EventGroupHandle_t   spiEventGroupHandle;


static uint8_t              s_spix_UseCtr                     = 0;

/* Buffer used for transmission */
volatile uint8_t            spi1TxBuffer[SPI1_BUFFERSIZE]     = { 0 };

/* Buffer used for reception */
volatile uint8_t            spi1RxBuffer[SPI1_BUFFERSIZE]     = { 0 };


SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;


/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t taskWoken = 0;

  if (&hspi1 == hspi) {
    uint8_t spi1BusInUse = 0U;

    #if 0
    /* Enable clocks of GPIOs */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    #endif

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_SPI_SEL_C_GPIO_Port, GPIO_SPI_SEL_C_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(GPIO_SPI_SEL_C_GPIO_Port, GPIO_SPI_SEL_C_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1_C__BUS_DONE, &taskWoken);

    } else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_SPI_SEL_L_GPIO_Port, GPIO_SPI_SEL_L_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(GPIO_SPI_SEL_L_GPIO_Port, GPIO_SPI_SEL_L_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1_L__BUS_DONE, &taskWoken);

    } else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_SPI_SEL_EXT_GPIO_Port, GPIO_SPI_SEL_EXT_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(GPIO_SPI_SEL_EXT_GPIO_Port, GPIO_SPI_SEL_EXT_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1_EXT__BUS_DONE, &taskWoken);

    } else {
      spi1BusInUse = 1U;
    }

    #if 0
    /* Disable clocks again */
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    #endif

    if (!spi1BusInUse) {
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1__BUS_FREE, &taskWoken);
    }
  }
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t taskWoken = 0;

  if (&hspi1 == hspi) {
    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_SPI_SEL_C_GPIO_Port, GPIO_SPI_SEL_C_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(GPIO_SPI_SEL_C_GPIO_Port, GPIO_SPI_SEL_C_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1_C__BUS_DONE, &taskWoken);
    }

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_SPI_SEL_L_GPIO_Port, GPIO_SPI_SEL_L_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(GPIO_SPI_SEL_L_GPIO_Port, GPIO_SPI_SEL_L_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1_L__BUS_DONE, &taskWoken);
    }

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_SPI_SEL_EXT_GPIO_Port, GPIO_SPI_SEL_EXT_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(GPIO_SPI_SEL_EXT_GPIO_Port, GPIO_SPI_SEL_EXT_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1_EXT__BUS_DONE, &taskWoken);
    }

    xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI1__BUS_ERROR, &taskWoken);
  }
}


const uint16_t spiWait_EGW_MaxWaitTicks = 500;
uint8_t spiProcessSpiReturnWait(void)
{
  EventBits_t eb = xEventGroupWaitBits(spiEventGroupHandle,
      EG_SPI1_C__BUS_DONE | EG_SPI1_L__BUS_DONE | EG_SPI1_EXT__BUS_DONE | EG_SPI1__BUS_FREE | EG_SPI1__BUS_ERROR,
      0,
      pdFALSE, spiWait_EGW_MaxWaitTicks);
  if (eb & (EG_SPI1_C__BUS_DONE | EG_SPI1_L__BUS_DONE | EG_SPI1_EXT__BUS_DONE | EG_SPI1__BUS_FREE)) {
    return HAL_OK;
  }

  if (eb & EG_SPI1__BUS_ERROR) {
    Error_Handler();
  }
  return HAL_ERROR;
}

uint8_t spiProcessSpi1MsgLocked(SPI1_CHIPS_t chip, uint8_t msgLen, uint8_t waitComplete)
{
  HAL_StatusTypeDef status    = HAL_OK;
  uint8_t           errCnt    = 0U;

  GPIO_TypeDef*     GPIOx;
  uint16_t          GPIO_Pin;
  switch (chip) {
  case SPI1_C:
    GPIOx     = GPIO_SPI_SEL_C_GPIO_Port;
    GPIO_Pin  = GPIO_SPI_SEL_C_Pin;

    #if 0
    /* Activate clock for this GPIO */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    #endif
    break;

  case SPI1_L:
    GPIOx     = GPIO_SPI_SEL_L_GPIO_Port;
    GPIO_Pin  = GPIO_SPI_SEL_L_Pin;

    #if 0
    /* Activate clock for this GPIO */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    #endif
    break;

  case SPI1_EXT:
    GPIOx     = GPIO_SPI_SEL_EXT_GPIO_Port;
    GPIO_Pin  = GPIO_SPI_SEL_EXT_Pin;

    #if 0
    /* Activate clock for this GPIO */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    #endif
    break;

  default:
    GPIOx     = 0;
    GPIO_Pin  = 0;
  }

  /* Sanity check */
  if (!GPIOx) {
    return HAL_ERROR;
  }

  /* Wait for free select line */
  do {
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
      break;
    }

    /* Wait some time for free bus */
    osDelay(2UL);
  } while (1);

  /* Activate low active NSS/SEL transaction */
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

  do {
    status = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*) spi1TxBuffer, (uint8_t *) spi1RxBuffer, msgLen);
    if (status == HAL_BUSY)
    {
      osThreadYield();

      if (++errCnt >= 100U) {
        /* Transfer error in transmission process */
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
        Error_Handler();
      }
    }
  } while (status == HAL_BUSY);

  if ((status == HAL_OK) && waitComplete) {
    /* Wait until the data is transfered */
    status = spiProcessSpiReturnWait();
  }

  #if 0
  /* Disable GPIO clocks again */
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  #endif

  return status;
}

uint8_t spiProcessSpi1MsgTemplateLocked(SPI1_CHIPS_t chip, uint16_t templateLen, const uint8_t* templateBuf, uint8_t waitComplete)
{
  /* Wait for SPI1 mutex */
  if (osOK != osSemaphoreWait(spi1_BSemHandle, 1000)) {
    return HAL_BUSY;
  }

  /* Copy from Template */
  memcpy((void*) spi1TxBuffer, (void*) templateBuf, templateLen);

  /* Execute SPI1 communication and leave with locked SPI3 mutex for read purpose */
  return spiProcessSpi1MsgLocked(chip, templateLen, waitComplete);
}

uint8_t spiProcessSpi1MsgTemplate(SPI1_CHIPS_t chip, uint16_t templateLen, const uint8_t* templateBuf)
{
  const uint8_t ret = spiProcessSpi1MsgTemplateLocked(chip, templateLen, templateBuf, 0U);

  /* Release SPI1 mutex */
  osSemaphoreRelease(spi1_BSemHandle);

  return ret;
}


static uint8_t spix_getDevIdx(SPI_HandleTypeDef* dev)
{
  if (&hspi1 == dev) {
    return 0U;

  #if 0
  } else if (&hspi3 == dev) {
    return 1U;
  #endif
  }

  Error_Handler();
  return 0U;
}


void spix_Init(SPI_HandleTypeDef* dev, osSemaphoreId semaphoreHandle)
{
  const uint8_t devIdx = spix_getDevIdx(dev);

  osSemaphoreWait(semaphoreHandle, osWaitForever);

  if (!s_spix_UseCtr++) {
    switch (devIdx) {
    case 0:
    //__HAL_RCC_GPIOx_CLK_ENABLE();                                                                   // SPI1: MCU_SPI1_SCK
      MX_SPI1_Init();
      break;

    #if 0
    case 1:
      __HAL_RCC_GPIOC_CLK_ENABLE();                                                                   // SPI3: MCU_SPI3_SCK, MCU_SPI3_MISO, MCU_SPI3_MOSI, MCU_OUT_AUDIO_DAC_SEL
      __HAL_RCC_GPIOD_CLK_ENABLE();                                                                   // SPI3: MCU_OUT_AUDIO_ADC_SEL
      __HAL_RCC_GPIOE_CLK_ENABLE();                                                                   // SPI3: MCU_OUT_AX_SEL, MCU_OUT_SX_SEL
      MX_SPI3_Init();
      break;
    #endif

    default: { }
    }
  }

  osSemaphoreRelease(semaphoreHandle);
}

void spix_DeInit(SPI_HandleTypeDef* dev, osSemaphoreId semaphoreHandle)
{
  //const uint8_t devIdx = spix_getDevIdx(dev);

  osSemaphoreWait(semaphoreHandle, osWaitForever);

  if (!--s_spix_UseCtr) {
    HAL_SPI_MspDeInit(dev);

  } else if (s_spix_UseCtr == 255U) {
    /* Underflow */
    s_spix_UseCtr = 0U;
  }

  osSemaphoreRelease(semaphoreHandle);
}
