/*
 * bus_spi.h
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

#ifndef BUS_SPI_H_
#define BUS_SPI_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"
#include "main.h"

//#include "LoRaWAN.h"


#define SPI1_BUFFERSIZE 256

#define SPI_WR_FLAG   (1 << 7)
#define SPI_RD_FLAG   (0 << 7)


typedef enum EG_SPI1_ENUM {

  EG_SPI1_C__BUS_DONE                                         = 0x0001U,
  EG_SPI1_L__BUS_DONE                                         = 0x0002U,
  EG_SPI1_EXT__BUS_DONE                                       = 0x0004U,

  EG_SPI1__BUS_FREE                                           = 0x0010U,
  EG_SPI1__BUS_ERROR                                          = 0x0080U,

} EG_SPI1_t;


typedef enum SPI1_CHIPS_ENUM {

  SPI1_C                                                      = 1,
  SPI1_L,
  SPI1_EXT,

} SPI1_CHIPS_t;


extern void _Error_Handler(char *, int);


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

uint8_t spiProcessSpiReturnWait(void);
uint8_t spiProcessSpi1MsgLocked(SPI1_CHIPS_t chip, uint8_t msgLen, uint8_t waitComplete);
uint8_t spiProcessSpi1MsgTemplateLocked(SPI1_CHIPS_t chip, uint16_t templateLen, const uint8_t* templateBuf, uint8_t waitComplete);
uint8_t spiProcessSpi1MsgTemplate(SPI1_CHIPS_t chip, uint16_t templateLen, const uint8_t* templateBuf);

void spix_Init(SPI_HandleTypeDef* dev, osSemaphoreId semaphoreHandle);
void spix_DeInit(SPI_HandleTypeDef* dev, osSemaphoreId semaphoreHandle);

#endif /* BUS_SPI_H_ */
