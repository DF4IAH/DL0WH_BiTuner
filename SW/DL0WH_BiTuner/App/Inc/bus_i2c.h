/*
 * bus_i2c.h
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

#ifndef BUS_I2C_H_
#define BUS_I2C_H_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"


/* Module */

#define I2C_TXBUFSIZE                                         32U
#define I2C_RXBUFSIZE                                         32U

#define I2C_DIGPOT_ADDR                                       0x28


void i2cBusAddrScan(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle);
uint32_t i2cSequenceWriteLong(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle, uint8_t addr, uint8_t i2cReg, uint16_t count, const uint8_t i2cWriteAryLong[]);
uint32_t i2cSequenceRead(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle, uint8_t addr, uint8_t i2cRegLen, uint8_t i2cReg[], uint16_t readlen);

void i2cx_Init();
void i2cx_DeInit();

#endif /* BUS_I2C_H_ */
