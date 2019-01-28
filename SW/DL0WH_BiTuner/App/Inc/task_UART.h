/*
 * task_UART.h
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */

#ifndef INC_TASK_UART_H_
#define INC_TASK_UART_H_


 /* Bit-mask for the uartEventGroup */
typedef enum Uart_EGW_BM {
} Gpscom_EGW_BM_t;



void uartTaskInit(void);
void uartTaskLoop(void);

#endif /* INC_TASK_UART_H_ */
