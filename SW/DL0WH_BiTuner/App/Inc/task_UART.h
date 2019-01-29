/*
 * task_UART.h
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */

#ifndef INC_TASK_UART_H_
#define INC_TASK_UART_H_


typedef enum UartCmds_ENUM {

  MsgUartTx__InitDo                                           = 0x01U,
  MsgUartTx__InitDone,
  MsgUartRx__InitDo,
  MsgUartRx__InitDone,

//MsgUart__SetVar01_x                                         = 0x41U,

//MsgUart__GetVar01_y                                         = 0x81U,

//MsgUart__CallFunc01_z                                       = 0xc1U,

} UartCmds_t;


typedef enum UART_EG_ENUM {

  UART_EG__TX_BUF_EMPTY                                       = (1UL <<  0U),
  UART_EG__TX_ECHO_ON                                         = (1UL <<  1U),

  UART_EG__DMA_TX_RUN                                         = (1UL <<  8U),
  UART_EG__DMA_TX_END                                         = (1UL <<  9U),

  UART_EG__DMA_RX_RUN                                         = (1UL << 16U),
  UART_EG__DMA_RX_END                                         = (1UL << 17U),

} UART_EG_t;


void uartLogLen(const char* str, int len);
void uartLog(const char* str);

void uartTxPutterTask(void const * argument);
void uartTxTaskInit(void);
void uartTxTaskLoop(void);

void uartRxGetterTask(void const * argument);
void uartRxTaskInit(void);
void uartRxTaskLoop(void);

#endif /* INC_TASK_UART_H_ */
