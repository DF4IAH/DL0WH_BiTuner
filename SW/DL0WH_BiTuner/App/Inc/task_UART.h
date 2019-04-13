/*
 * task_UART.h
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */

#ifndef INC_TASK_UART_H_
#define INC_TASK_UART_H_


typedef enum UartTxCmds_ENUM {

  MsgUartTx__InitDo                                           = 0x01U,
  MsgUartTx__InitDone,

//MsgUartTx__SetVar01_x                                       = 0x41U,

//MsgUartTx__GetVar01_y                                       = 0x81U,

//MsgUartTx__CallFunc01_z                                     = 0xc1U,

} UartTxCmds_t;

typedef enum UartRxCmds_ENUM {

  MsgUartRx__InitDo                                           = 0x01U,
  MsgUartRx__InitDone,

//MsgUartRx__SetVar01_x                                       = 0x41U,

//MsgUartRx__GetVar01_y                                       = 0x81U,

//MsgUartRx__CallFunc01_z                                     = 0xc1U,

} UartRxCmds_t;


typedef enum UART_EG_ENUM {

  UART_EG__TX_BUF_EMPTY                                       = (1UL <<  0U),
  UART_EG__TX_ECHO_ON                                         = (1UL <<  1U),

  UART_EG__DMA_TX_RUN                                         = (1UL <<  8U),
  UART_EG__DMA_TX_END                                         = (1UL <<  9U),

  UART_EG__DMA_RX_RUN                                         = (1UL << 16U),
  UART_EG__DMA_RX_END                                         = (1UL << 17U),

} UART_EG_t;


void uartLogLen(const char* str, int len, _Bool doWait);

uint32_t uartRxPullFromQueue(uint8_t* msgAry, uint32_t size, uint32_t waitMs);

void uartTxPutterTask(void const * argument);
void uartTxTaskInit(void);
void uartTxTaskLoop(void);

void uartRxGetterTask(void const * argument);
void uartRxTaskInit(void);
void uartRxTaskLoop(void);

#endif /* INC_TASK_UART_H_ */
