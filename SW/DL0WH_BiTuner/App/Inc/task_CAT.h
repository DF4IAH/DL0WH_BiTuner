/*
 * task_CAT.h
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */

#ifndef INC_TASK_CAT_H_
#define INC_TASK_CAT_H_

typedef enum CatCmds_ENUM {

  MsgCatTx__InitDo                                            = 0x01U,
  MsgCatTx__InitDone,
  MsgCatRx__InitDo,
  MsgCatRx__InitDone,

//MsgCat__SetVar01_x                                          = 0x41U,

//MsgCat__GetVar01_y                                          = 0x81U,

//MsgCat__CallFunc01_z                                        = 0xc1U,

} CatCmds_t;


typedef enum CAT_EG_ENUM {

  CAT_EG__BUF_EMPTY                                           = (1UL <<  0U),
  CAT_EG__ECHO_ON                                             = (1UL <<  1U),

  CAT_EG__DMA_TX_RUN                                          = (1UL <<  8U),
  CAT_EG__DMA_TX_END                                          = (1UL <<  9U),

  CAT_EG__DMA_RX_RUN                                          = (1UL << 16U),
  CAT_EG__DMA_RX_END                                          = (1UL << 17U),

} CAT_EG_t;



void HAL_CAT_TxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_CAT_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_CAT_ErrorCallback(UART_HandleTypeDef *UartHandle);

uint32_t catRxPullFromQueue(uint8_t* msgAry, uint32_t waitMs);

void catTxPutterTask(void const * argument);
void catTxTaskInit(void);
void catTxTaskLoop(void);

void catRxGetterTask(void const * argument);
void catRxTaskInit(void);
void catRxTaskLoop(void);

#endif /* INC_TASK_CAT_H_ */
