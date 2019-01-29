/*
 * task_CAT.h
 *
 *  Created on: 28.01.2019
 *      Author: DF4IAH
 */

#ifndef INC_TASK_CAT_H_
#define INC_TASK_CAT_H_
#if 0

typedef enum CatCmds_ENUM {

  MsgCat__InitDo                                              = 0x01U,
  MsgCat__InitDone,

//MsgCat__SetVar01_x                                          = 0x41U,

//MsgCat__GetVar01_y                                          = 0x81U,

//MsgCat__CallFunc01_z                                        = 0xc1U,

} CatCmds_t;


typedef enum CAT_EG_ENUM {

  CAT_EG__BUF_EMPTY                                           = (1UL <<  0U),
  CAT_EG__ECHO_ON                                             = (1UL <<  1U),

} CAT_EG_t;


void catTaskInit(void);
void catTaskLoop(void);

#endif
#endif /* INC_TASK_CAT_H_ */
