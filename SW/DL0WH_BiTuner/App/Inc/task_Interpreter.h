/*
 * task_Interpreter.h
 *
 *  Created on: 22.01.2019
 *      Author: DF4IAH
 */

#ifndef INC_TASK_INTERPRETER_H_
#define INC_TASK_INTERPRETER_H_

#include "main.h"


typedef enum InterpreterCmds_ENUM {

  MsgInterpreter__InitDo                                      = 0x01U,
  MsgInterpreter__InitDone,

  MsgInterpreter__SetVar01_L                                  = 0x41U,
  MsgInterpreter__SetVar02_C,
  MsgInterpreter__SetVar03_CL,
  MsgInterpreter__SetVar04_LC,

//MsgInterpreter__GetVar01_y                                  = 0x81U,

  MsgInterpreter__CallFunc01_Restart                          = 0xc1U,
  MsgInterpreter__CallFunc02_PrintLC,

} InterpreterCmds_t;


void interpreterPrintHelp(void);
void interpreterShowCursor(void);
void interpreterClearScreen(void);

void interpreterInterpreterTaskInit(void);
void interpreterInterpreterTaskLoop(void);

#endif /* INC_TASK_INTERPRETER_H_ */
