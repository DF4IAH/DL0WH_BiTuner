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

//MsgInterpreter__SetVar01_x                                  = 0x41U,

//MsgInterpreter__GetVar01_y                                  = 0x81U,

//MsgInterpreter__CallFunc01_z                                = 0xc1U,

} InterpreterCmds_t;


void interpreterConsolePush(const char* buf, int bufLen);
void interpreterPrintHelp(void);
void interpreterShowCursor(void);
void interpreterClearScreen(void);

void interpreterGetterTask(void const * argument);
void interpreterTaskInit(void);
void interpreterTaskLoop(void);

#endif /* INC_TASK_INTERPRETER_H_ */
