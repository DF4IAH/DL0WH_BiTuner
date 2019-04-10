/*
 * usb.h
 *
 *  Created on: 06.01.2019
 *      Author: DF4IAH
 */

#ifndef TASK_USB_H_
#define TASK_USB_H_

#include <stddef.h>
#include <sys/_stdint.h>


typedef enum usbMsgUsbCmds_ENUM {

  MsgUsb__InitDo                                              = 0x01U,
  MsgUsb__InitDone,

  MsgUsb__DeInitDo                                            = 0x05U,

//MsgUsb__SetVar01_x                                          = 0x41U,

//MsgUsb__GetVar01_y                                          = 0x81U,

//MsgUsb__CallFunc01_x                                        = 0xc1U,

} usbMsgUsbCmds_t;


typedef enum USB_TO_HOST_EG_ENUM {

  USB_TO_HOST_EG__BUF_EMPTY                                   = (1UL <<  0U),
  USB_TO_HOST_EG__ECHO_ON                                     = (1UL <<  1U),

} USB_TO_HOST_EG_t;


/* Called from the application */
void usbLogLen(const char* str, int len, _Bool doWait);

uint32_t usbPullFromOutQueue(uint8_t* msgAry, uint32_t waitMs);


/* Called from the USB CDC backend */
void usbFromHostFromIRQ(const uint8_t* buf, uint32_t len);

void usbStartUsbToHostTask(void const * argument);
void usbUsbToHostTaskInit(void);
void usbUsbToHostTaskLoop(void);

void usbStartUsbFromHostTask(void const * argument);
void usbUsbFromHostTaskInit(void);
void usbUsbFromHostTaskLoop(void);

#endif /* TASK_USB_H_ */
