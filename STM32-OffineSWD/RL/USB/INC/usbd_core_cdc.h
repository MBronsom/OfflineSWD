/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_core_cdc.h
 *      Purpose: USB Device Core Communication Device Class (CDC) specific 
 *               header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_CORE_CDC_H__
#define __USBD_CORE_CDC_H__


/*--------------------------- Core overridable class specific functions ------*/

extern BOOL USBD_EndPoint0_Setup_CDC_ReqToIF (void);
extern BOOL USBD_EndPoint0_Out_CDC_ReqToIF   (void);


#endif  /* __USBD_CORE_CDC_H__ */
