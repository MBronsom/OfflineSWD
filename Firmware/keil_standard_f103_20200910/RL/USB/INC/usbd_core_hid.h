/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_core_hid.h
 *      Purpose: USB Device Core Human Interface Class (HID) specific 
 *               header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_CORE_HID_H__
#define __USBD_CORE_HID_H__


/*--------------------------- Core overridable class specific functions ------*/

extern BOOL USBD_ReqGetDescriptor_HID        (U8 **pD, U32 *len);
extern BOOL USBD_EndPoint0_Setup_HID_ReqToIF (void);
extern BOOL USBD_EndPoint0_Out_HID_ReqToIF   (void);


#endif  /* __USBD_CORE_HID_H__ */
