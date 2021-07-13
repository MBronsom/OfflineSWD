/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_core_cls.h
 *      Purpose: USB Device Core Custom Class (CLS) specific header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_CORE_CLS_H__
#define __USBD_CORE_CLS_H__


/*--------------------------- Core overridable class specific functions ------*/

extern BOOL USBD_EndPoint0_Setup_CLS_ReqToDEV (void);
extern BOOL USBD_EndPoint0_Setup_CLS_ReqToIF  (void);
extern BOOL USBD_EndPoint0_Setup_CLS_ReqToEP  (void);
extern BOOL USBD_EndPoint0_Out_CLS_ReqToDEV   (void);
extern BOOL USBD_EndPoint0_Out_CLS_ReqToIF    (void);
extern BOOL USBD_EndPoint0_Out_CLS_ReqToEP    (void);


#endif  /* __USBD_CORE_CLS_H__ */
