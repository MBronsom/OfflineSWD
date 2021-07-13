/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_core_msc.h
 *      Purpose: USB Device Core Mass Storage Device (MSC) specific 
 *               header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_CORE_MSC_H__
#define __USBD_CORE_MSC_H__


/*--------------------------- Core overridable class specific functions ------*/

extern void USBD_ReqClrFeature_MSC           (U32 EPNum);
extern BOOL USBD_EndPoint0_Setup_MSC_ReqToIF (void);
extern BOOL USBD_EndPoint0_Out_MSC_ReqToIF   (void);


#endif  /* __USBD_CORE_MSC_H__ */
