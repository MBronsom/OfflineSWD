/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_core_adc.h
 *      Purpose: USB Device Core Audio Class (ADC) specific header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_CORE_ADC_H__
#define __USBD_CORE_ADC_H__


/*--------------------------- Core overridable class specific functions ------*/

extern BOOL USBD_EndPoint0_Setup_ADC_ReqToIF (void);
extern BOOL USBD_EndPoint0_Setup_ADC_ReqToEP (void);
extern BOOL USBD_EndPoint0_Out_ADC_ReqToIF   (void);
extern BOOL USBD_EndPoint0_Out_ADC_ReqToEP   (void);


#endif  /* __USBD_CORE_ADC_H__ */
