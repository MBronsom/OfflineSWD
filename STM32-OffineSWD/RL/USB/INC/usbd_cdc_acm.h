/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_cdc_acm.h
 *      Purpose: USB Device Communication Device Class Abstract Control Model 
 *               header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_CDC_ACM_H__
#define __USBD_CDC_ACM_H__


/*--------------------------- Event handling routines ------------------------*/

extern           void USBD_CDC_ACM_Reset_Event          (void);

extern           void USBD_CDC_ACM_SOF_Event            (void);

extern           void USBD_CDC_ACM_EP_INTIN_Event       (U32 event);
extern           void USBD_CDC_ACM_EP_BULKIN_Event      (U32 event);
extern           void USBD_CDC_ACM_EP_BULKOUT_Event     (U32 event);
extern           void USBD_CDC_ACM_EP_BULK_Event        (U32 event);

extern    __task void USBD_RTX_CDC_ACM_EP_INTIN_Event   (void);
extern    __task void USBD_RTX_CDC_ACM_EP_BULKIN_Event  (void);
extern    __task void USBD_RTX_CDC_ACM_EP_BULKOUT_Event (void);
extern    __task void USBD_RTX_CDC_ACM_EP_BULK_Event    (void);


#endif  /* __USBD_CDC_ACM_H__ */
