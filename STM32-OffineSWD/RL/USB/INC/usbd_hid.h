/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_hid.h
 *      Purpose: USB Device Human Interface Device Class header file
 *      Rev.:    V4.74
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_HID_H__
#define __USBD_HID_H__


/*--------------------------- Global constants -------------------------------*/

/* USB HID Class API enumerated constants                                     */
enum {
  USBD_HID_REQ_EP_CTRL = 0,             /* Request from control endpoint      */
  USBD_HID_REQ_EP_INT,                  /* Request from interrupt endpoint    */
  USBD_HID_REQ_PERIOD_UPDATE            /* Request from periodic update       */
};


/*--------------------------- Event handling routines ------------------------*/

extern        void USBD_HID_Configure_Event    (void);
extern        void USBD_HID_SOF_Event          (void);

extern        void USBD_HID_EP_INTIN_Event     (U32 event);
extern        void USBD_HID_EP_INTOUT_Event    (U32 event);
extern        void USBD_HID_EP_INT_Event       (U32 event);

extern __task void USBD_RTX_HID_EP_INTIN_Event (void);
extern __task void USBD_RTX_HID_EP_INTOUT_Event(void);
extern __task void USBD_RTX_HID_EP_INT_Event   (void);


#endif  /* __USBD_HID_H__ */
