/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_msc.h
 *      Purpose: USB Device Mass Storage Device Class header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_MSC_H__
#define __USBD_MSC_H__


/*--------------------------- Global variables -------------------------------*/

/* USB Device Mass Storage Device Class Global Variables */
extern       BOOL USBD_MSC_MediaReady;
extern       BOOL USBD_MSC_ReadOnly;
extern       U32  USBD_MSC_MemorySize;
extern       U32  USBD_MSC_BlockSize;
extern       U32  USBD_MSC_BlockGroup;
extern       U32  USBD_MSC_BlockCount;
extern       U8  *USBD_MSC_BlockBuf;


/*--------------------------- Event handling routines ------------------------*/

extern        void USBD_MSC_EP_BULKIN_Event      (U32 event);
extern        void USBD_MSC_EP_BULKOUT_Event     (U32 event);
extern        void USBD_MSC_EP_BULK_Event        (U32 event);

extern __task void USBD_RTX_MSC_EP_BULKIN_Event  (void);
extern __task void USBD_RTX_MSC_EP_BULKOUT_Event (void);
extern __task void USBD_RTX_MSC_EP_BULK_Event    (void);


#endif  /* __USBD_MSC_H__ */
