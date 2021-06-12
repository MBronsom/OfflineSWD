/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_core.h
 *      Purpose: USB Device Core header file
 *      Rev.:    V4.75
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_CORE_H__
#define __USBD_CORE_H__


/*--------------------------- Data structures --------------------------------*/

/* USB Device Core Endpoint Data Structure */
typedef struct _USBD_EP_DATA {
  U8 *pData;
  U16 Count;
} USBD_EP_DATA;


/*--------------------------- Global variables -------------------------------*/

/* USB Device Core Global Variables */
extern U16         USBD_DeviceStatus;
extern U8          USBD_DeviceAddress;
extern U8          USBD_Configuration;
extern U32         USBD_EndPointMask;
extern U32         USBD_EndPointHalt;
extern U32         USBD_EndPointNoHaltClr;
extern U8          USBD_NumInterfaces;
extern U8          USBD_HighSpeed;
extern U8          USBD_ZLP;

extern USBD_EP_DATA     USBD_EP0Data;
extern USB_SETUP_PACKET USBD_SetupPacket;

extern OS_TID      USBD_RTX_DevTask;
extern OS_TID      USBD_RTX_EPTask[];
extern OS_TID      USBD_RTX_CoreTask;


/*--------------------------- Functions exported to class specific files -----*/

extern void        USBD_SetupStage     (void);
extern void        USBD_DataInStage    (void);
extern void        USBD_DataOutStage   (void);
extern void        USBD_StatusInStage  (void);
extern void        USBD_StatusOutStage (void);


/*--------------------------- Event handling routines ------------------------*/

extern        void usbd_class_init    (void);

extern        void USBD_EndPoint0     (U32 event);

extern __task void USBD_RTX_EndPoint0 (void);


#endif  /* __USBD_CORE_H__ */
