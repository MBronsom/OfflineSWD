/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_MKxx.h
 *      Purpose: USB Host Freescale Kinetis MKxx Driver header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBH_MK_H__
#define __USBH_MK_H__

#pragma anon_unions


/************************* Structures *****************************************/

typedef __packed struct {               /* Buffer Descriptor (BD)             */
  __packed union {
    U32 CTRL;                           /* Buffer Descriptor Control          */
    __packed union {
      __packed struct {
        U32 ZEROS     :  2;             /* 2 zero bits                        */
        U32 TOK_PID   :  4;             /* Current Token PID                  */
        U32 DATA0_1   :  1;             /* DATA Type                          */
        U32 OWN       :  1;             /* Buffer Owner                       */
        U32 RSVD0     :  8;             /* Reserved                           */
        U32 BC        : 10;             /* Byte Count                         */
        U32 RSVD1     :  6;             /* Reserved                           */
      };
      __packed struct {
        U32 DUMMY0    :  2;             /* Defined in previous struct         */
        U32 BDT_STALL :  1;             /* Issue STALL                        */
        U32 DTS       :  1;             /* Perform Data Toggle Synchronization*/
        U32 NINC      :  1;             /* No Increment for DMA address       */
        U32 KEEP      :  1;             /* Used for ISO Endpoints             */
      };
    };
  };
  U32 ADDR;                             /* Buffer Address                     */
} USBH_MK_BD;

typedef __packed struct {               /* Endpoint Buffer Descriptors (EBD)  */
  __packed union {
    __packed USBH_MK_BD BD[4];
    __packed struct {
      __packed USBH_MK_BD RX;
      __packed USBH_MK_BD RX_ODD;
      __packed USBH_MK_BD TX;
      __packed USBH_MK_BD TX_ODD;
    };
  };
} USBH_MK_EBD;

typedef __packed struct {               /* Device and Endpoint Descriptor     */
  __packed U8  DEV_ADR;                 /* Device address of endpoint         */
  __packed U8  DEV_SPD;                 /* Device speed                       */
  __packed USB_ENDPOINT_DESCRIPTOR EP_DESC; /* Endpoint Descriptors           */
           USBH_URB *ptr_URB;           /* Active URB on this endpoint        */
  __packed U8  cntInterval;             /* Interrupt endpoint rearm count     */
} USBH_MK_DEV_EP;


/************************* Driver Functions ***********************************/

void USBH_Get_Capabilities (USBH_HCI_CAP *cap);
void USBH_Delay_ms         (U32 ms);
BOOL USBH_Pins_Config      (BOOL on);
U32  USBH_Init             (BOOL on);
BOOL USBH_Port_Power       (BOOL on);
BOOL USBH_Port_Reset       (U8 port);
U32  USBH_Get_Connect      (void);
U32  USBH_Get_Speed        (void);
U32  USBH_EP_Add           (          U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD);
BOOL USBH_EP_Config        (U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD);
BOOL USBH_EP_Remove        (U32 hndl);
BOOL USBH_URB_Submit       (U32 hndl, USBH_URB *ptrURB);
BOOL USBH_URB_Cancel       (U32 hndl, USBH_URB *ptrURB);


/************************* Exported Driver Structure **************************/

extern USBH_HCD usbh0_hcd;


/************************* Constant Definitions *******************************/

#pragma no_anon_unions


#endif  /* __USBH_MK_H__ */
