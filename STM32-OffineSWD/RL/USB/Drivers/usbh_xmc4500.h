/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_xmc4500.h
 *      Purpose: Host Infineon XMC4500 Driver header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBH_XMC4500_H__
#define __USBH_XMC4500_H__

#pragma anon_unions


/************************* Structures *****************************************/

typedef __packed struct {               /* Channel typedef (CH)               */
  __packed union {
    U32 HCCHAR;                         /* Channel Characteristics            */
    __packed struct {
      U32 MPS                  : 11;    /* Endpoint Maximum Packet Size       */
      U32 EPNUM                :  4;    /* Endpoint Number                    */
      U32 EPDIR                :  1;    /* Endpoint Direction                 */
      U32 Reserved0            :  1;    /* Reserved                           */
      U32 LSPDDEV              :  1;    /* Low-speed Device                   */
      U32 EPTYP                :  2;    /* Endpoint Type                      */
      U32 MC_EC                :  2;    /* Periodic Endpoint Multicount/Errcnt*/
      U32 DEVADDR              :  7;    /* Device Address                     */
      U32 ODDFRM               :  1;    /* Periodic Transaction Odd Frame     */
      U32 CHDIS                :  1;    /* Channel Disable                    */
      U32 CHENA                :  1;    /* Channel Enable                     */
    };
  };
  U32 ReservedU32_0;                    /* Reserved                           */
  __packed union {
    U32 HCINT;                          /* Channel Interrupt                  */
    __packed struct {
      U32 XFERCOMPL            :  1;    /* Transfer Completed                 */
      U32 CHHLTD               :  1;    /* Channel Halted                     */
      U32 AHBERR               :  1;    /* AHB Error                          */
      U32 STALL                :  1;    /* STALL Response                     */
      U32 NAK                  :  1;    /* NAK Response                       */
      U32 ACK                  :  1;    /* ACK Response                       */
      U32 NYET                 :  1;    /* NYET Response                      */
      U32 XACTERR              :  1;    /* Transaction Error                  */
      U32 BBLERR               :  1;    /* Babble Error                       */
      U32 FRMOVRUN             :  1;    /* Frame Overrun                      */
      U32 DATATGLERR           :  1;    /* Data Toggle Error                  */
      U32 BNAINTR              :  1;    /* Buffer Not Available Error         */
      U32 XCS_XACT_ERR         :  1;    /* Excessive Transaction Error        */
      U32 DESC_LST_ROLLINTR    :  1;    /* Descriptor Rollover Interrupt      */
    };
  };
  __packed union {
    U32 HCINTMSK;                       /* Channel Interrupt Mask             */
    __packed struct {
      U32 XFERCOMPLMSK         :  1;    /* Transfer Completed Mask            */
      U32 CHHLTDMSK            :  1;    /* Channel Halted Mask                */
      U32 AHBERRMSK            :  1;    /* AHB Error Mask                     */
      U32 STALLMSK             :  1;    /* STALL Response Mask                */
      U32 NAKMSK               :  1;    /* NAK Response Mask                  */
      U32 ACKMSK               :  1;    /* ACK Response Mask                  */
      U32 NYETMSK              :  1;    /* NYET Response Mask                 */
      U32 XACTERRMSK           :  1;    /* Transaction Error Mask             */
      U32 BBLERRMSK            :  1;    /* Babble Error Mask                  */
      U32 FRMOVRUNMSK          :  1;    /* Frame Overrun Mask                 */
      U32 DATATGLERRMSK        :  1;    /* Data Toggle Error Mask             */
      U32 BNAINTRMSK           :  1;    /* Buffer Not Available Error Mask    */
      U32 Reserved1            :  1;    /* Reserved                           */
      U32 DESC_LST_ROLLINTRMSK :  1;    /* Descriptor Rollover Interrupt Mask */
    };
  };
  __packed union {
    U32 HCTSIZ;                         /* Channel Transfer Size              */
    __packed struct {
      U32 XFERSIZE             : 19;    /* Transfer Size                      */
      U32 PKTCNT               : 10;    /* Packet Count                       */
      U32 PID                  :  2;    /* Data PID                           */
      U32 Reserved2            :  1;    /* Reserved                           */
    };
  };
  __packed union {
    U32 HCDMA;                          /* Host Channel DMA Address Register  */
    __packed struct {
      U32 DMAADDR;                      /* DMA Address                        */
    };
  };
  U32 ReservedU32;                      /* Reserved                           */
  __packed union {
    U32 HCDMAB;                         /* Host Channel DMA Buffer Address Reg*/
    __packed struct {
      U32 BUFFER_ADDRESS;               /* Buffer Address                     */
    };
  };
} USBH_XMC4500_CH;


/************************* Driver Functions ***********************************/

void USBH_XMC4500_Get_Capabilities (USBH_HCI_CAP *cap);
void USBH_XMC4500_Delay_ms         (U32 ms);
BOOL USBH_XMC4500_Pins_Config      (BOOL on);
BOOL USBH_XMC4500_Init             (BOOL on);
BOOL USBH_XMC4500_Port_Power       (BOOL on);
BOOL USBH_XMC4500_Port_Reset       (U8 port);
U32  USBH_XMC4500_Get_Connect      (void);
U32  USBH_XMC4500_Get_Speed        (void);
U32  USBH_XMC4500_EP_Add           (          U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD);
BOOL USBH_XMC4500_EP_Config        (U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD);
BOOL USBH_XMC4500_EP_Remove        (U32 hndl);
BOOL USBH_XMC4500_URB_Submit       (U32 hndl, USBH_URB *ptrURB);
BOOL USBH_XMC4500_URB_Cancel       (U32 hndl, USBH_URB *ptrURB);


/************************* Exported Driver Structure **************************/

extern USBH_HCD usbh0_hcd;


/************************* Constant Definitions *******************************/


#pragma no_anon_unions

#endif  /* __USBH_XMC4500_H__ */
