/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_stm32f20x.h
 *      Purpose: Host STM32F20x Driver header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBH_STM32_H__
#define __USBH_STM32_H__

#pragma anon_unions


/************************* Structures *****************************************/

typedef __packed struct {               /* Channel typedef (CH)               */
  __packed union {
    U32 HCCHAR;                         /* Channel Characteristics            */
    __packed struct {
      U32 MPSIZ     : 11;               /* Endpoint Maximum Packet Size       */
      U32 EPNUM     :  4;               /* Endpoint Number                    */
      U32 EPDIR     :  1;               /* Endpoint Direction                 */
      U32 Reserved0 :  1;               /* Reserved                           */
      U32 LSDEV     :  1;               /* Endpoint Low-speed                 */
      U32 EPTYP     :  2;               /* Endpoint Type                      */
      U32 MCNT      :  2;               /* Periodic Endpoint Multicount       */
      U32 DAD       :  7;               /* Device Address                     */
      U32 ODDFRM    :  1;               /* Periodic Transaction Odd Frame     */
      U32 CHDIS     :  1;               /* Channel Disable                    */
      U32 CHENA     :  1;               /* Channel Enable                     */
    };
  };
  U32 ReservedU32_0;                    /* Reserved                           */
  __packed union {
    U32 HCINT;                          /* Channel Interrupt                  */
    __packed struct {
      U32 XFCR      :  1;               /* Transfer Completed                 */
      U32 CHH       :  1;               /* Channel Halted                     */
      U32 Reserved1 :  1;               /* Reserved                           */
      U32 STALL     :  1;               /* STALL Response Received Interrupt  */
      U32 NAK       :  1;               /* NAK Response Received Interrupt    */
      U32 ACK       :  1;               /* ACK Response Rece/Transmit Int     */
      U32 Reserved2 :  1;               /* Reserved                           */
      U32 TXERR     :  1;               /* Transaction Error                  */
      U32 BBERR     :  1;               /* Babble Error                       */
      U32 FRMOR     :  1;               /* Frame Overrun                      */
      U32 DTERR     :  1;               /* Data Toggle Error                  */
    };
  };
  __packed union {
    U32 HCINTMSK;                       /* Channel Interrupt Mask             */
    __packed struct {
      U32 XFCRM     :  1;               /* Transfer Completed Mask            */
      U32 CHHM      :  1;               /* Channel Halted Mask                */
      U32 Reserved3 :  1;               /* Reserved                           */
      U32 STALLM    :  1;               /* STALL Response Rece Interrupt Mask */
      U32 NAKM      :  1;               /* NAK Response Rece Interrupt Mask   */
      U32 ACKM      :  1;               /* ACK Response Rece/Transmit Int Mask*/
      U32 Reserved4 :  1;               /* Reserved                           */
      U32 TXERRM    :  1;               /* Transaction Error Mask             */
      U32 BBERRM    :  1;               /* Babble Error Mask                  */
      U32 FRMORM    :  1;               /* Frame Overrun Mask                 */
      U32 DTERRM    :  1;               /* Data Toggle Error Mask             */
    };
  };
  __packed union {
    U32 HCTSIZ;                         /* Channel Transfer Size              */
    __packed struct {
      U32 XFRSIZ    : 19;               /* Transfer Size                      */
      U32 PKTCNT    : 10;               /* Packet Count                       */
      U32 DPID      :  2;               /* Data PID                           */
      U32 Reserved5 :  1;               /* Reserved                           */
    };
  };
  U32 ReservedU32_1;                    /* Reserved                           */
  U32 ReservedU32_2;                    /* Reserved                           */
  U32 ReservedU32_3;                    /* Reserved                           */
} USBH_STM32_CH;


/************************* Driver Functions ***********************************/

void USBH_STM32_Get_Capabilities (USBH_HCI_CAP *cap);
void USBH_STM32_Delay_ms         (U32 ms);
BOOL USBH_STM32_Pins_Config      (U8 ctrl, BOOL on);
BOOL USBH_STM32_Init             (U8 ctrl, BOOL on);
BOOL USBH_STM32_Port_Power       (U8 ctrl, BOOL on);
BOOL USBH_STM32_Port_Reset       (U8 ctrl, U8 port);
U32  USBH_STM32_Get_Connect      (U8 ctrl);
U32  USBH_STM32_Get_Speed        (U8 ctrl);
U32  USBH_STM32_EP_Add           (U8 ctrl,           U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD);
BOOL USBH_STM32_EP_Config        (U8 ctrl, U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD);
BOOL USBH_STM32_EP_Remove        (U8 ctrl, U32 hndl);
BOOL USBH_STM32_URB_Submit       (U8 ctrl, U32 hndl, USBH_URB *ptrURB);
BOOL USBH_STM32_URB_Cancel       (U8 ctrl, U32 hndl, USBH_URB *ptrURB);
void OTG_xS_IRQHandler           (U8 ctrl);

/************************* Exported Driver Structure **************************/

extern USBH_HCD usbh0_hcd;
extern USBH_HCD usbh1_hcd;


/************************* Constant Definitions *******************************/

/* OTG_FS_GAHBCFG Register bits                                               */
#define USBH_STM32_GAHBCFG_PTXFELVL     ((1         ) <<  8)
#define USBH_STM32_GAHBCFG_TXFELVL      ((1         ) <<  7)
#define USBH_STM32_GAHBCFG_GINTMSK      ((1         ) <<  0)

/* OTG_FS_GUSBCFG Register bits                                               */
#define USBH_STM32_GUSBCFG_CTXPKT       ((1UL       ) << 31)
#define USBH_STM32_GUSBCFG_FDMOD        ((1         ) << 30)
#define USBH_STM32_GUSBCFG_FHMOD        ((1         ) << 29)
#define USBH_STM32_GUSBCFG_TRDT(x)      ((x &   0x0F) << 10)
#define USBH_STM32_GUSBCFG_HNPCAP       ((1         ) <<  9)
#define USBH_STM32_GUSBCFG_SRPCAP       ((1         ) <<  8)
#define USBH_STM32_GUSBCFG_PHYSEL       ((1         ) <<  6)
#define USBH_STM32_GUSBCFG_TOCAL(x)     ((x &   0x07) <<  0)

/* OTG_FS_GRSTCTL Register bits                                               */
#define USBH_STM32_GRSTCTL_AHBIDL       ((1UL       ) << 31)
#define USBH_STM32_GRSTCTL_TXFNUM(x)    ((x &   0x1F) <<  6)
#define USBH_STM32_GRSTCTL_TXFFLSH      ((1         ) <<  5)
#define USBH_STM32_GRSTCTL_RXFFLSH      ((1         ) <<  4)
#define USBH_STM32_GRSTCTL_FCRST        ((1         ) <<  2)
#define USBH_STM32_GRSTCTL_HSRST        ((1         ) <<  1)
#define USBH_STM32_GRSTCTL_CSRST        ((1         ) <<  0)

/* OTG_FS_GINTSTS Register bits                                               */
#define USBH_STM32_GINTSTS_WKUINT       ((1UL       ) << 31)
#define USBH_STM32_GINTSTS_SRQINT       ((1         ) << 30)
#define USBH_STM32_GINTSTS_DISCINT      ((1         ) << 29)
#define USBH_STM32_GUSBCFG_PTCI         ((1         ) << 24)
#define USBH_STM32_GUSBCFG_PCCI         ((1         ) << 23)
#define USBH_STM32_GUSBCFG_ULPIEVBUSI   ((1         ) << 21)
#define USBH_STM32_GUSBCFG_ULPIEVBUSD   ((1         ) << 20)
#define USBH_STM32_GINTSTS_CIDSCHG      ((1         ) << 28)
#define USBH_STM32_GINTSTS_PTXFE        ((1         ) << 26)
#define USBH_STM32_GINTSTS_HCINT        ((1         ) << 25)
#define USBH_STM32_GINTSTS_HPRTINT      ((1         ) << 24)
#define USBH_STM32_GINTSTS_IPXFR        ((1         ) << 21)
#define USBH_STM32_GINTSTS_INCOMPISOOUT ((1         ) << 21)
#define USBH_STM32_GINTSTS_IISOIXFR     ((1         ) << 20)
#define USBH_STM32_GINTSTS_OEPINT       ((1         ) << 19)
#define USBH_STM32_GINTSTS_IEPINT       ((1         ) << 18)
#define USBH_STM32_GINTSTS_EOPF         ((1         ) << 15)
#define USBH_STM32_GINTSTS_ISOODRP      ((1         ) << 14)
#define USBH_STM32_GINTSTS_ENUMDNE      ((1         ) << 13)
#define USBH_STM32_GINTSTS_USBRST       ((1         ) << 12)
#define USBH_STM32_GINTSTS_USBSUSP      ((1         ) << 11)
#define USBH_STM32_GINTSTS_ESUSP        ((1         ) << 10)
#define USBH_STM32_GINTSTS_GOUTNAKEFF   ((1         ) <<  7)
#define USBH_STM32_GINTSTS_GINAKEFF     ((1         ) <<  6)
#define USBH_STM32_GINTSTS_NPTXFE       ((1         ) <<  5)
#define USBH_STM32_GINTSTS_RXFLVL       ((1         ) <<  4)
#define USBH_STM32_GINTSTS_SOF          ((1         ) <<  3)
#define USBH_STM32_GINTSTS_OTGINT       ((1         ) <<  2)
#define USBH_STM32_GINTSTS_MMIS         ((1         ) <<  1)
#define USBH_STM32_GINTSTS_CMOD         ((1         ) <<  0)

/* OTG_FS_GINTMSK Register bits                                               */
#define USBH_STM32_GINTMSK_WUIM         ((1UL       ) << 31)
#define USBH_STM32_GINTMSK_SRQIM        ((1         ) << 30)
#define USBH_STM32_GINTMSK_DISCINT      ((1         ) << 29)
#define USBH_STM32_GINTMSK_CIDSCHGM     ((1         ) << 28)
#define USBH_STM32_GINTMSK_PTXFEM       ((1         ) << 26)
#define USBH_STM32_GINTMSK_HCIM         ((1         ) << 25)
#define USBH_STM32_GINTMSK_PRTIM        ((1         ) << 24)
#define USBH_STM32_GINTMSK_IPXFRM       ((1         ) << 21)
#define USBH_STM32_GINTMSK_IISOOXFRM    ((1         ) << 21)
#define USBH_STM32_GINTMSK_IISOIXFRM    ((1         ) << 20)
#define USBH_STM32_GINTMSK_OEPINT       ((1         ) << 19)
#define USBH_STM32_GINTMSK_IEPINT       ((1         ) << 18)
#define USBH_STM32_GINTMSK_EPMISM       ((1         ) << 15)
#define USBH_STM32_GINTMSK_ISOODRPM     ((1         ) << 14)
#define USBH_STM32_GINTMSK_ENUMDNEM     ((1         ) << 13)
#define USBH_STM32_GINTMSK_USBRST       ((1         ) << 12)
#define USBH_STM32_GINTMSK_USBSUSPM     ((1         ) << 11)
#define USBH_STM32_GINTMSK_ESUSPM       ((1         ) << 10)
#define USBH_STM32_GINTMSK_GONAKEFFM    ((1         ) <<  7)
#define USBH_STM32_GINTMSK_GINAKEFFM    ((1         ) <<  6)
#define USBH_STM32_GINTMSK_NPTXFEM      ((1         ) <<  5)
#define USBH_STM32_GINTMSK_RXFLVLM      ((1         ) <<  4)
#define USBH_STM32_GINTMSK_SOFM         ((1         ) <<  3)
#define USBH_STM32_GINTMSK_OTGINT       ((1         ) <<  2)
#define USBH_STM32_GINTMSK_MMISM        ((1         ) <<  1)

/* OTG_FS_GCCFG Register bits                                                 */
#define USBH_STM32_GCCFG_NOVBUSSENS     ((1         ) << 21)
#define USBH_STM32_GCCFG_SOFOUTEN       ((1         ) << 20)
#define USBH_STM32_GCCFG_VBUSBSEN       ((1         ) << 19)
#define USBH_STM32_GCCFG_VBUSASEN       ((1         ) << 18)
#define USBH_STM32_GCCFG_PWRDWN         ((1         ) << 16)

/* OTG_FS_HCFG Register bits                                                  */
#define USBH_STM32_HCFG_FSLS(x)         ((x &      1) <<  2)
#define USBH_STM32_HCFG_FSLSPCS(x)      ((x &      3) <<  0)

/* OTG_FS_HFIR Register bits                                                  */
#define USBH_STM32_HFIR_FRIVL(x)        ((x & 0xFFFF) <<  0)

/* OTG_FS_HPTXSTS Register bits                                               */
#define USBH_STM32_HPTXSTS_PTXFSAVL(x)  ((x & 0xFFFF) <<  0)

/* OTG_FS_HAINTMSK Register bits                                              */
#define USBH_STM32_HAINTMSK_HAINTM(x)   ((x & 0xFFFF) <<  0)

/* OTG_FS_HPRT Register bits                                                  */
#define USBH_STM32_HPRT_PTCTL(x)        ((x &   0x0F) << 13)
#define USBH_STM32_HPRT_PPWR            ((1         ) << 12)
#define USBH_STM32_HPRT_PRST            ((1         ) <<  8)
#define USBH_STM32_HPRT_PSUSP           ((1         ) <<  7)
#define USBH_STM32_HPRT_PRES            ((1         ) <<  6)
#define USBH_STM32_HPRT_POCCHNG         ((1         ) <<  5)
#define USBH_STM32_HPRT_POCA            ((1         ) <<  4)
#define USBH_STM32_HPRT_PENCHNG         ((1         ) <<  3)
#define USBH_STM32_HPRT_PENA            ((1         ) <<  2)
#define USBH_STM32_HPRT_PCDET           ((1         ) <<  1)
#define USBH_STM32_HPRT_PCSTS           ((1         ) <<  0)

/* OTG_FS_HCCHARx Register bits                                               */
#define USBH_STM32_HCCHAR_CHENA         ((1UL       ) << 31)
#define USBH_STM32_HCCHAR_CHDIS         ((1         ) << 30)
#define USBH_STM32_HCCHAR_ODDFRM        ((1         ) << 29)
#define USBH_STM32_HCCHAR_DAD           ((0x7F      ) << 22)
#define USBH_STM32_HCCHAR_MCNT          ((3         ) << 20)
#define USBH_STM32_HCCHAR_MCNT0         ((0         ) << 20)
#define USBH_STM32_HCCHAR_MCNT1         ((1         ) << 20)
#define USBH_STM32_HCCHAR_MCNT2         ((2         ) << 20)
#define USBH_STM32_HCCHAR_MCNT3         ((3         ) << 20)
#define USBH_STM32_HCCHAR_EPTYP         ((3         ) << 18)
#define USBH_STM32_HCCHAR_LSDEV         ((1         ) << 17)
#define USBH_STM32_HCCHAR_EPDIR         ((1         ) << 15)
#define USBH_STM32_HCCHAR_EPNUM         ((0x0F      ) << 11)
#define USBH_STM32_HCCHAR_MPSIZ         ((0x7FF     ) <<  0)

/* OTG_FS_HCINTx Register bits                                                */
#define USBH_STM32_HCINT_DTERR          ((1         ) << 10)
#define USBH_STM32_HCINT_FRMOR          ((1         ) <<  9)
#define USBH_STM32_HCINT_BBERR          ((1         ) <<  8)
#define USBH_STM32_HCINT_TXERR          ((1         ) <<  7)
#define USBH_STM32_HCINT_NYET           ((1         ) <<  6)
#define USBH_STM32_HCINT_ACK            ((1         ) <<  5)
#define USBH_STM32_HCINT_NAK            ((1         ) <<  4)
#define USBH_STM32_HCINT_STALL          ((1         ) <<  3)
#define USBH_STM32_HCINT_CHH            ((1         ) <<  1)
#define USBH_STM32_HCINT_XFRC           ((1         ) <<  0)
#define USBH_STM32_HCINT_ERR            (USBH_STM32_HCINT_DTERR | \
                                         USBH_STM32_HCINT_FRMOR | \
                                         USBH_STM32_HCINT_BBERR | \
                                         USBH_STM32_HCINT_TXERR   )

/* OTG_FS_HCINTMSKx Register bits                                             */
#define USBH_STM32_HCINTMSK_DTERRM      ((1         ) << 10)
#define USBH_STM32_HCINTMSK_FRMORM      ((1         ) <<  9)
#define USBH_STM32_HCINTMSK_BBERRM      ((1         ) <<  8)
#define USBH_STM32_HCINTMSK_TXERRM      ((1         ) <<  7)
#define USBH_STM32_HCINTMSK_NYET        ((1         ) <<  6)
#define USBH_STM32_HCINTMSK_ACKM        ((1         ) <<  5)
#define USBH_STM32_HCINTMSK_NAKM        ((1         ) <<  4)
#define USBH_STM32_HCINTMSK_STALLM      ((1         ) <<  3)
#define USBH_STM32_HCINTMSK_CHHM        ((1         ) <<  1)
#define USBH_STM32_HCINTMSK_XFRCM       ((1         ) <<  0)

/* OTG_FS_HCTSIZx Register bits                                               */
#define USBH_STM32_HCTSIZ_RESERVED      ((1UL       ) << 31)
#define USBH_STM32_HCTSIZ_DOPING        ((1UL       ) << 31)
#define USBH_STM32_HCTSIZ_DPID          ((3         ) << 29)
#define USBH_STM32_HCTSIZ_DPID_DATA0    ((0         ) << 29)
#define USBH_STM32_HCTSIZ_DPID_DATA2    ((1         ) << 29)
#define USBH_STM32_HCTSIZ_DPID_DATA1    ((2         ) << 29)
#define USBH_STM32_HCTSIZ_DPID_MDATA    ((3         ) << 29)


#pragma no_anon_unions

#endif  /* __USBH_STM32_H__ */
