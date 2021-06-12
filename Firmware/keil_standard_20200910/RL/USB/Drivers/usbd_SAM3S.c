/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_SAM3S.c
 *      Purpose: Hardware Layer module for Atmel AT91SAM3S
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <SAM3S.H>

#define __NO_USB_LIB_C
#include "usb_config.c"


const U8  DualBankEP = 0xF6;            /* Dual Bank Endpoint Bit Mask        */

const U32 RX_DATA_BK[2] = {
  UDP_CSR_RX_DATA_BK0,
  UDP_CSR_RX_DATA_BK1
};


U8  RxDataBank[USBD_EP_NUM+1];
U8  TxDataBank[USBD_EP_NUM+1];


/*
 *  Set / Clear functions to modify UDP_CSR register
 */

static void USBD_SetCSR(U32 EPNum, U32 flags) {
  U32 timeout = 100;

  EPNum &= 0x0F;
  UDP->UDP_CSR[EPNum] |= (flags);
  while ((UDP->UDP_CSR[EPNum] & (flags)) != (flags)) {
    if (!timeout--) break;
  }
}

static void USBD_ClrCSR(U32 EPNum, U32 flags) {
  U32 timeout = 100;

  EPNum &= 0x0F;
  UDP->UDP_CSR[EPNum] &= ~(flags);
  while (UDP->UDP_CSR[EPNum] & (flags)) {
    if (!timeout--) break;
  }
}


/*
 *  Retrieve maximum EP size Function
 *   Called during EndPoint configuration
 *    Return Value:    maximum size for given EP
 */

static int USB_GetSizeEP (U32 EPNum) {
  switch (EPNum & 0x0F) {
    case 0:
    case 1:
    case 2:
    case 3:
      return (64);                      /* Maximum size is 64 bytes           */
    case 4:
    case 5:
      return (512);                     /* Maximum size is 512 bytes          */
    case 6:
    case 7:
      return (64);                      /* Maximum size is 64 bytes           */
    default:
      return (0);                       /* Non existant endpoint              */
  }
}


/*
 *  USB Device Interrupt enable
 *   Called by USBD_Init to enable the USB Interrupt
 *    Return Value:    None
 */

#ifdef __RTX
void __svc(1) USBD_IntrEna (void);
void __SVC_1               (void) {
#else
void          USBD_IntrEna (void) {
#endif
  NVIC_EnableIRQ(UDP_IRQn);             /* Enable USB interrupt               */
}


/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB Device
 *    Return Value:    None
 */

void USBD_Init (void) {
  /* Enables the 48MHz USB Clock UDPCK and System Peripheral USB Clock        */
                                        /* PLLB is configured for 96MHz       */
  PMC->PMC_PCER1 = PMC_PCER1_PID34;     /* Enable master clock for UPD        */
  PMC->PMC_USB   = ((0x01 << 8) |       /* USBDIV = 1  (96MHz / (USBDIV+1))   */
                    (0x01 << 0)  );     /* USB Clock Input is PLLB            */

  UDP->UDP_TXVC &= ~UDP_TXVC_PUON;      /* Disconnect pullup on DP            */

  PMC->PMC_SCER  =  PMC_SCER_UDP;       /* Enable 48MHz clock for UDP         */
  while (!(PMC->PMC_SCSR & PMC_SCER_UDP));

  USBD_IntrEna();                       /* Enable USB interrupt               */
}


/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect (BOOL con) {
  if (con) {
    UDP->UDP_TXVC &= ~UDP_TXVC_TXVDIS;            /* Enable Transciever       */
    UDP->UDP_TXVC |=  UDP_TXVC_PUON;              /* Connect pullup on DP     */
  } else {
    UDP->UDP_TXVC &= ~UDP_TXVC_PUON;              /* Disconnect pullup on DP  */
    UDP->UDP_TXVC |=  UDP_TXVC_TXVDIS;            /* Disable Transciever      */
  }
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {
  U32 ep;

  /* Global USB Device Reset                                                  */
  UDP->UDP_GLB_STAT   = 0;                        /* Reset global status to 0 */
  UDP->UDP_FADDR      = UDP_FADDR_FEN;            /* Set address to 0         */
  UDP->UDP_ICR        = 0xFFFFFFFF;               /* Clear all pending ints   */

  /* Reset & Disable USB Device Endpoints                                     */
  for (ep = 0; ep <= USBD_EP_NUM; ep++) {
    UDP->UDP_CSR[ep]  = 0;
    RxDataBank[ep]    = 0;
    TxDataBank[ep]    = 0;
  }
  UDP->UDP_RST_EP     = 0xFFFFFFFF;
  UDP->UDP_RST_EP     = 0;

  /* Setup USB Interrupts                                                     */
#ifdef __RTX
  UDP->UDP_IER =  ((USBD_RTX_DevTask     != 0) ? UDP_IER_RXSUSP   : 0) |
                  ((USBD_RTX_DevTask     != 0) ? UDP_IER_RXRSM    : 0) |
                  ((USBD_RTX_DevTask     != 0) ? UDP_IER_SOFINT   : 0) |
                  ((USBD_RTX_DevTask     != 0) ? UDP_IER_WAKEUP   : 0) |
#else
  UDP->UDP_IER =  ((USBD_P_Suspend_Event != 0) ? UDP_IER_RXSUSP   : 0) |
                  ((USBD_P_Resume_Event  != 0) ? UDP_IER_RXRSM    : 0) |
                  ((USBD_P_SOF_Event     != 0) ? UDP_IER_SOFINT   : 0) |
                  ((USBD_P_WakeUp_Event  != 0) ? UDP_IER_WAKEUP   : 0) |
#endif
                  ((1 << (USBD_EP_NUM+1))-1);

  /* Setup Control Endpoint 0                                                 */
  USBD_SetCSR(0, UDP_CSR_EPEDS | UDP_CSR_EPTYPE_CTRL);
}


/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg (BOOL cfg) {
  if (cfg) {
    UDP->UDP_GLB_STAT |=  UDP_GLB_STAT_RMWUPE;
  } else {
    UDP->UDP_GLB_STAT &= ~UDP_GLB_STAT_RMWUPE;
  }
}


/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *                     setup: Called in setup stage (!=0), else after status stage
 *    Return Value:    None
 */

void USBD_SetAddress (U32 adr, U32 setup) {
  if (setup) return;
  UDP->UDP_FADDR = UDP_FADDR_FEN | (UDP_FADDR_FADD(adr));  /* set the address */
  if (adr) {                                      /* If address is non-zero   */
    UDP->UDP_GLB_STAT |=  UDP_GLB_STAT_FADDEN;    /* Device enters adr state  */
  } else {
    UDP->UDP_GLB_STAT &= ~UDP_GLB_STAT_FADDEN;    /* Device enters def state  */
  }
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure (BOOL cfg) {
  if (cfg) {                                      /* If config is non-zero    */
    UDP->UDP_GLB_STAT |=  UDP_GLB_STAT_CONFG;     /* Device enters cfg state  */
  } else {
    UDP->UDP_GLB_STAT &= ~UDP_GLB_STAT_CONFG;     /* Device clears cfg state  */
  }
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
  U32 ep, dir, type, csr;

  ep   = pEPD->bEndpointAddress & 0x0F;
  type = pEPD->bmAttributes     & USB_ENDPOINT_TYPE_MASK;
  dir  = pEPD->bEndpointAddress >> 7;
  csr  = ((type | (dir << 2)) << 8);

  /* Check if MaxPacketSize fits for EndPoint                                 */
  if (pEPD->wMaxPacketSize <= USB_GetSizeEP(ep)) {
    USBD_SetCSR(ep, csr);               /* Configure the EP                   */
  }
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP (U32 dir) {
  if (dir) {
    USBD_SetCSR(0, UDP_CSR_DIR);
  } else {
    USBD_ClrCSR(0, UDP_CSR_DIR);
  }
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (U32 EPNum) {
  EPNum &= 0x0F;                        /* Get endpoint number                */
  UDP->UDP_IER = (1 << EPNum);          /* Enable EP interrupts               */
  USBD_SetCSR(EPNum, UDP_CSR_EPEDS);
}


/*
 *  Disable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (U32 EPNum) {
  EPNum &= 0x0F;                        /* Get endpoint number                */
  USBD_ClrCSR(EPNum, UDP_CSR_EPEDS);
  UDP->UDP_IDR = (1 << EPNum);          /* Disable EP interrupts              */
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (U32 EPNum) {
  EPNum &= 0x0F;
  USBD_ClrCSR(EPNum, (UDP_CSR_TXCOMP            | UDP_CSR_RXSETUP      |
                      UDP_CSR_RX_DATA_BK0       | UDP_CSR_RX_DATA_BK1  |
                                                  UDP_CSR_FORCESTALL   |
                      UDP_CSR_STALLSENT                                 ));

  if (UDP->UDP_CSR[EPNum] & UDP_CSR_TXPKTRDY) {
    USBD_ClrCSR(EPNum, UDP_CSR_TXPKTRDY);
    if (DualBankEP & (1 << EPNum)) {
      USBD_SetCSR(EPNum, UDP_CSR_TXPKTRDY);
      USBD_ClrCSR(EPNum, UDP_CSR_TXPKTRDY);
    }
  }

  UDP->UDP_RST_EP  |=  (1 << EPNum);
  UDP->UDP_RST_EP  &= ~(1 << EPNum);
  TxDataBank[EPNum] =   0;
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP (U32 EPNum) {
  USBD_SetCSR(EPNum & 0x0F, UDP_CSR_FORCESTALL);
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP (U32 EPNum) {
  EPNum &= 0x0F;
  USBD_ClrCSR(EPNum, UDP_CSR_FORCESTALL);
  UDP->UDP_RST_EP  |=  (1 << EPNum);
  UDP->UDP_RST_EP  &= ~(1 << EPNum);
}


/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

U32 USBD_ReadEP (U32 EPNum, U8 *pData) {
  U32 cnt, n;
#ifdef __RTX
  U8 *pDat = pData;
#endif

  EPNum &= 0x0F;
  cnt = (UDP->UDP_CSR[EPNum] >> 16) & 0x07FF;
  for (n = 0; n < cnt; n++) {           /* Read data                          */
    *pData++ = (U8)UDP->UDP_FDR[EPNum];
  }
  USBD_ClrCSR(EPNum, RX_DATA_BK[RxDataBank[EPNum]]);
  if (DualBankEP & (1 << EPNum)) {
    RxDataBank[EPNum] ^= 1;
  }
#ifdef __RTX
  /* Leave RXSETUP bit and interrupt disabled if next packet is IN and we  
     need to send data                                                        */
  if (EPNum || !((UDP->UDP_CSR[0] & UDP_CSR_RXSETUP) && (pDat[0] & 0x80))) {
    USBD_ClrCSR(EPNum, UDP_CSR_RXSETUP);
    UDP->UDP_IER = (1 << EPNum);        /* Reenable EP int                    */
  }
#else
  USBD_ClrCSR(EPNum, UDP_CSR_RXSETUP);
#endif

  return (cnt);
}


/*
 *  Write USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

U32 USBD_WriteEP (U32 EPNum, U8 *pData, U32 cnt) {
  U32 n;

  EPNum &= 0x0F;
  if (UDP->UDP_CSR[EPNum] & UDP_CSR_FORCESTALL) {     /* If stalled don't send*/
    return (cnt);
  }

  if (TxDataBank[EPNum]) {              /* If hardware not available to send  */
    return (0);
  }

  if (UDP->UDP_CSR[EPNum] & UDP_CSR_TXPKTRDY) {
    if ((DualBankEP & (1 << EPNum)) && (TxDataBank[EPNum] == 0)) {
      TxDataBank[EPNum] = 1;
    } else {
      return (0);
    }
  }
  USBD_ClrCSR(EPNum, UDP_CSR_RXSETUP);
  for (n = 0; n < cnt; n++) {           /* Write data                         */
    UDP->UDP_FDR[EPNum] = *pData++;
  }
#ifdef __RTX
  UDP->UDP_IER = (1 << EPNum);          /* Reenable EP int                    */
#endif
  USBD_SetCSR(EPNum, UDP_CSR_TXPKTRDY);

  return (cnt);
}


/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

U32 USBD_GetFrame (void) {
  U32 val;

  while ((UDP->UDP_FRM_NUM & (UDP_FRM_NUM_FRM_OK | UDP_FRM_NUM_FRM_ERR)) == 0);
  if (UDP->UDP_FRM_NUM & UDP_FRM_NUM_FRM_OK) {
    val = UDP->UDP_FRM_NUM & UDP_FRM_NUM_FRM_NUM_Msk;
  } else {
    val = 0xFFFFFFFF;
  }

  return (val);
}


#ifdef __RTX
U32 LastError;                          /* Last Error                         */

/*
 *  Get USB Device Last Error Code
 *    Parameters:      None
 *    Return Value:    Error Code
 */

U32 USBD_GetError (void) {
  return (LastError);
}
#endif


/*
 *  USB Device Interrupt Service Routine
 */

void UDP_IRQHandler (void) {
  U32 isr, csr, bkm, n;

  isr = UDP->UDP_ISR & UDP->UDP_IMR;

  /* End of Bus Reset Interrupt                                               */
  if (isr & UDP_ISR_ENDBUSRES) {
    USBD_Reset();
    usbd_reset_core();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_Reset_Event) {
      USBD_P_Reset_Event();
    }
#endif
    UDP->UDP_ICR = UDP_ISR_ENDBUSRES;
  }

  /* USB Suspend Interrupt                                                    */
  if (isr & UDP_ISR_RXSUSP) {
    USBD_Suspend();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_Suspend_Event) {
      USBD_P_Suspend_Event();
    }
#endif
    UDP->UDP_ICR = UDP_ISR_RXSUSP;
  }

  /* USB Resume Interrupt                                                     */
  if (isr & UDP_ISR_RXRSM) {
    USBD_Resume();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_RESUME,  USBD_RTX_DevTask);
    }
#else
    if (USBD_P_Resume_Event) {
      USBD_P_Resume_Event();
    }
#endif
    UDP->UDP_ICR = UDP_ISR_RXRSM;
  }

  /* Start of Frame Interrupt                                                 */
  if (isr & UDP_ISR_SOFINT) {
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_SOF_Event) {
      USBD_P_SOF_Event();
    }
#endif
    UDP->UDP_ICR = UDP_ISR_SOFINT;
  }

  /* USB Wakeup Interrupt                                                     */
  if (isr & UDP_ISR_WAKEUP) {
    USBD_WakeUp();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_WAKEUP,  USBD_RTX_DevTask);
    }
#else
    if (USBD_P_WakeUp_Event) {
      USBD_P_WakeUp_Event();
    }
#endif
    UDP->UDP_ICR = UDP_ISR_WAKEUP;
  }

  /* Endpoint Interrupts                                                      */
  for (n = 0; n <= USBD_EP_NUM; n++) {
    if (isr & (1 << n)) {
      csr = UDP->UDP_CSR[n];

      /* Data Packet Sent Interrupt                                           */
      if (csr & UDP_CSR_TXCOMP) {
        USBD_ClrCSR(n, UDP_CSR_TXCOMP);
#ifdef __RTX
        if (USBD_RTX_EPTask[n]) {         /* IN Packet                        */
          isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[n]);
        }
#else
        if (USBD_P_EP[n]) {
          USBD_P_EP[n](USBD_EVT_IN);
        }
#endif
      }

      /* Data Packet Received Interrupt                                       */
      bkm = RX_DATA_BK[RxDataBank[n]];
      if (csr & bkm) {
#ifdef __RTX
        if (USBD_RTX_EPTask[n]) {       /* OUT Packet                         */
          isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[n]);
          UDP->UDP_IDR = (1 << n);      /* Disable EP int until read          */
        }
#else
        if (USBD_P_EP[n]) {
          USBD_P_EP[n](USBD_EVT_OUT);
        }
#endif
      }

      /* STALL Packet Sent Interrupt                                          */
      if (csr & UDP_CSR_STALLSENT) {
        if ((csr & UDP_CSR_EPTYPE_Msk) == UDP_CSR_EPTYPE_CTRL) {
#ifdef __RTX
          if (USBD_RTX_EPTask[n]) {
            isr_evt_set(USBD_EVT_IN_STALL, USBD_RTX_EPTask[n]);
          }
#else
          if (USBD_P_EP[n]) {
            USBD_P_EP[n](USBD_EVT_IN_STALL);
          }
#endif
        }
        USBD_ClrCSR(n, UDP_CSR_STALLSENT);
      }

      /* Setup Packet Received Interrupt                                      */
      if (csr & UDP_CSR_RXSETUP) {
#ifdef __RTX
        if (USBD_RTX_EPTask[n]) {       /* SETUP Packet                       */
          isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[n]);
          UDP->UDP_IDR = (1 << n);      /* Disable EP int until read          */
        }
#else
        if (USBD_P_EP[n]) {
          USBD_P_EP[n](USBD_EVT_SETUP);
        }
#endif
      }
    }
  }
}
