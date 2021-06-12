/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_LPC17xx.c
 *      Purpose: Hardware Layer module for NXP LPC17xx
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include "usbreg.h"
#include <LPC17xx.H>

#define __NO_USB_LIB_C
#include "usb_config.c"

#pragma diag_suppress 1441

#define EP_MSK_CTRL 0x0001      /* Control Endpoint Logical Address Mask      */
#define EP_MSK_BULK 0xC924      /* Bulk Endpoint Logical Address Mask         */
#define EP_MSK_INT  0x2492      /* Interrupt Endpoint Logical Address Mask    */
#define EP_MSK_ISO  0x1248      /* Isochronous Endpoint Logical Address Mask  */

uint32_t CDC_OUT = ((USBD_CDC_ACM_ENABLE == 1) << USBD_CDC_ACM_EP_BULKOUT);
uint8_t  PendingPckts[16];

#ifdef __RTX
        BOOL        USBD_Int_Active = __FALSE;
static  OS_MUT      USBD_HW_Mutex;
#endif

void USBD_Reset      (void);
void USBD_SetAddress (U32 adr, U32 setup);

/*
 *  Usb interrupt enable/disable
 *    Parameters:      ena: enable/disable
 *                       0: disable interrupt
 *                       1: enable interrupt
 */

#ifdef __RTX
void USBD_Intr (int ena) {
  if (ena) {
    NVIC_EnableIRQ(USB_IRQn);           /* Enable USB interrupt               */
  } else {
    NVIC_DisableIRQ(USB_IRQn);          /* Disable USB interrupt              */
  }
}
#endif


/*
 *  Get Endpoint Physical Address
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    Endpoint Physical Address
 */

static U32 EPAdr (U32 EPNum) {
  U32 val;

  val = (EPNum & 0x0F) << 1;
  if (EPNum & 0x80) {
    val += 1;
  }
  return (val);
}


/*
 *  Write Command
 *    Parameters:      cmd:   Command
 *    Return Value:    None
 */

static void WrCmd (U32 cmd) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->USBDevIntClr = CCEMTY_INT;
  LPC_USB->USBCmdCode = cmd;
  while ((LPC_USB->USBDevIntSt & CCEMTY_INT) == 0);

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* Enable USB interrupt               */
#endif
}


/*
 *  Write Command Data
 *    Parameters:      cmd:   Command
 *                     val:   Data
 *    Return Value:    None
 */

static void WrCmdDat (U32 cmd, U32 val) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->USBDevIntClr = CCEMTY_INT;
  LPC_USB->USBCmdCode = cmd;
  while ((LPC_USB->USBDevIntSt & CCEMTY_INT) == 0);
  LPC_USB->USBDevIntClr = CCEMTY_INT;
  LPC_USB->USBCmdCode = val;
  while ((LPC_USB->USBDevIntSt & CCEMTY_INT) == 0);

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* Enable USB interrupt               */
#endif
}


/*
 *  Write Command to Endpoint
 *    Parameters:      cmd:   Command
 *                     val:   Data
 *    Return Value:    None
 */

static void WrCmdEP (U32 EPNum, U32 cmd) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->USBDevIntClr = CCEMTY_INT;
  LPC_USB->USBCmdCode = CMD_SEL_EP(EPAdr(EPNum));
  while ((LPC_USB->USBDevIntSt & CCEMTY_INT) == 0);
  LPC_USB->USBDevIntClr = CCEMTY_INT;
  LPC_USB->USBCmdCode = cmd;
  while ((LPC_USB->USBDevIntSt & CCEMTY_INT) == 0);

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* Enable USB interrupt               */
#endif
}


/*
 *  Read Command Data
 *    Parameters:      cmd:   Command
 *    Return Value:    Data Value
 */

static U32 RdCmdDat (U32 cmd) {
  uint32_t USBCmdData;

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->USBDevIntClr = CCEMTY_INT | CDFULL_INT;
  LPC_USB->USBCmdCode = cmd;
  while ((LPC_USB->USBDevIntSt & CDFULL_INT) == 0);
  USBCmdData = LPC_USB->USBCmdData; 

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* Enable USB interrupt               */
#endif

  return (USBCmdData);
}


/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB Device
 *    Return Value:    None
 */

void USBD_Init (void) {

#ifdef __RTX
  os_mut_init(&USBD_HW_Mutex);
#endif

  LPC_PINCON->PINSEL1 &= ~((3<<26)|(3<<28));  /* P0.29 D+, P0.30 D-           */
  LPC_PINCON->PINSEL1 |=  ((1<<26)|(1<<28));  /* PINSEL1 26.27, 28.29  = 01   */

  LPC_PINCON->PINSEL3 &= ~((3<< 4)|(3<<28));  /* P1.18 GoodLink, P1.30 VBUS   */
  LPC_PINCON->PINSEL3 |=  ((1<< 4)|(2<<28));  /* PINSEL3 4.5 = 01, 28.29 = 10 */

  LPC_PINCON->PINSEL4 &= ~((3<<18)        );  /* P2.9 SoftConnect             */
  LPC_PINCON->PINSEL4 |=  ((1<<18)        );  /* PINSEL4 18.19 = 01           */

  LPC_SC->PCONP       |= (1UL<<31);           /* USB PCLK -> enable USB Peri  */

  LPC_USB->USBClkCtrl = 0x1A;                  /* Dev, PortSel, AHB clock en  */
  while ((LPC_USB->USBClkSt & 0x1A) != 0x1A); 

#ifdef __RTX
  USBD_Intr(1);                                /* enable USB interrupt        */
#else
  NVIC_EnableIRQ(USB_IRQn);                    /* enable USB interrupt        */
#endif

#if 1 /* Partial Manual Reset since Automatic Bus Reset is not working        */
  USBD_Reset();
  USBD_SetAddress(0, 0);
#endif
}


/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect (BOOL con) {
  WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(con ? DEV_CON : 0));
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {
  int i;

  for (i = 0; i < 16; i++) 
    PendingPckts[i] = 0;

  LPC_USB->USBEpInd     = 0;
  LPC_USB->USBMaxPSize  = USBD_MAX_PACKET0;
  LPC_USB->USBEpInd     = 1;
  LPC_USB->USBMaxPSize  = USBD_MAX_PACKET0;
  while ((LPC_USB->USBDevIntSt & EP_RLZED_INT) == 0);

  LPC_USB->USBEpIntClr  = 0xFFFFFFFF;
  LPC_USB->USBEpIntEn   = 0xFFFFFFFF;
  LPC_USB->USBDevIntClr = 0xFFFFFFFF;
  LPC_USB->USBDevIntEn  = DEV_STAT_INT    | EP_SLOW_INT    |
#ifdef __RTX
              ((USBD_RTX_DevTask   != 0) ? FRAME_INT : 0) |   /* SOF event    */
              ((USBD_RTX_DevTask   != 0) ? ERR_INT   : 0) ;   /* Error event  */
#else
              ((USBD_P_SOF_Event   != 0) ? FRAME_INT : 0) |   /* SOF event    */
              ((USBD_P_Error_Event != 0) ? ERR_INT   : 0) ;   /* Error event  */
#endif
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

  if (USBD_DeviceStatus & USB_GETSTATUS_REMOTE_WAKEUP) {
    WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(DEV_CON));
  }
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg (BOOL cfg) {
  /* Not needed */
}


/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *                     setup: Called in setup stage (!=0), else after status stage
 *    Return Value:    None
 */

void USBD_SetAddress (U32 adr, U32 setup) {
  if (setup) return;
  WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /* Don't wait for next   */
  WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /*  Setup Status Phase   */
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure (BOOL cfg) {

  WrCmdDat(CMD_CFG_DEV, DAT_WR_BYTE(cfg ? CONF_DVICE : 0));

  LPC_USB->USBReEp = 0x00000003;
  while ((LPC_USB->USBDevIntSt & EP_RLZED_INT) == 0);
  LPC_USB->USBDevIntClr = EP_RLZED_INT;
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
  U32 num;

  num = EPAdr(pEPD->bEndpointAddress);
  LPC_USB->USBReEp |= (1 << num);
  LPC_USB->USBEpInd = num;
  LPC_USB->USBMaxPSize = pEPD->wMaxPacketSize;
  while ((LPC_USB->USBDevIntSt & EP_RLZED_INT) == 0);
  LPC_USB->USBDevIntClr = EP_RLZED_INT;
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP (U32 dir) {
  /* Not needed */
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Disable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_DA));
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_ST));
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf (U32 EPNum) {
  WrCmdEP(EPNum, CMD_CLR_BUF);
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
  os_mut_wait(&USBD_HW_Mutex, 0xFFFF);
#endif

  if (PendingPckts[EPNum]) PendingPckts[EPNum]--;

  LPC_USB->USBCtrl = ((EPNum & 0x0F) << 2) | CTRL_RD_EN;

  do {
    cnt = LPC_USB->USBRxPLen;
  } while ((cnt & PKT_RDY) == 0);
  cnt &= PKT_LNGTH_MASK;

  for (n = 0; n < (cnt + 3) / 4; n++) {
    *((__packed U32 *)pData) = LPC_USB->USBRxData;
    pData += 4;
  }

  LPC_USB->USBCtrl = 0;

  if (((EP_MSK_ISO >> (EPNum & 0x0F)) & 1) == 0) {   /* Non-Isochronous EP    */
    WrCmdEP(EPNum, CMD_CLR_BUF);
  }

#ifdef __RTX
  os_mut_release(&USBD_HW_Mutex);
#endif

  return (cnt);
}


/*
 *  Write USB Device Endpoint Data
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

U32 USBD_WriteEP (U32 EPNum, U8 *pData, U32 cnt) {
  U32 n;

#ifdef __RTX
  os_mut_wait(&USBD_HW_Mutex, 0xFFFF);
#endif

  LPC_USB->USBCtrl = ((EPNum & 0x0F) << 2) | CTRL_WR_EN;

  LPC_USB->USBTxPLen = cnt;

  for (n = 0; n < (cnt + 3) / 4; n++) {
    LPC_USB->USBTxData = *((__packed U32 *)pData);
    pData += 4;
  }

  LPC_USB->USBCtrl = 0;

  WrCmdEP(EPNum, CMD_VALID_BUF);

#ifdef __RTX
  os_mut_release(&USBD_HW_Mutex);
#endif

  return (cnt);
}


/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

U32 USBD_GetFrame (void) {
  U32 val;

  WrCmd(CMD_RD_FRAME);
  val = RdCmdDat(DAT_RD_FRAME);
  val = val | (RdCmdDat(DAT_RD_FRAME) << 8);

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

void USB_IRQHandler (void) {
  U32 disr, val, n, m;
  U32 episr, episrCur;
  U8  skip_callback;

#ifdef __RTX
  USBD_Int_Active = __TRUE;
#endif

  disr = LPC_USB->USBDevIntSt & LPC_USB->USBDevIntEn; /* Device Int Mask Stat */

  /* Device Status Interrupt (Reset, Connect change, Suspend/Resume)          */
  if (disr & DEV_STAT_INT) {
    LPC_USB->USBDevIntClr = DEV_STAT_INT;
    WrCmd(CMD_GET_DEV_STAT);
    val = RdCmdDat(DAT_GET_DEV_STAT);   /* Device Status                      */
    if (val & DEV_RST) {                /* Reset                              */
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
    }

    if (val & DEV_CON_CH) {             /* Connect change                     */
#ifdef __RTX
      if (USBD_RTX_DevTask) {
        if (val & DEV_CON) {
          isr_evt_set(USBD_EVT_POWER_ON,  USBD_RTX_DevTask);
        } else {
          isr_evt_set(USBD_EVT_POWER_OFF, USBD_RTX_DevTask);
        }
      }
#else
      if (USBD_P_Power_Event) {
        USBD_P_Power_Event(val & DEV_CON);
      }
#endif
    }

    if (val & DEV_SUS_CH) {             /* Suspend/Resume                     */
      if (val & DEV_SUS) {              /* Suspend                            */
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
      } else {                          /* Resume                             */
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
      }
    }

    goto isr_end;
  }

  /* Start of Frame Interrupt */
  if (disr & FRAME_INT) {
    LPC_USB->USBDevIntClr = FRAME_INT;
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_SOF_Event) {
      USBD_P_SOF_Event();
    }
#endif
  }

  /* Error Interrupt */
  if (disr & ERR_INT) {
    LPC_USB->USBDevIntClr = ERR_INT;
    WrCmd(CMD_RD_ERR_STAT);
#ifdef __RTX
    LastError = RdCmdDat(DAT_RD_ERR_STAT);
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_ERROR, USBD_RTX_DevTask);
    }
#else
    val = RdCmdDat(DAT_RD_ERR_STAT);
    if (USBD_P_Error_Event) {
      USBD_P_Error_Event(val);
    }
#endif
  }

  /* Endpoint's Slow Interrupt */
  if (disr & EP_SLOW_INT) {
    episrCur = 0;
    episr    = LPC_USB->USBEpIntSt;
    for (n=0; n<((USBD_EP_NUM+1)<<1); n++) {/* Check All Enabled Endpoints    */
      if (episr == episrCur) break;         /* break if all EP ints handled   */
      if (episr & (1 << n)) {
        episrCur |= (1 << n);
        m = n >> 1;
        LPC_USB->USBEpIntClr = (1 << n);
        while ((LPC_USB->USBDevIntSt & CDFULL_INT) == 0);
        val = LPC_USB->USBCmdData;

        if ((n & 1) == 0) {             /* OUT Endpoint                       */
          if (n == 0) {                 /* Control OUT Endpoint               */
            if (val & EP_SEL_STP) {                           /* SETUP Packet */
#ifdef __RTX
              if (USBD_RTX_EPTask[m]) {
                isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[m]);
              }
              continue;
#else
              if (USBD_P_EP[m]) {
                USBD_P_EP[m](USBD_EVT_SETUP);
                continue;
              }
#endif
            }
          }
#ifdef __RTX
          if (USBD_RTX_EPTask[m]) {                           /* OUT Packet   */
            skip_callback = 0;
            if (CDC_OUT & (1 << m)) {
              if ((val & (3 << 5)) == (3 << 5)) { 
                if (PendingPckts[m] == 2) 
                  skip_callback = 1;
                else 
                  PendingPckts[m] = 2;
              } else if (val & (1 << 5)) {
                if (PendingPckts[m] == 1) 
                  skip_callback = 1;
                else 
                  PendingPckts[m] = 1;
              } else if (val & (1 << 6)) {
                if (PendingPckts[m] == 1) 
                  skip_callback = 1;
                else 
                  PendingPckts[m] = 1;
              } else {
                PendingPckts[m] = 0;
                skip_callback = 1;
              }
            }
            if (!skip_callback) {
              isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[m]);
            }
          }
#else
          if (USBD_P_EP[m]) {
            skip_callback = 0;
            if (CDC_OUT & (1 << m)) {
              if ((val & (3 << 5)) == (3 << 5)) { 
                if (PendingPckts[m] == 2) 
                  skip_callback = 1;
                else 
                  PendingPckts[m] = 2;
              } else if (val & (1 << 5)) {
                if (PendingPckts[m] == 1) 
                  skip_callback = 1;
                else 
                  PendingPckts[m] = 1;
              } else if (val & (1 << 6)) {
                if (PendingPckts[m] == 1) 
                  skip_callback = 1;
                else 
                  PendingPckts[m] = 1;
              } else {
                PendingPckts[m] = 0;
                skip_callback = 1;
              }
            }
            if (!skip_callback) {
              USBD_P_EP[m](USBD_EVT_OUT);
            }
          }
#endif
        } else {                                              /* IN Packet    */
#ifdef __RTX
          if (USBD_RTX_EPTask[m]) {
            isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[m]);
          }
#else
          if (USBD_P_EP[m]) {
            USBD_P_EP[m](USBD_EVT_IN);
          }
#endif
        }
      }
    }
    LPC_USB->USBDevIntClr = EP_SLOW_INT;
  }

isr_end:
#ifdef __RTX
  USBD_Int_Active = __FALSE;
#else
  ;
#endif
}
