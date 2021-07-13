/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_LPC134x.c
 *      Purpose: Hardware Layer module for NXP LPC134x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include "usbreg.h"
#include <LPC13xx.H>

#define __NO_USB_LIB_C
#include "usb_config.c"

#pragma diag_suppress 1441

//*** <<< Use Configuration Wizard in Context Menu >>> ***

/*
// <h> LPC13xx Specific Configuration
//   <e0>USB Device FIQ Select
//     <i>Setting all bits to ‘1’ at the same time is not allowed.
//     <o1.0>FRAME: Route FRAME interrupt to HighPriority interrupt line
//     <o1.1>BULKOUT: Route Bulk OUT EP3 IRQ to HighPriority interrupt line
//     <o1.2>BULKIN: Route Bukk IN EP3 IRQ to HighPriority interrupt line
//   </e>
// </h>
*/
#define USB_LPC13XX_FIQ        0
#define USB_LPC13XX_FIQ_VAL    0

#define EP_MSK_CTRL 0x0001      /* Control Endpoint Logical Address Mask      */
#define EP_MSK_BULK 0x000E      /* Bulk Endpoint Logical Address Mask         */
#define EP_MSK_INT  0x000E      /* Interrupt Endpoint Logical Address Mask    */
#define EP_MSK_ISO  0x0010      /* Isochronous Endpoint Logical Address Mask  */

#ifdef __RTX
BOOL    USBD_FIQ_Active = __FALSE;
BOOL    USBD_IRQ_Active = __FALSE;
#define USBD_Int_Active  (USBD_FIQ_Active | USBD_IRQ_Active)
static OS_MUT USBD_HW_Mutex;
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
void __svc(1) USBD_Intr (int ena);
void __SVC_1            (int ena) {
  if (ena) {
#if USB_LPC13XX_FIQ
    NVIC_EnableIRQ(USB_FIQn);           /* enable USB HighPriority interrupt  */
#endif
    NVIC_EnableIRQ(USB_IRQn);           /* enable USB interrupt               */
  } else {
#if USB_LPC13XX_FIQ
    NVIC_DisableIRQ(USB_FIQn);          /* disable USB HighPriority interrupt */
#endif
    NVIC_DisableIRQ(USB_IRQn);          /* disable USB interrupt              */
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

U32 EPAdr (U32 EPNum) {
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

void WrCmd (U32 cmd) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* enable USB interrupt               */
#endif
}


/*
 *  Write Command Data
 *    Parameters:      cmd:   Command
 *                     val:   Data
 *    Return Value:    None        
 */

void WrCmdDat (U32 cmd, U32 val) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = val;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* enable USB interrupt               */
#endif
}


/*
 *  Write Command to Endpoint
 *    Parameters:      cmd:   Command
 *                     val:   Data
 *    Return Value:    None
 */

void WrCmdEP (U32 EPNum, U32 cmd) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = CMD_SEL_EP(EPAdr(EPNum));
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* enable USB interrupt               */
#endif
}


/*
 *  Read Command Data
 *    Parameters:      cmd:   Command
 *    Return Value:    Data Value
 */

U32 RdCmdDat (U32 cmd) {
  uint32_t USBCmdData;

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT | CDFULL_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CDFULL_INT) == 0);
  USBCmdData = LPC_USB->CmdData; 

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* enable USB interrupt               */
#endif

  return (USBCmdData);
}


/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB
 *    Return Value:    None
 */

void USBD_Init (void) {

#ifdef __RTX
  os_mut_init(&USBD_HW_Mutex);
#endif

  /* Enable AHB clock for USB_REG. and AHB clock for IO configuration block.  */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1<<14) | (1<<16)  );
                                            
  /* D+, D- needs not to be configured                                        */
  LPC_IOCON->PIO0_3  &= ~0x07;          /* P0.3 VBUS                          */
  LPC_IOCON->PIO0_3  |=  0x01;          /* Select function USB_VBUS           */
  LPC_IOCON->PIO0_6  &= ~0x07;          /* P0.6 SoftConnect                   */
  LPC_IOCON->PIO0_6  |=  0x01;          /* Selects function USB_CONNECT       */

#if USB_LPC13XX_FIQ
  /* only BULK EP3 and FRAME(ISO) can be 
     routed to FIQ but not all at the same time.                              */
  LPC_USB->DevFIQSel = USB_LPC13XX_FIQ_VAL;
#endif 

#ifdef __RTX
  USBD_Intr(1);                         /* enable USB interrupt               */
#else

#if USB_LPC13XX_FIQ
  NVIC_EnableIRQ(USB_FIQn);             /* enable USB HighPriority interrupt  */
#endif

  NVIC_EnableIRQ(USB_IRQn);             /* enable USB interrupt               */
#endif

#if 1 /* Partial Manual Reset since Automatic Bus Reset is not working        */
  USBD_Reset();
  USBD_SetAddress(0, 0);
#endif
}


/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect (BOOL con) {
  WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(con ? DEV_CON : 0));
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {

  LPC_USB->DevIntClr = 0xFFFFFFFF;
  LPC_USB->DevIntEn  = (0xFF << 1)  |   /* enable EP0..7        interrupt     */
                       DEV_STAT_INT |   /* enable device status interrupt     */
#ifdef __RTX
              ((USBD_RTX_DevTask   != 0) ? FRAME_INT : 0);   /* SOF event     */
#else
              ((USBD_P_SOF_Event   != 0) ? FRAME_INT : 0);   /* SOF event     */
#endif

}


/*
 *  USB Device Suspend Function
 *   Called automatically on USB Suspend
 *    Return Value:    None
 */

void USBD_Suspend (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Resume
 *    Return Value:    None
 */

void USBD_Resume (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp (void) {

  if (USBD_DeviceStatus & USB_GETSTATUS_REMOTE_WAKEUP) {
    WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(DEV_CON));
  }
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Enable/Disable
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
  WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr));  /* Don't wait for next  */
  WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr));  /*  Setup Status Phase  */
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure (BOOL cfg) {

  WrCmdDat(CMD_CFG_DEV, DAT_WR_BYTE(cfg ? CONF_DVICE : 0));
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
  /*
     EPs need not to be configured.
     EPs use fix maxPacketSize: Control, Bulk, Interrupt  64Byte
                                Isochronous              512Byte              */
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
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Disable USB Device Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_DA));
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_ST));
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP (U32 EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf (U32 EPNum) {
  WrCmdEP(EPNum, CMD_CLR_BUF);
}


/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Endpoint Number
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

  LPC_USB->Ctrl = ((EPNum & 0x0F) << 2) | CTRL_RD_EN;
  for (n = 0; n < 3; n++) __nop();      /* 3 clock cycles to fetch 
                                           the packet length from RAM.        */

  do {
    cnt = LPC_USB->RxPLen;
  } while ((cnt & PKT_DV) == 0);
  cnt &= PKT_LNGTH_MASK;

  for (n = 0; n < (cnt + 3) / 4; n++) {
    *((__packed U32 *)pData) = LPC_USB->RxData;
    pData += 4;
  }

  LPC_USB->Ctrl = 0;

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

  LPC_USB->Ctrl = ((EPNum & 0x0F) << 2) | CTRL_WR_EN;
  for (n = 0; n < 3; n++) __nop();      /* 3 clock cycles to fetch 
                                           the packet length from RAM.        */
  LPC_USB->TxPLen = cnt;

  for (n = 0; n < (cnt + 3) / 4; n++) {
    LPC_USB->TxData = *((__packed U32 *)pData);
    pData += 4;
  }
  
  if (n == 0) LPC_USB->TxData = 0;      /* Dummy write to send ZLP            */ 

  LPC_USB->Ctrl = 0;

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
 *  USB Device Low Priority Interrupt Service Routine
 */

void USB_IRQHandler (void) {
  U32 disr, val, n, m;
  U32 episr, episrCur;

#ifdef __RTX
  USBD_IRQ_Active = __TRUE;
#endif

  disr = LPC_USB->DevIntSt;                        /* Device Interrupt Status */
  LPC_USB->DevIntClr = disr;                       /* clear interrupts        */

  /* Device Status Interrupt (Reset, Connect change, Suspend/Resume)          */
  if (disr & DEV_STAT_INT) {
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

  /* logical Endpoint0..3 Interrupts */
  if (disr & EP_INT) {
    episrCur = 0;
    episr    = (disr & EP_INT) >> 1;    /* handle only EP interrupts          */
    for (n = 0; n < 8; n++) {           /* Check All Endpoints                 
                                           ISO EP do not create interrupts)   */
      if (episr == episrCur) break;     /* break if all EP interrupts handled */
      if (episr & (1 << n)) {
        episrCur |= (1 << n);
        m = n >> 1;
        /* clear EP interrupt by sending cmd to the command engine.           */
        WrCmd(CMD_SEL_EP_CLRI(n));      
        val = RdCmdDat(DAT_SEL_EP_CLRI(n));

  
        if ((n & 1) == 0) {             /* OUT Endpoint                       */
          if (n == 0) {                 /* Control OUT Endpoint               */
            if (val & EP_SEL_STP) {     /* Setup Packet                       */
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
          if (USBD_RTX_EPTask[m]) {     /* OUT Packet                         */
            isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[m]);
          }
#else
          if (USBD_P_EP[m]) {
            USBD_P_EP[m](USBD_EVT_OUT);
          }
#endif
        } else {                        /* IN Endpoint                        */
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
  }
isr_end:
  ;
#ifdef __RTX
  USBD_IRQ_Active = __FALSE;
#endif
}

#if USB_LPC13XX_FIQ
#pragma diag_suppress 177, 550
/*
 *  USB High Priority Interrupt Service Routine
 */

void USB_FIQHandler(void) {
  U32 disr, val, n, m = 0;
  U32 episr, episrCur;

#ifdef __RTX
  USBD_FIQ_Active = __TRUE;
#endif

  disr = LPC_USB->DevIntSt;             /* Device Interrupt Status            */
  LPC_USB->DevIntClr = disr;            /* clear interrupts                   */

#if (USB_LPC13XX_FIQ_VAL & 0x01)
  /* Start of Frame Interrupt */
  if (disr & FRAME_INT) {
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
#endif

#if (USB_LPC13XX_FIQ_VAL & 0x06)
  /* logical Endpoint3 Interrupts                                             */
  if (disr & (EP7_INT | EP6_INT)) {
    episrCur = 0;
    episr    = (disr & (EP7_INT | EP6_INT)) >> 1;  /* only logical EP3 intr   */
    for (n = 6; n < 8; n++) {           /* Check logical Endpoint3            */
      if (episr == episrCur) break;     /* break if all EP3 interrupts handled*/
      if (episr & (1 << n)) {
        episrCur |= (1 << n);
        m = n >> 1;
        /* clear EP interrupt by sending cmd to the command engine.           */
        WrCmd(CMD_SEL_EP_CLRI(n));
        val = RdCmdDat(DAT_SEL_EP_CLRI(n));
  
        if ((n & 1) == 0) {             /* OUT Endpoint                       */
#ifdef __RTX
          if (USBD_RTX_EPTask[m]) {     /* OUT Packet                         */
            isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[m]);
          }
#else
          if (USBD_P_EP[m]) {
            USBD_P_EP[m](USBD_EVT_OUT);
          }
#endif
          }
        } else {                        /* IN Endpoint                        */
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
#endif
#ifdef __RTX
  USBD_FIQ_Active = __FALSE;
#endif
}
#endif
