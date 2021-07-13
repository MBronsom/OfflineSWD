/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_LPC177x_8x.c
 *      Purpose: Hardware Layer module for NXP LPC177x/8x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include "usbreg.h"
#include <LPC177x_8x.h>                      /* LPC177x_8x definitions        */

#define __NO_USB_LIB_C
#include "usb_config.c"

#pragma diag_suppress 1441

#define EP_MSK_CTRL 0x0001      /* Control Endpoint Logical Address Mask      */
#define EP_MSK_BULK 0xC924      /* Bulk Endpoint Logical Address Mask         */
#define EP_MSK_INT  0x2492      /* Interrupt Endpoint Logical Address Mask    */
#define EP_MSK_ISO  0x1248      /* Isochronous Endpoint Logical Address Mask  */

#ifdef __RTX
        BOOL        USBD_Int_Active = __FALSE;
static  OS_MUT      USBD_HW_Mutex;
#endif

#ifdef __RTX
#define TICK  10000
#else
static uint32_t calDelay = 0;
#endif

void USBD_Reset      (void);
void USBD_SetAddress (uint32_t adr, uint32_t setup);

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

static uint32_t EPAdr (uint32_t EPNum) {
  uint32_t val;

  val = (EPNum & 0x0F) << 1;
  if (EPNum & 0x80) {
    val += 1;
  }
  return (val);
}

/*
 *  usbd_delay_ms
 *    Parameter:  ms:         Number of milliseconds to delay execution for
 *    Return:
 */
void usbd_delay_ms (uint32_t ms) {
  
#ifdef __RTX
  float val = TICK / 1000;
  os_dly_wait ((uint16_t)((ms + val - 1.0) / val));
#else
  uint32_t cnt = 0, vals = 0, vale = 0;

start:
  if (!calDelay) {                      /* If not calibrated                  */
    cnt = 1000;
    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)) {
                                        /* If SysTick timer not running       */
      vals = 0xFFFFFF;
      SysTick->LOAD  = 0xFFFFFF;
      SysTick->VAL   = 0xFFFFFF;
      SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | 
                       SysTick_CTRL_ENABLE_Msk;
    } else {
      vals = SysTick->VAL;              /* Timer start value                  */
    }
  } else {
    cnt = ms * calDelay;
  }

  while (cnt--);

  if (!calDelay) {                      /* If not calibrated                  */
    vale = SysTick->VAL;                /* Timer end value                    */
    if (vale >= vals)                   /* If timer reloaded                  */
      vals += SysTick->LOAD;
    SystemCoreClockUpdate();            /* Update SystemCoreClock variable    */
    calDelay = SystemCoreClock / (vals - vale);   /* Calibrated value         */
    if (vals == 0xFFFFFF)               /* Stop timer if we started it        */
      SysTick->CTRL  = 0;
    goto start;
  }
#endif
}

/*
 *  usbd_i2c_read
 *    Parameters:  i2c_adr:    Address of the I2C chip
 *                 reg_adr:    Address of the register to read
 *    Return:                  0 .. 0xFF = read value, -1 = error
 */

int usbd_i2c_read (uint8_t i2c_adr, uint8_t reg_adr) {
  int32_t tout;

  LPC_USB->I2C_TX    = (      1 << 8) | /* START bit on transmit start        */
                       (i2c_adr << 1) | /* I2C Address                        */
                       (      0 << 0) ; /* Write request                      */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (-1);
    if (tout <= 10) usbd_delay_ms (10);
  }
  LPC_USB->I2C_TX    =  reg_adr;        /* Register address to read from      */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (-1);
    if (tout <= 10) usbd_delay_ms (10);
  }
  LPC_USB->I2C_TX    = (      1 << 8) | /* START bit on transmit start        */
                       (i2c_adr << 1) | /* I2C Address                        */
                       (      1 << 0) ; /* Read request                       */
  LPC_USB->I2C_TX    = (      1 << 9) | /* STOP bit on end                    */
                        0x55;           /* Dummy data transmit to receive     */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 0)) break;       /* Wait for transaction done*/
    if (tout ==  0) return (-1);
    if (tout <= 10) usbd_delay_ms (10);
  }
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (!(LPC_USB->I2C_STS & (1 << 5))) break;    /* Wait for STOP condition  */
    if (tout ==  0) return (-1);
    if (tout <= 10) usbd_delay_ms (10);
  }
  return (LPC_USB->I2C_RX & 0xFF);
}


/*
 *  usbd_i2c_write
 *    Parameters:  i2c_adr:    Address of the I2C chip
 *                 reg_adr:    Address of the register to write
 *                 reg_val:    Value to be written
 *    Return:                 __TRUE = success, __FALSE = error
 */

int usbd_i2c_write (uint8_t i2c_adr, uint8_t reg_adr, uint8_t reg_val) {
  int32_t tout;

  LPC_USB->I2C_TX    = (      1 << 8) | /* START bit on transmit start        */
                       (i2c_adr << 1) | /* I2C Address                        */
                       (      0 << 0) ; /* Write request                      */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (__FALSE);
    if (tout <= 10) usbd_delay_ms (10);
  }
  LPC_USB->I2C_TX    =  reg_adr;        /* Register address to write to       */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (__FALSE);
    if (tout <= 10) usbd_delay_ms (10);
  }
  LPC_USB->I2C_TX    = (      1 << 9) | /* STOP bit on end                    */
                        reg_val;        /* Register value to write            */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (!(LPC_USB->I2C_STS & (1 << 5))) break;    /* Wait for STOP condition  */
    if (tout ==  0) return (__FALSE);
    if (tout <= 10) usbd_delay_ms (10);
  }
  return (__TRUE);
}


/*
 *  Write Command
 *    Parameters:      cmd:   Command
 *    Return Value:    None
 */

static void WrCmd (uint32_t cmd) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);

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

static void WrCmdDat (uint32_t cmd, uint32_t val) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = val;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);

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

static void WrCmdEP (uint32_t EPNum, uint32_t cmd) {

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = CMD_SEL_EP(EPAdr(EPNum));
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
  LPC_USB->DevIntClr = CCEMTY_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(1);   /* Enable USB interrupt               */
#endif
}


/*
 *  Read Command Data
 *    Parameters:      cmd:   Command
 *    Return Value:    Data Value
 */

static uint32_t RdCmdDat (uint32_t cmd) {
  uint32_t USBCmdData;

#ifdef __RTX
  if (!USBD_Int_Active) USBD_Intr(0);   /* Disable USB interrupt              */
#endif

  LPC_USB->DevIntClr = CCEMTY_INT | CDFULL_INT;
  LPC_USB->CmdCode = cmd;
  while ((LPC_USB->DevIntSt & CDFULL_INT) == 0);
  USBCmdData = LPC_USB->CmdData; 

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

  LPC_SC->PCONP       |=  (1UL << 31);  /* Enable USB interface power/clock   */
  
  LPC_USB->USBClkCtrl = 0x1A;                  /* Dev, PortSel, AHB clock en  */
  while ((LPC_USB->USBClkSt & 0x1A) != 0x1A);
  
#ifdef USBD_2
  LPC_USB->StCtrl   =  0x02;            /* USB device maped to U2 port        */
  
  /* Set pin functions                                                        */
  LPC_IOCON->P1_30 = 2;
  LPC_IOCON->P0_14 = 3;                 /* USB_CONNECT2                       */
  LPC_IOCON->P0_31 = 1;                 /* USB_D+2                            */
                                        /* USB_D-2 is dedicated pin           */
  LPC_IOCON->P1_30 = 0;
  LPC_IOCON->P0_13 = 1 | (1 << 7);      /* USB_UP_LED2                        */

#else
  /* Enable USB1 Transceiver                                                  */
  LPC_USB->StCtrl   =  0x00;            /* USB device maped to U1 port        */
  LPC_USB->OTGClkCtrl =  0x1A;          /* Enable Device, OTG, AHB mster clk  */
  while (!((LPC_USB->OTGClkSt & 0x1A) == 0x1A));   /* Wait clock enable       */

  /* Enable pins                                                              */
  LPC_IOCON->P0_29 = 1;                 /* USB_D+1                            */
  LPC_IOCON->P0_30 = 1;                 /* USB_D-1                            */
  LPC_IOCON->P1_28 = 1;                 /* USB_SCL1                           */
  LPC_IOCON->P1_29 = 1;                 /* USB_SDA1                           */
  LPC_IOCON->P1_18 = 1;                 /* USB_UP_LED1                        */

  LPC_USB->OTGClkCtrl |= (1 << 2);      /* Enable I2C                         */
  while (!(LPC_USB->OTGClkSt & (1 << 2)));          /* Wait I2C clock enable  */

  LPC_USB->I2C_CTL   = 1 << 8;          /* I2C reset                          */
  while (LPC_USB->I2C_CTL & (1 << 8));  /* Wait reset end                     */

                                        /* Set I2C clock to 100kHz            */
  LPC_USB->I2C_CLKHI = 240;             /* 48MHz /200kHz = 240 clock cycles   */
  LPC_USB->I2C_CLKLO = 240;             /* 48MHz /200kHz = 240 clock cycles   */
#endif

  USBD_Reset();
  USBD_SetAddress(0, 0);

#ifdef __RTX
  USBD_Intr(1);                         /* enable USB interrupt               */
#else
  NVIC_EnableIRQ(USB_IRQn);             /* enable USB interrupt               */
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
  
#ifndef USBD_2
  switch (usbd_i2c_read (0x2D, 0x00) | (usbd_i2c_read (0x2D, 0x01) << 8)) {
    case 0x058D:                        /* MIC2555 Transceiver                */
      /* Control Register 2 = DP - Enable Pull-up                             */
      usbd_i2c_write (0x2D, 0x06, 0x01);
      break;
  }
#endif
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {

  LPC_USB->EpInd     = 0;
  LPC_USB->MaxPSize  = USBD_MAX_PACKET0;
  LPC_USB->EpInd     = 1;
  LPC_USB->MaxPSize  = USBD_MAX_PACKET0;
  while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0);

  LPC_USB->EpIntClr  = 0xFFFFFFFF;
  LPC_USB->EpIntEn   = 0xFFFFFFFF;
  LPC_USB->DevIntClr = 0xFFFFFFFF;
  LPC_USB->DevIntEn  = DEV_STAT_INT    | EP_SLOW_INT    |
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

void USBD_SetAddress (uint32_t adr, uint32_t setup) {
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

  LPC_USB->ReEp = 0x00000003;
  while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0);
  LPC_USB->DevIntClr = EP_RLZED_INT;
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
  uint32_t num;

  num = EPAdr(pEPD->bEndpointAddress);
  LPC_USB->ReEp |= (1 << num);
  LPC_USB->EpInd = num;
  LPC_USB->MaxPSize = pEPD->wMaxPacketSize;
  while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0);
  LPC_USB->DevIntClr = EP_RLZED_INT;
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP (uint32_t dir) {
  /* Not needed */
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (uint32_t EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Disable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (uint32_t EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_DA));
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (uint32_t EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP (uint32_t EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_ST));
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP (uint32_t EPNum) {
  WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}


/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf (uint32_t EPNum) {
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

uint32_t USBD_ReadEP (uint32_t EPNum, uint8_t *pData) {
  uint32_t cnt, n;

#ifdef __RTX
  os_mut_wait(&USBD_HW_Mutex, 0xFFFF);
#endif

  LPC_USB->Ctrl = ((EPNum & 0x0F) << 2) | CTRL_RD_EN;

  do {
    cnt = LPC_USB->RxPLen;
  } while ((cnt & PKT_RDY) == 0);
  cnt &= PKT_LNGTH_MASK;

  for (n = 0; n < (cnt + 3) / 4; n++) {
    *((__packed uint32_t *)pData) = LPC_USB->RxData;
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

uint32_t USBD_WriteEP (uint32_t EPNum, uint8_t *pData, uint32_t cnt) {
  uint32_t n;

#ifdef __RTX
  os_mut_wait(&USBD_HW_Mutex, 0xFFFF);
#endif

  LPC_USB->Ctrl = ((EPNum & 0x0F) << 2) | CTRL_WR_EN;

  LPC_USB->TxPLen = cnt;

  for (n = 0; n < (cnt + 3) / 4; n++) {
    LPC_USB->TxData = *((__packed uint32_t *)pData);
    pData += 4;
  }

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

uint32_t USBD_GetFrame (void) {
  uint32_t val;

  WrCmd(CMD_RD_FRAME);
  val = RdCmdDat(DAT_RD_FRAME);
  val = val | (RdCmdDat(DAT_RD_FRAME) << 8);

  return (val);
}


#ifdef __RTX
uint32_t LastError;                     /* Last Error                         */

/*
 *  Get USB Device Last Error Code
 *    Parameters:      None
 *    Return Value:    Error Code
 */

uint32_t USBD_GetError (void) {
  return (LastError);
}
#endif


/*
 *  USB Device Interrupt Service Routine
 */

void USB_IRQHandler (void) {
  uint32_t disr, val, n, m;
  uint32_t episr, episrCur;

#ifdef __RTX
  USBD_Int_Active = __TRUE;
#endif

  disr = LPC_USB->DevIntSt & LPC_USB->DevIntEn; /* Device Int Mask Stat       */

  /* Device Status Interrupt (Reset, Connect change, Suspend/Resume)          */
  if (disr & DEV_STAT_INT) {
    LPC_USB->DevIntClr = DEV_STAT_INT;
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
    LPC_USB->DevIntClr = FRAME_INT;
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
    LPC_USB->DevIntClr = ERR_INT;
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
    episr    = LPC_USB->EpIntSt;
    for (n=0; n<((USBD_EP_NUM+1)<<1); n++) {/* Check All Enabled Endpoints    */
      if (episr == episrCur) break;         /* break if all EP ints handled   */
      if (episr & (1 << n)) {
        episrCur |= (1 << n);
        m = n >> 1;
        LPC_USB->EpIntClr = (1 << n);
        while ((LPC_USB->DevIntSt & CDFULL_INT) == 0);
        val = LPC_USB->CmdData;

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
            isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[m]);
          }
#else
          if (USBD_P_EP[m]) {
            USBD_P_EP[m](USBD_EVT_OUT);
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
    LPC_USB->DevIntClr = EP_SLOW_INT;
  }

isr_end:
#ifdef __RTX
  USBD_Int_Active = __FALSE;
#else
  ;
#endif
}
