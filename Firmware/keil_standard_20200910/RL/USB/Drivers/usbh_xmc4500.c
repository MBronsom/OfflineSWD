/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_xmc4500.c
 *      Purpose: Full/Low-speed Host Infineon XMC4500 Driver module
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include "usbh_xmc4500.h"
#include "System_XMC4500.h"
#include <xmc4500.h>
#include <string.h>


/************************** Host Controller Driver Structure ******************/

USBH_HCD usbh0_hcd = {                  /* Host Controller Driver structure   */
  USBH_XMC4500_Get_Capabilities,        /* Get Host Ctrl Driver capabilities  */
  USBH_XMC4500_Delay_ms,                /* Delay in ms                        */
  USBH_XMC4500_Pins_Config,             /* Config/Unconfig pins               */
  USBH_XMC4500_Init,                    /* Init/Uninit Host Controller        */
  USBH_XMC4500_Port_Power,              /* On/Off Port Power                  */
  USBH_XMC4500_Port_Reset,              /* Reset port                         */
  USBH_XMC4500_Get_Connect,             /* Get port conn/disconn status       */
  USBH_XMC4500_Get_Speed,               /* Get port enumerated speed          */
  USBH_XMC4500_EP_Add,                  /* Add Endpoint                       */
  USBH_XMC4500_EP_Config,               /* (Re)Configure Endpoint             */
  USBH_XMC4500_EP_Remove,               /* Remove Endpoint                    */
  USBH_XMC4500_URB_Submit,              /* Submit USB Block Request           */
  USBH_XMC4500_URB_Cancel               /* Cancel USB Block Request           */
};


/************************** Driver Settings ***********************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

//  <o0> Size of memory used by the USB Host Controller <1-1048576>
//  <i> This is a size of memory (in bytes) that the USB Host Controller 
//  <i> will use for USB communication data.
#define USBH_XMC4500_SZ_MPOOL     0x00000234


/************************** Variable and Memory Definitons ********************/

#define MAX_TIMEOUT_COUNT         100
#define DEBOUNCE_500ms            500

#define USB0_MAX_CH               14

static U32 *USB0_DFIFO[USB0_MAX_CH]={ (U32 *)(USB0_BASE + 0x1000), (U32 *)(USB0_BASE + 0x2000), 
                                      (U32 *)(USB0_BASE + 0x3000), (U32 *)(USB0_BASE + 0x4000), 
                                      (U32 *)(USB0_BASE + 0x5000), (U32 *)(USB0_BASE + 0x6000), 
                                      (U32 *)(USB0_BASE + 0x7000), (U32 *)(USB0_BASE + 0x8000), 
                                      (U32 *)(USB0_BASE + 0x9000), (U32 *)(USB0_BASE + 0xA000), 
                                      (U32 *)(USB0_BASE + 0xB000), (U32 *)(USB0_BASE + 0xC000), 
                                      (U32 *)(USB0_BASE + 0xD000), (U32 *)(USB0_BASE + 0xE000) }; 

/* Reserve memory for memory pool */ 
       U32       USBH_MPOOL[(USBH_XMC4500_SZ_MPOOL+3)>>2];

static BOOL      HW_Accessing                      = { __FALSE };
static USBH_URB *CHURB             [USB0_MAX_CH]   = { 0 };
static U8        cntInterval       [USB0_MAX_CH]   = { 0 };
static U8        cntIntervalMax    [USB0_MAX_CH]   = { 0 };
static U16       cntDebounce                       = { 0 };
static U32       calDelay                          =   0  ;
static U32       Port_Discon_Evt                   = { 0 };
static U32       Port_Speed                        = { 0 };
static U32       Port_Con                          = { 0 };


/************************** Local Module Functions ****************************/


/***----------------------- Channel Functions ------------------------------***/

/*------------------------- USBH_XMC4500_CH_GetIndexFromCH ---------------------
 *
 *  Get the Index of Channel from it's Address
 *
 *  Parameter:  ptrCH:      Pointer to the Channel
 *  Return:                 Index of the Channel
 *----------------------------------------------------------------------------*/

static U32 USBH_XMC4500_CH_GetIndexFromCH (USBH_XMC4500_CH *ptrCH) {
  return (ptrCH - (USBH_XMC4500_CH *)(USB0_CH0));
}


/*------------------------- USBH_XMC4500_CH_GetCHFromIndex ---------------------
 *
 *  Get the Channel Address from it's Index
 *
 *  Parameter:  idx:        Index of the Channel
 *  Return:                 Address of the Channel
 *----------------------------------------------------------------------------*/

static USBH_XMC4500_CH *USBH_XMC4500_CH_GetCHFromIndex (U32 idx) {
  return ((USBH_XMC4500_CH *)(USB0_CH0) + idx);
}


/*------------------------- USBH_XMC4500_CH_FindFree ---------------------------
 *
 *  Find a free Channel
 *
 *  Parameter:
 *  Return:                 Pointer to the first free Channel
 *                          (0 = no free Channel is available)
 *----------------------------------------------------------------------------*/

static void *USBH_XMC4500_CH_FindFree (void) {
  USBH_XMC4500_CH *ptr_CH;
  U32              i;

  ptr_CH = (USBH_XMC4500_CH *)(USB0_CH0);

  for (i = 0; i < USB0_MAX_CH; i++) {
    if (!ptr_CH->HCCHAR) {
      return (ptr_CH);
    }
    ptr_CH++;
  }

  return (0);
}


/*------------------------- USBH_XMC4500_CH_Disable ----------------------------
 *
 *  Disable the Channel
 *
 *  Parameter:  ptrCH:      Pointer to the Channel
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_XMC4500_CH_Disable (USBH_XMC4500_CH *ptrCH) {
  S32 tout;

  if (!ptrCH) 
    return (__FALSE);

  ptrCH->HCINTMSK  =  0;
  USBH_XMC4500_Delay_ms (2);
  if (ptrCH->HCCHAR & USB_CH_HCCHAR_ChEna_Msk) {
    ptrCH->HCINT     =  ~USB_CH_HCINT_ChHltd_Msk;
    ptrCH->HCCHAR    =  (ptrCH->HCCHAR  |  USB_CH_HCCHAR_ChEna_Msk);
    USBH_XMC4500_Delay_ms (2);
    ptrCH->HCCHAR    =  (ptrCH->HCCHAR  & ~USB_CH_HCCHAR_ChEna_Msk) | USB_CH_HCCHAR_ChDis_Msk;
    for (tout=1000; tout>=0; tout--) {            /* Wait                     */            
      if (ptrCH->HCINT & USB_CH_HCINT_ChHltd_Msk) /* If transaction done      */
        break;
      if ((ptrCH->HCCHAR & (USB_CH_HCCHAR_ChEna_Msk | USB_CH_HCCHAR_ChDis_Msk)) == (USB_CH_HCCHAR_ChEna_Msk | USB_CH_HCCHAR_ChDis_Msk)) 
        break;
      if (tout ==  0) return (__FALSE);
      if (tout <= 10) USBH_XMC4500_Delay_ms (10); /* for max 100 ms           */
    }
  }

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_CH_TransferEnqueue --------------------
 *
 *  Enqueue the Transfer
 *
 *  Parameter:  ptrCH:      Pointer to the channel on which transfer will take place
 *              tgl_typ:    Toggle (bit 5..4: bit 5 - force toggle, bit 4 - value) and 
 *                          Packet type (bit 3..0: USBH_PACKET_IN, USBH_PACKET_OUT or USBH_PACKET_SETUP)
 *              buf:        Start of the receive or transmit data buffer
 *              len:        Length of the data to be received or sent
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_XMC4500_CH_TransferEnqueue (USBH_XMC4500_CH *ptrCH, U32 tgl_typ, U8 *buf, U32 len) {
  U32  hcchar;
  U32  hctsiz;
  U32  hcintmsk;
  U32  mps;
  U32  ch_idx;
  U32  num;
  U32 *ptr_src, *ptr_dst;
  BOOL load_data;

  if (!ptrCH)
    return (__FALSE);

  if (!Port_Con)
    return (__FALSE);

  hcchar   = ptrCH->HCCHAR;                       /* Read channel characterist*/
  hctsiz   = ptrCH->HCTSIZ;                       /* Read channel size info   */
  hcintmsk = 0;

  /* Prepare transfer                                                         */
                                                  /* Prepare HCCHAR register  */
  hcchar        &= USB_CH_HCCHAR_OddFrm_Msk  |    /* Keep ODDFRM              */
                   USB_CH_HCCHAR_DevAddr_Msk |    /* Keep DEVADDR             */
                   USB_CH_HCCHAR_MC_EC_Msk   |    /* Keep MC_EC               */
                   USB_CH_HCCHAR_EPType_Msk  |    /* Keep EPTYP               */
                   USB_CH_HCCHAR_LSpdDev_Msk |    /* Keep LSPDDEV             */
                   USB_CH_HCCHAR_EPNum_Msk   |    /* Keep EPNUM               */
                   USB_CH_HCCHAR_MPS_Msk ;        /* Keep MPS                 */
  switch (tgl_typ & 0x0F) {
    case USBH_PACKET_IN:
      hcchar   |=  USB_CH_HCCHAR_EPDir_Msk;
      hcintmsk  =  USB_CH_HCINTMSK_DataTglErrMsk_Msk | 
                   USB_CH_HCINTMSK_BblErrMsk_Msk     | 
                   USB_CH_HCINTMSK_XactErrMsk_Msk    | 
                   USB_CH_HCINTMSK_AckMsk_Msk        | 
                   USB_CH_HCINTMSK_NakMsk_Msk        | 
                   USB_CH_HCINTMSK_StallMsk_Msk      | 
                   USB_CH_HCINTMSK_XferComplMsk_Msk  ;
      break;
    case USBH_PACKET_OUT:
      hcchar   &= ~USB_CH_HCCHAR_EPDir_Msk;
      hcintmsk  =  USB_CH_HCINTMSK_XactErrMsk_Msk    | 
                   USB_CH_HCINTMSK_NyetMsk_Msk       | 
                   USB_CH_HCINTMSK_NakMsk_Msk        | 
                   USB_CH_HCINTMSK_StallMsk_Msk      | 
                   USB_CH_HCINTMSK_XferComplMsk_Msk  ;
      break;
    case USBH_PACKET_SETUP:
      hcchar   &= ~USB_CH_HCCHAR_EPDir_Msk;
      hcintmsk  =  USB_CH_HCINTMSK_XactErrMsk_Msk    | 
                   USB_CH_HCINTMSK_NakMsk_Msk        | 
                   USB_CH_HCINTMSK_StallMsk_Msk      | 
                   USB_CH_HCINTMSK_XferComplMsk_Msk  ;
      break;
  }
  hcchar &= ~USB_CH_HCCHAR_ChDis_Msk;
  hcchar |=  USB_CH_HCCHAR_ChEna_Msk;

                                                  /* Prepare HCTSIZ register  */
  hctsiz &= USB_CH_HCTSIZ_BUFFERMODE_Pid_Msk;     /* Keep Data PID            */
  if ((tgl_typ & 0x0F) == USBH_PACKET_SETUP) {    /* If setup pckt DPID=MDATA */
    hctsiz &= ~USB_CH_HCTSIZ_BUFFERMODE_Pid_Msk;
    hctsiz |= (3 << USB_CH_HCTSIZ_BUFFERMODE_Pid_Pos);      /* MDATA/SETUP PID*/
  } else if ((tgl_typ >> 5) & 1) {                /* If toggle force bit activ*/
    hctsiz &= ~USB_CH_HCTSIZ_BUFFERMODE_Pid_Msk;
    if ((tgl_typ >> 4) & 1) {                     /* Toggle bit value         */
      hctsiz |=  (2 << USB_CH_HCTSIZ_BUFFERMODE_Pid_Pos);   /* DATA1 PID      */
    } else {
      hctsiz |=  (0 << USB_CH_HCTSIZ_BUFFERMODE_Pid_Pos);   /* DATA0 PID      */
    }
  }

  mps = hcchar & 0x7FF;                           /* Maximum packet size      */
  if (len) {                                      /* Normal packet            */
    hctsiz |= ((len+mps-1) / mps) << 19;          /* Prepare PKTCNT field     */
    hctsiz |= ( len             ) <<  0;          /* Prepare XFERSIZE field   */
  } else {                                        /* Zero length packet       */
    hctsiz |= ( 1               ) << 19;          /* Prepare PKTCNT field     */
    hctsiz |= ( 0               ) <<  0;          /* Prepare XFERSIZE field   */
  }

  ch_idx  = USBH_XMC4500_CH_GetIndexFromCH (ptrCH);

  /* load_data == __TRUE if there is data to be loaded to FIFO 
    (If packet is OUT or SETUP and len > 0)                                   */
  load_data = (((tgl_typ & 0x0F) == USBH_PACKET_OUT)    || 
               ((tgl_typ & 0x0F) == USBH_PACKET_SETUP)) && 
                 len; 
  num = (len + 3) / 4;
  ptr_dst = USB0_DFIFO[ch_idx];
  ptr_src = (U32 *) buf;

  ptrCH->HCINTMSK = hcintmsk;                     /* Enable channel interrupts*/
  ptrCH->HCTSIZ   = hctsiz;                       /* Write ch transfer size   */
//ptrCH->DMAADDR  = (U32)buf;                     /* In DMA Buffer mode addr  */
  ptrCH->HCCHAR   = hcchar;                       /* Write ch characteristics */

  if (load_data) {
    while (num--) {
      while (!((USB0->GNPTXSTS & 0x00FF0000) && (USB0->GNPTXSTS & 0x0000FFFF)));
      *ptr_dst = *ptr_src++;
    }
  }

  return (__TRUE);
}


/************************** Module Functions **********************************/

/*------------------------- USBH_XMC4500_Get_Capabilities ----------------------
 *
 *  Get capabilities of Host Controller Driver
 *
 *  Parameter:  cap:        Pointer to USBH_HCI_CAP structure where 
 *                          capabilities are loaded
 *  Return:
 *----------------------------------------------------------------------------*/

void USBH_XMC4500_Get_Capabilities (USBH_HCI_CAP *cap) {
  cap->MultiPckt = __FALSE;
  cap->MaxDataSz = 512;
  cap->CtrlNAKs  = 100000;
  cap->BulkNAKs  = 1000000;
}


/*------------------------- USBH_XMC4500_Delay_ms ------------------------------
 *
 *  Delay execution (in milliseconds)
 *  Calibration is done if global variable calDelay is 0
 *
 *  Parameter:  ms:         Number of milliseconds to delay execution for
 *  Return:
 *----------------------------------------------------------------------------*/

void USBH_XMC4500_Delay_ms (U32 ms) {
  U32 cnt = 0, vals = 0, vale = 0;

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
    calDelay = SystemCoreClock / (vals - vale);   /* Calibrated value         */
    if (vals == 0xFFFFFF)               /* Stop timer if we started it        */
      SysTick->CTRL  = 0;
    goto start;
  }
}


/*------------------------- USBH_XMC4500_Pins_Config ---------------------------
 *
 *  Configure or unconfigure pins used by the USB Host
 *
 *  Parameter:  on:         __TRUE = configure, __FALSE = unconfigure
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_Pins_Config (BOOL on) {

  if (on) {
    /* Set P3.2 as alternate function 1 (USB.DRIVEVBUS), Push-pull            */
    PORT3->IOCR0  &= ~PORT3_IOCR0_PC2_Msk;
    PORT3->IOCR0  |= (0x11 << PORT3_IOCR0_PC2_Pos);
  } else {
    PORT3->IOCR0  &= ~PORT3_IOCR0_PC2_Msk;
  }

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_Init ----------------------------------
 *
 *  Initialize or uninitialize the USB Host Controller
 *
 *  Parameter:  on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_Init (BOOL on) {
  S32 tout;

  if (on) {
    /* Initialize memory pool for data                                        */
    if (!usbh_mem_init(0, (U32 *)&USBH_MPOOL, USBH_XMC4500_SZ_MPOOL))
      return (__FALSE);

    SCU_CLK->CLKSET |= 1;                           /* USB Clock Enable       */
    for (tout=1000; tout>=0; tout--) {              /* Wait                   */
      if (SCU_CLK->CLKSTAT & 1) break;
      if (tout ==  0) return (__FALSE);
      if (tout <= 10) USBH_XMC4500_Delay_ms (10);   /* for max 100 ms         */
    }

    SCU_RESET->PRCLR2 |=  (1 <<  7);                /* USB Peri Reset Clear   */
    SCU_POWER->PWRSET |=  (1 << 16) | (1 << 17);    /* USB PHY En + OTG En    */

    USB0->GRSTCTL |= USB_GRSTCTL_CSftRst_Msk;       /* USB Core Soft Reset    */
    for (tout=1000; tout>=0; tout--) {              /* Wait                   */
      if (!(USB0->GRSTCTL & USB_GRSTCTL_CSftRst_Msk)) break;
      if (tout ==   0) return (__FALSE);
      if (tout <= 100) USBH_XMC4500_Delay_ms (10);  /* for max 1000 ms        */
    }

    USBH_XMC4500_Delay_ms (100);                    /* Wait ~100 ms           */

    /* Core initialization                                                    */
//  USB0->GAHBCFG  |=  USB_GAHBCFG_DMAEn_Msk;       /* Enable buffered DMA    */
    USB0->GAHBCFG  |=  USB_GAHBCFG_GlblIntrMsk_Msk; /* Enable interrupts      */
    USB0->GUSBCFG  |= (USB_GUSBCFG_ForceHstMode_Msk     |   /* Force host mode*/
                      (15 << USB_GUSBCFG_USBTrdTim_Pos) |   /* TrdTim = 15    */
                       USB_GUSBCFG_PHYSel_Msk);             /* Full-speed trns*/
    USBH_XMC4500_Delay_ms (100);                    /* Wait ~100 ms           */

    USB0->GRXFSIZ   =  282;                         /* RxFIFO depth is 1128 by*/
    USB0->GNPTXFSIZ_HOSTMODE = (16 << 16) | (282);  /* Non-periodic TxFIFO    */
    USB0->HPTXFSIZ  = (16 << 16) | (282+16);        /* Periodic TxFIFO mem    */

    USB0->GINTMSK_HOSTMODE  |=  USB_GINTMSK_HOSTMODE_DisconnIntMsk_Msk | /* Enable ints*/
                       USB_GINTMSK_HOSTMODE_HChIntMsk_Msk     | 
                       USB_GINTMSK_HOSTMODE_PrtIntMsk_Msk     | 
                       USB_GINTMSK_HOSTMODE_RxFLvlMsk_Msk     | 
                       USB_GINTMSK_HOSTMODE_SofMsk_Msk        ;
    if (!(USB0->HCFG & 3)) {
      USB0->HCFG   |= (1 << USB_CH_HCFG_FSLSPclkSel_Pos) |  /* PHY clk 48MHz  */
                      (1 << USB_CH_HCFG_FSLSSupp_Pos)    ;  /* FS/LS only     */
    }
    USB0->HAINTMSK  =  0x1FFF;                        /* Enable all ch ints   */

    NVIC_SetPriority (USB0_0_IRQn, 0);  /* Set USB0 interrupt highest priority*/
    NVIC_EnableIRQ   (USB0_0_IRQn);     /* Enable USB0 interrupt              */
  } else {
    NVIC_DisableIRQ  (USB0_0_IRQn);     /* Disable USB0 interrupt             */

    USB0->HAINTMSK  =  0;
    USB0->HPRT      =  0;
    USB0->HCFG      =  0x200;
    USB0->GINTMSK_HOSTMODE   =  0;
    USB0->GUSBCFG   =  0x1440;
    calDelay = 0;                                 /* Force delay recalibrate  */
  }

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_Port_Power ----------------------------
 *
 *  Turn USB Host port power on or off
 *
 *  Parameter:  on:         __TRUE = power on, __FALSE = power off
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_Port_Power (BOOL on) {

  if (on) {
    USB0->HPRT     |=  USB_HPRT_PrtPwr_Msk;       /* Port power on            */
  } else { 
    USB0->HPRT     &= ~USB_HPRT_PrtPwr_Msk;       /* Port power off           */
  }

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_Port_Reset ----------------------------
 *
 *  Reset Port
 *
 *  Parameter:  port:       Root Hub port to be reset (only 0 is available)
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_Port_Reset (U8 port) {
  S32 tout;

  if (!Port_Con)
    return (__FALSE);

  USB0->HPRT |=  USB_HPRT_PrtRst_Msk;                 /* Port reset           */
  USBH_XMC4500_Delay_ms (17);                         /* Wait ~17 ms          */
  USB0->HPRT &= ~USB_HPRT_PrtRst_Msk;                 /* Clear port reset     */

  for (tout=1000; tout>=0; tout--) {                  /* Wait                 */            
    if ((USB0->HPRT & USB_HPRT_PrtEna_Msk)) {         /* If port enabled      */
      USB0->HFIR  = 48000;
      break;
    }
    if (tout ==  0) return (__FALSE);
    if (tout <= 10) USBH_XMC4500_Delay_ms (10);       /* for max 100 ms       */
  }

  USBH_XMC4500_Delay_ms (20);                         /* Wait ~20 ms          */

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_Get_Connect ---------------------------
 *
 *  Returns connect/disconnect port events (also does debouncing)
 *
 *  Parameter:              None
 *  Return:                 Connection/Disconnection events
 *----------------------------------------------------------------------------*/

U32 USBH_XMC4500_Get_Connect (void) {
  U32 ret, stat;

  stat = (USB0->HPRT & USB_HPRT_PrtConnSts_Msk);
  ret  = Port_Discon_Evt;
  ret |= ((stat ^ Port_Con) && (Port_Con));

  if (ret){                             /* If port disconnect occured         */
    Port_Discon_Evt &= ~ret;
    return (ret << 16);
  }

  if (cntDebounce) {
    cntDebounce--;
    if (!cntDebounce) {                 /* If debounce time expired           */
      ret = ((stat ^ Port_Con) && (!Port_Con));
                                        /* If debounce expired on conn 0 -> 1 */
      Port_Con |= ret;
    } else {
      USBH_XMC4500_Delay_ms (1);
    }
  } else if ((stat ^ Port_Con) && (!Port_Con)) {
                                        /* Restart debouncing if connect      */
    cntDebounce = DEBOUNCE_500ms;
  }

  return (ret);
}


/*------------------------- USBH_XMC4500_Get_Speed -----------------------------
 *
 *  Returns port speeds
 *
 *  Parameter:              None
 *  Return:                 Port speeds
 *----------------------------------------------------------------------------*/

U32 USBH_XMC4500_Get_Speed (void) {
  return (Port_Speed); 
}


/*------------------------- USBH_XMC4500_EP_Config -----------------------------
 *
 *  (Re)Configure some parameters of the Endpoint
 *
 *  Parameter:  hndl:       Handle to the Configured Endpoint
 *              dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_EP_Config (U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {
  USBH_XMC4500_CH *ptr_CH;

  if (!hndl) 
    return (__FALSE);

  ptr_CH = (USBH_XMC4500_CH *)(hndl);
  if (!USBH_XMC4500_CH_Disable (ptr_CH)) 
    return (__FALSE);

  /* Fill in all fields of Endpoint Descriptor                                */
  ptr_CH->HCCHAR = (((ptrEPD->wMaxPacketSize       ) & 0x07FF)       <<  0) | 
                   (((ptrEPD->bEndpointAddress     ) & 0x000F)       << 11) | 
                   (((ptrEPD->bEndpointAddress >> 7) & 0x0001)       << 15) | 
                   ((ptrEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK)  << 18) | 
                   ((dev_adr                         & 0x007F)       << 22) ;

  switch (ptrEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK) {
    case USB_ENDPOINT_TYPE_CONTROL:
    case USB_ENDPOINT_TYPE_BULK:
      break;
    case USB_ENDPOINT_TYPE_ISOCHRONOUS:
    case USB_ENDPOINT_TYPE_INTERRUPT:
      cntIntervalMax[USBH_XMC4500_CH_GetIndexFromCH (ptr_CH)] = ptrEPD->bInterval;
      ptr_CH->HCCHAR |= 1 << USB_CH_HCCHAR_MC_EC_Pos;
      break;
  }

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_EP_Add --------------------------------
 *
 *  Add the Endpoint and return handle (address of the Endpoint)
 *
 *  Parameter:  dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 Handle to the Configured Endpoint (0 = FAIL)
 *----------------------------------------------------------------------------*/

U32 USBH_XMC4500_EP_Add (U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {
  USBH_XMC4500_CH *ptr_CH;

  ptr_CH = (USBH_XMC4500_CH *)(USBH_XMC4500_CH_FindFree ());  /* Find free Ch */
  if (!ptr_CH) {                                              /* If no free   */
    return (0);
  }

  if (USBH_XMC4500_EP_Config ((U32)ptr_CH, dev_adr, ep_spd, ptrEPD)) {
    return ((U32)ptr_CH);
  }

  return (0);
}


/*------------------------- USBH_XMC4500_EP_Remove -----------------------------
 *
 *  Remove the Endpoint
 *
 *  Parameter:  hndl:       Handle to the Configured Endpoint
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_EP_Remove (U32 hndl) {
  USBH_XMC4500_CH *ptr_CH;
  USBH_URB        *ptr_URB;
  U32              ch_idx;

  if (!hndl) 
    return (__FALSE);

  ptr_CH = (USBH_XMC4500_CH *)(hndl);
  ch_idx              = USBH_XMC4500_CH_GetIndexFromCH (ptr_CH);
  ptr_URB = CHURB[ch_idx];              /* Pointer to channels URB            */
  if (ptr_URB) {                        /* If URB exists cancel it            */
    if (!USBH_XMC4500_URB_Cancel (hndl, ptr_URB)) 
      return (__FALSE);
  }

  ptr_CH->HCCHAR      = 0;
  ptr_CH->HCINT       = 0;
  ptr_CH->HCINTMSK    = 0;
  ptr_CH->HCTSIZ      = 0;

  cntInterval[ch_idx] = 0;

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_URB_Submit ----------------------------
 *
 *  Submit the URB (USB Request Block) to be processed
 *
 *  Parameter:  hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_URB_Submit (U32 hndl, USBH_URB *ptrURB) {
  USBH_XMC4500_CH *ptr_CH;
  U32              ch_idx;
  U8               resp_type;

  if (!hndl) 
    return (__FALSE);

  if (!ptrURB) 
    return (__FALSE);

  if (!Port_Con)
    return (__FALSE);

  if ((ptrURB->Submitted == 1) || (ptrURB->InProgress == 1)) 
    return (__FALSE);

  resp_type               = ptrURB->ResponsePacketType;

  ptrURB->DataTransferred = 0;
  ptrURB->Status          = 0;

  ptrURB->TimeoutCount    = MAX_TIMEOUT_COUNT;
  ptr_CH                  = (USBH_XMC4500_CH *)(hndl);
  ch_idx                  = USBH_XMC4500_CH_GetIndexFromCH (ptr_CH);
  CHURB[ch_idx]           = ptrURB;

  if (ptr_CH->EPTYP == USB_ENDPOINT_TYPE_INTERRUPT) {
                                        /* If interrupt endpoint transfer     */
    if (resp_type == USBH_PACKET_NAK) {
      cntInterval[ch_idx] = cntIntervalMax[ch_idx];
    } else {
      cntInterval[ch_idx] = 1;          /* Enable transmission on next SOF    */
    }
    ptrURB->Submitted     = 1;
  } else if ((ptr_CH->EPTYP == USB_ENDPOINT_TYPE_CONTROL) || (ptr_CH->EPTYP == USB_ENDPOINT_TYPE_BULK)) {
    if ((Port_Speed & 3) != USBH_LS) {
      HW_Accessing        = __TRUE;
      ptrURB->Submitted   = 1;
      ptrURB->InProgress  = 1;
      USBH_XMC4500_CH_TransferEnqueue (ptr_CH, (U8)ptrURB->Parameters, (U8 *)ptrURB->ptrDataBuffer, (U32)ptrURB->DataLength);
      HW_Accessing        = __FALSE;
    } else {
      ptrURB->Submitted   = 1;
    }
  }

  return (__TRUE);
}


/*------------------------- USBH_XMC4500_URB_Cancel ----------------------------
 *
 *  Cancel the URB (USB Request Block)
 *
 *  Parameter:  hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_XMC4500_URB_Cancel (U32 hndl, USBH_URB *ptrURB) {
  USBH_XMC4500_CH *ptr_CH;
  U32              ch_idx;

  if (!hndl) 
    return (__FALSE);

  if (!ptrURB) 
    return (__FALSE);

  if (!ptrURB->Submitted) 
    return (__TRUE);

  ptrURB->Submitted = 0;

  ptr_CH = (USBH_XMC4500_CH *)(hndl);
  if (ptrURB->InProgress == 1) {
    if (!USBH_XMC4500_CH_Disable (ptr_CH))  /* Stop Endpoint processing       */
      return (__FALSE);
    ptrURB->InProgress  = 0;
  }

  ch_idx = USBH_XMC4500_CH_GetIndexFromCH (ptr_CH);

  if (!cntInterval[ch_idx]) {           /* If interrupt endpoint transfer     */
    cntInterval[ch_idx] = 0;            /* Disable transmission on next SOF   */
  }
  CHURB[ch_idx] = 0;
  ptrURB->Cancelled = 1;

  return (__TRUE);
}


/*------------------------- USB0_0_IRQHandler ----------------------------------
 *
 *  XMC4500 USB Interrupt Handler Routine
 *
 *  Parameter:
 *  Return:
 *----------------------------------------------------------------------------*/

void USB0_0_IRQHandler (void) {
  USBH_XMC4500_CH  *ptr_CH;
  USBH_URB         *ptr_URB;
  U8               *ptrData8;
  U32              *ptrData32;
  U32              *DFIFO;
  U32               gintsts, hprt, haint, hcint, pktcnt, mps;
  U32               grxsts, bcnt, ch, dat, len, len_rest;
  U32               act;

  /* Read global interrupt register                                           */
  gintsts =  USB0->GINTSTS_HOSTMODE & USB0->GINTMSK_HOSTMODE;
  hprt    =  USB0->HPRT;
  haint   =  USB0->HAINT;

  /* Analyze interrupt                                                        */
  if (gintsts & USB_GINTSTS_HOSTMODE_Sof_Msk) {   /* If start of frame int    */
    for (ch = 0; ch < USB0_MAX_CH; ch++) {
      if (CHURB[ch]) {
        if (CHURB[ch]->TimeoutCount) {            /* If timeout not 0         */
          if (!(--CHURB[ch]->TimeoutCount)) {
            CHURB[ch]->Timeout = 1;               /* If timeout expired       */
          }
        }
      }
    }
  }

  if (gintsts&USB_GINTSTS_HOSTMODE_PrtInt_Msk) {  /* If host port interrupt   */
    if (hprt & USB_HPRT_PrtConnDet_Msk) {         /* Port connect detected    */
      if (!Port_Con) {
        cntDebounce = DEBOUNCE_500ms;
      }
    }
    if (hprt & USB_HPRT_PrtEnChng_Msk) {          /* If port enable changed   */
      if (hprt & USB_HPRT_PrtEna_Msk) {           /* If device connected      */
        Port_Speed &= ~3;
        switch ((hprt >> 17) & 3) {
          case 0:
            break;
          case 1:
            Port_Speed |= USBH_FS;
            break;
          case 2:
            break;
          case 3:
            break;
        }
      } 
      if (!(hprt & USB_HPRT_PrtConnSts_Msk) &&    /* If device disconnected   */
            Port_Con) { 
        Port_Discon_Evt = 1;
        Port_Con        = 0;
      }
    }
    hprt &= ~USB_HPRT_PrtEna_Msk;                 /* Leave PENA bit           */
    USB0->HPRT = hprt;                            /* Clear host port interrupt*/
  }
  if ((gintsts & USB_GINTSTS_HOSTMODE_DisconnInt_Msk) &&    /* If disconnected*/
       Port_Con) { 
    Port_Discon_Evt = 1;
    Port_Con        = 0;
  }

  if (Port_Discon_Evt) {                          /* If disconnect detected   */
    ptr_CH = (USBH_XMC4500_CH *)(USB0_CH0);
    for (ch = 0; ch < USB0_MAX_CH; ch++) {
      if (CHURB[ch]) 
        USBH_XMC4500_URB_Cancel ((U32)USBH_XMC4500_CH_GetCHFromIndex (ch), CHURB[ch]);
      ptr_CH++;
    }
  }
                                                  /* Handle reception int     */
  if (gintsts & USB_GINTSTS_HOSTMODE_RxFLvl_Msk) {/* If RXFIFO non-empty int  */
    USB0->GINTMSK_HOSTMODE &= ~USB_GINTMSK_HOSTMODE_RxFLvlMsk_Msk;
    grxsts = USB0->GRXSTSR_HOSTMODE;
    if (((grxsts >> 17) & 0x0F) == 0x02){         /* If PKTSTS = 0x02         */
      grxsts     = (USB0->GRXSTSP_HOSTMODE);
      ch         = (grxsts >> 0) & 0x00F;
      bcnt       = (grxsts >> 4) & 0x7FF;
      ptr_CH     = USBH_XMC4500_CH_GetCHFromIndex (ch);
      DFIFO      = USB0_DFIFO[ch];
      ptr_URB    = CHURB[ch];                     /* Pointer to channels URB  */
      ptrData32  = (U32 *)(ptr_URB->ptrDataBuffer + ptr_URB->DataTransferred);
      len        = bcnt / 4;                      /* Received number of bytes */
      len_rest   = bcnt & 3;                      /* Number of bytes left     */
      while (len--) {
        *ptrData32++    = *DFIFO;
        ptr_URB->DataTransferred += 4;
      }
      if (len_rest) {
        dat      = *DFIFO;
        ptrData8 = (U8 *)ptrData32;
        while (len_rest--) {
          *ptrData8++   = dat;
          dat         >>= 8;
          ptr_URB->DataTransferred ++;
        }
      }
    } else {                                      /* If PKTSTS != 0x02        */
      grxsts     = USB0->GRXSTSP_HOSTMODE;
    }
    USB0->GINTMSK_HOSTMODE |= USB_GINTMSK_HOSTMODE_RxFLvlMsk_Msk;
  }
                                                  /* Handle transmission int  */
  if (gintsts & USB_GINTSTS_HOSTMODE_HChInt_Msk) {/* If host channel interrupt*/
    ptr_CH = (USBH_XMC4500_CH *)(USB0_CH0);
    for (ch = 0; ch < USB0_MAX_CH; ch++) {
      if (haint & (1 << ch)) {                    /* If channels interrupt act*/
        hcint   = ptr_CH->HCINT & ptr_CH->HCINTMSK;
        ptr_URB = CHURB[ch];                      /* Pointer to channels URB  */
        ptr_URB->Error |= ((hcint & 0x3F80) >> 6) | ((hcint & 0x0004) >> 2);
        if ((ptr_URB->PacketType == USBH_PACKET_OUT) ||     /* If OUT packet  ----------*/
            (ptr_URB->PacketType == USBH_PACKET_SETUP)) {   /* or SETUP packet----------*/
          if (hcint & USB_CH_HCINT_XferCompl_Msk){/* If data transfer finished*/
            ptr_CH->HCINTMSK         = 0;
            ptr_URB->DataTransferred = ptr_URB->DataLength;
            ptr_URB->Status          = 0;
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Completed       = 1;
            CHURB[ch]                = 0;
          } else if (hcint & USB_CH_HCINT_STALL_Msk) {      /* If STALL event */
            ptr_URB->ResponsePacketType = USBH_PACKET_STALL;
            ptr_CH->HCINTMSK         = USB_CH_HCINTMSK_ChHltdMsk_Msk;
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USB_CH_HCCHAR_ChDis_Msk;
          } else if ((hcint & USB_CH_HCINT_NAK_Msk)   ||    /* If NAK received*/
                     (hcint & USB_CH_HCINT_XactErr_Msk)) {  /* If TXERR rece  */
            if (hcint & USB_CH_HCINT_NAK_Msk) {
              ptr_URB->ResponsePacketType = USBH_PACKET_NAK;
              ptr_CH->HCINTMSK       = USB_CH_HCINTMSK_ChHltdMsk_Msk;
            } else {
              ptr_URB->Error         = 0x02;
              ptr_CH->HCINTMSK       = USB_CH_HCINTMSK_AckMsk_Msk | USB_CH_HCINTMSK_ChHltdMsk_Msk;
            }
            if (ptr_URB->DataLength) {
                                                  /* Update transfer info     */
              pktcnt                 = (ptr_CH->HCTSIZ >> 19) & 0x3FF;
              mps                    = (ptr_CH->HCCHAR >>  0) & 0x7FF;

              ptr_URB->DataTransferred = (((ptr_URB->DataLength + mps - 1) / mps) - pktcnt) * mps;
            }
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USB_CH_HCCHAR_ChEna_Msk | USB_CH_HCCHAR_ChDis_Msk;
          } else if (hcint&USB_CH_HCINT_ChHltd_Msk) { /* If channel halted    */
            if ((ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT)   && 
                (ptr_URB->ResponsePacketType == USBH_PACKET_NAK) && 
                (CHURB[ch]->NAKRetries--)) {
                                                  /* Reenable channel         */
              USBH_XMC4500_CH_TransferEnqueue (ptr_CH, (U8)CHURB[ch]->Parameters, (U8 *)CHURB[ch]->ptrDataBuffer+CHURB[ch]->DataTransferred, (U32)CHURB[ch]->DataLength-CHURB[ch]->DataTransferred);
              CHURB[ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
              ptr_CH->HCINTMSK       = 0;
              ptr_URB->Submitted     = 0;
              ptr_URB->InProgress    = 0;
              ptr_URB->Completed     = 1;
              CHURB[ch]              = 0;
            }
          } else if (hcint&USB_CH_HCINT_ACK_Msk) {/* If ACK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            ptr_CH->HCINTMSK        &=~USB_CH_HCINTMSK_AckMsk_Msk;
          }
        } else if (ptr_URB->PacketType == USBH_PACKET_IN) { /* If IN packet   ----------*/
          if (hcint & USB_CH_HCINT_XferCompl_Msk) {
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            ptr_CH->HCINTMSK         = USB_CH_HCINTMSK_ChHltdMsk_Msk;
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USB_CH_HCCHAR_ChEna_Msk | USB_CH_HCCHAR_ChDis_Msk;
          } else if (hcint&USB_CH_HCINT_NAK_Msk) {/* If NAK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_NAK;
            if ((Port_Con) && (ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT) && (CHURB[ch]->NAKRetries--)) {
                                                  /* Reenable channel         */
              ptr_CH->HCINTMSK       = USB_CH_HCINTMSK_DataTglErrMsk_Msk | 
                                       USB_CH_HCINTMSK_BblErrMsk_Msk     | 
                                       USB_CH_HCINTMSK_XactErrMsk_Msk    | 
                                       USB_CH_HCINTMSK_AckMsk_Msk        | 
                                       USB_CH_HCINTMSK_NakMsk_Msk        | 
                                       USB_CH_HCINTMSK_StallMsk_Msk      | 
                                       USB_CH_HCINTMSK_XferComplMsk_Msk  ;
              ptr_CH->HCCHAR        |= USB_CH_HCCHAR_ChEna_Msk;
              CHURB[ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
                                                  /* Halt the channel         */
              ptr_CH->HCINTMSK       = USB_CH_HCINTMSK_ChHltdMsk_Msk;
              ptr_CH->HCCHAR        |= USB_CH_HCCHAR_ChEna_Msk | USB_CH_HCCHAR_ChDis_Msk;
            }
          } else if ((hcint & USB_CH_HCINT_XactErr_Msk) ||  /* If TXERR event */
                     (hcint & USB_CH_HCINT_BblErr_Msk)  ||  /* If BBERR event */
                     (hcint & USB_CH_HCINT_STALL_Msk))   {  /* If STALL event */
            ptr_CH->HCINTMSK         = USB_CH_HCINTMSK_ChHltdMsk_Msk;
            if (hcint & USB_CH_HCINT_XactErr_Msk) {
              ptr_URB->Error         = 0x02;
              ptr_CH->HCINTMSK      |= USB_CH_HCINTMSK_AckMsk_Msk;
            } else if (hcint & USB_CH_HCINT_BblErr_Msk) {
              ptr_URB->Error         = 0x04;
            } else {
              ptr_URB->ResponsePacketType = USBH_PACKET_STALL;
            }
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USB_CH_HCCHAR_ChEna_Msk | USB_CH_HCCHAR_ChDis_Msk;
          } else if (hcint&USB_CH_HCINT_ChHltd_Msk) { /* If Channel Halted    */
            ptr_CH->HCINTMSK         = 0;
            ptr_URB->Submitted       = 0;
            ptr_URB->InProgress      = 0;
            ptr_URB->Completed       = 1;
            CHURB[ch]                = 0;
            if (ptr_URB->Completed && ptr_URB->CompletedCallback) {
              ptr_URB->CompletedCallback();
            }
          } else if (hcint&USB_CH_HCINT_ACK_Msk) {/* If ACK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            if (ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT) {
              ptr_CH->HCINTMSK       = USB_CH_HCINTMSK_DataTglErrMsk_Msk | 
                                       USB_CH_HCINTMSK_BblErrMsk_Msk     | 
                                       USB_CH_HCINTMSK_XactErrMsk_Msk    | 
                                       USB_CH_HCINTMSK_AckMsk_Msk        | 
                                       USB_CH_HCINTMSK_NakMsk_Msk        | 
                                       USB_CH_HCINTMSK_StallMsk_Msk      | 
                                       USB_CH_HCINTMSK_XferComplMsk_Msk  ;
                                                  /* Reenable channel         */
              ptr_CH->HCCHAR        |= USB_CH_HCCHAR_ChEna_Msk;
              CHURB[ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
              ptr_CH->HCINTMSK       = USB_CH_HCINTMSK_ChHltdMsk_Msk;
                                                  /* Halt the channel         */
              ptr_CH->HCCHAR        |= USB_CH_HCCHAR_ChEna_Msk | USB_CH_HCCHAR_ChDis_Msk;
            }
          } else if (hcint&USB_CH_HCINT_DataTglErr_Msk) { /* If DTERR received*/
          }
        }
        ptr_CH->HCINT = 0x3FFF;
      }
      ptr_CH++;
    }
  }  
  USB0->GINTSTS_HOSTMODE = gintsts;               /* Clear core interrupts    */
  if (gintsts & USB_GINTSTS_HOSTMODE_Sof_Msk) {   /* If start of frame int    */
    if (HW_Accessing) {
      act = 1;
    } else {
      act = 0;
      for (ch = 0; ch < USB0_MAX_CH; ch++) {
        if (CHURB[ch]) {
          if (CHURB[ch]->InProgress == 1) {       /* If any URB in progress   */
            act = 1;                              /* Set act to 1             */
            break;
          }
        }
      }
    }

    /* At this point act == 1 if there is USB bus activity                    */
    ptr_CH = (USBH_XMC4500_CH *)(USB0_CH0);
    for (ch = 0; ch < USB0_MAX_CH; ch++) {
      ptr_URB = CHURB[ch];                        /* Pointer to channels URB  */
      if (ptr_URB) {
        if ((ptr_URB->Submitted  == 1) &&         /* If URB is submitted      */
            (ptr_URB->InProgress == 0)) {         /* If URB not in progress   */
          if (ptr_CH->EPTYP == USB_ENDPOINT_TYPE_INTERRUPT) {
            if (cntInterval[ch]) {
              if ((act && (cntInterval[ch] > 1)) || !act)
                cntInterval[ch]--;
              }
              if (!act) {
                if (!cntInterval[ch]) {           /* If period expired        */
                  ptr_URB->InProgress = 1;
                  USBH_XMC4500_CH_TransferEnqueue (ptr_CH, (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrDataBuffer, (U32)ptr_URB->DataLength);
                  act = 1;
                }
              }
          } else if (!act) {
            ptr_URB->InProgress = 1;
            USBH_XMC4500_CH_TransferEnqueue (ptr_CH, (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrDataBuffer, (U32)ptr_URB->DataLength);
            act = 1;
          }
        }
      }
      ptr_CH++;
    }
  }
}
