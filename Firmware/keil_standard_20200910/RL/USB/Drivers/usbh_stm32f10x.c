/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_stm32f10x.c
 *      Purpose: Full/Low-speed Host STM32F10x Driver module
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include "usbh_stm32f10x.h"
#include <stm32f10x_cl.h>
#include <string.h>


/************************** Host Controller Driver Structure ******************/

USBH_HCD usbh0_hcd = {                  /* Host Controller Driver structure   */
  USBH_STM32_Get_Capabilities,          /* Get Host Ctrl Driver capabilities  */
  USBH_STM32_Delay_ms,                  /* Delay in ms                        */
  USBH_STM32_Pins_Config,               /* Config/Unconfig pins               */
  USBH_STM32_Init,                      /* Init/Uninit Host Controller        */
  USBH_STM32_Port_Power,                /* On/Off Port Power                  */
  USBH_STM32_Port_Reset,                /* Reset port                         */
  USBH_STM32_Get_Connect,               /* Get port conn/disconn status       */
  USBH_STM32_Get_Speed,                 /* Get port enumerated speed          */
  USBH_STM32_EP_Add,                    /* Add Endpoint                       */
  USBH_STM32_EP_Config,                 /* (Re)Configure Endpoint             */
  USBH_STM32_EP_Remove,                 /* Remove Endpoint                    */
  USBH_STM32_URB_Submit,                /* Submit USB Block Request           */
  USBH_STM32_URB_Cancel                 /* Cancel USB Block Request           */
};


/************************** Driver Settings ***********************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

//  <o0> Size of memory used by the OTG_FS USB Host Controller <1-1048576>
//  <i> This is a size of memory (in bytes) that the USB Host Controller 
//  <i> will use for USB communication data.
#define USBH_STM32_SZ_MPOOL         0x00000234


/************************** Variable and Memory Definitons ********************/

#define MAX_TIMEOUT_COUNT         100
#define DEBOUNCE_500ms            500

#define OTG_MAX_CH                8

#define OTG                       OTG_FS

static U32 *OTG_DFIFO[OTG_MAX_CH] = { OTG_FS_DFIFO0, OTG_FS_DFIFO1, 
                                      OTG_FS_DFIFO2, OTG_FS_DFIFO3, 
                                      OTG_FS_DFIFO4, OTG_FS_DFIFO5, 
                                      OTG_FS_DFIFO6, OTG_FS_DFIFO7  };

/* Reserve memory for memory pool */ 
       U32       USBH_MPOOL[(USBH_STM32_SZ_MPOOL+3)>>2];

static BOOL      HW_Accessing                      = { __FALSE };
static USBH_URB *CHURB             [OTG_MAX_CH]    = { 0 };
static U8        cntInterval       [OTG_MAX_CH]    = { 0 };
static U8        cntIntervalMax    [OTG_MAX_CH]    = { 0 };
static U16       cntDebounce                       = { 0 };
static U32       calDelay                          =   0  ;
static U32       Port_Discon_Evt                   = { 0 };
static U32       Port_Speed                        = { 0 };
static U32       Port_Con                          = { 0 };


/************************** Local Module Functions ****************************/

/***----------------------- DMA Functions ----------------------------------***/

/*------------------------- USBH_STM32_DMA_Enable ------------------------------
 *
 *  Enable the DMA so it can be used for USB
 *
 *  Parameter:              None
 *  Return:                 None
 *----------------------------------------------------------------------------*/

__inline static void USBH_STM32_DMA_Enable (void) {
  RCC->AHBENR  |=  1;                             /* Enable DMA1 clock, DMA1  */
                                                  /* will be used to load     */
                                                  /* TX FIFO                  */
}


/*------------------------- USBH_STM32_DMA_Disable -----------------------------
 *
 *  Disable the DMA previoucly used for USB
 *
 *  Parameter:              None
 *  Return:                 None
 *----------------------------------------------------------------------------*/
#if 0
__inline static void USBH_STM32_DMA_Disable (void) {
  RCC->AHBENR  &= ~1;                             /* Disable DMA1 clock       */
}
#endif


/*------------------------- USBH_STM32_DMA_Stop --------------------------------
 *
 *  Stop the DMA transfer
 *
 *  Parameter:              None
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

__inline static BOOL USBH_STM32_DMA_Stop (void) {
  S32 tout;

  DMA1_CH1->CCR = 0;                              /* Stop the DMA transfer    */
  for (tout = 1010; tout >= 0; tout--) {          /* Wait max 100 ms          */
    if (!(DMA1_CH1->CCR & 1))                     /* Wait for DMA to disable  */
      break;
    if (!tout) 
      return (__FALSE);
    if (tout <= 10)
      USBH_STM32_Delay_ms (10);
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_DMA_Start -------------------------------
 *
 *  Start the DMA transfer
 *
 *  Parameter:  ptrDest:    Pointer to Destination addres of transfer
 *              ptrSrc:     Pointer to Source address of transfer
 *              len:        Number of bytes to transfer
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

__inline static BOOL USBH_STM32_DMA_Start (U32 *ptrDest, U32 *ptrSrc, U16 len) {

  DMA1->IFCR      =  0x0F;                        /* Clear DMA interrupts     */
  DMA1_CH1->CMAR  = (uint32_t)ptrSrc;             /* Load source address      */
  DMA1_CH1->CPAR  = (uint32_t)ptrDest;            /* Load destination address */
  DMA1_CH1->CNDTR = (len+3)/4;                    /* Set size of transfer     */
  DMA1_CH1->CCR   = (1 << 14) |                   /* MEM2MEM enable           */
                    (0 << 12) |                   /* Priority Level low (PL=0)*/
                    (2 << 10) |                   /* Memory size 32 bits      */
                    (2 <<  8) |                   /* Peripheral size 32 bits  */
                    (1 <<  7) |                   /* Memory increment enable  */
                    (1 <<  6) |                   /* Peripheral increment en  */
                    (0 <<  5) |                   /* CIRC = 0                 */
                    (1 <<  4) |                   /* Direction from memory    */
                    (0 <<  3) |                   /* TEIE = 0                 */
                    (0 <<  2) |                   /* HTIE = 0                 */
                    (0 <<  1) ;                   /* TCIE = 0                 */
  DMA1_CH1->CCR  |= (1 <<  0) ;                   /* Channel enable           */

  return (__TRUE);
}


/*------------------------- USBH_STM32_DMA_Wait --------------------------------
 *
 *  Wait for the DMA transfer to finish and cleanup DMA for next transfer 
 *
 *  Parameter:              None
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

__inline static BOOL USBH_STM32_DMA_Wait (void) {
  S32 tout;

  for (tout = 1010; tout >= 0; tout--) {          /* Wait max 100 ms          */
    if (!(DMA1_CH1->CCR&1) || !(DMA1_CH1->CNDTR)) /* If prev trans completed  */
      break;
    if (!tout) 
      return (__FALSE);
    if (tout <= 10)
      USBH_STM32_Delay_ms (10);
  }
  if (!USBH_STM32_DMA_Stop())                     /* Stop the DMA transfer    */
    return (__FALSE);

  return (__TRUE);
}


/***----------------------- Channel Functions ------------------------------***/

/*------------------------- USBH_STM32_CH_GetIndexFromCH -----------------------
 *
 *  Get the Index of Channel from it's Address
 *
 *  Parameter:  ptrCH:      Pointer to the Channel
 *  Return:                 Index of the Channel
 *----------------------------------------------------------------------------*/

static U32 USBH_STM32_CH_GetIndexFromCH (USBH_STM32_CH *ptrCH) {
  return (ptrCH - (USBH_STM32_CH *)(&(OTG->HCCHAR0)));
}


/*------------------------- USBH_STM32_CH_GetCHFromIndex -----------------------
 *
 *  Get the Channel Address from it's Index
 *
 *  Parameter:  idx:        Index of the Channel
 *  Return:                 Address of the Channel
 *----------------------------------------------------------------------------*/

static USBH_STM32_CH *USBH_STM32_CH_GetCHFromIndex (U32 idx) {
  return ((USBH_STM32_CH *)(&(OTG->HCCHAR0)) + idx);
}


/*------------------------- USBH_STM32_CH_FindFree -----------------------------
 *
 *  Find a free Channel
 *
 *  Parameter:
 *  Return:                 Pointer to the first free Channel
 *                          (0 = no free Channel is available)
 *----------------------------------------------------------------------------*/

static void *USBH_STM32_CH_FindFree (void) {
  USBH_STM32_CH *ptr_CH;
  U32            i;

  ptr_CH = (USBH_STM32_CH *)(&(OTG->HCCHAR0));

  for (i = 0; i < OTG_MAX_CH; i++) {
    if (!ptr_CH->HCCHAR) {
      return (ptr_CH);
    }
    ptr_CH++;
  }

  return (0);
}


/*------------------------- USBH_STM32_CH_Disable ------------------------------
 *
 *  Disable the Channel
 *
 *  Parameter:  ptrCH:      Pointer to the Channel
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_STM32_CH_Disable (USBH_STM32_CH *ptrCH) {
  S32 tout;

  if (!ptrCH) 
    return (__FALSE);

  ptrCH->HCINTMSK  =  0;
  USBH_STM32_Delay_ms (2);
  if (ptrCH->HCCHAR & USBH_STM32_HCCHAR_CHENA) {
    if (!(ptrCH->HCCHAR & USBH_STM32_HCCHAR_EPDIR) && (ptrCH->HCTSIZ)) {
      if (!USBH_STM32_DMA_Stop())
        return (__FALSE);
    }
    ptrCH->HCINT     =  ~USBH_STM32_HCINT_CHH;
    ptrCH->HCCHAR    =  (ptrCH->HCCHAR  |  USBH_STM32_HCCHAR_CHENA);
    USBH_STM32_Delay_ms (2);
    ptrCH->HCCHAR    =  (ptrCH->HCCHAR  & ~USBH_STM32_HCCHAR_CHENA) | USBH_STM32_HCCHAR_CHDIS;
    for (tout = 1010; tout >= 0; tout--) {        /* Wait max 100 ms          */
      if (ptrCH->HCINT & USBH_STM32_HCINT_CHH) 
        break;
      if ((ptrCH->HCCHAR & (USBH_STM32_HCCHAR_CHENA | USBH_STM32_HCCHAR_CHDIS)) == (USBH_STM32_HCCHAR_CHENA | USBH_STM32_HCCHAR_CHDIS)) 
        break;
      if (!tout) {
        return (__FALSE);
      }
      if (tout <= 10)
        USBH_STM32_Delay_ms (10);
    }
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_CH_TransferEnqueue ----------------------
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

static BOOL USBH_STM32_CH_TransferEnqueue (USBH_STM32_CH *ptrCH, U32 tgl_typ, U8 *buf, U32 len) {
  U32  hcchar;
  U32  hctsiz;
  U32  hcintmsk;
  U32  mpsiz;
  U32  ch_idx;
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
  hcchar &= USBH_STM32_HCCHAR_ODDFRM|             /* Keep ODDFRM              */
            USBH_STM32_HCCHAR_DAD   |             /* Keep DAD                 */
            USBH_STM32_HCCHAR_MCNT  |             /* Keep MCNT                */
            USBH_STM32_HCCHAR_EPTYP |             /* Keep EPTYP               */
            USBH_STM32_HCCHAR_LSDEV |             /* Keep LSDEV               */
            USBH_STM32_HCCHAR_EPNUM |             /* Keep EPNUM               */
            USBH_STM32_HCCHAR_MPSIZ ;             /* Keep MPSIZ               */
  switch (tgl_typ & 0x0F) {
    case USBH_PACKET_IN:
      hcchar   |=  USBH_STM32_HCCHAR_EPDIR;
      hcintmsk  =  USBH_STM32_HCINTMSK_DTERRM | 
                   USBH_STM32_HCINTMSK_BBERRM | 
                   USBH_STM32_HCINTMSK_TXERRM | 
                   USBH_STM32_HCINTMSK_ACKM   | 
                   USBH_STM32_HCINTMSK_NAKM   | 
                   USBH_STM32_HCINTMSK_STALLM | 
                   USBH_STM32_HCINTMSK_XFRCM  ;
      break;
    case USBH_PACKET_OUT:
      hcchar   &= ~USBH_STM32_HCCHAR_EPDIR;
      hcintmsk  =  USBH_STM32_HCINTMSK_TXERRM | 
                   USBH_STM32_HCINTMSK_NYET   | 
                   USBH_STM32_HCINTMSK_NAKM   | 
                   USBH_STM32_HCINTMSK_STALLM | 
                   USBH_STM32_HCINTMSK_XFRCM  ;
      break;
    case USBH_PACKET_SETUP:
      hcchar   &= ~USBH_STM32_HCCHAR_EPDIR;
      hcintmsk  =  USBH_STM32_HCINTMSK_TXERRM | 
                   USBH_STM32_HCINTMSK_NAKM   | 
                   USBH_STM32_HCINTMSK_STALLM | 
                   USBH_STM32_HCINTMSK_XFRCM  ;
      break;
  }
  hcchar &= ~USBH_STM32_HCCHAR_CHDIS;
  hcchar |=  USBH_STM32_HCCHAR_CHENA;

                                                  /* Prepare HCTSIZ register  */
  hctsiz &= USBH_STM32_HCTSIZ_DPID;               /* Keep DPID                */
  if ((tgl_typ & 0x0F) == USBH_PACKET_SETUP) {    /* If setup pckt DPID=MDATA */
    hctsiz &= ~USBH_STM32_HCTSIZ_DPID;
    hctsiz |=  USBH_STM32_HCTSIZ_DPID_MDATA;
  } else if ((tgl_typ >> 5) & 1) {                /* If toggle force bit activ*/
    if ((tgl_typ >> 4) & 1) {                     /* Toggle bit value         */
      hctsiz &= ~USBH_STM32_HCTSIZ_DPID;
      hctsiz |=  USBH_STM32_HCTSIZ_DPID_DATA1;
    } else {
      hctsiz &= ~USBH_STM32_HCTSIZ_DPID;
      hctsiz |=  USBH_STM32_HCTSIZ_DPID_DATA0;
    }
  }

  mpsiz = hcchar & 0x7FF;                         /* Maximum packet size      */
  if (len) {                                      /* Normal packet            */
    hctsiz |= ((len+mpsiz-1) / mpsiz) << 19;      /* Prepare PKTCNT field     */
    hctsiz |= ( len                 ) <<  0;      /* Prepare XFRSIZ field     */
  } else {                                        /* Zero length packet       */
    hctsiz |= ( 1                   ) << 19;      /* Prepare PKTCNT field     */
    hctsiz |= ( 0                   ) <<  0;      /* Prepare XFRSIZ field     */
  }

  ch_idx  = USBH_STM32_CH_GetIndexFromCH (ptrCH);

  ptrCH->HCINTMSK = hcintmsk;                     /* Enable channel interrupts*/
  ptrCH->HCTSIZ   = hctsiz;                       /* Write ch transfer size   */

  /* load_data == __TRUE if there is data to be loaded to FIFO 
    (If packet is OUT or SETUP and len > 0)                                   */
  load_data = (((tgl_typ & 0x0F) == USBH_PACKET_OUT)    || 
               ((tgl_typ & 0x0F) == USBH_PACKET_SETUP)) && 
                 len; 

  if (load_data)
    USBH_STM32_DMA_Wait();

  ptrCH->HCCHAR = hcchar;                         /* Write ch characteristics */

  if (load_data)
    USBH_STM32_DMA_Start (OTG_DFIFO[ch_idx], (U32 *)(buf), len);

  return (__TRUE);
}


/************************** Module Functions **********************************/

/*------------------------- USBH_STM32_Get_Capabilities ------------------------
 *
 *  Get capabilities of Host Controller Driver
 *
 *  Parameter:  cap:        Pointer to USBH_HCI_CAP structure where 
 *                          capabilities are loaded
 *  Return:
 *----------------------------------------------------------------------------*/

void USBH_STM32_Get_Capabilities (USBH_HCI_CAP *cap) {
  cap->MultiPckt = __TRUE;
  cap->MaxDataSz = 512;
  cap->CtrlNAKs  = 100000;
  cap->BulkNAKs  = 1000000;
}


/*------------------------- USBH_STM32_Delay_ms --------------------------------
 *
 *  Delay execution (in milliseconds)
 *  Calibration is done if global variable calDelay is 0
 *
 *  Parameter:  ms:         Number of milliseconds to delay execution for
 *  Return:
 *----------------------------------------------------------------------------*/

void USBH_STM32_Delay_ms (U32 ms) {
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
    calDelay = SystemFrequency / (vals - vale);   /* Calibrated value         */
    if (vals == 0xFFFFFF)               /* Stop timer if we started it        */
      SysTick->CTRL  = 0;
    goto start;
  }
}


/*------------------------- USBH_STM32_Pins_Config -----------------------------
 *
 *  Configurate or unconfigurate pins used by the USB Host
 *
 *  Parameter:  on:         __TRUE = configurate, __FALSE = unconfigurate
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Pins_Config (BOOL on) {

  if (on) {
    RCC->APB2ENR  |=  (   1 <<  4);     /* Enable GPIOC port clock            */
    GPIOC->BSRR    =  (   1 <<  9);     /* PC9 drive high                     */
    GPIOC->CRH    &= ~(0x0F <<  4);     /* Clear PC9 configuration            */
    GPIOC->CRH    |=  (   1 <<  6)|     /* PC9 general purpose open-drain     */
                      (   1 <<  4);     /* PC9 output mode (10 MHz)           */
    USBH_STM32_Port_Power ( __FALSE);   /* Turn off port power                */
  } else {
    GPIOC->CRH    &= ~(0x0F <<  4);     /* Clear PC9 configuration            */
    GPIOC->CRH    |=  (0x04 <<  4);     /* Float Input (reset state)          */
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_Init ------------------------------------
 *
 *  Initialize or uninitialize the USB Host Controller
 *
 *  Parameter:  on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Init (BOOL on) {
  S32 tout;

  if (on) {
    /* Initialize memory pool for data                                        */
    if (!usbh_mem_init(0, (U32 *)&USBH_MPOOL, USBH_STM32_SZ_MPOOL))
      return (__FALSE);

    RCC->AHBENR     |=  (   1 << 12);   /* OTG FS clock enable                */

    OTG->GRSTCTL |= 1;                            /* USB Core Reset           */
    for (tout = 100; tout >= 0; tout--) {         /* Wait max 1 second        */
      if (!(OTG->GRSTCTL & 1))                    /* Wait for Core Reset end  */
        break;
      if (!tout) {
        return (__FALSE);
      }
      USBH_STM32_Delay_ms (10);
    }
    USBH_STM32_DMA_Enable ();

    USBH_STM32_Delay_ms (100);                    /* Wait ~100 ms             */

    /* Core initialization                                                    */
    OTG->GAHBCFG   |=  USBH_STM32_GAHBCFG_GINTMSK;    /* Enable interrupts    */
    OTG->GUSBCFG   &=~(USBH_STM32_GUSBCFG_FHMOD  |    /* Clear register       */
                       USBH_STM32_GUSBCFG_TRDT(15)|
                       USBH_STM32_GUSBCFG_PHYSEL);
    OTG->GUSBCFG    =  USBH_STM32_GUSBCFG_FHMOD  |    /* Force host mode      */
                       USBH_STM32_GUSBCFG_PHYSEL |    /* Full-spd transceiver */
                       USBH_STM32_GUSBCFG_TRDT(5);    /* Turnaround time      */
    USBH_STM32_Delay_ms (100);                    /* Wait ~100 ms             */

    OTG->GRXFSIZ    =  (512/4) +                    /* RxFIFO depth is 512 by */
                             2  +                   /* 8 bytes for Int EP     */
                             4  ;                   /* Packet info and status */
    OTG->GNPTXFSIZ  = ((512/4)<<16) | ((512/4)+6);    /* Non-periodic TxFIFO  */
    OTG->HPTXFSIZ   = (16<<16) |  (((512/4)*2)+6);    /* Periodic TxFIFO mem  */
    OTG->GCCFG     |=  USBH_STM32_GCCFG_SOFOUTEN |    /* Enable SOF output    */
                       USBH_STM32_GCCFG_PWRDWN   ;    /* Disable power down   */

    OTG->GINTMSK   |=  USBH_STM32_GINTMSK_DISCINT|    /* Enable disconn int   */
                       USBH_STM32_GINTMSK_HCIM   |    /* Enable host ch int   */
                       USBH_STM32_GINTMSK_PRTIM  |    /* Enable host prt int  */
                       USBH_STM32_GINTMSK_RXFLVLM|    /* Enable RXFIFO int    */
                       USBH_STM32_GINTMSK_SOFM   ;    /* Enable SOF int       */
    if (!(OTG->HCFG & 3)) {
      OTG->HCFG     =  USBH_STM32_HCFG_FSLSPCS(1)|    /* PHY clk at 48MHz     */
                       USBH_STM32_HCFG_FSLS(1)   ;    /* FS/LS only           */
    }
    OTG->HAINTMSK   =  0xFF;                          /* Enable all ch ints   */

    NVIC_SetPriority (OTG_FS_IRQn, 0);  /* Set OTG interrupt highest priority */
    NVIC_EnableIRQ   (OTG_FS_IRQn);     /* Enable OTG interrupt               */
  } else {
    NVIC_DisableIRQ  (OTG_FS_IRQn);     /* Disable OTG interrupt              */

    OTG->HAINTMSK  &= ~0xFF;            /* Disable channel interrupts         */
    RCC->AHBSTR    |=  (   1 << 12);    /* OTG FS reset                       */
    RCC->AHBSTR    &= ~(   1 << 12);    /* OTG FS not reset                   */
    RCC->AHBENR    &= ~(   1 << 12);    /* OTG FS clock disable               */

    OTG->HPRT       =  0;                           /* Reset host port control*/
    OTG->HCFG       =  0;                           /* Reset host config      */
    OTG->GINTMSK   &=~(USBH_STM32_GINTMSK_DISCINT|  /* Disable interrupts     */
                       USBH_STM32_GINTMSK_HCIM   |
                       USBH_STM32_GINTMSK_PRTIM  |
                       USBH_STM32_GINTMSK_RXFLVLM|
                       USBH_STM32_GINTMSK_SOFM  );
    OTG->GUSBCFG   &=~(USBH_STM32_GUSBCFG_FHMOD  |  /* Reset USB config fields*/
                       USBH_STM32_GUSBCFG_PHYSEL | 
                       USBH_STM32_GUSBCFG_TRDT(15));
    OTG->GUSBCFG   |=  USBH_STM32_GUSBCFG_TRDT(2);  /* Set init values        */
    calDelay = 0;                                 /* Force delay recalibrate  */
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_Port_Power ------------------------------
 *
 *  Turn USB Host port power on or off
 *
 *  Parameter:  on:         __TRUE = power on, __FALSE = power off
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Port_Power (BOOL on) {

  if (on) {
    OTG->HPRT    |=  USBH_STM32_HPRT_PPWR;        /* Port power on            */
    GPIOC->BRR    =  (   1 <<  9);                /* PC9 drive low            */
  } else { 
    GPIOC->BSRR   =  (   1 <<  9);                /* PC9 drive high           */
    OTG->HPRT    &= ~USBH_STM32_HPRT_PPWR;        /* Port power off           */
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_Port_Reset ------------------------------
 *
 *  Reset Port
 *
 *  Parameter:  port:       Root Hub port to be reset (only 0 is available)
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Port_Reset (U8 port) {
  U32 hcfg, hprt;
  S32 tout;

  if (!Port_Con)
    return (__FALSE);

  hcfg = OTG->HCFG;
  hprt = OTG->HPRT;
  switch ((hprt >> 17) & 3) {
    case 0:                             /* High-speed detected                */
      break;
    case 1:                             /* Full-speed detected                */
      OTG->HFIR  = 48000;
      if ((hcfg & 3) != 1) {
        hcfg       = (hcfg & ~USBH_STM32_HCFG_FSLSPCS(3)) | USBH_STM32_HCFG_FSLSPCS(1);
        OTG->HCFG  =  hcfg;
      }
      break;
    case 2:                             /* Low-speed detected                 */
      OTG->HFIR  = 6000;
      if ((hcfg & 3) != 2) {
        hcfg       = (hcfg & ~USBH_STM32_HCFG_FSLSPCS(3)) | USBH_STM32_HCFG_FSLSPCS(2);
        OTG->HCFG  =  hcfg;
      }
      break;
    case 3:
      break;
  }

  OTG->HPRT |=  USBH_STM32_HPRT_PRST;                 /* Port reset           */
  USBH_STM32_Delay_ms (17);                           /* Wait ~17 ms          */
  OTG->HPRT &= ~USBH_STM32_HPRT_PRST;                 /* Clear port reset     */

  for (tout = 10100; tout >= 0; tout--) {             /* Wait for max 1 s     */
    if ((OTG->HPRT & USBH_STM32_HPRT_PENA))           /* If port enabled      */
      break;
    if (!tout) 
      return (__FALSE);
    if (tout <= 100)
      USBH_STM32_Delay_ms (10);                       /* Wait ~10 ms          */
  }
  USBH_STM32_Delay_ms (20);                           /* Wait ~20 ms          */

  return (__TRUE);
}


/*------------------------- USBH_STM32_Get_Connect -----------------------------
 *
 *  Returns connect/disconnect port events (also does debouncing)
 *
 *  Parameter:              None
 *  Return:                 Connection/Disconnection events
 *----------------------------------------------------------------------------*/

U32 USBH_STM32_Get_Connect (void) {
  U32 ret, stat;

  stat = (OTG->HPRT & USBH_STM32_HPRT_PCSTS);
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
      USBH_STM32_Delay_ms (1);
    }
  } else if ((stat ^ Port_Con) && (!Port_Con)) {
                                        /* Restart debouncing if connect      */
    cntDebounce = DEBOUNCE_500ms;
  }

  return (ret);
}


/*------------------------- USBH_STM32_Get_Speed -------------------------------
 *
 *  Returns port speeds
 *
 *  Parameter:              None
 *  Return:                 Port speeds
 *----------------------------------------------------------------------------*/

U32 USBH_STM32_Get_Speed (void) {
  return (Port_Speed); 
}


/*------------------------- USBH_STM32_EP_Add ----------------------------------
 *
 *  Add the Endpoint and return handle (address of the Endpoint)
 *
 *  Parameter:  dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 Handle to the Configured Endpoint (0 = FAIL)
 *----------------------------------------------------------------------------*/

U32 USBH_STM32_EP_Add (U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {
  USBH_STM32_CH *ptr_CH;

  ptr_CH = (USBH_STM32_CH *)(USBH_STM32_CH_FindFree ());  /* Find free Channel*/
  if (!ptr_CH) {                                          /* If no free exists */
    return (0);
  }

  /* Fill in all fields from the USB Standard Endpoint Descriptor             */
  ptr_CH->HCCHAR = (((ptrEPD->wMaxPacketSize       ) & 0x07FF)       <<  0) | 
                   (((ptrEPD->bEndpointAddress     ) & 0x000F)       << 11) | 
                   (((ptrEPD->bEndpointAddress >> 7) & 0x0001)       << 15) | 
                   ((ep_spd == USBH_LS)                              << 17) |
                   ((ptrEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK)  << 18) | 
                   ((dev_adr                         & 0x007F)       << 22) ;

  switch (ptrEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK) {
    case USB_ENDPOINT_TYPE_CONTROL:
    case USB_ENDPOINT_TYPE_BULK:
      break;
    case USB_ENDPOINT_TYPE_ISOCHRONOUS:
    case USB_ENDPOINT_TYPE_INTERRUPT:
      cntIntervalMax[USBH_STM32_CH_GetIndexFromCH (ptr_CH)] = ptrEPD->bInterval;
      ptr_CH->HCCHAR |= USBH_STM32_HCCHAR_MCNT1;
      break;
  }

  return ((U32)ptr_CH);
}


/*------------------------- USBH_STM32_EP_Config -------------------------------
 *
 *  (Re)Configure some parameters of the Endpoint
 *
 *  Parameter:  hndl:       Handle to the Configured Endpoint
 *              dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_EP_Config (U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {
  USBH_STM32_CH *ptr_CH;

  if (!hndl) 
    return (__FALSE);

  ptr_CH = (USBH_STM32_CH *)(hndl);
  if (!USBH_STM32_CH_Disable (ptr_CH)) 
    return (__FALSE);

  /* Fill in all fields of Endpoint Descriptor                                */
  ptr_CH->HCCHAR = (((ptrEPD->wMaxPacketSize       ) & 0x07FF)       <<  0) | 
                   (((ptrEPD->bEndpointAddress     ) & 0x000F)       << 11) | 
                   (((ptrEPD->bEndpointAddress >> 7) & 0x0001)       << 15) | 
                   ((ep_spd == USBH_LS)                              << 17) |
                   ((ptrEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK)  << 18) | 
                   ((dev_adr                         & 0x007F)       << 22) ;

  switch (ptrEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK) {
    case USB_ENDPOINT_TYPE_CONTROL:
    case USB_ENDPOINT_TYPE_BULK:
      break;
    case USB_ENDPOINT_TYPE_ISOCHRONOUS:
    case USB_ENDPOINT_TYPE_INTERRUPT:
      ptr_CH->HCCHAR |= USBH_STM32_HCCHAR_MCNT1;
      break;
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_EP_Remove -------------------------------
 *
 *  Remove the Endpoint
 *
 *  Parameter:  hndl:       Handle to the Configured Endpoint
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_EP_Remove (U32 hndl) {
  USBH_STM32_CH *ptr_CH;
  USBH_URB      *ptr_URB;
  U32            ch_idx;

  if (!hndl) 
    return (__FALSE);

  ptr_CH = (USBH_STM32_CH *)(hndl);
  ch_idx              = USBH_STM32_CH_GetIndexFromCH (ptr_CH);
  ptr_URB = CHURB[ch_idx];              /* Pointer to channels URB            */
  if (ptr_URB) {                        /* If URB exists cancel it            */
    if (!USBH_STM32_URB_Cancel (hndl, ptr_URB)) 
      return (__FALSE);
  }

  ptr_CH->HCCHAR      = 0;
  ptr_CH->HCINT       = 0;
  ptr_CH->HCINTMSK    = 0;
  ptr_CH->HCTSIZ      = 0;

  cntInterval[ch_idx] = 0;

  return (__TRUE);
}


/*------------------------- USBH_STM32_URB_Submit ------------------------------
 *
 *  Submit the URB (USB Request Block) to be processed
 *
 *  Parameter:  hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_URB_Submit (U32 hndl, USBH_URB *ptrURB) {
  USBH_STM32_CH *ptr_CH;
  U32            ch_idx;
  U8             resp_type;

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
  ptr_CH                  = (USBH_STM32_CH *)(hndl);
  ch_idx                  = USBH_STM32_CH_GetIndexFromCH (ptr_CH);
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
      USBH_STM32_CH_TransferEnqueue (ptr_CH, (U8)ptrURB->Parameters, (U8 *)ptrURB->ptrDataBuffer, (U32)ptrURB->DataLength);
      HW_Accessing        = __FALSE;
    } else {
      ptrURB->Submitted   = 1;
    }
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_URB_Cancel ------------------------------
 *
 *  Cancel the URB (USB Request Block)
 *
 *  Parameter:  hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_URB_Cancel (U32 hndl, USBH_URB *ptrURB) {
  USBH_STM32_CH *ptr_CH;
  U32            ch_idx;

  if (!hndl) 
    return (__FALSE);

  if (!ptrURB) 
    return (__FALSE);

  if (!ptrURB->Submitted) 
    return (__TRUE);

  ptrURB->Submitted = 0;

  ptr_CH = (USBH_STM32_CH *)(hndl);
  if (ptrURB->InProgress == 1) {
    if (!USBH_STM32_CH_Disable (ptr_CH))/* Stop Endpoint processing           */
      return (__FALSE);
    ptrURB->InProgress  = 0;
  }

  ch_idx = USBH_STM32_CH_GetIndexFromCH (ptr_CH);

  if (!cntInterval[ch_idx]) {           /* If interrupt endpoint transfer     */
    cntInterval[ch_idx] = 0;            /* Disable transmission on next SOF   */
  }
  CHURB[ch_idx] = 0;
  ptrURB->Cancelled = 1;

  return (__TRUE);
}


/*------------------------- OTG_FS_IRQHandler ----------------------------------
 *
 *  STM32 USB Interrupt Handler Routine
 *
 *  Parameter:
 *  Return:
 *----------------------------------------------------------------------------*/

void OTG_FS_IRQHandler (void) {
  USBH_STM32_CH  *ptr_CH;
  USBH_URB       *ptr_URB;
  U8             *ptrData8;
  U32            *ptrData32;
  U32            *DFIFO;
  U32             gintsts, hprt, haint, hcint, pktcnt, mpsiz;
  U32             grxsts, bcnt, ch, dat, len, len_rest;
  U32             act;

  /* Read global interrupt register                                           */
  gintsts =  OTG->GINTSTS & OTG->GINTMSK;
  hprt    =  OTG->HPRT;
  haint   =  OTG->HAINT;

  /* Analyze interrupt                                                        */
  if (gintsts & USBH_STM32_GINTSTS_SOF) {         /* If start of frame int    */
    for (ch = 0; ch < OTG_MAX_CH; ch++) {
      if (CHURB[ch]) {
        if (CHURB[ch]->TimeoutCount) {            /* If timeout not 0         */
          if (!(--CHURB[ch]->TimeoutCount)) {
            CHURB[ch]->Timeout = 1;               /* If timeout expired       */
          }
        }
      }
    }
  }

  if (gintsts&USBH_STM32_GINTSTS_HPRTINT) {       /* If host port interrupt   */
    if (hprt & USBH_STM32_HPRT_PCDET) {           /* Port connect detected    */
      if (!Port_Con) {
        cntDebounce = DEBOUNCE_500ms;
      }
    }
    if (hprt & USBH_STM32_HPRT_PENCHNG) {         /* If port enable changed   */
      if (hprt & USBH_STM32_HPRT_PENA) {          /* If device connected      */
        Port_Speed &= ~3;
        switch ((hprt >> 17) & 3) {
          case 0:
            break;
          case 1:
            Port_Speed |= USBH_FS;
            break;
          case 2:
            Port_Speed |= USBH_LS;
            break;
          case 3:
            break;
        }
      } 
      if (!(hprt & USBH_STM32_HPRT_PCSTS) &&      /* If device disconnected   */
            Port_Con) { 
        Port_Discon_Evt = 1;
        Port_Con        = 0;
      }
    }
    hprt &= ~USBH_STM32_HPRT_PENA;                /* Leave PENA bit           */
    OTG->HPRT = hprt;                             /* Clear host port interrupt*/
  }
  if ((gintsts & USBH_STM32_GINTSTS_DISCINT) &&   /* If device disconnected   */
       Port_Con) { 
    Port_Discon_Evt = 1;
    Port_Con        = 0;
  }

  if (Port_Discon_Evt) {                          /* If disconnect detected   */
    ptr_CH = (USBH_STM32_CH *)(&(OTG->HCCHAR0));
    for (ch = 0; ch < OTG_MAX_CH; ch++) {
      if (CHURB[ch]) 
        USBH_STM32_URB_Cancel ((U32)USBH_STM32_CH_GetCHFromIndex (ch), CHURB[ch]);
      ptr_CH++;
    }
  }
                                                  /* Handle reception int     */
  if (gintsts & USBH_STM32_GINTSTS_RXFLVL) {      /* If RXFIFO non-empty int  */
    OTG->GINTMSK &= ~USBH_STM32_GINTMSK_RXFLVLM;
    grxsts = OTG->GRXSTSR;
    if (((grxsts >> 17) & 0x0F) == 0x02){         /* If PKTSTS = 0x02         */
      grxsts     = (OTG->GRXSTSP);
      ch         = (grxsts >> 0) & 0x00F;
      bcnt       = (grxsts >> 4) & 0x7FF;
      ptr_CH     = USBH_STM32_CH_GetCHFromIndex (ch);
      DFIFO      = OTG_DFIFO[ch];
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
      grxsts     = OTG->GRXSTSP;
    }
    OTG->GINTMSK |= USBH_STM32_GINTMSK_RXFLVLM;
  }
                                                  /* Handle transmission int  */
  if (gintsts & USBH_STM32_GINTSTS_HCINT) {       /* If host channel interrupt*/
    ptr_CH = (USBH_STM32_CH *)(&(OTG->HCCHAR0));
    for (ch = 0; ch < OTG_MAX_CH; ch++) {
      if (haint & (1 << ch)) {                    /* If channels interrupt act*/
        hcint   = ptr_CH->HCINT & ptr_CH->HCINTMSK;
        ptr_URB = CHURB[ch];                      /* Pointer to channels URB  */
        ptr_URB->Error |= (hcint & USBH_STM32_HCINT_ERR) >> 7;
        if ((ptr_URB->PacketType == USBH_PACKET_OUT) ||     /* If OUT packet  ----------*/
            (ptr_URB->PacketType == USBH_PACKET_SETUP)) {   /* or SETUP packet----------*/
          if (hcint & USBH_STM32_HCINT_XFRC) {    /* If data transfer finished*/
            USBH_STM32_DMA_Stop();                /* Stop the DMA transfer    */
            ptr_CH->HCINTMSK         = 0;
            ptr_URB->DataTransferred = ptr_URB->DataLength;
            ptr_URB->Status          = 0;
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Completed       = 1;
            CHURB[ch]                = 0;
          } else if (hcint & USBH_STM32_HCINT_STALL) {      /* If STALL event */
            USBH_STM32_DMA_Stop();                /* Stop the DMA transfer    */
            ptr_URB->ResponsePacketType = USBH_PACKET_STALL;
            ptr_CH->HCINTMSK         = USBH_STM32_HCINT_CHH;
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USBH_STM32_HCCHAR_CHDIS;
          } else if ((hcint & USBH_STM32_HCINT_NAK)   ||    /* If NAK received*/
                     (hcint & USBH_STM32_HCINT_TXERR)) {    /* If TXERR rece  */
            USBH_STM32_DMA_Stop();                /* Stop the DMA transfer    */
            if (hcint & USBH_STM32_HCINT_NAK) {
              ptr_URB->ResponsePacketType = USBH_PACKET_NAK;
              ptr_CH->HCINTMSK       = USBH_STM32_HCINT_CHH;
            } else {
              ptr_URB->Error         = USBH_STM32_HCINT_TXERR >> 7;
              ptr_CH->HCINTMSK       = USBH_STM32_HCINT_ACK | USBH_STM32_HCINT_CHH;
            }
            if (ptr_URB->DataLength) {
                                                  /* Update transfer info     */
              pktcnt                 = (ptr_CH->HCTSIZ >> 19) & 0x3FF;
              mpsiz                  = (ptr_CH->HCCHAR >>  0) & 0x7FF;

              ptr_URB->DataTransferred = (((ptr_URB->DataLength + mpsiz - 1) / mpsiz) - pktcnt) * mpsiz;
            }
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USBH_STM32_HCCHAR_CHENA | USBH_STM32_HCCHAR_CHDIS;
          } else if (hcint&USBH_STM32_HCINT_CHH) {/* If channel halted        */
            if ((ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT)   && 
                (ptr_URB->ResponsePacketType == USBH_PACKET_NAK) && 
                (CHURB[ch]->NAKRetries--)) {
                                                  /* Reenable channel         */
              USBH_STM32_CH_TransferEnqueue (ptr_CH, (U8)CHURB[ch]->Parameters, (U8 *)CHURB[ch]->ptrDataBuffer+CHURB[ch]->DataTransferred, (U32)CHURB[ch]->DataLength-CHURB[ch]->DataTransferred);
              CHURB[ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
              ptr_CH->HCINTMSK       = 0;
              ptr_URB->Submitted     = 0;
              ptr_URB->InProgress    = 0;
              ptr_URB->Completed     = 1;
              CHURB[ch]              = 0;
            }
          } else if (hcint&USBH_STM32_HCINT_ACK) {/* If ACK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            ptr_CH->HCINTMSK        &=~USBH_STM32_HCINT_ACK;
          }
        } else if (ptr_URB->PacketType == USBH_PACKET_IN) { /* If IN packet   ----------*/
          if (hcint & USBH_STM32_HCINT_XFRC) {
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            ptr_CH->HCINTMSK         = USBH_STM32_HCINT_CHH;
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USBH_STM32_HCCHAR_CHENA | USBH_STM32_HCCHAR_CHDIS;
          } else if (hcint&USBH_STM32_HCINT_NAK) {/* If NAK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_NAK;
            if ((Port_Con) && (ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT) && (CHURB[ch]->NAKRetries--)) {
                                                  /* Reenable channel         */
              ptr_CH->HCINTMSK       = USBH_STM32_HCINTMSK_DTERRM | 
                                       USBH_STM32_HCINTMSK_BBERRM | 
                                       USBH_STM32_HCINTMSK_TXERRM | 
                                       USBH_STM32_HCINTMSK_ACKM   | 
                                       USBH_STM32_HCINTMSK_NAKM   | 
                                       USBH_STM32_HCINTMSK_STALLM | 
                                       USBH_STM32_HCINTMSK_XFRCM  ;
              ptr_CH->HCCHAR        |= USBH_STM32_HCCHAR_CHENA;
              CHURB[ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
                                                  /* Halt the channel         */
              ptr_CH->HCINTMSK       = USBH_STM32_HCINT_CHH;
              ptr_CH->HCCHAR        |= USBH_STM32_HCCHAR_CHENA | USBH_STM32_HCCHAR_CHDIS;
            }
          } else if ((hcint & USBH_STM32_HCINT_TXERR) ||    /* If TXERR event */
                     (hcint & USBH_STM32_HCINT_BBERR) ||    /* If BBERR event */
                     (hcint & USBH_STM32_HCINT_STALL)) {    /* If STALL event */
            ptr_CH->HCINTMSK         = USBH_STM32_HCINT_CHH;
            if (hcint & USBH_STM32_HCINT_TXERR) {
              ptr_URB->Error         = USBH_STM32_HCINT_TXERR >> 7;
              ptr_CH->HCINTMSK      |= USBH_STM32_HCINT_ACK;
            } else if (hcint & USBH_STM32_HCINT_BBERR) {
              ptr_URB->Error         = USBH_STM32_HCINT_BBERR >> 7;
            } else {
              ptr_URB->ResponsePacketType = USBH_PACKET_STALL;
            }
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USBH_STM32_HCCHAR_CHENA | USBH_STM32_HCCHAR_CHDIS;
          } else if (hcint&USBH_STM32_HCINT_CHH) {/* If Channel Halted        */
            ptr_CH->HCINTMSK         = 0;
            ptr_URB->Submitted       = 0;
            ptr_URB->InProgress      = 0;
            ptr_URB->Completed       = 1;
            CHURB[ch]                = 0;
            if (ptr_URB->Completed && ptr_URB->CompletedCallback) {
              ptr_URB->CompletedCallback();
            }
          } else if (hcint&USBH_STM32_HCINT_ACK) {/* If ACK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            if (ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT) {
              ptr_CH->HCINTMSK       = USBH_STM32_HCINTMSK_DTERRM | 
                                       USBH_STM32_HCINTMSK_BBERRM | 
                                       USBH_STM32_HCINTMSK_TXERRM | 
                                       USBH_STM32_HCINTMSK_ACKM   | 
                                       USBH_STM32_HCINTMSK_NAKM   | 
                                       USBH_STM32_HCINTMSK_STALLM | 
                                       USBH_STM32_HCINTMSK_XFRCM  ;
                                                  /* Reenable channel         */
              ptr_CH->HCCHAR        |= USBH_STM32_HCCHAR_CHENA;
              CHURB[ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
              ptr_CH->HCINTMSK       = USBH_STM32_HCINT_CHH;
                                                  /* Halt the channel         */
              ptr_CH->HCCHAR        |= USBH_STM32_HCCHAR_CHENA | USBH_STM32_HCCHAR_CHDIS;
            }
          } else if (hcint&USBH_STM32_HCINT_DTERR) {/* If DTERR received      */
          }
        }
        ptr_CH->HCINT = 0x7FF;
      }
      ptr_CH++;
    }
    OTG->HAINT = haint;
  }  
  OTG->GINTSTS = gintsts;                         /* Clear core interrupts    */
  if (gintsts & USBH_STM32_GINTSTS_SOF) {         /* If start of frame int    */
    if (HW_Accessing) {
      act = 1;
    } else {
      act = 0;
      for (ch = 0; ch < OTG_MAX_CH; ch++) {
        if (CHURB[ch]) {
          if (CHURB[ch]->InProgress == 1) {       /* If any URB in progress   */
            act = 1;                              /* Set act to 1             */
            break;
          }
        }
      }
    }

    /* At this point act == 1 if there is USB bus activity                    */
    ptr_CH = (USBH_STM32_CH *)(&(OTG->HCCHAR0));
    for (ch = 0; ch < OTG_MAX_CH; ch++) {
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
                  USBH_STM32_CH_TransferEnqueue (ptr_CH, (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrDataBuffer, (U32)ptr_URB->DataLength);
                  act = 1;
                }
              }
          } else if (!act) {
            ptr_URB->InProgress = 1;
            USBH_STM32_CH_TransferEnqueue (ptr_CH, (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrDataBuffer, (U32)ptr_URB->DataLength);
            act = 1;
          }
        }
      }
      ptr_CH++;
    }
  }
}
