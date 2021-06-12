/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_MK53.c
 *      Purpose: USB Full/Low-speed Host Freescale Kinetis MKxx Driver module
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include "usbh_MKxx.h"
#include <MK53N512ZCMD100.H>
#include <string.h>


/************************** Host Controller Driver Structure ******************/

USBH_HCD usbh0_hcd = {                  /* Host Controller Driver structure   */
  USBH_Get_Capabilities,                /* Get driver capabilities            */
  USBH_Delay_ms,                        /* Delay in ms                        */
  USBH_Pins_Config,                     /* Config/Unconfig pins               */
  USBH_Init,                            /* Init/Uninit Host Controller        */
  USBH_Port_Power,                      /* On/Off Port Power                  */
  USBH_Port_Reset,                      /* Reset port                         */
  USBH_Get_Connect,                     /* Get port conn/disconn status       */
  USBH_Get_Speed,                       /* Get port enumerated speed          */
  USBH_EP_Add,                          /* Add Endpoint                       */
  USBH_EP_Config,                       /* (Re)Configure Endpoint             */
  USBH_EP_Remove,                       /* Remove Endpoint                    */
  USBH_URB_Submit,                      /* Submit USB Block Request           */
  USBH_URB_Cancel                       /* Cancel USB Block Request           */
};


/************************** Driver Settings ***********************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

//  <o0> Size of memory used by the OTG-FS USB Host Controller <1-1048576>
//  <i> This is a size of memory (in bytes) that the USB Host Controller 
//  <i> will use for USB communication data.
#define USBH_MK_SZ_MPOOL          0x00001034


/************************** Variable and Memory Definitons ********************/

#define DEBOUNCE_500ms            500

/* Maximum number of endpoints */
#define OTG_MAX_EP                16

/* Position Buffer Descriptor Table in memory */
static USBH_MK_BD     USBH_MK_BDT    [4] __attribute__((aligned(512)));

/* Reserve memory for memory pool */ 
static U32            USBH_MK_MPOOL  [(USBH_MK_SZ_MPOOL+3)>>2];

static USBH_MK_DEV_EP USBH_DEV_EP    [OTG_MAX_EP<<1] = { 0 };

static BOOL HW_Busy           = { __FALSE };
static U16  cntDebounce       = { 0 };
static U32  calDelay          =   0  ;
static U32  Port_Discon_Evt   = { 0 };
static U32  Port_Speed        = { 0 };
static U32  Port_Con          = { 0 };
static U32  EP_Data           = { 0 };


/************************** Local Module Functions ****************************/

/***----------------------- Auxiliary Functions ----------------------------***/

/*------------------------- USBH_MK_GetEPIndexFromHandle -----------------------
 *
 *  Get the Endpoint index from it's handle
 *
 *  Parameter:  hndl:       Pointer to the Endpoint
 *  Return:                 Endpoint index (even - INs, odd - OUTs)
 *----------------------------------------------------------------------------*/

__inline U8 USBH_MK_GetEPIndexFromHandle (U32 hndl) {
  return (((hndl - ((U32)&USBH_DEV_EP[0])) / (sizeof(USBH_MK_DEV_EP))));
}


/*------------------------- USBH_MK_TransferCancel -----------------------------
 *
 *  Cancel the pending Transfer
 *
 *  Parameter:  hndl:       Pointer to the endpoint on which transfer should be canceled
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_MK_TransferCancel (U32 hndl) {
  U8 bd_idx;

  if (!hndl)
    return (__FALSE);

  bd_idx = (USBH_MK_GetEPIndexFromHandle (hndl) & 1) << 1;
  while (USBH_MK_BDT[bd_idx + 0].OWN);
  while (USBH_MK_BDT[bd_idx + 1].OWN);

  memset (&USBH_MK_BDT[bd_idx], 0, (sizeof(USBH_MK_BD) << 1));

  return (__TRUE);
}


/*------------------------- USBH_MK_TransferEnqueue ----------------------------
 *
 *  Enqueue the Transfer
 *
 *  Parameter:  hndl:       Pointer to the endpoint on which transfer will take place
 *              tgl_typ:    Toggle (bit 5..4: bit 5 - force toggle, bit 4 - value) and 
 *                          Packet type (bit 3..0: USBH_PACKET_IN, USBH_PACKET_OUT or USBH_PACKET_SETUP)
 *              buf:        Start of the receive or transmit data buffer
 *              len:        Length of the data to be received or sent
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_MK_TransferEnqueue (U32 hndl, U32 tgl_typ, U8 *buf, U32 len) {
  U8  ep_idx, bd_idx, addr, endpnt, token;
  U32 bd_ctrl;
  static U8 in_bd, out_bd;

  if ((!hndl) || (!Port_Con))
    return (__FALSE);

  ep_idx = USBH_MK_GetEPIndexFromHandle(hndl);
  if (ep_idx == 1) {
    ep_idx = 0;
  }
  addr   = ((USBH_DEV_EP[ep_idx].DEV_SPD == USBH_LS) << 7) | (USBH_DEV_EP[ep_idx].DEV_ADR & 0x7F);

  endpnt = 0;
  switch (USBH_DEV_EP[ep_idx].EP_DESC.bmAttributes & USB_ENDPOINT_TYPE_MASK) {
    case USB_ENDPOINT_TYPE_INTERRUPT:
      endpnt = 0x4D;
      break;
    case USB_ENDPOINT_TYPE_CONTROL:
      endpnt = 0x0D;
      break;
    case USB_ENDPOINT_TYPE_BULK:
      endpnt = 0x1D;
      break;
    case USB_ENDPOINT_TYPE_ISOCHRONOUS:
      endpnt = 0x0C;
      break;
  }
  if ((Port_Speed & 3) == USBH_LS) {    /* If Low-speed                       */
    endpnt |= 0x80;
  }

  token  = USB_TOKEN_TOKENENDPT(ep_idx >> 1);
  bd_idx = 0;
  switch (tgl_typ & 0x0F) {             /* Determine BD index to use          */
    case USBH_PACKET_IN:
      bd_idx  = (0 << 1) + (in_bd << 0);
      in_bd  ^= 1;
      token  |= USB_TOKEN_TOKENPID(0x9);
      break;
    case USBH_PACKET_OUT:
      bd_idx  = (1 << 1) + (out_bd << 0);
      out_bd ^= 1;
      token  |= USB_TOKEN_TOKENPID(0x1);
      break;
    case USBH_PACKET_SETUP:
      bd_idx  = (1 << 1) + (out_bd << 0);
      out_bd ^= 1;
      endpnt |= 0x40;                   /* Disable NAK retries for setup pckt */
      token  |= USB_TOKEN_TOKENPID(0xD);
      break;
  }

  if (len > USBH_DEV_EP[ep_idx].EP_DESC.wMaxPacketSize) {   /* Limit to max sz*/
    len = USBH_DEV_EP[ep_idx].EP_DESC.wMaxPacketSize;
  }
  bd_ctrl = ((len & 0x3FF) << 16);                /* Lenght of data           */
  if ((tgl_typ >> 5) & 1) {                       /* If toggle force bit activ*/
    if ((tgl_typ >> 4) & 1)                       /* Toggle bit value         */
      bd_ctrl |= (1 << 6);
  } else {                                        /* Toggle force not active  */
    if (EP_Data & (1 << ep_idx)) 
      bd_ctrl |= (1 << 6);
  }
  bd_ctrl |= (1 << 7);                            /* OWN bit to 1             */

  USB0->ADDR                   = addr;
  USB0->ENDPOINT[0].ENDPT      = endpnt;
  USBH_MK_BDT[bd_idx].ADDR     = (U32)buf;
  USBH_MK_BDT[bd_idx].CTRL     = bd_ctrl;
  HW_Busy                      = 1;
  while (USB0->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK);      /* Wait for ready */
  USB0->TOKEN                  = token;

  return (__TRUE);
}


/************************** Module Functions **********************************/

/*------------------------- USBH_Get_Capabilities ------------------------------
 *
 *  Get capabilities of Host Controller Driver
 *
 *  Parameter:  cap:        Pointer to USBH_HCI_CAP structure where 
 *                          capabilities are loaded
 *  Return:
 *----------------------------------------------------------------------------*/

void USBH_Get_Capabilities (USBH_HCI_CAP *cap) {
  cap->MultiPckt = __TRUE;
  cap->MaxDataSz = 4096;
  cap->CtrlNAKs  = 10000;
  cap->BulkNAKs  = 1000000;
}


/*------------------------- USBH_Delay_ms --------------------------------------
 *
 *  Delay execution (in milliseconds)
 *  Calibration is done if global variable calDelay is 0
 *
 *  Parameter:  ms:         Number of milliseconds to delay execution for
 *  Return:
 *----------------------------------------------------------------------------*/

static void USBH_Delay_ms (U32 ms) {
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
    SystemCoreClockUpdate();            /* Update information on core clock   */
    calDelay = SystemCoreClock / (vals - vale);   /* Calibrated value         */
    if (vals == 0xFFFFFF)               /* Stop timer if we started it        */
      SysTick->CTRL  = 0;
    goto start;
  }
}


/*------------------------- USBH_Pins_Config -----------------------------------
 *
 *  Configurate or unconfigurate pins used by the USB Host
 *
 *  Parameter:  on:         __TRUE = configurate, __FALSE = unconfigurate
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_Pins_Config (BOOL on) {

  /* Dedicated pins are enabled by default                                    */
  return (__TRUE);
}


/*------------------------- USBH_Init ------------------------------------------
 *
 *  Initialize or uninitialize the USB Host Controller
 *
 *  Parameter:  on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_Init (BOOL on) {

  if (on) {
    USBH_Delay_ms (10);                           /* To calibrate delay       */

    /* Initialize memory pool for data                                        */
    if (!usbh_mem_init(0, (U32 *)USBH_MK_MPOOL, USBH_MK_SZ_MPOOL))
      return (__FALSE);

    /* Enable all clocks needed for USB to function                           */
    /* Set USB clock to 48 MHz                                                */
    SIM->SOPT2   |=   SIM_SOPT2_USBSRC_MASK     | /* MCGPLLCLK used as src    */
                      SIM_SOPT2_PLLFLLSEL_MASK  ; /* Select MCGPLLCLK as clock*/
    SIM->CLKDIV2 &= ~(SIM_CLKDIV2_USBFRAC_MASK  | /* Clear CLKDIV2 FS values  */
                      SIM_CLKDIV2_USBDIV_MASK);
    SIM->CLKDIV2  =   SIM_CLKDIV2_USBDIV(1)     ; /* USB clk = (PLL*1/2)      */
                                                  /*         = ( 96*1/2)=48   */
    SIM->SCGC4   |=   SIM_SCGC4_USBOTG_MASK;      /* Enable USBOTG clock      */

    /* Initialize the USB controller                                          */
    USB0->USBTRC0|=   USB_USBTRC0_USBRESET_MASK;  /* Reset the USB controller */
    while (USB0->USBTRC0 & USB_USBTRC0_USBRESET_MASK);      /* Wait to finish */
    USB0->USBCTRL =   0;                          /* Reset USB CTRL register  */
    USB0->USBCTRL|=   USB_USBCTRL_PDE_MASK;       /* Ena weak PD              */
    USB0->CTL    |=   USB_CTL_ODDRST_MASK;        /* ODD reset                */
    USB0->BDTPAGE1= ((U32)USBH_MK_BDT >>  8)&0xFE;/* BDT address bit  9-15    */
    USB0->BDTPAGE2= ((U32)USBH_MK_BDT >> 16);     /* BDT address bit 16-23    */
    USB0->BDTPAGE3= ((U32)USBH_MK_BDT >> 24);     /* BDT address bit 24-31    */
    USB0->SOFTHLD =   1;                          /* Max threshold            */
    USB0->CTL     =   USB_CTL_HOSTMODEEN_MASK;    /* Enable Host mode         */
    USB0->ISTAT   =   0xFF;                       /* Clear all interrupts     */
    USB0->INTEN   =   USB_INTEN_ATTACHEN_MASK;    /* Enable ATTACH interrupt  */
    NVIC_SetPriority (USB0_IRQn, 0);    /* Set USB interrupt highest priority */
    NVIC_EnableIRQ   (USB0_IRQn);       /* Enable USB interrupt               */
  } else {
    NVIC_DisableIRQ  (USB0_IRQn);       /* Disable OTG interrupt              */

    /* Reset all initialized values to reset state                            */
    USB0->USBTRC0|=   USB_USBTRC0_USBRESET_MASK;  /* Reset the USB controller */
    while (USB0->USBTRC0 & USB_USBTRC0_USBRESET_MASK);      /* Wait to finish */
    SIM->SCGC4   &=  ~SIM_SCGC4_USBOTG_MASK;      /* Disable USBOTG clock     */
    SIM->CLKDIV2 &= ~(SIM_CLKDIV2_USBFRAC_MASK  | /* Clear CLKDIV2 FS values  */
                      SIM_CLKDIV2_USBDIV_MASK);
    SIM->SOPT2   &= ~(SIM_SOPT2_PLLFLLSEL_MASK | 
                      SIM_SOPT2_USBSRC_MASK)   ;

    calDelay      = 0;                            /* Force delay recalibrate  */
  }

  return (__TRUE);
}


/*------------------------- USBH_Port_Power ------------------------------------
 *
 *  Turn USB Host port power on or off
 *
 *  Parameter:  on:         __TRUE = power on, __FALSE = power off
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_Port_Power (BOOL on) {

  /* Set jumpers on TWR-SER: J10 (1-2) and J16 (1-2) for power on VBUS        */
  return (__TRUE);
}


/*------------------------- USBH_Port_Reset ------------------------------------
 *
 *  Reset Port
 *
 *  Parameter:  port:       Root Hub port to be reset (only 0 is available)
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_Port_Reset (U8 port) {

  USB0->INTEN  &= ~USB_INTEN_USBRSTEN_MASK;       /* Disable reset interrupt  */
  USB0->CTL    |=  USB_CTL_RESET_MASK;            /* Generate USB reset       */
  USBH_Delay_ms (10);                             /* Wait ~10 ms              */
  EP_Data       =  0;                             /* Endpoint data toggle rst */
  USB0->CTL    &= ~USB_CTL_RESET_MASK;            /* Stop USB reset signaling */
  USB0->CTL    |=  USB_CTL_USBENSOFEN_MASK;       /* Enable SOF generation    */
  USB0->SOFTHLD =  110;                           /* Max threshold            */
  USB0->ISTAT   =  0xFF;                          /* Clear all USB interrupts */
  USB0->INTEN  |=  USB_INTEN_TOKDNEEN_MASK |      /* Enable TOKDNE interrupt  */
                   USB_INTEN_SOFTOKEN_MASK |      /* Enable SOFTOKNE interrupt*/
                   USB_INTEN_ERROREN_MASK  |      /* Enable ERROR interrupt   */
                   USB_INTEN_USBRSTEN_MASK ;      /* Enable USBRST interrupt  */
  USB0->ERRSTAT =  0xFF;                          /* Clear all error ints     */
  USB0->ERREN  |=  USB_ERREN_BTSERREN_MASK |      /* Enable all error ints    */
                   USB_ERREN_DMAERREN_MASK |
                   USB_ERREN_BTOERREN_MASK |
                   USB_ERREN_DFN8EN_MASK   |
                   USB_ERREN_CRC16EN_MASK  |
                   USB_ERREN_CRC5EOFEN_MASK|
                   USB_ERREN_PIDERREN_MASK ;

  return (__TRUE);
}


/*------------------------- USBH_Get_Connect -----------------------------------
 *
 *  Returns connect/disconnect port events (also does debouncing)
 *
 *  Parameter:  
 *  Return:                 Connection/Disconnection events
 *----------------------------------------------------------------------------*/

static U32 USBH_Get_Connect (void) {
  U32 stat, ret;

  stat = (USB0->ISTAT >> 6) & 1;
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
      USBH_Delay_ms (1);
    }
  } else if ((stat ^ Port_Con) && (!Port_Con)) {
                                        /* Restart debouncing if connect      */
    cntDebounce = DEBOUNCE_500ms;
  }

  return (ret);
}


/*------------------------- USBH_Get_Speed -------------------------------------
 *
 *  Returns port speeds
 *
 *  Parameter:  
 *  Return:                 Port speeds
 *----------------------------------------------------------------------------*/

static U32 USBH_Get_Speed (void) {
  
  return (Port_Speed); 
}


/*------------------------- USBH_EP_Add ----------------------------------------
 *
 *  Add the Endpoint and return handle (address of the Endpoint)
 *
 *  Parameter:  dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 Handle to the Configured Endpoint (0 = FAIL)
 *----------------------------------------------------------------------------*/

static U32 USBH_EP_Add (U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {
  U8 ep_idx;

  ep_idx = ((ptrEPD->bEndpointAddress & 0x0F) << 1);
  if (ep_idx) {
    ep_idx += (((ptrEPD->bEndpointAddress & 0x80) >> 7) ^ 1);
  }

  /* Store all of endpoint descriptor parameters to local structure           */
  USBH_DEV_EP[ep_idx].DEV_ADR = dev_adr;
  USBH_DEV_EP[ep_idx].DEV_SPD = ep_spd;
  memcpy (&USBH_DEV_EP[ep_idx].EP_DESC, ptrEPD, sizeof(USB_ENDPOINT_DESCRIPTOR));

  return ((U32)&USBH_DEV_EP[ep_idx]);
}


/*------------------------- USBH_EP_Config -------------------------------------
 *
 *  (Re)Configure some parameters of the Endpoint
 *
 *  Parameter:  hndl:       Handle to the Configured Endpoint
 *              dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_EP_Config (U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {

  if (!hndl) 
    return (__FALSE);

  if (USBH_EP_Add (dev_adr, ep_spd, ptrEPD) != hndl) 
    return (__FALSE);

  return (__TRUE);
}


/*------------------------- USBH_EP_Remove -------------------------------------
 *
 *  Remove the Endpoint
 *
 *  Parameter:  hndl:       Handle to the Configured Endpoint
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_EP_Remove (U32 hndl) {
  U8         ep_idx;
  USBH_URB  *ptr_URB;

  if (!hndl) 
    return (__FALSE);

  ep_idx = USBH_MK_GetEPIndexFromHandle(hndl);
  ptr_URB = USBH_DEV_EP[ep_idx].ptr_URB;/* Pointer to endpoint URB            */
  if (ptr_URB) {                        /* If URB exists cancel it            */
    if (!USBH_URB_Cancel (hndl, ptr_URB)) 
      return (__FALSE);
  }

  /* Clear all of endpoint descriptor parameters in local structure           */
  memset (&USBH_DEV_EP[ep_idx], 0, sizeof(USBH_MK_DEV_EP));

  return (__TRUE);
}


/*------------------------- USBH_URB_Submit ------------------------------------
 *
 *  Submit the URB (USB Request Block) to be processed
 *
 *  Parameter:  hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_URB_Submit (U32 hndl, USBH_URB *ptrURB) {
  U32 ep_idx;
  U32 ep_typ;

  if ((!hndl) || (!ptrURB) || (!Port_Con) || (ptrURB->Submitted == 1) || (ptrURB->InProgress == 1))
    return (__FALSE);
  
  ptrURB->ptrCurrentDataBuffer = ptrURB->ptrDataBuffer;
  ptrURB->DataTransferred = 0;
  ptrURB->Status          = 0;

  ep_idx                  = USBH_MK_GetEPIndexFromHandle(hndl);
  ep_typ                  = USBH_DEV_EP[ep_idx].EP_DESC.bmAttributes & USB_ENDPOINT_TYPE_MASK;
  USBH_DEV_EP[ep_idx].ptr_URB = ptrURB;

  if ((ep_typ == USB_ENDPOINT_TYPE_INTERRUPT) && (!USBH_DEV_EP[ep_idx].cntInterval)) {
                                        /* If interrupt endpoint transfer     */
    USBH_DEV_EP[ep_idx].cntInterval = USBH_DEV_EP[ep_idx].EP_DESC.bInterval;
    if (!USBH_DEV_EP[ep_idx].cntInterval) 
      USBH_DEV_EP[ep_idx].cntInterval = 1;
    ptrURB->Submitted     = 1;
  } else if ((ep_typ == USB_ENDPOINT_TYPE_CONTROL) || (ep_typ == USB_ENDPOINT_TYPE_BULK)) {
    ptrURB->Submitted     = 1;          /* Transfer will start on the next SOF*/
  }

  return (__TRUE);
}


/*------------------------- USBH_URB_Cancel ------------------------------------
 *
 *  Cancel the URB (USB Request Block)
 *
 *  Parameter:  hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_URB_Cancel (U32 hndl, USBH_URB *ptrURB) {
  U8 ep_idx;

  if ((!hndl) || (!ptrURB)) 
    return (__FALSE);

  if (!ptrURB->Submitted) 
    return (__TRUE);

  ptrURB->Submitted = 0;

  ep_idx = USBH_MK_GetEPIndexFromHandle(hndl);
  if (ptrURB->InProgress == 1) {
    if (USBH_MK_TransferCancel (hndl))  /* Stop Transfer                      */
      return (__FALSE);
    ptrURB->InProgress  = 0;
  }

  if (!USBH_DEV_EP[ep_idx].cntInterval){/* If interrupt endpoint transfer     */
    USBH_DEV_EP[ep_idx].cntInterval = 0;/* Disable transmission on next SOF   */
  }
  ptrURB->Cancelled = 1;

  return (__TRUE);
}


/*------------------------- IRQ Handler ----------------------------------------
 *
 *  USB Interrupt Handler Routine
 *
 *  Parameter:
 *  Return:
 *----------------------------------------------------------------------------*/

void USB0_IRQHandler (void) {
  U8         istat, inten, errstat, ep_idx, bd_idx, ep, stat;
  U16        len;
  static U32 error_msk, completed_msk, retransmit_msk, transmit_msk;
  USBH_URB  *ptr_URB;

  istat    = USB0->ISTAT;
  inten    = USB0->INTEN;
  istat   &= inten;
  errstat  = USB0->ERRSTAT;

  if (istat & (1 << 6)) {               /* If ATTACH interrupt occured        */
    if (!Port_Con) {
      cntDebounce  = DEBOUNCE_500ms;
      Port_Speed   = (USB0->CTL >> 7)&1;/* Determine attached device speed    */
      if (!Port_Speed) {
        USB0->ADDR   |= 0x80;           /* Enable low speed                   */
      }
      error_msk       = 0;
      completed_msk   = 0;
      retransmit_msk  = 0;
      transmit_msk    = 0;
    }
    USB0->INTEN  &= ~USB_INTEN_ATTACHEN_MASK;     /* Disable ATTACH interrupt */
  }

  if (istat & (1 << 1)) {               /* If ERROR  interrupt occured        */
    for (ep = 0; ep < (OTG_MAX_EP*2); ep++){/* Go through all Endpoints       */
      ptr_URB = USBH_DEV_EP[ep].ptr_URB;    /* Pointer to endpoints URB       */
      if (ptr_URB) {                              /* If URB is valid (!= 0)   */
        if ((ptr_URB->Submitted  == 1) &&         /* If URB is submitted      */
            (ptr_URB->InProgress == 1)) {         /* If URB is in progress    */
          switch (errstat) {
            case USB_ERRSTAT_BTSERR_MASK:
              ptr_URB->Error  = USBH_TR_ERROR_BSTUFF;
              error_msk |= (1 << ep);
              break;              
            case USB_ERRSTAT_DMAERR_MASK:
              ptr_URB->Error  = USBH_TR_ERROR_DOVER;
              error_msk |= (1 << ep);
              break;              
            case USB_ERRSTAT_BTOERR_MASK:
              ptr_URB->Error  = USBH_TR_ERROR_BUSTT;
              retransmit_msk |= (1 << ep);
              break;              
            case USB_ERRSTAT_DFN8_MASK:
              ptr_URB->Error  = USBH_TR_ERROR_OTHER;
              error_msk |= (1 << ep);
              break;              
            case USB_ERRSTAT_CRC16_MASK:
              ptr_URB->Error  = USBH_TR_ERROR_CRC;
              error_msk |= (1 << ep);
              break;              
            case USB_ERRSTAT_CRC5EOF_MASK:
              ptr_URB->Error  = USBH_TR_ERROR_FEOP;
              error_msk |= (1 << ep);
              break;              
            case USB_ERRSTAT_PIDERR_MASK:
              ptr_URB->Error  = USBH_TR_ERROR_PID;
              error_msk |= (1 << ep);
              break;              
          }
          break;
        }
      }
    }
    USB0->ERRSTAT = errstat;            /* Clear all errors                   */
  }

  if (istat & (1 << 3)) {               /* If TOKDNE interrupt occured        */
    stat    =    USB0->STAT & 0x0F;
                                        /* Find out Endpoint Index            */
    ep_idx  = (((USB0->TOKEN) & 0x0F) << 1) + ((stat >> 3) & 0x01);
    if (ep_idx == 1)
      ep_idx = 0;
    bd_idx  =   (stat >> 2)   & 0x03;   /* Find out Buffer Descriptor Index   */
                                        /* Only ENDPNT0 is used               */
    ptr_URB =  USBH_DEV_EP[ep_idx].ptr_URB;               /* Pointer to ep URB*/
    switch (USBH_MK_BDT[bd_idx].TOK_PID) {
      case 0x00:                        /* Bus Timeout                        */
        ptr_URB->ResponsePacketType = USBH_PACKET_RESERVED;
        retransmit_msk             |= (1 << ep_idx);
        break;
      case 0x02:                        /* ACK                                */
        ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
        if (ep_idx <= 1) {
          EP_Data ^= 0x03;              /* Toggle last EP0 data type          */
        } else {
          EP_Data ^= (1 << ep_idx);     /* Toggle last data type              */
        }
        break;
      case 0x03:                        /* DATA0                              */
        ptr_URB->ResponsePacketType = USBH_PACKET_DATA0;
        ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
        if (ep_idx <= 1) {
          EP_Data ^= 0x03;              /* Toggle last EP0 data type          */
        } else {
          EP_Data ^= (1 << ep_idx);     /* Toggle last data type              */
        }
        break;
      case 0x0A:                        /* NAK                                */
        ptr_URB->ResponsePacketType = USBH_PACKET_NAK;
        break;
      case 0x0B:                        /* DATA1                              */
        ptr_URB->ResponsePacketType = USBH_PACKET_DATA1;
        ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
        if (ep_idx <= 1) {
          EP_Data ^= 0x03;              /* Toggle last EP0 data type          */
        } else {
          EP_Data ^= (1 << ep_idx);     /* Toggle last data type              */
        }
        break;
      case 0x0E:                        /* STALL                              */
        ptr_URB->ResponsePacketType = USBH_PACKET_STALL;
        break;
      case 0x0F:                        /* Data Error                         */
        ptr_URB->ResponsePacketType = USBH_PACKET_RESERVED;
        error_msk                  |= (1 << ep_idx);
        break;
      default:                          /* Undefined                          */
        ptr_URB->ResponsePacketType = USBH_PACKET_RESERVED;
        error_msk                  |= (1 << ep_idx);
        break;
    }
    len = USBH_MK_BDT[bd_idx].BC;
    if (retransmit_msk & (1 << ep_idx))
      len = 0;
    if (!(error_msk & (1 << ep_idx))) { /* If there was no error              */
      ptr_URB->DataTransferred      += len;
      ptr_URB->ptrCurrentDataBuffer += len;
      if (ptr_URB->DataTransferred >= ptr_URB->DataLength) {
        completed_msk  |= 1 << ep_idx;
      } else if (ptr_URB->DataTransferred < ptr_URB->DataLength) { 
        if ((USBH_DEV_EP[ep_idx].EP_DESC.bmAttributes & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_INTERRUPT) 
          completed_msk |= 1 << ep_idx;
        else
          transmit_msk  |= 1 << ep_idx;
      }
    }

    HW_Busy             = 0;
    if (error_msk      & (1 << ep_idx)) {
      ptr_URB->Submitted  = 0;
      ptr_URB->InProgress = 0;
      ptr_URB->Completed  = 1;
      USBH_DEV_EP[ep_idx].ptr_URB = 0;
    } else if (completed_msk  & (1 << ep_idx)) {
      if (ptr_URB->DataTransferred > ptr_URB->DataLength)
        ptr_URB->Error    = USBH_TR_ERROR_DOVER;
      ptr_URB->Submitted  = 0;
      ptr_URB->InProgress = 0;
      ptr_URB->Completed  = 1;
      USBH_DEV_EP[ep_idx].ptr_URB = 0;
      if (ptr_URB->Completed && ptr_URB->CompletedCallback) {
        ptr_URB->CompletedCallback();
      }
    } else if (retransmit_msk & (1 << ep_idx)) {
      retransmit_msk     &= ~(1 << ep_idx);
      ptr_URB->Status     = 0;
      ptr_URB->Submitted  = 1;
    } else if (transmit_msk & (1 << ep_idx)) {
      retransmit_msk     &= ~(1 << ep_idx);
      ptr_URB->Status     = 0;
      ptr_URB->Submitted  = 1;
      ptr_URB->InProgress = 1;
      ptr_URB->Completed  = 0;
      USBH_MK_TransferEnqueue ((U32)&USBH_DEV_EP[ep_idx], (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrCurrentDataBuffer, (U32)ptr_URB->DataLength-(U32)ptr_URB->DataTransferred);
    }
    error_msk      &= ~(1 << ep_idx);
    completed_msk  &= ~(1 << ep_idx);
    retransmit_msk &= ~(1 << ep_idx);
    transmit_msk   &= ~(1 << ep_idx);
  }

  if ((istat & (1 << 0)) && Port_Con) { /* If RESET and port was connected    */
    Port_Discon_Evt  =  __TRUE;         /* Port disconnect event              */
    Port_Con         =  0;              /* Device is disconnected             */
  }

  if (Port_Discon_Evt) {                          /* If disconnect detected   */
    Port_Speed       =  0;              /* Clear port speed                   */
    USB0->ADDR       =  0;              /* Clear address                      */
    USB0->CTL       &= ~USB_CTL_USBENSOFEN_MASK;      /* Disable SOFs         */
    USB0->INTEN      =  USB_INTEN_ATTACHEN_MASK;      /* Enable ATTACH int    */
    for (ep = 0; ep < (OTG_MAX_EP*2); ep++){/* Go through all endpoints       */
      ptr_URB = USBH_DEV_EP[ep].ptr_URB;/* Pointer to endpoint's URB          */
      if (ptr_URB)                      /* If pointer is valid                */
        USBH_URB_Cancel ((U32)&USBH_DEV_EP[ep], ptr_URB);
    }
  }

  if (istat & (1 << 2)) {               /* If start of frame interrupt        */

                                        /* Check transfer timeout             */
    for (ep = 0; ep < (OTG_MAX_EP*2); ep++){/* Go through all endpoints       */
      ptr_URB = USBH_DEV_EP[ep].ptr_URB;/* Pointer to endpoint's URB          */
      if (ptr_URB) {                    /* If pointer is valid                */
        if (ptr_URB->TimeoutCount--) {
          if (!ptr_URB->TimeoutCount) {
            ptr_URB->Timeout = 1;       /* If timeout expired                 */
          }
        }
      }
    }

    if (!HW_Busy) {
                                        /* Handle start of control transfers  */
      for (ep = 0; ep<(OTG_MAX_EP*2); ep++){  /* Go through all endpoints     */
        ptr_URB=USBH_DEV_EP[ep].ptr_URB;/* Pointer to endpoint's URB          */
        if (ptr_URB) {                  /* If pointer is valid                */
          if ((USBH_DEV_EP[ep].EP_DESC.bmAttributes & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_CONTROL) {
            if ((ptr_URB->Submitted  == 1) &&     /* If URB is submitted      */
              (ptr_URB->InProgress == 0)) {       /* If URB not in progress   */
              ptr_URB->InProgress = 1;
              USBH_MK_TransferEnqueue ((U32)&USBH_DEV_EP[ep], (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrCurrentDataBuffer, (U32)ptr_URB->DataLength-(U32)ptr_URB->DataTransferred);
              break;
            }
          }
        }
      }
    }
                                        /* Handle start of interrupt transfers*/
    for (ep = 0; ep<(OTG_MAX_EP*2); ep++){    /* Go through all endpoints     */
      ptr_URB=USBH_DEV_EP[ep].ptr_URB;  /* Pointer to endpoint's URB          */
      if (ptr_URB) {                    /* If pointer is valid                */
        if ((USBH_DEV_EP[ep].EP_DESC.bmAttributes & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_INTERRUPT) {
          if ((ptr_URB->Submitted  == 1) &&       /* If URB is submitted      */
              (ptr_URB->InProgress == 0)) {       /* If URB not in progress   */
            if (USBH_DEV_EP[ep].cntInterval) {
              if (!HW_Busy) {
                USBH_DEV_EP[ep].cntInterval--;
                if (!USBH_DEV_EP[ep].cntInterval){/* If period expired        */
                  ptr_URB->InProgress = 1;
                  USBH_MK_TransferEnqueue ((U32)&USBH_DEV_EP[ep], (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrCurrentDataBuffer, (U32)ptr_URB->DataLength-(U32)ptr_URB->DataTransferred);
                }
              } else {
                if (USBH_DEV_EP[ep].cntInterval > 1) 
                  USBH_DEV_EP[ep].cntInterval--;
              }
            }
          }
        }
      }
    }
    if (!HW_Busy) {
                                        /* Handle start of bulk transfers     */
      for (ep = 0; ep<(OTG_MAX_EP*2); ep++){  /* Go through all endpoints     */
        ptr_URB=USBH_DEV_EP[ep].ptr_URB;/* Pointer to endpoint's URB          */
        if (ptr_URB) {                  /* If pointer is valid                */
          if ((USBH_DEV_EP[ep].EP_DESC.bmAttributes & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_BULK) {
            if ((ptr_URB->Submitted  == 1) &&     /* If URB is submitted      */
              (ptr_URB->InProgress == 0)) {       /* If URB not in progress   */
              ptr_URB->InProgress = 1;
              USBH_MK_TransferEnqueue ((U32)&USBH_DEV_EP[ep], (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrCurrentDataBuffer, (U32)ptr_URB->DataLength-(U32)ptr_URB->DataTransferred);
              break;
            }
          }
        }
      }
    }
  }

  USB0->ISTAT = istat;                  /* Clear handled interrupts           */
}
