/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_stm32f20x.c
 *      Purpose: High/Full/Low-speed Host STM32F20x Driver module
 *      Rev.:    V4.70
 *      Comment: Known issue in frequent connect/disconnect if transfer was 
 *               started but not ended before disconnect, cancel of transfer 
 *               never happens (channel never halts) (this is hardware issue)
 *      Defines: __MCBSTM32F200        - retarget pins for Keil MCBSTM32F200 
 *                                       board, OTG_HS used with external ULPI 
 *                                       PHY
 *               __STM3220G            - retargeted pins for ST STM3220G-EVAL
 *                                       board, OTG_HS used with external ULPI 
 *                                       PHY
 *               __XYNERGY             - retargeted pins for Silica XYNERGY 
 *                                       board, only OTG_FS available
 *               __OTG_HS_EMBEDDED_PHY - OTG_HS is used with Embedded PHY, 
 *                                       allows only Low/Full-speed 
 *                                       functionality
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include "usbh_stm32f20x.h"
#include <stm32f2xx.h>
#include <string.h>


/************************** Host Controller Driver Structure ******************/

BOOL USBH0_STM32_Pins_Config (BOOL on)                    { return (USBH_STM32_Pins_Config (0, on)); }
BOOL USBH0_STM32_Init        (BOOL on)                    { return (USBH_STM32_Init        (0, on)); }
BOOL USBH0_STM32_Port_Power  (BOOL on)                    { return (USBH_STM32_Port_Power  (0, on)); }
BOOL USBH0_STM32_Port_Reset  (U8 port)                    { return (USBH_STM32_Port_Reset  (0, port)); }
U32  USBH0_STM32_Get_Connect (void)                       { return (USBH_STM32_Get_Connect (0)); }
U32  USBH0_STM32_Get_Speed   (void)                       { return (USBH_STM32_Get_Speed   (0)); }
U32  USBH0_STM32_EP_Add      (          U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) { 
                                                            return (USBH_STM32_EP_Add      (0,       dev_adr, ep_spd, ptrEPD)); }
BOOL USBH0_STM32_EP_Config   (U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) { 
                                                            return (USBH_STM32_EP_Config   (0, hndl, dev_adr, ep_spd, ptrEPD)); }
BOOL USBH0_STM32_EP_Remove   (U32 hndl)                   { return (USBH_STM32_EP_Remove   (0, hndl)); }
BOOL USBH0_STM32_URB_Submit  (U32 hndl, USBH_URB *ptrURB) { return (USBH_STM32_URB_Submit  (0, hndl, ptrURB)); }
BOOL USBH0_STM32_URB_Cancel  (U32 hndl, USBH_URB *ptrURB) { return (USBH_STM32_URB_Cancel  (0, hndl, ptrURB)); }
void OTG_FS_IRQHandler       (void)                       { OTG_xS_IRQHandler              (0); }

BOOL USBH1_STM32_Pins_Config (BOOL on)                    { return (USBH_STM32_Pins_Config (1, on)); }
BOOL USBH1_STM32_Init        (BOOL on)                    { return (USBH_STM32_Init        (1, on)); }
BOOL USBH1_STM32_Port_Power  (BOOL on)                    { return (USBH_STM32_Port_Power  (1, on)); }
BOOL USBH1_STM32_Port_Reset  (U8 port)                    { return (USBH_STM32_Port_Reset  (1, port)); }
U32  USBH1_STM32_Get_Connect (void)                       { return (USBH_STM32_Get_Connect (1)); }
U32  USBH1_STM32_Get_Speed   (void)                       { return (USBH_STM32_Get_Speed   (1)); }
U32  USBH1_STM32_EP_Add      (          U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) { 
                                                            return (USBH_STM32_EP_Add      (1,       dev_adr, ep_spd, ptrEPD)); }
BOOL USBH1_STM32_EP_Config   (U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) { 
                                                            return (USBH_STM32_EP_Config   (1, hndl, dev_adr, ep_spd, ptrEPD)); }
BOOL USBH1_STM32_EP_Remove   (U32 hndl)                   { return (USBH_STM32_EP_Remove   (1, hndl)); }
BOOL USBH1_STM32_URB_Submit  (U32 hndl, USBH_URB *ptrURB) { return (USBH_STM32_URB_Submit  (1, hndl, ptrURB)); }
BOOL USBH1_STM32_URB_Cancel  (U32 hndl, USBH_URB *ptrURB) { return (USBH_STM32_URB_Cancel  (1, hndl, ptrURB)); }
void OTG_HS_IRQHandler       (void)                       { OTG_xS_IRQHandler              (1); }

USBH_HCD usbh0_hcd = {                  /* Host Controller Driver structure   */
  USBH_STM32_Get_Capabilities,          /* Get Host Ctrl Driver capabilities  */
  USBH_STM32_Delay_ms,                  /* Delay in ms                        */
  USBH0_STM32_Pins_Config,              /* Config/Unconfig pins               */
  USBH0_STM32_Init,                     /* Init/Uninit Host Controller        */
  USBH0_STM32_Port_Power,               /* On/Off Port Power                  */
  USBH0_STM32_Port_Reset,               /* Reset port                         */
  USBH0_STM32_Get_Connect,              /* Get port conn/disconn status       */
  USBH0_STM32_Get_Speed,                /* Get port enumerated speed          */
  USBH0_STM32_EP_Add,                   /* Add Endpoint                       */
  USBH0_STM32_EP_Config,                /* (Re)Configure Endpoint             */
  USBH0_STM32_EP_Remove,                /* Remove Endpoint                    */
  USBH0_STM32_URB_Submit,               /* Submit USB Block Request           */
  USBH0_STM32_URB_Cancel                /* Cancel USB Block Request           */
};

USBH_HCD usbh1_hcd = {                  /* Host Controller Driver structure   */
  USBH_STM32_Get_Capabilities,          /* Get Host Ctrl Driver capabilities  */
  USBH_STM32_Delay_ms,                  /* Delay in ms                        */
  USBH1_STM32_Pins_Config,              /* Config/Unconfig pins               */
  USBH1_STM32_Init,                     /* Init/Uninit Host Controller        */
  USBH1_STM32_Port_Power,               /* On/Off Port Power                  */
  USBH1_STM32_Port_Reset,               /* Reset port                         */
  USBH1_STM32_Get_Connect,              /* Get port conn/disconn status       */
  USBH1_STM32_Get_Speed,                /* Get port enumerated speed          */
  USBH1_STM32_EP_Add,                   /* Add Endpoint                       */
  USBH1_STM32_EP_Config,                /* (Re)Configure Endpoint             */
  USBH1_STM32_EP_Remove,                /* Remove Endpoint                    */
  USBH1_STM32_URB_Submit,               /* Submit USB Block Request           */
  USBH1_STM32_URB_Cancel                /* Cancel USB Block Request           */
};


/************************** Driver Settings ***********************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

//  <o0> Size of memory used by the OTG_FS USB Host Controller <1-1048576>
//  <i> This is a size of memory (in bytes) that the USB Host Controller 
//  <i> will use for USB communication data.
//
//  <o1> Size of memory used by the OTG_HS USB Host Controller <1-1048576>
//  <i> This is a size of memory (in bytes) that the USB Host Controller 
//  <i> will use for USB communication data.
#define USBH0_STM32_SZ_MPOOL        0x00000234
#define USBH1_STM32_SZ_MPOOL        0x00000234


/************************** Variable and Memory Definitons ********************/

typedef enum    {PORT_A = 0, PORT_B, PORT_C, PORT_D, PORT_E, PORT_F, PORT_G, PORT_H, PORT_I} PORTS;

#define MAX_TIMEOUT_COUNT         100
#define DEBOUNCE_500ms            500

#define OTG_FS_MAX_CH             8
#define OTG_HS_MAX_CH             16

static U32 *OTG[2]                      = { (U32 *)OTG_FS, (U32 *)OTG_HS };
static U32 *OTG_DFIFO[2][OTG_HS_MAX_CH] = { { OTG_FS_DFIFO0,  OTG_FS_DFIFO1, 
                                              OTG_FS_DFIFO2,  OTG_FS_DFIFO3, 
                                              OTG_FS_DFIFO4,  OTG_FS_DFIFO5, 
                                              OTG_FS_DFIFO6,  OTG_FS_DFIFO7,
                                              0                               },
                                            { OTG_HS_DFIFO0,  OTG_HS_DFIFO1, 
                                              OTG_HS_DFIFO2,  OTG_HS_DFIFO3, 
                                              OTG_HS_DFIFO4,  OTG_HS_DFIFO5, 
                                              OTG_HS_DFIFO6,  OTG_HS_DFIFO7, 
                                              OTG_HS_DFIFO8,  OTG_HS_DFIFO9, 
                                              OTG_HS_DFIFO10, OTG_HS_DFIFO11, 
                                              OTG_HS_DFIFO12, OTG_HS_DFIFO13, 
                                              OTG_HS_DFIFO14, OTG_HS_DFIFO15  }};

/* Reserve memory for memory pools */
       U32       USBH0_MPOOL[(USBH0_STM32_SZ_MPOOL+3)>>2];
       U32       USBH1_MPOOL[(USBH1_STM32_SZ_MPOOL+3)>>2];

static BOOL      HW_Accessing   [2]                = { __FALSE };
static USBH_URB *CHURB          [2][OTG_HS_MAX_CH] = { 0 };
static U16       cntInterval    [2][OTG_HS_MAX_CH] = { 0 };
static U8        cntIntervalMax [2][OTG_HS_MAX_CH] = { 0 };
static U16       cntDebounce[2]                    = { 0 };
static U32       calDelay                          =   0  ;
static U32       Port_Discon_Evt[2]                = { 0 };
static U32       Port_Speed[2]                     = { 0 };
static U32       Port_Con[2]                       = { 0 };


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
  RCC->AHB1ENR  |=  (1 << 22);                    /* Enable DMA2 clock, DMA2  */
                                                  /* will be used to load     */
                                                  /* TX FIFOs                 */
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
  RCC->AHB1ENR  &= ~(1 << 22);                    /* Disable DMA2 clock       */
}
#endif


/*------------------------- USBH_STM32_DMA_Stop --------------------------------
 *
 *  Stop the DMA transfer
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

__inline static BOOL USBH_STM32_DMA_Stop (U8 ctrl) {
  S32 tout;
  DMA_Stream_TypeDef *ptr_DMA_Stream;

  if (!ctrl) {
    ptr_DMA_Stream = DMA2_Stream0;
  } else {
    ptr_DMA_Stream = DMA2_Stream1;
  }

  ptr_DMA_Stream->CR = (ctrl << 25);              /* Disable the DMA stream   */
  for (tout = 1010; tout >= 0; tout--) {          /* Wait max 100 ms          */
    if (!(ptr_DMA_Stream->CR & 1))                /* Wait for DMA to disable  */
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
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              ptrDest:    Pointer to Destination addres of transfer
 *              ptrSrc:     Pointer to Source address of transfer
 *              len:        Number of bytes to transfer
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

__inline static BOOL USBH_STM32_DMA_Start (U8 ctrl, U32 *ptrDest, U32 *ptrSrc, U16 len) {
  DMA_Stream_TypeDef *ptr_DMA_Stream;

  if (!ctrl) {
    ptr_DMA_Stream = DMA2_Stream0;
  } else {
    ptr_DMA_Stream = DMA2_Stream1;
  }

  DMA2->LIFCR          = (0x3D << (ctrl*6));      /* Clear DMA2 interrupts    */
  ptr_DMA_Stream->PAR  = (uint32_t)ptrSrc;        /* Load source address      */
  ptr_DMA_Stream->M1AR =  0;                      /* Not used                 */
  ptr_DMA_Stream->M0AR = (uint32_t)ptrDest;       /* Load destination address */
  ptr_DMA_Stream->NDTR = (len+3)/4;               /* Set size of transfer     */
  ptr_DMA_Stream->FCR  =  0;                      /* FIFO not used            */
  ptr_DMA_Stream->CR   = (ctrl << 25);            /* Select channel (CHSEL)   */
  ptr_DMA_Stream->CR  |= (0 << 23) |              /* MBURST = 0               */
                         (0 << 21) |              /* PBURST = 0               */
                         (0 << 19) |              /* CT = 0                   */
                         (0 << 18) |              /* DBM = 0                  */
                         (0 << 16) |              /* Priority Level low (PL=0)*/
                         (0 << 15) |              /* PINCOS = 0               */
                         (2 << 13) |              /* Memory size 32 bits      */
                         (2 << 11) |              /* Peripheral size 32 bits  */
                         (1 << 10) |              /* Memory increment enable  */
                         (1 <<  9) |              /* Peripheral increment en  */
                         (0 <<  8) |              /* CIRC = 0                 */
                         (2 <<  6) |              /* Direction mem-to-mem     */
                         (0 <<  5) |              /* DMA is flow controller   */
                         (0 <<  4) |              /* TCIE = 0                 */
                         (0 <<  3) |              /* HTIE = 0                 */
                         (0 <<  2) |              /* TEIE = 0                 */
                         (0 <<  1) ;              /* DMEIE = 0                */
  ptr_DMA_Stream->CR  |= (1 <<  0) ;              /* Channel enable           */

  return (__TRUE);
}


/*------------------------- USBH_STM32_DMA_Wait --------------------------------
 *
 *  Wait for the DMA transfer to finish and cleanup DMA for next transfer 
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

__inline static BOOL USBH_STM32_DMA_Wait (U8 ctrl) {
  S32 tout;
  DMA_Stream_TypeDef *ptr_DMA_Stream;

  if (!ctrl) {
    ptr_DMA_Stream = DMA2_Stream0;
  } else {
    ptr_DMA_Stream = DMA2_Stream1;
  }

  for (tout = 1010; tout >= 0; tout--) {          /* Wait max 100 ms          */
    if (!(ptr_DMA_Stream->CR & 1) || !(ptr_DMA_Stream->NDTR)) /* If completed */
      break;
    if (!tout) 
      return (__FALSE);
    if (tout <= 10)
      USBH_STM32_Delay_ms (10);
  }
  if (!USBH_STM32_DMA_Stop(ctrl))                 /* Stop the DMA transfer    */
    return (__FALSE);

  return (__TRUE);
}


/***----------------------- Pin Functions ----------------------------------***/

/*------------------------- usbh_stm32_cfg_pin ---------------------------------
 *
 *  Configurate USB pin
 *
 *  Parameter:  port:       Port number   ( A = 0, B = 1, ..)
 *              pin:        Pin number    ( 0 .. 15)
 *              param:      Pin parameter ( 0 ..  1: mode,
 *                                          2      : otype,
 *                                          3 ..  4: ospeed,
 *                                          5 ..  6: pupd,
 *                                          7 .. 10: af
 *                                         11      : output value for GPO mode)
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_stm32_cfg_pin (U32 port, U32 pin, U32 param) {
  GPIO_TypeDef *gpio;

  if (port >  8) return;
  if (pin  > 15) return;
  gpio  = ((GPIO_TypeDef *) (0x40020000 + 0x400*port));

  RCC->AHB1ENR      |=  (                 1  <<  port);
  if ((param & 3) == 1) {
    if ((param >> 11) & 1) { 
      gpio->BSRRL    =  (                 1 <<   pin);
    } else {
      gpio->BSRRH    =  (                 1 <<   pin);
    }
  }
  gpio->MODER       &= ~(                 3  << (pin<<1));
  gpio->MODER       |=  (((param >> 0) &  3) << (pin<<1));
  gpio->OTYPER      &= ~(                 1  <<  pin);
  gpio->OTYPER      |=  (((param >> 2) &  1) <<  pin);
  gpio->OSPEEDR     &= ~(                 3  << (pin<<1));
  gpio->OSPEEDR     |=  (((param >> 3) &  3) << (pin<<1));
  gpio->PUPDR       &= ~(                 3  << (pin<<1));
  gpio->PUPDR       |=  (((param >> 5) &  3) << (pin<<1));
  gpio->AFR[pin>=8] &= ~(                15  << ((pin&7)<<2));
  gpio->AFR[pin>=8] |=  (((param >> 7) & 15) << ((pin&7)<<2));
}


/*------------------------- usbh_stm32_uncfg_pin -------------------------------
 *
 *  Unconfigurate USB pin (configurate pin as after reset)
 *
 *  Parameter:  port:       Port number (A = 0, B = 1, ..)
 *              pin:        Pin number  (0 .. 15)
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_stm32_uncfg_pin (U32 port, U32 pin) {
  GPIO_TypeDef *gpio;

  if (port >  8) return;
  if (pin  > 15) return;
  gpio  = ((GPIO_TypeDef *) (0x40020000 + 0x400*port));  

  gpio->MODER       &= ~( 3  << (pin<<1));
  gpio->OTYPER      &= ~( 1  <<  pin);
  gpio->OSPEEDR     &= ~( 3  << (pin<<1));
  gpio->PUPDR       &= ~( 3  << (pin<<1));
  gpio->AFR[pin>=8] &= ~(15  << ((pin&7)<<2));
}


/***----------------------- Channel Functions ------------------------------***/

/*------------------------- USBH_STM32_CH_GetIndexFromCH -----------------------
 *
 *  Get the Index of Channel from it's Address
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              ptrCH:      Pointer to the Channel
 *  Return:                 Index of the Channel
 *----------------------------------------------------------------------------*/

static U32 USBH_STM32_CH_GetIndexFromCH (U8 ctrl, USBH_STM32_CH *ptrCH) {
  return (ptrCH - (USBH_STM32_CH *)(&((OTG_HS_TypeDef *)OTG[ctrl])->HCCHAR0));
}


/*------------------------- USBH_STM32_CH_GetCHFromIndex -----------------------
 *
 *  Get the Channel Address from it's Index
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              idx:        Index of the Channel
 *  Return:                 Address of the Channel
 *----------------------------------------------------------------------------*/

static USBH_STM32_CH *USBH_STM32_CH_GetCHFromIndex (U8 ctrl, U32 idx) {
  return ((USBH_STM32_CH *)(&((OTG_HS_TypeDef *)OTG[ctrl])->HCCHAR0) + idx);
}


/*------------------------- USBH_STM32_CH_FindFree -----------------------------
 *
 *  Find a free Channel
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *  Return:                 Pointer to the first free Channel
 *                          (0 = no free Channel is available)
 *----------------------------------------------------------------------------*/

static void *USBH_STM32_CH_FindFree (U8 ctrl) {
  USBH_STM32_CH *ptr_CH;
  U32            i, max_ch;

  ptr_CH = (USBH_STM32_CH *)(&((OTG_HS_TypeDef *)OTG[ctrl])->HCCHAR0);
  max_ch = (ctrl ? OTG_HS_MAX_CH : OTG_FS_MAX_CH);

  for (i = 0; i < max_ch; i++) {
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
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              ptrCH:      Pointer to the Channel
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_STM32_CH_Disable (U8 ctrl, USBH_STM32_CH *ptrCH) {
  S32 tout;

  if (!ptrCH) 
    return (__FALSE);

  ptrCH->HCINTMSK  =  0;
  USBH_STM32_Delay_ms (2);
  if (ptrCH->HCCHAR & USBH_STM32_HCCHAR_CHENA) {
    if (!(ptrCH->HCCHAR & USBH_STM32_HCCHAR_EPDIR) && (ptrCH->HCTSIZ)) {
      if (!USBH_STM32_DMA_Stop(ctrl))
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
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              ptrCH:      Pointer to the channel on which transfer will take place
 *              tgl_typ:    Toggle (bit 5..4: bit 5 - force toggle, bit 4 - value) and 
 *                          Packet type (bit 3..0: USBH_PACKET_IN, USBH_PACKET_OUT or USBH_PACKET_SETUP)
 *              buf:        Start of the receive or transmit data buffer
 *              len:        Length of the data to be received or sent
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

static BOOL USBH_STM32_CH_TransferEnqueue (U8 ctrl, USBH_STM32_CH *ptrCH, U32 tgl_typ, U8 *buf, U32 len) {
  U32  hcchar;
  U32  hctsiz;
  U32  hcintmsk;
  U32  mpsiz;
  U32  ch_idx;
  BOOL load_data;

  if (!ptrCH)
    return (__FALSE);

  if (!Port_Con[ctrl])
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
    case USBH_PACKET_PING:
      hcchar   &= ~USBH_STM32_HCCHAR_EPDIR;
      hcintmsk  =  USBH_STM32_HCINTMSK_TXERRM | 
                   USBH_STM32_HCINTMSK_ACKM   | 
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
  if ((tgl_typ & 0x0F) == USBH_PACKET_PING) {     /* If OUT pckt DOPING       */
    hctsiz |= USBH_STM32_HCTSIZ_DOPING;
  }

  mpsiz = hcchar & 0x7FF;                         /* Maximum packet size      */
  if (len) {                                      /* Normal packet            */
    hctsiz |= ((len+mpsiz-1) / mpsiz) << 19;      /* Prepare PKTCNT field     */
    hctsiz |= ( len                 ) <<  0;      /* Prepare XFRSIZ field     */
  } else {                                        /* Zero length packet       */
    hctsiz |= ( 1                   ) << 19;      /* Prepare PKTCNT field     */
    hctsiz |= ( 0                   ) <<  0;      /* Prepare XFRSIZ field     */
  }

  ch_idx  = USBH_STM32_CH_GetIndexFromCH (ctrl, ptrCH);

  ptrCH->HCINTMSK = hcintmsk;                     /* Enable channel interrupts*/
  ptrCH->HCTSIZ   = hctsiz;                       /* Write ch transfer size   */

  /* load_data == __TRUE if there is data to be loaded to FIFO 
    (If packet is OUT or SETUP and len > 0)                                   */
  load_data = (((tgl_typ & 0x0F) == USBH_PACKET_OUT)    || 
               ((tgl_typ & 0x0F) == USBH_PACKET_SETUP)) && 
                 len; 

  if (load_data)
    USBH_STM32_DMA_Wait(ctrl);

  ptrCH->HCCHAR = hcchar;                         /* Write ch characteristics */

  if (load_data)
    USBH_STM32_DMA_Start (ctrl, OTG_DFIFO[ctrl][ch_idx], (U32 *)(buf), len);

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
    calDelay = SystemCoreClock / (vals - vale);   /* Calibrated value         */
    if (vals == 0xFFFFFF)               /* Stop timer if we started it        */
      SysTick->CTRL  = 0;
    goto start;
  }
}


/*------------------------- USBH_STM32_Pins_Config -----------------------------
 *
 *  Configurate or unconfigurate pins used by the USB Host
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              on:         __TRUE = configurate, __FALSE = unconfigurate
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Pins_Config (U8 ctrl, BOOL on) {
  if (!ctrl) {
    if (on) {
      usbh_stm32_cfg_pin (PORT_A, 10, (2)|(0<<3)|(10 << 7));/* PA10: OTG_FS_ID*/
      usbh_stm32_cfg_pin (PORT_A, 11, (2)|(3<<3)|(10 << 7));/* PA11: OTG_FS_DM*/
      usbh_stm32_cfg_pin (PORT_A, 12, (2)|(3<<3)|(10 << 7));/* PA12: OTG_FS_DP*/
#ifdef __XYNERGY
      usbh_stm32_cfg_pin (PORT_B,  2, (1)|(1<<11));         /* PB2:  OTG_SW_ON*/
#else
      usbh_stm32_cfg_pin (PORT_H,  5, (1)|(1<<11));         /* PH5:  OTG_FS_PS*/
#endif
      USBH_STM32_Port_Power (0, __FALSE);         /* Turn off port power      */
    } else {
#ifdef __XYNERGY
      usbh_stm32_uncfg_pin (PORT_B,  2);          /* PB2:  reset mode         */
#else
      usbh_stm32_uncfg_pin (PORT_H,  5);          /* PH5:  reset mode         */
#endif
      usbh_stm32_uncfg_pin (PORT_A, 10);          /* PA10: reset mode         */
      usbh_stm32_uncfg_pin (PORT_A, 11);          /* PA11: reset mode         */
      usbh_stm32_uncfg_pin (PORT_A, 12);          /* PA12: reset mode         */
    }
  } else if (ctrl == 1) {
    if (on) {
#ifdef __OTG_HS_EMBEDDED_PHY
      usbh_stm32_cfg_pin (PORT_B, 14, (2)|(3<<3)|(12 << 7));/* PB14: OTG_HS_DM*/
      usbh_stm32_cfg_pin (PORT_B, 15, (2)|(3<<3)|(12 << 7));/* PB15: OTG_HS_DP*/
#else
      usbh_stm32_cfg_pin (PORT_H,  4, (2)|(3<<3)|(10 << 7));/* PH4:  ULPI_NXT */
      usbh_stm32_cfg_pin (PORT_I, 11, (2)|(3<<3)|(10 << 7));/* PI11: ULPI_DIR */
      usbh_stm32_cfg_pin (PORT_C,  0, (2)|(3<<3)|(10 << 7));/* PC0:  ULPI_STP */
      usbh_stm32_cfg_pin (PORT_A,  5, (2)|(3<<3)|(10 << 7));/* PA5:  ULPI_CLK */
      usbh_stm32_cfg_pin (PORT_B,  5, (2)|(3<<3)|(10 << 7));/* PB5:  ULPI_D7  */
      usbh_stm32_cfg_pin (PORT_B, 13, (2)|(3<<3)|(10 << 7));/* PB13: ULPI_D6  */
      usbh_stm32_cfg_pin (PORT_B, 12, (2)|(3<<3)|(10 << 7));/* PB12: ULPI_D5  */
      usbh_stm32_cfg_pin (PORT_B, 11, (2)|(3<<3)|(10 << 7));/* PB11: ULPI_D4  */
      usbh_stm32_cfg_pin (PORT_B, 10, (2)|(3<<3)|(10 << 7));/* PB10: ULPI_D3  */
      usbh_stm32_cfg_pin (PORT_B,  1, (2)|(3<<3)|(10 << 7));/* PB1:  ULPI_D2  */
      usbh_stm32_cfg_pin (PORT_B,  0, (2)|(3<<3)|(10 << 7));/* PB0:  ULPI_D1  */
      usbh_stm32_cfg_pin (PORT_A,  3, (2)|(3<<3)|(10 << 7));/* PA5:  ULPI_D0  */
#ifdef __MCBSTM32F200
      usbh_stm32_cfg_pin (PORT_C,  2, (1)|(1<<2)|(1 << 11));/* PC2:  OD output*/
#endif
#endif
      USBH_STM32_Port_Power (1, __FALSE);         /* Turn off port power      */
    } else {
#ifdef __OTG_HS_EMBEDDED_PHY
      usbh_stm32_uncfg_pin (PORT_B, 14);          /* PB14: reset mode         */
      usbh_stm32_uncfg_pin (PORT_B, 15);          /* PB15: reset mode         */
#else
      usbh_stm32_uncfg_pin (PORT_H,  4);          /* PH4:  reset mode         */
      usbh_stm32_uncfg_pin (PORT_I, 11);          /* PI11: reset mode         */
      usbh_stm32_uncfg_pin (PORT_C,  0);          /* PC0:  reset mode         */
      usbh_stm32_uncfg_pin (PORT_A,  5);          /* PA5:  reset mode         */
      usbh_stm32_uncfg_pin (PORT_B,  5);          /* PB5:  reset mode         */
      usbh_stm32_uncfg_pin (PORT_B, 13);          /* PB13: reset mode         */
      usbh_stm32_uncfg_pin (PORT_B, 12);          /* PB12: reset mode         */
      usbh_stm32_uncfg_pin (PORT_B, 11);          /* PB11: reset mode         */
      usbh_stm32_uncfg_pin (PORT_B, 10);          /* PB10: reset mode         */
      usbh_stm32_uncfg_pin (PORT_B,  1);          /* PB1:  reset mode         */
      usbh_stm32_uncfg_pin (PORT_B,  0);          /* PB0:  reset mode         */
      usbh_stm32_uncfg_pin (PORT_A,  3);          /* PA5:  reset mode         */
#ifdef __MCBSTM32F200
      usbh_stm32_uncfg_pin (PORT_C,  2);          /* PC2:  reset mode         */
#endif
#endif
    }
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_Init ------------------------------------
 *
 *  Initialize or uninitialize the USB Host Controller
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Init (U8 ctrl, BOOL on) {
  OTG_HS_TypeDef *ptr_OTG;
  S32             tout;

  ptr_OTG = (OTG_HS_TypeDef *)OTG[ctrl];

  if (on) {
    USBH_STM32_Delay_ms (10);                     /* Delay to calibrate delay */
    if (!ctrl) {
      /* Initialize memory pool for data                                      */
      if (!usbh_mem_init(0, (U32 *)&USBH0_MPOOL, USBH0_STM32_SZ_MPOOL))
        return (__FALSE);

      RCC->AHB2ENR     |=  (   1 <<  7);          /* OTG FS clock enable      */
      USBH_STM32_Delay_ms (10);                   /* Wait ~10 ms              */
      RCC->AHB2RSTR    |=  (   1 <<  7);          /* Reset OTG FS clock       */
      USBH_STM32_Delay_ms (10);                   /* Wait ~10 ms              */
      RCC->AHB2RSTR    &= ~(   1 <<  7);
      USBH_STM32_Delay_ms (40);                   /* Wait ~40 ms              */
    } else if (ctrl == 1) {
      /* Initialize memory pool for data                                      */
      if (!usbh_mem_init(1, (U32 *)&USBH1_MPOOL, USBH1_STM32_SZ_MPOOL))
        return (__FALSE);

      RCC->AHB1ENR     |=  (   1 << 29);          /* OTG HS clock enable      */
      USBH_STM32_Delay_ms (10);                   /* Wait ~10 ms              */
      RCC->AHB1RSTR    |=  (   1 << 29);          /* Reset OTG HS clock       */
      USBH_STM32_Delay_ms (10);                   /* Wait ~10 ms              */
      RCC->AHB1RSTR    &= ~(   1 << 29);
      USBH_STM32_Delay_ms (40);                   /* Wait ~40 ms              */
#ifndef __OTG_HS_EMBEDDED_PHY
      RCC->AHB1ENR     |=  (   1 << 30);          /* OTG HS ULPI clock enable */
#endif
    }

    if (!ctrl) {
      ptr_OTG->GUSBCFG |=  1 << 6;                        /* Full-spd trnscvr */
    } else {
#ifndef __OTG_HS_EMBEDDED_PHY
      ptr_OTG->GUSBCFG &=~(1 << 6);                       /* High-spd trnscvr */
      ptr_OTG->GUSBCFG |=  USBH_STM32_GUSBCFG_PTCI       |/* Ind. pass through*/
                           USBH_STM32_GUSBCFG_PCCI       |/* Ind. complement  */
                           USBH_STM32_GUSBCFG_ULPIEVBUSI |/* ULPI ext Vbus ind*/
                           USBH_STM32_GUSBCFG_ULPIEVBUSD; /* ULPI ext Vbus drv*/
#else
      ptr_OTG->GUSBCFG |=   1 << 6;                       /* Full-spd trnscvr */
      ptr_OTG->GCCFG   &= ~(1UL << 19); /* Disable VBUS sensing of "B" device */
      ptr_OTG->GCCFG   &= ~(1UL << 18); /* Disable VBUS sensing of "A" device */
      ptr_OTG->GCCFG   |=  (1UL << 21); /* No VBUS sensing                    */
#endif
    }
    USBH_STM32_Delay_ms (20);                     /* Wait ~20 ms              */

    for (tout = 1100; tout >= 0; tout--) {        /* Wait max 1 second        */
      if (ptr_OTG->GRSTCTL & (1UL << 31))         /* Wait for AHB Idle        */
        break;
      if (!tout) 
        return (__FALSE);
      if (tout <= 100)
        USBH_STM32_Delay_ms (10);
    }
    ptr_OTG->GRSTCTL |= 1;                        /* USB Core Reset           */
    for (tout = 1100; tout >= 0; tout--) {        /* Wait max 1 second        */
      if (!(ptr_OTG->GRSTCTL & 1))                /* Wait for Core Reset end  */
        break;
      if (!tout) 
        return (__FALSE);
      if (tout <= 100)
        USBH_STM32_Delay_ms (10);
    }
    USBH_STM32_Delay_ms (3);                      /* Wait ~3 ms               */

    ptr_OTG->GAHBCFG   &= ~USBH_STM32_GAHBCFG_GINTMSK;    /* Disable ints     */

    if (!ctrl) {
      ptr_OTG->GCCFG   |=  USBH_STM32_GCCFG_SOFOUTEN |    /* Enable SOF output*/
                           USBH_STM32_GCCFG_PWRDWN   ;    /* Disable power dwn*/
    } else {
      ptr_OTG->GCCFG   |=  
#ifdef __OTG_HS_EMBEDDED_PHY
                           USBH_STM32_GCCFG_NOVBUSSENS|   /* No VBUS sensing  */
#endif
                           USBH_STM32_GCCFG_SOFOUTEN |    /* Enable SOF output*/
                           USBH_STM32_GCCFG_PWRDWN   ;    /* Disable power dwn*/
    }

    USBH_STM32_DMA_Enable ();

    ptr_OTG->GUSBCFG   &=~(USBH_STM32_GUSBCFG_TRDT(15));
    ptr_OTG->GUSBCFG   |=  USBH_STM32_GUSBCFG_FHMOD   |   /* Force host mode  */
                           USBH_STM32_GUSBCFG_TRDT(5);    /* Turnaround time  */
    USBH_STM32_Delay_ms (100);                    /* Wait ~100 ms             */

    /* Core initialization                                                    */
    ptr_OTG->GRXFSIZ    =  (512/4) +                /* RxFIFO depth is 512 by */
                                2  +                /* 8 bytes for Int EP     */
                                4  ;                /* Packet info and status */
    if (!ctrl) 
      OTG_FS->HNPTXFSIZ = ((512/4)<<16) | ((512/4)+6);    /* Non-peri TxFIFO  */
    else if (ctrl == 1)
      ptr_OTG->GNPTXFSIZ= ((512/4)<<16) | ((512/4)+6);    /* Non-peri TxFIFO  */
    ptr_OTG->HPTXFSIZ   = (16<<16) |  (((512/4)*2)+6);    /* Peri TxFIFO mem  */

    ptr_OTG->GINTMSK   |=  USBH_STM32_GINTMSK_DISCINT|    /* En disconn int   */
                           USBH_STM32_GINTMSK_HCIM   |    /* En host ch int   */
                           USBH_STM32_GINTMSK_PRTIM  |    /* En host prt int  */
                           USBH_STM32_GINTMSK_RXFLVLM|    /* Enable RXFIFO int*/
                           USBH_STM32_GINTMSK_SOFM   ;    /* Enable SOF int   */
    if (!(ptr_OTG->HCFG & 3)) {
      if (!ctrl) {
        ptr_OTG->HCFG   =  USBH_STM32_HCFG_FSLSPCS(1)|    /* PHY clk at 48MHz */
                           USBH_STM32_HCFG_FSLS(1);       /* FS/LS only       */
      } else if (ctrl == 1) {
#ifdef __OTG_HS_EMBEDDED_PHY
        ptr_OTG->HCFG   =  USBH_STM32_HCFG_FSLSPCS(1)|    /* PHY clk at 48MHz */
                           USBH_STM32_HCFG_FSLS(1);       /* FS/LS only       */
#else
        ptr_OTG->HCFG   =  USBH_STM32_HCFG_FSLSPCS(1);    /* PHY clk at 48MHz */
#endif
      }
    }
    ptr_OTG->HAINTMSK   =  0xFFFF;                        /* En all ch ints   */

    if (!ctrl) {
      ptr_OTG->HAINTMSK =  0xFF;                          /* En all ch ints   */
      NVIC_SetPriority (OTG_FS_IRQn, 0);/* Set OTG interrupt highest priority */
      NVIC_EnableIRQ   (OTG_FS_IRQn);   /* Enable OTG interrupt               */
    } else if (ctrl == 1) {
      ptr_OTG->HAINTMSK =  0xFFFF;                        /* En all ch ints   */
      NVIC_SetPriority (OTG_HS_IRQn, 0);/* Set OTG interrupt highest prio     */
      NVIC_EnableIRQ   (OTG_HS_IRQn);   /* Enable OTG interrupt               */
    }
    ptr_OTG->GAHBCFG   |=  USBH_STM32_GAHBCFG_GINTMSK;    /* Enable interrupts*/
  } else {
    if (!ctrl) {
      NVIC_DisableIRQ  (OTG_FS_IRQn);   /* Disable OTG interrupt              */

      ptr_OTG->HAINTMSK&= ~0xFF;        /* Disable channel interrupts         */
      RCC->AHB2RSTR    |= (   1 <<  7); /* OTG FS reset                       */
      USBH_STM32_Delay_ms (10);         /* Wait ~10 ms                        */
      RCC->AHB2RSTR    &=~(   1 <<  7); /* OTG FS not reset                   */
      USBH_STM32_Delay_ms (10);         /* Wait ~10 ms                        */
      RCC->AHB2ENR     &=~(   1 <<  7); /* OTG FS clock disable               */
      USBH_STM32_Delay_ms (40);         /* Wait ~40 ms                        */
    } else if (ctrl == 1) {
      NVIC_DisableIRQ  (OTG_HS_IRQn);   /* Disable OTG interrupt              */

      ptr_OTG->HAINTMSK&= ~0xFFFF;      /* Disable all channel interrupts     */
      RCC->AHB1RSTR    |= (   1 << 29); /* OTG HS reset                       */
      USBH_STM32_Delay_ms (10);         /* Wait ~10 ms                        */
      RCC->AHB1RSTR    &=~(   1 << 29); /* OTG HS not reset                   */
      USBH_STM32_Delay_ms (10);         /* Wait ~10 ms                        */
      RCC->AHB1ENR     &=~(   1 << 29); /* OTG HS clock disable               */
      RCC->AHB1ENR     &=~(   1 << 30); /* OTG HS ULPI clock disable          */
      USBH_STM32_Delay_ms (40);         /* Wait ~40 ms                        */
    }

    ptr_OTG->HPRT       =  0;                             /* Rst host prt ctrl*/
    ptr_OTG->HCFG       =  0;                             /* Reset host config*/
    ptr_OTG->GINTMSK   &=~(USBH_STM32_GINTMSK_DISCINT|    /* Disable ints     */
                           USBH_STM32_GINTMSK_HCIM   |
                           USBH_STM32_GINTMSK_PRTIM  |
                           USBH_STM32_GINTMSK_RXFLVLM|
                           USBH_STM32_GINTMSK_SOFM  );
    ptr_OTG->GUSBCFG   &=~(USBH_STM32_GUSBCFG_FHMOD  |    /* Reset USB cfg    */
                           USBH_STM32_GUSBCFG_TRDT(15));
    if (!ctrl)
      ptr_OTG->GUSBCFG &= ~USBH_STM32_GUSBCFG_PHYSEL; 
    ptr_OTG->GUSBCFG   |=  USBH_STM32_GUSBCFG_TRDT(2);    /* Set init values  */
    calDelay = 0;                                 /* Force delay recalibrate  */
    USBH_STM32_Delay_ms (100);          /* Wait ~100 ms                       */
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_Port_Power ------------------------------
 *
 *  Turn USB Host port power on or off
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              on:         __TRUE = power on, __FALSE = power off
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Port_Power (U8 ctrl, BOOL on) {
  OTG_HS_TypeDef *ptr_OTG;
  
  ptr_OTG = (OTG_HS_TypeDef *)OTG[ctrl];

  if (on) {
    ptr_OTG->HPRT    |=  USBH_STM32_HPRT_PPWR;  /* Port power on              */
    if (!ctrl) {
#ifdef __XYNERGY
      GPIOB->BSRRH   =  (   1 <<  2);           /* PB2 drive low              */
#else
      GPIOH->BSRRH   =  (   1 <<  5);           /* PH5 drive low              */
#endif
    }
#ifdef __MCBSTM32F200
    else {
      GPIOC->BSRRH   =  (   1 <<  2);           /* PC2 drive low              */
    }
#endif
  } else {
    if (!ctrl){
#ifdef __XYNERGY
      GPIOB->BSRRL   =  (   1 <<  2);           /* PB2 drive high             */
#else
      GPIOH->BSRRL   =  (   1 <<  5);           /* PH5 drive high             */
#endif
    }
#ifdef __MCBSTM32F200
    else {
      GPIOC->BSRRL   =  (   1 <<  2);           /* PC2 drive  high            */
    }
#endif
    ptr_OTG->HPRT    &= ~USBH_STM32_HPRT_PPWR;  /* Port power off             */
  }

  return (__TRUE);
}


/*------------------------- USBH_STM32_Port_Reset ------------------------------
 *
 *  Reset Port
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              port:       Root Hub port to be reset (only 0 is available)
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_Port_Reset (U8 ctrl, U8 port) {
  OTG_HS_TypeDef *ptr_OTG;
  U32             hcfg, hprt;
  S32             tout;

  if (!Port_Con[ctrl])
    return (__FALSE);
  
  ptr_OTG = (OTG_HS_TypeDef *)OTG[ctrl];

  hcfg = ptr_OTG->HCFG;
  hprt = ptr_OTG->HPRT;
  switch ((hprt >> 17) & 3) {
    case 0:                             /* High-speed detected                */
      ptr_OTG->HFIR   = 48000;
      if ((hcfg & 3)  != 1) {
        hcfg          = (hcfg & ~USBH_STM32_HCFG_FSLSPCS(3)) | USBH_STM32_HCFG_FSLSPCS(1);
        ptr_OTG->HCFG =  hcfg;
      }
      break;
    case 1:                             /* Full-speed detected                */
      ptr_OTG->HFIR   = 48000;
      if ((hcfg & 3)  != 1) {
        hcfg          = (hcfg & ~USBH_STM32_HCFG_FSLSPCS(3)) | USBH_STM32_HCFG_FSLSPCS(1);
        ptr_OTG->HCFG =  hcfg;
      }
      break;
    case 2:                             /* Low-speed detected                 */
      ptr_OTG->HFIR   = 6000;
      if ((hcfg & 3)  != 2) {
        hcfg          = (hcfg & ~USBH_STM32_HCFG_FSLSPCS(3)) | USBH_STM32_HCFG_FSLSPCS(2);
        ptr_OTG->HCFG =  hcfg;
      }
      break;
    case 3:
      break;
  }

  ptr_OTG->HPRT |=  USBH_STM32_HPRT_PRST;             /* Port reset           */
  USBH_STM32_Delay_ms (17);                           /* Wait ~17 ms          */
  ptr_OTG->HPRT &= ~USBH_STM32_HPRT_PRST;             /* Clear port reset     */

  for (tout = 10100; tout >= 0; tout--) {             /* Wait for max 1 s     */
    if ((ptr_OTG->HPRT & USBH_STM32_HPRT_PENA))       /* If port enabled      */
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
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *  Return:                 Connection/Disconnection events
 *----------------------------------------------------------------------------*/

U32 USBH_STM32_Get_Connect (U8 ctrl) {
  U32 ret, stat;

  stat = (((OTG_HS_TypeDef *)OTG[ctrl])->HPRT & USBH_STM32_HPRT_PCSTS);
  ret  = Port_Discon_Evt[ctrl];
  ret |= ((stat ^ Port_Con[ctrl]) && (Port_Con[ctrl]));

  if (ret){                             /* If port disconnect occured         */
    Port_Discon_Evt[ctrl] &= ~ret;
    return (ret << 16);
  }

  if (cntDebounce[ctrl]) {
    cntDebounce[ctrl]--;
    if (!cntDebounce[ctrl]) {           /* If debounce time expired           */
      ret = ((stat ^ Port_Con[ctrl]) && (!Port_Con[ctrl]));
                                        /* If debounce expired on conn 0 -> 1 */
      Port_Con[ctrl] |= ret;
    } else {
      USBH_STM32_Delay_ms (1);
    }
  } else if ((stat ^ Port_Con[ctrl]) && (!Port_Con[ctrl])) {
                                        /* Restart debouncing if connect      */
    cntDebounce[ctrl] = DEBOUNCE_500ms;
  }

  return (ret);
}


/*------------------------- USBH_STM32_Get_Speed -------------------------------
 *
 *  Returns port speeds
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *  Return:                 Port speeds
 *----------------------------------------------------------------------------*/

U32 USBH_STM32_Get_Speed (U8 ctrl) {
  return (Port_Speed[ctrl]); 
}


/*------------------------- USBH_STM32_EP_Add ----------------------------------
 *
 *  Add the Endpoint and return handle (address of the Endpoint)
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 Handle to the Configured Endpoint (0 = FAIL)
 *----------------------------------------------------------------------------*/

U32 USBH_STM32_EP_Add (U8 ctrl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {
  USBH_STM32_CH *ptr_CH;

  ptr_CH = (USBH_STM32_CH *)(USBH_STM32_CH_FindFree (ctrl));  /* Find free Ch */
  if (!ptr_CH) {                                              /* If no free   */
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
      cntIntervalMax[ctrl][USBH_STM32_CH_GetIndexFromCH (ctrl, ptr_CH)] = ptrEPD->bInterval;
      ptr_CH->HCCHAR |= USBH_STM32_HCCHAR_MCNT1;
      break;
  }

  return ((U32)ptr_CH);
}


/*------------------------- USBH_STM32_EP_Config -------------------------------
 *
 *  (Re)Configure some parameters of the Endpoint
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              hndl:       Handle to the Configured Endpoint
 *              dev_adr:    Device Address
 *              ep_spd:     Endpoint Speed
 *              ptrEPD:     Pointer to the USB Standard Endpoint Descriptor
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_EP_Config (U8 ctrl, U32 hndl, U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptrEPD) {
  USBH_STM32_CH *ptr_CH;

  if (!hndl) 
    return (__FALSE);

  ptr_CH = (USBH_STM32_CH *)(hndl);
  if (!USBH_STM32_CH_Disable (ctrl, ptr_CH)) 
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
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              hndl:       Handle to the Configured Endpoint
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_EP_Remove (U8 ctrl, U32 hndl) {
  USBH_STM32_CH *ptr_CH;
  USBH_URB      *ptr_URB;
  U32            ch_idx;

  if (!hndl) 
    return (__FALSE);

  ptr_CH = (USBH_STM32_CH *)(hndl);
  ch_idx              = USBH_STM32_CH_GetIndexFromCH (ctrl, ptr_CH);
  ptr_URB = CHURB[ctrl][ch_idx];        /* Pointer to channels URB            */
  if (ptr_URB) {                        /* If URB exists cancel it            */
    if (!USBH_STM32_URB_Cancel (ctrl, hndl, ptr_URB)) 
      return (__FALSE);
  }

  ptr_CH->HCCHAR      = 0;
  ptr_CH->HCINT       = 0;
  ptr_CH->HCINTMSK    = 0;
  ptr_CH->HCTSIZ      = 0;

  cntInterval[ctrl][ch_idx] = 0;

  return (__TRUE);
}


/*------------------------- USBH_STM32_URB_Submit ------------------------------
 *
 *  Submit the URB (USB Request Block) to be processed
 *
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_URB_Submit (U8 ctrl, U32 hndl, USBH_URB *ptrURB) {
  USBH_STM32_CH *ptr_CH;
  U32            ch_idx;
  U8             resp_type;
  U16            max_cnt;

  if (!hndl) 
    return (__FALSE);

  if (!ptrURB) 
    return (__FALSE);

  if (!Port_Con[ctrl])
    return (__FALSE);

  if ((ptrURB->Submitted == 1) || (ptrURB->InProgress == 1)) 
    return (__FALSE);

  resp_type               = ptrURB->ResponsePacketType;

  ptrURB->DataTransferred = 0;
  ptrURB->Status          = 0;

  ptrURB->TimeoutCount    = MAX_TIMEOUT_COUNT;
  ptr_CH                  = (USBH_STM32_CH *)(hndl);
  ch_idx                  = USBH_STM32_CH_GetIndexFromCH (ctrl, ptr_CH);
  CHURB[ctrl][ch_idx]     = ptrURB;

  if (ptr_CH->EPTYP == USB_ENDPOINT_TYPE_INTERRUPT) {
                                        /* If interrupt endpoint transfer     */
    if (resp_type == USBH_PACKET_NAK) {
      if (Port_Speed[ctrl] == USBH_HS) {
        max_cnt = (cntIntervalMax[ctrl][ch_idx] & 0x0F);
        if (!max_cnt)
          max_cnt = 1;
        max_cnt = 1 << max_cnt;
      } else if ((Port_Speed[ctrl] == USBH_FS) || (Port_Speed[ctrl] == USBH_LS)) {
        max_cnt = (cntIntervalMax[ctrl][ch_idx] & 0xFF);
        if (!max_cnt)
          max_cnt = 1;
      }
      cntInterval[ctrl][ch_idx] = max_cnt;
    } else {
      cntInterval[ctrl][ch_idx] = 1;    /* Enable transmission on next SOF    */
    }
    ptrURB->Submitted     = 1;
  } else if ((ptr_CH->EPTYP == USB_ENDPOINT_TYPE_CONTROL) || (ptr_CH->EPTYP == USB_ENDPOINT_TYPE_BULK)) {
    if ((Port_Speed[ctrl] & 3) != USBH_LS) {
      HW_Accessing[ctrl]  = __TRUE;
      ptrURB->Submitted   = 1;
      ptrURB->InProgress  = 1;
      USBH_STM32_CH_TransferEnqueue (ctrl, ptr_CH, (U8)ptrURB->Parameters, (U8 *)ptrURB->ptrDataBuffer, (U32)ptrURB->DataLength);
      HW_Accessing[ctrl]  = __FALSE;
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
 *  Parameter:  ctrl:       Controller index (0 = OTG_FS, 1 = OTG_HS)
 *              hndl:       Endpoint handle (Endpoint address in memory)
 *              ptrURB:     Pointer to the URB
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL USBH_STM32_URB_Cancel (U8 ctrl, U32 hndl, USBH_URB *ptrURB) {
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
    if (!USBH_STM32_CH_Disable (ctrl, ptr_CH))    /* Stop Endpoint processing */
      return (__FALSE);
    ptrURB->InProgress  = 0;
  }

  ch_idx = USBH_STM32_CH_GetIndexFromCH (ctrl, ptr_CH);

  if (!cntInterval[ctrl][ch_idx]) {     /* If interrupt endpoint transfer     */
    cntInterval[ctrl][ch_idx] = 0;      /* Disable transmission on next SOF   */
  }
  CHURB[ctrl][ch_idx] = 0;

  ptrURB->Cancelled = 1;

  return (__TRUE);
}


/*------------------------- OTG_xS_IRQHandler ----------------------------------
 *
 *  STM32 USB Interrupt Handler Routines
 *
 *  Parameter:
 *  Return:
 *----------------------------------------------------------------------------*/

void OTG_xS_IRQHandler (U8 ctrl) {
  OTG_HS_TypeDef *ptr_OTG;
  USBH_STM32_CH  *ptr_CH;
  USBH_URB       *ptr_URB;
  U8             *ptrData8;
  U32            *ptrData32;
  U32            *DFIFO;
  U32             gintsts, hprt, haint, hcint, pktcnt, mpsiz;
  U32             grxsts, bcnt, ch, dat, len, len_rest, max_ch;
  U32             act;

  ptr_OTG = (OTG_HS_TypeDef *)OTG[ctrl];

  /* Read global interrupt register                                           */
  gintsts =  ptr_OTG->GINTSTS & ptr_OTG->GINTMSK;
  hprt    =  ptr_OTG->HPRT;
  haint   =  ptr_OTG->HAINT;
  max_ch  = (ctrl ?  OTG_HS_MAX_CH  : OTG_FS_MAX_CH);

  /* Analyze interrupt                                                        */
  if (gintsts & USBH_STM32_GINTSTS_SOF) {         /* If start of frame int    */
    for (ch = 0; ch < max_ch; ch++) {
      if (CHURB[ctrl][ch]) {
        if (CHURB[ctrl][ch]->TimeoutCount) {      /* If timeout not 0         */
          if (!(--CHURB[ctrl][ch]->TimeoutCount)) {
            CHURB[ctrl][ch]->Timeout = 1;         /* If timeout expired       */
          }
        }
      }
    }
  }

  if (gintsts&USBH_STM32_GINTSTS_HPRTINT) {       /* If host port interrupt   */
    if (hprt & USBH_STM32_HPRT_PCDET) {           /* Port connect detected    */
      if (!Port_Con[ctrl]) {
        cntDebounce[ctrl] = DEBOUNCE_500ms;
      }
    }
    if (hprt & USBH_STM32_HPRT_PENCHNG) {         /* If port enable changed   */
      if (hprt & USBH_STM32_HPRT_PENA) {          /* If device connected      */
        Port_Speed[ctrl] &= ~3;
        switch ((hprt >> 17) & 3) {
          case 0:
            Port_Speed[ctrl] |= USBH_HS;
            break;
          case 1:
            Port_Speed[ctrl] |= USBH_FS;
            break;
          case 2:
            Port_Speed[ctrl] |= USBH_LS;
            break;
          case 3:
            break;
        }
      } 
      if (!(hprt & USBH_STM32_HPRT_PCSTS) &&      /* If device disconnected   */
            Port_Con[ctrl]) { 
        Port_Discon_Evt[ctrl] = 1;
        Port_Con[ctrl]        = 0;
      }
    }
    hprt &= ~USBH_STM32_HPRT_PENA;                /* Leave PENA bit           */
    ptr_OTG->HPRT = hprt;                         /* Clear host port interrupt*/
  }
  if ((gintsts & USBH_STM32_GINTSTS_DISCINT) &&   /* If device disconnected   */
       Port_Con[ctrl]) { 
    Port_Discon_Evt[ctrl] = 1;
    Port_Con[ctrl]        = 0;
  }

  if (Port_Discon_Evt[ctrl]) {                    /* If disconnect detected   */
    ptr_CH = (USBH_STM32_CH *)(&(ptr_OTG->HCCHAR0));
    for (ch = 0; ch < max_ch; ch++) {
      if (CHURB[ctrl][ch]) 
        USBH_STM32_URB_Cancel (ctrl, (U32)USBH_STM32_CH_GetCHFromIndex (ctrl, ch), CHURB[ctrl][ch]);
      ptr_CH++;
    }
  }
                                                  /* Handle reception int     */
  if (gintsts & USBH_STM32_GINTSTS_RXFLVL) {      /* If RXFIFO non-empty int  */
    ptr_OTG->GINTMSK &= ~USBH_STM32_GINTMSK_RXFLVLM;
    grxsts = ptr_OTG->GRXSTSR;
    if (((grxsts >> 17) & 0x0F) == 0x02){         /* If PKTSTS = 0x02         */
      grxsts     = (ptr_OTG->GRXSTSP);
      ch         = (grxsts >> 0) & 0x00F;
      bcnt       = (grxsts >> 4) & 0x7FF;
      ptr_CH     = USBH_STM32_CH_GetCHFromIndex (ctrl, ch);
      DFIFO      = OTG_DFIFO[ctrl][ch];
      ptr_URB    = CHURB[ctrl][ch];               /* Pointer to channels URB  */
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
      grxsts     = ptr_OTG->GRXSTSP;
    }
    ptr_OTG->GINTMSK |= USBH_STM32_GINTMSK_RXFLVLM;
  }
                                                  /* Handle transmission int  */
  if (gintsts & USBH_STM32_GINTSTS_HCINT) {       /* If host channel interrupt*/
    ptr_CH = (USBH_STM32_CH *)(&ptr_OTG->HCCHAR0);
    for (ch = 0; ch < max_ch; ch++) {
      if (haint & (1 << ch)) {                    /* If channels interrupt act*/
        hcint   = ptr_CH->HCINT & ptr_CH->HCINTMSK;
        ptr_URB = CHURB[ctrl][ch];                /* Pointer to channels URB  */
        ptr_URB->Error |= (hcint & USBH_STM32_HCINT_ERR) >> 7;
        if ((ptr_URB->PacketType == USBH_PACKET_OUT) ||     /* If OUT packet  ----------*/
            (ptr_URB->PacketType == USBH_PACKET_SETUP)) {   /* or SETUP packet----------*/
          if (hcint & USBH_STM32_HCINT_XFRC) {    /* If data transfer finished*/
            USBH_STM32_DMA_Stop(ctrl);            /* Stop the DMA transfer    */
            ptr_CH->HCINTMSK         = 0;
            ptr_URB->DataTransferred = ptr_URB->DataLength;
            ptr_URB->Status          = 0;
            if (hcint & USBH_STM32_HCINT_NYET)
              ptr_URB->ResponsePacketType = USBH_PACKET_NYET;
            else 
              ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Completed       = 1;
            CHURB[ctrl][ch]          = 0;
          } else if (hcint & USBH_STM32_HCINT_STALL) {      /* If STALL event */
            USBH_STM32_DMA_Stop(ctrl);            /* Stop the DMA transfer    */
            ptr_URB->ResponsePacketType = USBH_PACKET_STALL;
            ptr_CH->HCINTMSK         = USBH_STM32_HCINT_CHH;
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USBH_STM32_HCCHAR_CHDIS;
          } else if ((hcint & USBH_STM32_HCINT_NAK)   ||    /* If NAK received*/
                     (hcint & USBH_STM32_HCINT_NYET)  ||    /* If NYET rece   */
                     (hcint & USBH_STM32_HCINT_TXERR)) {    /* If TXERR rece  */
            USBH_STM32_DMA_Stop(ctrl);            /* Stop the DMA transfer    */
            if (hcint & USBH_STM32_HCINT_NAK) {
              ptr_URB->ResponsePacketType = USBH_PACKET_NAK;
              ptr_CH->HCINTMSK       = USBH_STM32_HCINT_CHH;
            } else if (hcint & USBH_STM32_HCINT_NYET) {
              ptr_URB->ResponsePacketType = USBH_PACKET_NYET;
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
             /* (Port_Speed[ctrl] != USBH_HS)                    && */
                (ptr_URB->ResponsePacketType == USBH_PACKET_NAK) && 
                (CHURB[ctrl][ch]->NAKRetries--)) {
                                                  /* Reenable channel         */
              USBH_STM32_CH_TransferEnqueue (ctrl, ptr_CH, (U8)CHURB[ctrl][ch]->Parameters, (U8 *)CHURB[ctrl][ch]->ptrDataBuffer+CHURB[ctrl][ch]->DataTransferred, (U32)CHURB[ctrl][ch]->DataLength-CHURB[ctrl][ch]->DataTransferred);
              CHURB[ctrl][ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
              ptr_CH->HCINTMSK       = 0;
              ptr_URB->Submitted     = 0;
              ptr_URB->InProgress    = 0;
              ptr_URB->Completed     = 1;
              CHURB[ctrl][ch]        = 0;
            }
          } else if (hcint&USBH_STM32_HCINT_ACK) {/* If ACK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            ptr_CH->HCINTMSK        &=~USBH_STM32_HCINT_ACK;
          }
        } else if (ptr_URB->PacketType == USBH_PACKET_PING){/* If PING packet ----------*/
          if (hcint & USBH_STM32_HCINT_STALL) {             /* If STALL event */
            ptr_URB->ResponsePacketType = USBH_PACKET_STALL;
            ptr_CH->HCINTMSK         = USBH_STM32_HCINT_CHH;
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USBH_STM32_HCCHAR_CHDIS;
          } else if ((hcint & USBH_STM32_HCINT_NAK)   ||    /* If NAK received*/
                     (hcint & USBH_STM32_HCINT_TXERR)) {    /* If TXERR rece  */
            if (hcint & USBH_STM32_HCINT_NAK) {
              ptr_URB->ResponsePacketType = USBH_PACKET_NAK;
              ptr_CH->HCINTMSK       = USBH_STM32_HCINT_CHH;
            } else {
              ptr_URB->Error         = USBH_STM32_HCINT_TXERR >> 7;
              ptr_CH->HCINTMSK       = USBH_STM32_HCINT_ACK | USBH_STM32_HCINT_CHH;
            }
                                                  /* Halt the channel         */
            ptr_CH->HCCHAR          |= USBH_STM32_HCCHAR_CHDIS;
          } else if (hcint&USBH_STM32_HCINT_CHH) {/* If channel halted        */
            if ((ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT) && (ptr_URB->ResponsePacketType == USBH_PACKET_NAK) && (CHURB[ctrl][ch]->NAKRetries--)) {
                                                  /* Reenable channel         */
              USBH_STM32_CH_TransferEnqueue (ctrl, ptr_CH, (U8)CHURB[ctrl][ch]->Parameters, (U8 *)CHURB[ctrl][ch]->ptrDataBuffer+CHURB[ctrl][ch]->DataTransferred, (U32)CHURB[ctrl][ch]->DataLength-CHURB[ctrl][ch]->DataTransferred);
              CHURB[ctrl][ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
            } else {
              ptr_CH->HCINTMSK       = 0;
              ptr_URB->Submitted     = 0;
              ptr_URB->InProgress    = 0;
              ptr_URB->Completed     = 1;
              CHURB[ctrl][ch]        = 0;
            }
          } else if (hcint&USBH_STM32_HCINT_ACK) {/* If ACK received          */
            ptr_URB->ResponsePacketType = USBH_PACKET_ACK;
            ptr_URB->Error           = 0;
            ptr_CH->HCINTMSK        &=~USBH_STM32_HCINT_ACK;
                                                  /* Halt the channel         */
            ptr_CH->HCINTMSK         = USBH_STM32_HCINT_CHH;
            ptr_CH->HCCHAR          |= /*USBH_STM32_HCCHAR_CHENA | */USBH_STM32_HCCHAR_CHDIS;
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
            if ((Port_Con[ctrl]) && (ptr_CH->EPTYP != USB_ENDPOINT_TYPE_INTERRUPT) && (CHURB[ctrl][ch]->NAKRetries--)) {
                                                  /* Reenable channel         */
              ptr_CH->HCINTMSK       = USBH_STM32_HCINTMSK_DTERRM | 
                                       USBH_STM32_HCINTMSK_BBERRM | 
                                       USBH_STM32_HCINTMSK_TXERRM | 
                                       USBH_STM32_HCINTMSK_ACKM   | 
                                       USBH_STM32_HCINTMSK_NAKM   | 
                                       USBH_STM32_HCINTMSK_STALLM | 
                                       USBH_STM32_HCINTMSK_XFRCM  ;
              ptr_CH->HCCHAR        |= USBH_STM32_HCCHAR_CHENA;
              CHURB[ctrl][ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
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
            CHURB[ctrl][ch]          = 0;
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
              CHURB[ctrl][ch]->TimeoutCount= MAX_TIMEOUT_COUNT;
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
    ptr_OTG->HAINT = haint;
  }  
  ptr_OTG->GINTSTS = gintsts;                     /* Clear core ints          */
  if (gintsts & USBH_STM32_GINTSTS_SOF) {         /* If start of frame int    */
    if (HW_Accessing[ctrl]) {
      act = 1;
    } else {
      act = 0;
      for (ch = 0; ch < max_ch; ch++) {
        if (CHURB[ctrl][ch]) {
          if (CHURB[ctrl][ch]->InProgress == 1) { /* If any URB in progress   */
            act = 1;                              /* Set act to 1             */
            break;
          }
        }
      }
    }

    /* At this point act == 1 if there is USB bus activity                    */
    ptr_CH = (USBH_STM32_CH *)(&(ptr_OTG->HCCHAR0));
    for (ch = 0; ch < max_ch; ch++) {
      ptr_URB = CHURB[ctrl][ch];                  /* Pointer to channels URB  */
      if (ptr_URB) {
        if ((ptr_URB->Submitted  == 1) &&         /* If URB is submitted      */
            (ptr_URB->InProgress == 0)) {         /* If URB not in progress   */
          if (ptr_CH->EPTYP == USB_ENDPOINT_TYPE_INTERRUPT) {
            if (cntInterval[ctrl][ch]) {
              if ((act && (cntInterval[ctrl][ch] > 1)) || !act)
                cntInterval[ctrl][ch]--;
              }
              if (!act) {
                if (!cntInterval[ctrl][ch]) {     /* If period expired        */
                  ptr_URB->InProgress = 1;
                  USBH_STM32_CH_TransferEnqueue (ctrl, ptr_CH, (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrDataBuffer, (U32)ptr_URB->DataLength);
                  act = 1;
                }
              }
          } else if (!act) {
            ptr_URB->InProgress = 1;
            USBH_STM32_CH_TransferEnqueue (ctrl, ptr_CH, (U8)ptr_URB->Parameters, (U8 *)ptr_URB->ptrDataBuffer, (U32)ptr_URB->DataLength);
            act = 1;
          }
        }
      }
      ptr_CH++;
    }
  }
}
