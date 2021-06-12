/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_xmc4500.c
 *      Purpose: Hardware Layer module for Infineon XMC4500
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <XMC4500.h>

#define __NO_USB_LIB_C
#include "usb_config.c"

#define RX_FIFO_SIZE        128
#define TX0_FIFO_SIZE       64
#define TX1_FIFO_SIZE       64
#define TX2_FIFO_SIZE       64
#define TX3_FIFO_SIZE       64
#define TX4_FIFO_SIZE       64
#define TX5_FIFO_SIZE       64
#define TX6_FIFO_SIZE       64

#define TX_FIFO(n)          *((__packed U32*)(USB0_BASE + 0x1000 + n*0x1000))
#define RX_FIFO             *((U32*)(USB0_BASE + 0x1000))

#define DIEPTSIZ(EPNum)     *(&USB0_EP0->DIEPTSIZ0 + EPNum * 8)
#define DIEPCTL(EPNum)      *(&USB0_EP0->DIEPCTL0  + EPNum * 8)
#define DTXFSTS(EPNum)      *(&USB0_EP0->DTXFSTS0  + EPNum * 8)
#define DOEPTSIZ(EPNum)     *(&USB0_EP0->DOEPTSIZ0 + EPNum * 8)
#define DOEPCTL(EPNum)      *(&USB0_EP0->DOEPCTL0  + EPNum * 8)
#define DIEPINT(EPNum)      *(&USB0_EP0->DIEPINT0  + EPNum * 8)
#define DOEPINT(EPNum)      *(&USB0_EP0->DOEPINT0  + EPNum * 8)

U32 OutMaxPacketSize[7] = {USBD_MAX_PACKET0,0,0,0,0,0,0};

/*
 *  usbd_xmc4500_delay
 *    Parameters:      delay:      Delay
 *    Return Value:    None
 */
void usbd_xmc4500_delay (U32 delay) {
  delay *= SystemCoreClock / 100000;
  while (delay--) {
    __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
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
  NVIC_EnableIRQ   (USB0_0_IRQn);       /* Enable OTG interrupt               */
}


/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB
 *   Return Value:    None
 */

void USBD_Init (void) {
  int32_t tout;

  SCU_CLK->CLKSET |= 1;                           /* USB Clock Enable         */
  for (tout=1000; tout>=0; tout--) {              /* Wait                     */
    if (SCU_CLK->CLKSTAT & 1) break;
    if (tout <= 10) usbd_xmc4500_delay(10);       /* for max 100 ms           */
  }

  SCU_RESET->PRCLR2 |=  (1 <<  7);                /* USB Peri Reset Clear     */
  SCU_POWER->PWRSET |=  (1 << 16) | (1 << 17);    /* USB PHY En + OTG En      */

  USB0->GRSTCTL |= USB_GRSTCTL_CSftRst_Msk;       /* USB Core Soft Reset      */
  for (tout=1000; tout>=0; tout--) {              /* Wait                     */
    if (!(USB0->GRSTCTL & USB_GRSTCTL_CSftRst_Msk)) break;
    if (tout <= 100) usbd_xmc4500_delay (10);     /* for max 1000 ms          */
  }

  usbd_xmc4500_delay (100);             /* Wait ~100 ms                       */

  USB0->GUSBCFG  = (1 << 30);           /* force device mode                  */
  usbd_xmc4500_delay (500);

  USBD_IntrEna();                       /* Enable OTG interrupt               */

  USB0->GAHBCFG |= 1 | (1 << 7);
  USB0->GUSBCFG  = (USB0->GUSBCFG & ~(15UL << 10)) |(9 << 10);/*turnaround*/

  USB0->DCFG    |=  3;                /* full speed phy                       */
  USB0->PCGCCTL  =  0;                /* Restart PHY clock                    */

  USB0->GINTMSK_DEVICEMODE  = (1UL << 11) | /* suspend int unmask             */
                              (1UL << 12) | /* reset int unmask               */
                              (1UL << 13) | /* enumeration done int unmask    */
                              (1UL << 4 ) | /* recive fifo non-empty int      */
                              (1UL << 18) | /* IN EP int unmask               */
                              (1UL << 19) | /* OUT EP int unmask              */
                              (1UL << 31) | /* resume int unmask              */
#ifdef __RTX
  ((USBD_RTX_DevTask   != 0) ? (1UL <<  3) : 0);   /* SOF int unmask          */
#else
  ((USBD_P_SOF_Event   != 0) ? (1UL <<  3) : 0);   /* SOF int unmask          */
#endif
}




/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect (BOOL con) {  
  if (con) {
    USB0->GOTGCTL   =   (1UL << 19) |   /* enable Vbus*                       */
                        (1UL << 16);    /* power down deactivated             */
    USB0->DCTL     &=  ~(1UL << 1 );    /* soft disconnect disabled           */

  }
  else {
    USB0->GOTGCTL  &= ~(1UL << 19);     /* disable Vbus*                      */
    USB0->DCTL     |=  (1UL << 1 );     /* soft disconnect enabled            */
  }
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {
  U32 i;

  for (i = 0; i < (USBD_EP_NUM + 1); i++) {
   if (DOEPCTL(i) & (1UL << 31))
     DOEPCTL(i) = (1UL << 30) | (1UL << 27); /* OUT EP disable, Set NAK       */
   if (DIEPCTL(i) & (1UL << 31))
     DIEPCTL(i) = (1UL << 30) | (1UL << 27); /* IN EP disable, Set NAK        */
  }

  USBD_SetAddress(0 , 1);

  USB0->DAINTMSK = 1 | (1UL << 16);     /* unmask IN&OUT EP0 interruts        */

  USB0->DOEPMSK  =  (1UL << 3) |        /* setup phase done                   */
                     1;                 /* transfer complete                  */
  USB0->DIEPMSK  =   1;                 /* transfer completed                 */

  USB0->GRXFSIZ  =   RX_FIFO_SIZE;
  USB0->HPTXFSIZ =  (TX0_FIFO_SIZE/4 << 16) | RX_FIFO_SIZE;

  USB0->DIEPTXF1 =  (RX_FIFO_SIZE + TX0_FIFO_SIZE) |
                    (TX1_FIFO_SIZE/4 << 16);

  USB0->DIEPTXF2 =  (RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE) |
                    (TX2_FIFO_SIZE/4 << 16);

  USB0->DIEPTXF3 =  (RX_FIFO_SIZE + TX0_FIFO_SIZE+ TX1_FIFO_SIZE +TX2_FIFO_SIZE) |
                    (TX3_FIFO_SIZE/4 << 16);

  USB0->DIEPTXF4 =  (RX_FIFO_SIZE + TX0_FIFO_SIZE+ TX1_FIFO_SIZE +TX2_FIFO_SIZE
                     + TX3_FIFO_SIZE) | (TX4_FIFO_SIZE/4 << 16);

  USB0->DIEPTXF5 =  (RX_FIFO_SIZE + TX0_FIFO_SIZE+ TX1_FIFO_SIZE +TX2_FIFO_SIZE
                     + TX3_FIFO_SIZE + TX4_FIFO_SIZE) | (TX5_FIFO_SIZE/4 << 16);

  USB0->DIEPTXF6 =  (RX_FIFO_SIZE + TX0_FIFO_SIZE+ TX1_FIFO_SIZE +TX2_FIFO_SIZE
                     + TX3_FIFO_SIZE + TX4_FIFO_SIZE + TX5_FIFO_SIZE) | (TX6_FIFO_SIZE/4 << 16);

  USB0_EP0->DOEPTSIZ0  =  (1UL << 29) |       /* setup count = 1              */
                          (1UL << 19) |       /* packet count                 */
                           USBD_MAX_PACKET0;
}


/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend (void) {
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume (void) {
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp (void) {
  USB0->DCTL |= 1;                      /* remote wakeup signaling            */
  usbd_xmc4500_delay (50);              /* Wait ~5 ms                         */
  USB0->DCTL &= ~1;
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg (BOOL cfg) {
  /* Not needed                                                               */
}


/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */

void USBD_SetAddress (U32  adr, U32 setup) {
  if (setup) {
    USB0->DCFG = (USB0->DCFG & ~(0x7f << 4)) | (adr << 4);
  }
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure (BOOL cfg) {
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
  U32 num, val, type;

  num  = pEPD->bEndpointAddress & ~(0x80);
  val  = pEPD->wMaxPacketSize;
  type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;

  if (pEPD->bEndpointAddress & USB_ENDPOINT_DIRECTION_MASK) {
    USB0->DAINTMSK   |= (1UL  << num);  /* unmask IN EP int                   */
    DIEPCTL(num)      = (num  <<  22) | /* fifo number                        */
                        (type <<  18) | /* ep type                            */
                         val;           /* max packet size                    */
    if ((type & 3) > 1)                 /* if interrupt or bulk EP            */
      DIEPCTL(num) |= (1 << 28);    
  }
  else {
    OutMaxPacketSize[num] = (val + 3) & ~0x03;
    USB0->DAINTMSK       |= (1UL  <<    num) << 16; /* unmask OUT EP int      */
    DOEPCTL(num)          = (type <<     18) |      /* EP type                */
                            (val  &   0x7FF);       /* max packet size        */
    DOEPTSIZ(num)         = (1UL  <<     19) |      /* packet count = 1       */
                            (val  & 0x7FFFF);       /* transfer size          */
    if ((type & 3) > 1)                             /* if int or bulk EP      */
      DOEPCTL(num)       |= (1 << 28);
  }
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP (U32 dir) {
  /* Not needed                                                               */
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (U32 EPNum) {
  if (EPNum & 0x80) {
    EPNum &= ~0x80;
    DIEPCTL(EPNum)    |= (1UL << 15) |  /* EP active                          */
                         (1UL << 27);   /* set EP NAK                         */
    if (DIEPCTL(EPNum)&  (1UL << 31))
      DIEPCTL(EPNum)  |= (1UL << 30);   /* disable EP                         */
  }
  else {
    DOEPCTL(EPNum)    |= (1UL << 15) |  /* EP active                          */
                         (1UL << 31) |  /* enable EP                          */
                         (1UL << 26);   /* clear EP NAK                       */
  }
}


/*
 *  Disable USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (U32 EPNum) {
  if (EPNum & 0x80) {
    EPNum &= ~0x80;
    if (DIEPCTL(EPNum) &   (1UL << 31))
      DIEPCTL(EPNum)   |=  (1UL << 30); /* disable EP                         */
    DIEPCTL(EPNum)     |=  (1UL << 27); /* set EP NAK                         */
    DIEPCTL(EPNum)     &= ~(1UL << 15); /* deactivate EP                      */
  }
  else {
    if (DOEPCTL(EPNum) &   (1UL << 31))
      DOEPCTL(EPNum)   |=  (1UL << 30); /* disable EP                         */
    DOEPCTL(EPNum)     |=  (1UL << 27); /* set EP NAK                         */
    DOEPCTL(EPNum)     &= ~(1UL << 15); /* deactivate EP                      */
  }
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (U32 EPNum) {
  if (EPNum & 0x80) {
    EPNum &= ~0x80;
    if (DIEPCTL(EPNum) &  (1UL << 31))
      DIEPCTL(EPNum)   |= (1UL << 30);  /* disable EP                         */
    DIEPCTL(EPNum)     |= (1UL << 27);  /* set EP NAK                         */

    USB0->GRSTCTL = (USB0->GRSTCTL & ~(0x1F << 6)) | /* flush EP fifo         */
                    (EPNum << 6)| (1UL << 5);
    while (USB0->GRSTCTL & (1UL << 5));
  }
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP (U32 EPNum) {
  if (!(EPNum & 0x80)) {
    DOEPCTL(EPNum) |= (1UL << 21);      /*set stall                           */
#ifdef __RTX
    USB0->GINTMSK_DEVICEMODE |= (1UL << 4);
#endif
  }
  else {
    EPNum &= ~0x80;
    if (DIEPCTL(EPNum) &  (1UL << 31)) {
      DIEPCTL(EPNum)   |= (1UL << 30);
    }
    DIEPCTL(EPNum)     |= (1UL << 21);  /*set stall                           */

    USB0->GRSTCTL     = (USB0->GRSTCTL & ~(0x1f << 6)) | /* flush EP fifo */
                          (EPNum << 6)| (1UL << 5);

    while (USB0->GRSTCTL & (1UL << 5));
  }
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP (U32 EPNum) {
  if (!(EPNum & 0x80)) {
    if (((DOEPCTL(EPNum) >> 18) & 3) > 1) {/* if interrupt or bulk EP         */
      DOEPCTL(EPNum)    =  (DOEPCTL(EPNum) & ~(1 << 21)) |  (1 << 28);
  }
#ifdef __RTX
    USB0->GINTMSK_DEVICEMODE |= (1UL << 4);
#endif
  }
  else {
    EPNum &= ~0x80;
    if (DIEPCTL(EPNum) &  (1UL << 31))
      DIEPCTL(EPNum)   |= (1UL << 30);
    DIEPCTL(EPNum)     |= (1UL << 27);

    USB0->GRSTCTL     = (USB0->GRSTCTL & ~(0x1f << 6)) | /* flush EP fifo */
                          (EPNum << 6)| (1UL << 5);
    while (USB0->GRSTCTL & (1UL << 5));

    if (((DIEPCTL(EPNum) >> 18) & 3) > 1)/* if interrupt or bulk EP           */
      DIEPCTL(EPNum)    =   (DIEPCTL(EPNum) & ~(1 << 21)) |  (1 << 28);
  }
}


/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf (U32 EPNum) {
  if (EPNum & 0x80) {
    EPNum &= ~0x80;
    USB0->GRSTCTL = (USB0->GRSTCTL & ~(0x1f << 6)) | /* flush EP fifo     */
                      (EPNum << 6)| (1UL << 5);
    while (USB0->GRSTCTL & (1UL << 5));
  }
  else {
    USB0->GRSTCTL |= (1UL << 4);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
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
  U32 i, sz;


  sz = ( USB0->GRXSTSP_DEVICEMODE >> 4) & 0x7FF;

  for (i = 0; i < (U32)((sz+3)/4); i++) {
    *((__packed U32 *)pData) = RX_FIFO;
    pData += 4;
  }

  while ((USB0->GINTSTS_DEVICEMODE & (1UL << 4)) == 0);
  USB0->GRXSTSP_DEVICEMODE;
#ifdef __RTX
  USB0->GINTMSK_DEVICEMODE |= (1UL << 4);
#endif

  return (sz);
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
  U32 i;

  EPNum &= ~(0x80);

  if (cnt) {
    while ((DTXFSTS(EPNum) * 4) < cnt);

    DIEPTSIZ(EPNum) = (DIEPTSIZ(EPNum) & ~0x1FFFFFFF) | cnt | (1UL << 19);
    DIEPCTL(EPNum) |= (1UL << 31) | (1UL << 26);
    for (i = 0; i < (cnt+3)/4; i++) {
      TX_FIFO(EPNum) = *((__packed U32 *)pData);
      pData +=4;
    }
  }
  else {
    DIEPTSIZ(EPNum) = (DIEPTSIZ(EPNum) & ~0x1FFFFFFF)| (1UL << 19);
    DIEPCTL(EPNum) |= (1UL << 31) | (1UL << 26);
  }
  return (cnt);
}


/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

U32 USBD_GetFrame (void) {
  return ((USB0->DSTS >> 8) & 0x3FFF);
}


/*
 *  USB Device Interrupt Service Routine
 */
void USB0_0_IRQHandler(void) {
  U32 istr, val, num, i;

  istr = USB0->GINTSTS_DEVICEMODE & USB0->GINTMSK_DEVICEMODE;

/* reset interrupt                                                            */
  if (istr & (1UL << 12)) {
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
    USB0->GINTSTS_DEVICEMODE |= (1UL << 12);
  }

/* suspend interrupt                                                          */
  if (istr & (1UL << 11)) {
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
    USB0->GINTSTS_DEVICEMODE |= (1UL << 11);
  }

/* resume interrupt                                                           */
  if (istr & (1UL << 31)) {
    USBD_Resume();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_RESUME, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_Resume_Event) {
      USBD_P_Resume_Event();
    }
#endif
    USB0->GINTSTS_DEVICEMODE |= (1UL << 31);
  }

/* speed enumeration completed                                                */
  if (istr & (1UL << 13)) {
    USB0_EP0->DIEPCTL0 &= ~(3UL);
    switch (USBD_MAX_PACKET0) {
      case 8:
        USB0_EP0->DIEPCTL0 |= 3;
      break;

      case 16:
        USB0_EP0->DIEPCTL0 |= 2;
      break;

      case 32:
        USB0_EP0->DIEPCTL0 |= 1;
      break;
    }
    USB0->DCTL    |= (1UL << 8);        /* clear global IN NAK                */
    USB0->DCTL    |= (1UL << 10);       /* clear global OUT NAK               */
    USB0->GINTSTS_DEVICEMODE |= (1UL << 13);
  }

/* Start Of Frame                                                             */
  if (istr & (1UL << 3)) {
#ifdef __RTX
    if (USBD_RTX_DevTask) {
      isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_SOF_Event) {
      USBD_P_SOF_Event();
    }
#endif
     USB0->GINTSTS_DEVICEMODE |= (1UL << 3);
  }

/* RxFIFO non-empty                                                           */
  if (istr & (1UL << 4)) {
    val = USB0->GRXSTSR_DEVICEMODE;
    num = val & 0x0F;

    switch ((val >> 17) & 0x0F) {
/* setup packet                                                               */
      case 6:
#ifdef __RTX
        USB0->GINTMSK_DEVICEMODE &= ~(1UL << 4);
        if (USBD_RTX_EPTask[num]) {
          isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[num]);
        }
#else
        if (USBD_P_EP[num]) {
          USBD_P_EP[num](USBD_EVT_SETUP);
        }
#endif
        break;

/* OUT packet                                                                 */
      case 2:
#ifdef __RTX
        USB0->GINTMSK_DEVICEMODE &= ~(1UL << 4);
        if (USBD_RTX_EPTask[num]) {
          isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[num]);
        }
#else
        if (USBD_P_EP[num]) {
          USBD_P_EP[num](USBD_EVT_OUT);
        }
#endif
        break;

      default:
      USB0->GRXSTSP_DEVICEMODE;
    }
  }

/* OUT Packet                                                                 */
  if (istr & (1UL << 19)) {

    num = (((USB0->DAINT & USB0->DAINTMSK) >> 16) & 0xFFFF);  
    for (i = 0; i < (USBD_EP_NUM+1); i++) {
      if ((num >> i) & 1) {
        num = i;
        break;
      }
    }

    if ((DOEPINT(num) & 1) | (DOEPINT(num) & (1 << 3))) {

      DOEPTSIZ(num)    = (1UL << 19) |                   /* packet count      */
                         (OutMaxPacketSize[num]);        /* transfer size     */
      if (num == 0) {
        DOEPTSIZ(0)   |= (1UL << 29);
      }
      DOEPCTL(num)    |= (1UL <<31) | (1UL << 26);       /* clr NAK, en EP    */

    }

    DOEPINT(num) |= 1;
  }

/* IN Packet                                                                  */
  if (istr & (1UL << 18)) {
    num = (USB0->DAINT & USB0->DAINTMSK & 0xFFFF);
    for (i = 0; i < (USBD_EP_NUM+1); i++) {
      if ((num >> i) & 1) {
        num = i;
        break;
      }
    }
    if (DIEPINT(num) & 1) {             /* TxFIFO completed                   */
#ifdef __RTX
      if (USBD_RTX_EPTask[num]) {
        isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[num]);
      }
#else
      if (USBD_P_EP[num]) {
        USBD_P_EP[num](USBD_EVT_IN);
      }
#endif
      DIEPINT(num) |= 1;
    }
  }
}
