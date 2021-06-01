/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_ehci_lpc43xx.c
 *      Purpose: EHCI Hardware Specific Layer Driver for the
 *               NXP LPC43xx Device Series
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  Note:  define 'LPC4330_XPLORER' in your project settings to use this file
         for the NGX LPC4330 Xplorer board.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <LPC43xx.h>


/************************* EHCI Hardware Driver Configuration *****************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <e0.0> USB0 Host EHCI Enable
//   <o1> USB Host EHCI Controller Base Address
//   <h> Root Hub ports used by EHCI Controller
//     <i> These are the ports that EHCI will use.
//     <o2.0>  Port 1
//     </h>
//
//   <o3> Size of memory used by EHCI <1-1048576>
//     <i> This is a size of memory (in bytes) that EHCI will use for
//     <i> USB communication data (maximum sum of data at a single point in
//     <i> time, for example for HID = size of max Configuration Descriptor +
//     <i> size of max HID Report Descriptor).
//
//   <o4> Maximum number of Queue Heads (Endpoints) used by EHCI <1-64>
//     <i> This is a maximum number of Queue Heads (Endpoints) that EHCI will use.
//
//   <o5> Maximum number of Transfer Descriptors used by EHCI <1-64>
//     <i> This is a maximum number of concurrent Transfers that EHCI will use.
// </e>
// <e0.1> USB1 Host EHCI Enable
//   <o9> USB Host EHCI Controller Base Address
//   <h> Root Hub ports used by EHCI Controller
//     <i> These are the ports that EHCI will use.
//     <o10.0>  Port 1
//     </h>
//
//   <o11> Size of memory used by EHCI <1-1048576>
//     <i> This is a size of memory (in bytes) that EHCI will use for
//     <i> USB communication data (maximum sum of data at a single point in
//     <i> time, for example for HID = size of max Configuration Descriptor +
//     <i> size of max HID Report Descriptor).
//
//   <o12> Maximum number of Queue Heads (Endpoints) used by EHCI <1-64>
//     <i> This is a maximum number of Queue Heads (Endpoints) that EHCI will use.
//
//   <o13> Maximum number of Transfer Descriptors used by EHCI <1-64>
//     <i> This is a maximum number of concurrent Transfers that EHCI will use.
// </e>
#define USBH_EHCI_EN                0x00000003

#define USBH0_EHCI_ADR              0x40006000
#define USBH0_EHCI_PORTS            0x00000001
#define USBH0_EHCI_MEM_DATA_SZ      4096
#define USBH0_EHCI_NUM_qH           3
#define USBH0_EHCI_NUM_qTD          2
#define USBH0_EHCI_NUM_iTD          0
#define USBH0_EHCI_NUM_siTD         0
#define USBH0_EHCI_NUM_FSTN         0

#define USBH1_EHCI_ADR              0x40007000
#define USBH1_EHCI_PORTS            0x00000001
#define USBH1_EHCI_MEM_DATA_SZ      4096
#define USBH1_EHCI_NUM_qH           3
#define USBH1_EHCI_NUM_qTD          2
#define USBH1_EHCI_NUM_iTD          0
#define USBH1_EHCI_NUM_siTD         0
#define USBH1_EHCI_NUM_FSTN         0

// *** <<< End of Configuration section             >>> ***


/************************** EHCI Host Controller Hardware Driver Variables ****/

#define USBH0_EHCI_MEM_SZ          (USBH0_EHCI_MEM_DATA_SZ + /* DATA sz    */  \
                                    USBH0_EHCI_NUM_qTD * 8 + /* alloc ovhd */  \
                                    4                      ) /* alloc ovhd */

#define USBH1_EHCI_MEM_SZ          (USBH1_EHCI_MEM_DATA_SZ + /* DATA sz    */  \
                                    USBH1_EHCI_NUM_qTD * 8 + /* alloc ovhd */  \
                                    4                      ) /* alloc ovhd */

#if    (USBH_EHCI_EN & 1)
U32     usbh0_ehci_pfl  [1024]                      __attribute__((aligned(4096)));
U32     usbh0_ehci_qh   [USBH0_EHCI_NUM_qH*16];
U32     usbh0_ehci_qtd  [USBH0_EHCI_NUM_qTD*8]      __attribute__((aligned(32)));
U32     usbh0_ehci_mpool[USBH0_EHCI_MEM_SZ /4];
U32     usbh0_ehci_tdurb[USBH0_EHCI_NUM_qTD*3];
#endif

#if    (USBH_EHCI_EN & 2)
U32     usbh1_ehci_pfl  [1024]                      __attribute__((aligned(4096)));
U32     usbh1_ehci_qh   [USBH1_EHCI_NUM_qH*16];
U32     usbh1_ehci_qtd  [USBH1_EHCI_NUM_qTD*8]      __attribute__((aligned(32)));
U32     usbh1_ehci_mpool[USBH1_EHCI_MEM_SZ /4];
U32     usbh1_ehci_tdurb[USBH0_EHCI_NUM_qTD*3];
#endif
static U32 calDelay = 0;
static U32 USBHx_EHCI_ADR[2] = { USBH0_EHCI_ADR,  USBH1_EHCI_ADR };


/************************** EHCI Host Controller Hardware Function Prototypes */

extern void USBH_EHCI_NXP_IRQHandler     (U8 ctrl);

void    usbh_ehci_hw_get_capabilities    (U8 ctrl, USBH_HCI_CAP *cap);
void    usbh_ehci_hw_delay_ms            (U32  ms);
void    usbh_ehci_hw_reg_wr              (U8 ctrl, U32 reg_ofs, U32 val);
U32     usbh_ehci_hw_reg_rd              (U8 ctrl, U32 reg_ofs);
BOOL    usbh_ehci_hw_pins_config         (U8 ctrl, BOOL on);
BOOL    usbh_ehci_hw_init                (U8 ctrl, BOOL on);
BOOL    usbh_ehci_hw_port_power          (U8 ctrl, BOOL on);
BOOL    usbh_ehci_hw_irq_en              (U8 ctrl, BOOL on);

#if    (USBH_EHCI_EN & 1)
void    usbh0_ehci_hw_get_capabilities   (USBH_HCI_CAP *cap)      {         usbh_ehci_hw_get_capabilities (1, cap         ) ; };
void    usbh0_ehci_hw_delay_ms           (U32  ms)                {         usbh_ehci_hw_delay_ms         (ms             ) ; };
void    usbh0_ehci_hw_reg_wr             (U32  reg_ofs, U32 val)  {         usbh_ehci_hw_reg_wr           (0, reg_ofs, val) ; };
U32     usbh0_ehci_hw_reg_rd             (U32  reg_ofs)           { return (usbh_ehci_hw_reg_rd           (0, reg_ofs     )); };
BOOL    usbh0_ehci_hw_pins_config        (BOOL on)                { return (usbh_ehci_hw_pins_config      (0, on          )); };
BOOL    usbh0_ehci_hw_init               (BOOL on)                { return (usbh_ehci_hw_init             (0, on          )); };
BOOL    usbh0_ehci_hw_port_power         (BOOL on)                { return (usbh_ehci_hw_port_power       (0, on          )); };
BOOL    usbh0_ehci_hw_irq_en             (BOOL on)                { return (usbh_ehci_hw_irq_en           (0, on          )); };
#endif
#if    (USBH_EHCI_EN & 2)
void    usbh1_ehci_hw_get_capabilities   (USBH_HCI_CAP *cap)      {         usbh_ehci_hw_get_capabilities (1, cap         ) ; };
void    usbh1_ehci_hw_delay_ms           (U32  ms)                {         usbh_ehci_hw_delay_ms         (ms             ) ; };
void    usbh1_ehci_hw_reg_wr             (U32  reg_ofs, U32 val)  {         usbh_ehci_hw_reg_wr           (1, reg_ofs, val) ; };
U32     usbh1_ehci_hw_reg_rd             (U32  reg_ofs)           { return (usbh_ehci_hw_reg_rd           (1, reg_ofs     )); };
BOOL    usbh1_ehci_hw_pins_config        (BOOL on)                { return (usbh_ehci_hw_pins_config      (1, on          )); };
BOOL    usbh1_ehci_hw_init               (BOOL on)                { return (usbh_ehci_hw_init             (1, on          )); };
BOOL    usbh1_ehci_hw_port_power         (BOOL on)                { return (usbh_ehci_hw_port_power       (1, on          )); };
BOOL    usbh1_ehci_hw_irq_en             (BOOL on)                { return (usbh_ehci_hw_irq_en           (1, on          )); };
#endif


/************************** EHCI Host Controller Hardware Driver Structure ****/

#if    (USBH_EHCI_EN & 1)
USBH_HWD_EHCI usbh0_hwd_ehci_NXP = {    /* EHCI0 Host Controller Hardware Drv */
  USBH0_EHCI_PORTS,                     /* Ports (bits 0..15)                 */
  USBH0_EHCI_NUM_qH,                    /* Maximum Queue Heads                */
  USBH0_EHCI_NUM_qTD,                   /* Maximum Queue Transfer Descriptors */
  USBH0_EHCI_NUM_iTD,                   /* Maximum Iso Transfer Descriptors   */
  USBH0_EHCI_NUM_siTD,                  /* Maximum Split Iso Transfer Descs   */
  USBH0_EHCI_NUM_FSTN,                  /* Maximum Peri Frame Span Trav Nodes */
  (U32 *) &usbh0_ehci_pfl,              /* Pointer to Periodic Frame List mem */
  (U32 *) &usbh0_ehci_qh,               /* Pointer to qH memory start         */
  (U32 *) &usbh0_ehci_qtd,              /* Pointer to qTD memory start        */
  NULL,                                 /* Pointer to iTD memory start        */
  NULL,                                 /* Pointer to siTD memory start       */
  NULL,                                 /* Pointer to FSTN memory start       */
  (U32 *) &usbh0_ehci_tdurb,            /* Pointer to TDURB memory start      */
  usbh0_ehci_hw_get_capabilities,       /* Get driver capabilities            */
  usbh0_ehci_hw_delay_ms,               /* Delay in ms                        */
  usbh0_ehci_hw_reg_wr,                 /* Write register                     */
  usbh0_ehci_hw_reg_rd,                 /* Read register                      */
  usbh0_ehci_hw_pins_config,            /* Config/Unconfig pins               */
  usbh0_ehci_hw_init,                   /* Init/Uninit Host Controller        */
  usbh0_ehci_hw_port_power,             /* On/Off Port Power                  */
  usbh0_ehci_hw_irq_en                  /* Enable/Disable interrupt           */
};
#endif
#if    (USBH_EHCI_EN & 2)
USBH_HWD_EHCI usbh1_hwd_ehci_NXP = {    /* EHCI1 Host Controller Hardware Drv */
  USBH1_EHCI_PORTS,                     /* Ports (bits 0..15)                 */
  USBH1_EHCI_NUM_qH,                    /* Maximum Queue Heads                */
  USBH1_EHCI_NUM_qTD,                   /* Maximum Queue Transfer Descriptors */
  USBH1_EHCI_NUM_iTD,                   /* Maximum Iso Transfer Descriptors   */
  USBH1_EHCI_NUM_siTD,                  /* Maximum Split Iso Transfer Descs   */
  USBH1_EHCI_NUM_FSTN,                  /* Maximum Peri Frame Span Trav Nodes */
  (U32 *) &usbh1_ehci_pfl,              /* Pointer to Periodic Frame List mem */
  (U32 *) &usbh1_ehci_qh,               /* Pointer to qH memory start         */
  (U32 *) &usbh1_ehci_qtd,              /* Pointer to qTD memory start        */
  NULL,                                 /* Pointer to iTD memory start        */
  NULL,                                 /* Pointer to siTD memory start       */
  NULL,                                 /* Pointer to FSTN memory start       */
  (U32 *) &usbh1_ehci_tdurb,            /* Pointer to TDURB memory start      */
  usbh1_ehci_hw_get_capabilities,       /* Get driver capabilities            */
  usbh1_ehci_hw_delay_ms,               /* Delay in ms                        */
  usbh1_ehci_hw_reg_wr,                 /* Write register                     */
  usbh1_ehci_hw_reg_rd,                 /* Read register                      */
  usbh1_ehci_hw_pins_config,            /* Config/Unconfig pins               */
  usbh1_ehci_hw_init,                   /* Init/Uninit Host Controller        */
  usbh1_ehci_hw_port_power,             /* On/Off Port Power                  */
  usbh1_ehci_hw_irq_en                  /* Enable/Disable interrupt           */
};
#endif


/************************** Imported Functions ********************************/

extern void USBH_EHCI_IRQHandler (U8 ctrl);


/************************** Module Functions **********************************/

/*------------------------- usbh_ehci_hw_get_capabilities ----------------------
 *
 *  Get capabilities of Host Controller Driver
 *
 *  Parameter:  ctrl:       Controller index (0 .. 1)
 *              cap:        Pointer to USBH_HCI_CAP structure where
 *                          capabilities are loaded
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ehci_hw_get_capabilities (U8 ctrl, USBH_HCI_CAP *cap) {
  cap->MultiPckt = __TRUE;
  if (!ctrl)
    cap->MaxDataSz = USBH0_EHCI_MEM_DATA_SZ;
  else
    cap->MaxDataSz = USBH1_EHCI_MEM_DATA_SZ;
  cap->CtrlNAKs  = 100;
  cap->BulkNAKs  = 1000000;
}


/*------------------------- usbh_ehci_hw_delay_ms ------------------------------
 *
 *  Delay execution (in milliseconds)
 *  Calibration is done if global variable calDelay is 0
 *
 *  Parameter:  ms:         Number of milliseconds to delay execution for
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ehci_hw_delay_ms (U32 ms) {
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
    SystemCoreClockUpdate();            /* Update SystemCoreClock variable    */
    calDelay = SystemCoreClock / (vals - vale);   /* Calibrated value         */
    if (vals == 0xFFFFFF)               /* Stop timer if we started it        */
      SysTick->CTRL  = 0;
    goto start;
  }
}


/*------------------------- usbh_ehci_hw_reg_wr --------------------------------
 *
 *  Write hardware register.
 *
 *  Parameter:  ctrl:       Controller index (0 .. 1)
 *              reg:        Register offset
 *              val:        Register value
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ehci_hw_reg_wr (U8 ctrl, U32 reg_ofs, U32 val) {
  *((U32 *)(USBHx_EHCI_ADR[ctrl & 1] + reg_ofs)) = val;
}


/*------------------------- usbh_ehci_hw_reg_rd --------------------------------
 *
 *  Read hardware register.
 *
 *  Parameter:  ctrl:       Controller index (0 .. 1)
 *              reg:        Register offset
 *  Return:                 Register value
 *----------------------------------------------------------------------------*/

U32 usbh_ehci_hw_reg_rd (U8 ctrl, U32 reg_ofs) {
  return (*((U32 *)(USBHx_EHCI_ADR[ctrl & 1] + reg_ofs)));
}


/*------------------------- usbh_ehci_hw_pins_cfg ------------------------------
 *
 *  Configurate or unconfigurate pins used by the USB Host.
 *
 *  Parameter:  ctrl:       Controller index (0 .. 1)
 *              on:         __TRUE = configurate, __FALSE = unconfigurate
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ehci_hw_pins_config (U8 ctrl, BOOL on) {

#ifdef LPC4330_XPLORER                           /* NGX LPC4330-Xplorer board */
  if (!ctrl) {
    if (on) {
      /* Set pin functions                                                    */
      LPC_SCU->SFSP1_7 = 4;             /* USB0_PPWR          (G4)            */
      LPC_SCU->SFSP2_1 = 3;             /* USB0_PWR_FAULT     (G7)            */
                                        /* USB0_IND0          (not connected) */
                                        /* USB0_IND1          (not connected) */
    } else {
      /* Reset pin functions                                                  */
      LPC_SCU->SFSP1_7 = 0;
      LPC_SCU->SFSP2_1 = 0;
    }
  } else {
    if (on) {
      LPC_GPIO_PORT->DIR[5] |=  (1 <<  6);
      LPC_GPIO_PORT->SET[5]  =  (1 <<  6);
      LPC_GPIO_PORT->DIR[1] &= ~(1 << 13);

      LPC_SCU->SFSP2_6  = 4;            /* USB1_PPWR       (GPIO5[6]          */
      LPC_SCU->SFSP2_13 = 0;            /* USB1_PWR_FAULT  (GPIO1[13]         */
                                        /* USB1_IND0          (not connected) */
                                        /* USB1_IND1          (not connected) */
    } else {
      /* Reset pin functions                                                  */
      LPC_SCU->SFSP2_6  = 0;
      LPC_SCU->SFSP2_13 = 0;
    }
  }
#else
  if (!ctrl) {
    if (on) {
      /* Set pin functions                                                    */
      LPC_SCU->SFSP6_3 = 1;             /* USB0_PPWR                          */
      LPC_SCU->SFSP6_6 = 3;             /* USB0_PWR_FAULT                     */
      LPC_SCU->SFSP8_2 = 1;             /* USB0_IND0                          */
      LPC_SCU->SFSP8_1 = 1;             /* USB0_IND1                          */
    } else {
      /* Reset pin functions                                                  */
      LPC_SCU->SFSP6_3 = 0;
      LPC_SCU->SFSP6_6 = 0;
      LPC_SCU->SFSP8_2 = 0;
      LPC_SCU->SFSP8_1 = 0;
    }
  } else {
    if (on) {
      /* Set pin functions                                                    */
      LPC_SCU->SFSP9_5 = 2;             /* USB1_PPWR                          */
      LPC_SCU->SFSP9_6 = 2;             /* USB1_PWR_FAULT                     */
      LPC_SCU->SFSP9_4 = 2;             /* USB1_IND0                          */
      LPC_SCU->SFSP9_3 = 2;             /* USB1_IND1                          */
    } else {
      /* Reset pin functions                                                  */
      LPC_SCU->SFSP9_5 = 0;
      LPC_SCU->SFSP9_6 = 0;
      LPC_SCU->SFSP9_4 = 0;
      LPC_SCU->SFSP9_3 = 0;
    }
  }
#endif

  return (__TRUE);
}


/*------------------------- usbh_ehci_hw_init ----------------------------------
 *
 *  Initialize or uninitialize the USB Host Controller.
 *
 *  Parameter:  ctrl:       Controller index (0 .. 1)
 *              on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ehci_hw_init (U8 ctrl, BOOL on) {
  U32 tout;

  if (!ctrl) {
#if    (USBH_EHCI_EN & 1)
    if (on) {
      usbh_ehci_hw_delay_ms (10);                     /* Calibrate wait       */

      /* Initialize memory pool for data                                      */
      if (!usbh_mem_init(0, (U32 *)&usbh0_ehci_mpool, sizeof(usbh0_ehci_mpool)))
        return (__FALSE);

      /* Enable USB0 base clock                                               */
      LPC_CCU1->CLK_M4_USB0_CFG |= 1;
      tout = 10100;
      while (!(LPC_CCU1->CLK_M4_USB0_STAT & 1)){      /* Wait clock enable    */
        if (tout-- <= 100) {
          if (tout == 0)
            return (__FALSE);
          usbh_ehci_hw_delay_ms (10);                 /* Wait ~10 ms          */
        }
      }
      /* Enable USB0 base clock                                               */
      LPC_CCU1->CLK_USB0_CFG |= 1;
      tout = 10100;
      while (!(LPC_CCU1->CLK_USB0_STAT & 1)){         /* Wait clock enable    */
        if (tout-- <= 100) {
          if (tout == 0)
            return (__FALSE);
          usbh_ehci_hw_delay_ms (10);                 /* Wait ~10 ms          */
        }
      }
      LPC_CREG->CREG0 &= ~(1 << 5);     /* Enable USB0 PHY                    */

      NVIC_SetPriority (USB0_IRQn, 0);  /* Set USB0 interrupt highest priority*/
    } else {
      /* Disable USB0 base clock                                              */
      LPC_CCU1->CLK_M4_USB0_CFG &=~1;
      tout = 10100;
      while (LPC_CCU1->CLK_M4_USB0_STAT & 1){         /* Wait clock enable    */
        if (tout-- <= 100) {
          if (tout == 0)
            return (__FALSE);
          usbh_ehci_hw_delay_ms (10);                 /* Wait ~10 ms          */
        }
      }
    }
#endif
  } else {
#if    (USBH_EHCI_EN & 2)
    if (on) {
      usbh_ehci_hw_delay_ms (10);                     /* Calibrate wait       */

      /* Initialize memory pool for data                                      */
      if (!usbh_mem_init(1, (U32 *)&usbh1_ehci_mpool, sizeof(usbh1_ehci_mpool)))
        return (__FALSE);

      /* Enable USB1 base clock                                               */
      LPC_CGU->BASE_USB1_CLK  = (0x01 << 11) |        /* Autoblock En         */
                                (0x0C << 24) ;        /* Clock source: IDIVA  */
      LPC_CCU1->CLK_M4_USB1_CFG |= 1;
      tout = 10100;
      while (!(LPC_CCU1->CLK_M4_USB1_STAT & 1)){      /* Wait clock enable    */
        if (tout-- <= 100) {
          if (tout == 0)
            return (__FALSE);
          usbh_ehci_hw_delay_ms (10);                 /* Wait ~10 ms          */
        }
      }
      LPC_SCU->SFSUSB = 0x17;           /* USB_AIM=1, USB_ESEA=1, USB_EPD = 1,
                                           USB_EPWR=1                         */

      NVIC_SetPriority (USB1_IRQn, 0);  /* Set USB1 interrupt highest priority*/
    } else {
      LPC_SCU->SFSUSB = 0x02;           /* USB_AIM=0, USB_ESEA=1, USB_EPWR=0  */

      /* Disable USB1 base clock                                              */
      LPC_CCU1->CLK_M4_USB1_CFG &=~1;
      tout = 10100;
      while (LPC_CCU1->CLK_M4_USB1_STAT & 1){         /* Wait clock enable    */
        if (tout-- <= 100) {
          if (tout == 0)
            return (__FALSE);
          usbh_ehci_hw_delay_ms (10);                 /* Wait ~10 ms          */
        }
      }
    }
#endif
  }

  return (__TRUE);
}


/*------------------------- usbh_ehci_hw_port_power ----------------------------
 *
 *  Turn Port Power on or off with pin if not handled through EHCI.
 *
 *  Parameter:  ctrl:       Controller index (0 .. 1)
 *              on:         __TRUE = turn power on, __FALSE = turn power off
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ehci_hw_port_power (U8 ctrl, BOOL on) {

  return (__TRUE);
}


/*------------------------- usbh_ehci_hw_irq_en --------------------------------
 *
 *  USB Host EHCI Controller interrupt enable or disable.
 *
 *  Parameter:  ctrl:       Controller index (0 .. 1)
 *              on:         __TRUE = enable, __FALSE = disable
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ehci_hw_irq_en (U8 ctrl, BOOL on) {

  if (!ctrl) {
    if (on) {
      NVIC_EnableIRQ  (USB0_IRQn);      /* Enable USB0 interrupt              */
    } else {
      NVIC_DisableIRQ (USB0_IRQn);      /* Disable USB0 interrupt             */
    }
  } else {
    if (on) {
      NVIC_EnableIRQ  (USB1_IRQn);      /* Enable USB1 interrupt              */
    } else {
      NVIC_DisableIRQ (USB1_IRQn);      /* Disable USB1 interrupt             */
    }
  }

  return (__TRUE);
}


/*------------------------- USBx_IRQHandler ------------------------------------
 *
 *  Hardware USB Interrupt Handler Routine.
 *
 *  Parameter:
 *  Return:
 *----------------------------------------------------------------------------*/

#if    (USBH_EHCI_EN & 1)
void USB0_IRQHandler (void) {
  USBH_EHCI_NXP_IRQHandler(0);
}
#endif
#if  (USBH_EHCI_EN & 2)
void USB1_IRQHandler (void) {
  USBH_EHCI_NXP_IRQHandler(1);
}
#endif
