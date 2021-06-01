/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_ohci_lpc177x_8x.c
 *      Purpose: OHCI Hardware Specific Layer Driver for the 
 *               NXP LPC177x/8x Device Series
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <LPC177x_8x.h>


/************************* OHCI Hardware Driver Configuration *****************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <h> USB Host OHCI Settings
//   <o0> USB Host OHCI Controller Base Address
//   <h> Root Hub ports used by OHCI Controller
//     <i> These are the ports that OHCI will use.
//     <o1.0>  Port 1
//     <o1.1>  Port 2
//     <o1.2>  Port 3
//     <o1.3>  Port 4
//     <o1.4>  Port 5
//     <o1.5>  Port 6
//     <o1.6>  Port 7
//     <o1.7>  Port 8
//     <o1.8>  Port 9
//     <o1.9>  Port 10
//     <o1.10> Port 11
//     <o1.11> Port 12
//     <o1.12> Port 13
//     <o1.13> Port 14
//     <o1.14> Port 15
//     </h>
//
//   <o2> Start address of memory used by OHCI
//     <i> This is a start address of memory that OHCI will use for descriptors 
//     <i> and USB communication data.
//
//   <o3> Size of memory used for USB data by OHCI <1-1048576>
//     <i> This is a size of memory (in bytes) that OHCI will use for  
//     <i> USB communication data (maximum sum of data at a single point in 
//     <i> time, for example for HID = size of max Configuration Descriptor + 
//     <i> size of max HID Report Descriptor).
//
//   <o4> Maximum number of Endpoint Descriptors used by OHCI <1-64>
//     <i> This is a maximum number of Endpoints that OHCI will use.
//
//   <o5> Maximum number of Transfer Descriptors used by OHCI <1-64>
//     <i> This is a maximum number of Transfers that OHCI will use.
//
//   <o6> Maximum number of Isochronous Transfer Descriptors used by OHCI <0-64>
//     <i> This is a maximum number of Isochronous Transfers that OHCI will use.
// </h>
#define USBH_OHCI_ADR               0x2008C000
#define USBH_OHCI_PORTS             0x00000003
#define USBH_OHCI_MEM_ADR           0x20000000
#define USBH_OHCI_MEM_DATA_SZ       4096
#define USBH_OHCI_NUM_ED            5
#define USBH_OHCI_NUM_TD            2
#define USBH_OHCI_NUM_ITD           0

// *** <<< End of Configuration section             >>> ***


/************************** OHCI Host Controller Hardware Driver Variables ****/

#define USBH_OHCI_MEM_SZ           (256                   +  /* HCCA sz    */  \
                                    USBH_OHCI_NUM_ED * 16 +  /* EDs sz     */  \
                                    USBH_OHCI_NUM_TD * 16 +  /* TDs sz     */  \
                                    USBH_OHCI_MEM_DATA_SZ +  /* DATA sz    */  \
                                    USBH_OHCI_NUM_TD * 8  +  /* alloc ovhd */  \
                                    4                     )  /* alloc ovhd */

#define USBH_OHCI_MEM_HCCA         (USBH_OHCI_MEM_ADR)
#define USBH_OHCI_MEM_ED           (USBH_OHCI_MEM_ADR+256)
#define USBH_OHCI_MEM_TD           (USBH_OHCI_MEM_ED+(USBH_OHCI_NUM_ED<<4))
#define USBH_OHCI_MEM_ITD          (USBH_OHCI_MEM_TD+(USBH_OHCI_NUM_TD<<4))
#define USBH_OHCI_MEM_MPOOL        (USBH_OHCI_MEM_ITD+(USBH_OHCI_NUM_ITD<<5))
#define USBH_OHCI_MEM_SZ_MPOOL     (USBH_OHCI_MEM_SZ-(USBH_OHCI_MEM_MPOOL-USBH_OHCI_MEM_ADR))
#define USBH_OHCI_NUM_TDURB        (USBH_OHCI_NUM_TD+USBH_OHCI_NUM_ITD)

U32     usbh_ohci_hcca  [64]                        __attribute__((at(USBH_OHCI_MEM_ADR)));
U32     usbh_ohci_ed    [USBH_OHCI_NUM_ED<<2]       __attribute__((at(USBH_OHCI_MEM_ED)));
U32     usbh_ohci_td    [USBH_OHCI_NUM_TD<<2]       __attribute__((at(USBH_OHCI_MEM_TD)));
#if    (USBH_OHCI_NUM_ITD > 0)
U32     usbh_ohci_itd   [USBH_OHCI_NUM_ITD<<3]      __attribute__((at(USBH_OHCI_MEM_ITD)));
#endif
U32     usbh_ohci_mpool [USBH_OHCI_MEM_SZ_MPOOL>>2] __attribute__((at(USBH_OHCI_MEM_MPOOL)));
U32     usbh_ohci_tdurb [USBH_OHCI_NUM_TDURB<<1];

static U32 calDelay = 0;


/************************** OHCI Host Controller Hardware Function Prototypes */

void usbh_ohci_hw_get_capabilities (USBH_HCI_CAP *cap);
void usbh_ohci_hw_delay_ms         (U32 ms);
void usbh_ohci_hw_reg_wr           (U32 reg_ofs, U32 val);
U32  usbh_ohci_hw_reg_rd           (U32 reg_ofs);
BOOL usbh_ohci_hw_pins_config      (BOOL on);
BOOL usbh_ohci_hw_init             (BOOL on);
BOOL usbh_ohci_hw_port_power       (BOOL on);
BOOL usbh_ohci_hw_irq_en           (BOOL on);


/************************** OHCI Host Controller Hardware Driver Structure ****/

USBH_HWD_OHCI usbh0_hwd_ohci = {        /* OHCI Host Controller Hardware Drv  */
  USBH_OHCI_PORTS,                      /* Ports (bits 0..15)                 */
  USBH_OHCI_NUM_ED,                     /* Maximum Endpoint Descriptors       */
  USBH_OHCI_NUM_TD,                     /* Maximum Transfer Descriptors       */
  USBH_OHCI_NUM_ITD,                    /* Maximum Iso Transfer Descriptors   */
  (U32 *) &usbh_ohci_hcca,              /* Pointer to HCCA memory start       */
  (U32 *) &usbh_ohci_ed,                /* Pointer to ED memory start         */
  (U32 *) &usbh_ohci_td,                /* Pointer to TD memory start         */
#if    (USBH_OHCI_NUM_ITD > 0)
  (U32 *) &usbh_ohci_itd,               /* Pointer to ITD memory start        */
#else
  NULL,
#endif
  (U32 *) &usbh_ohci_tdurb,             /* Pointer to TDURB memory start      */
  usbh_ohci_hw_get_capabilities,        /* Get driver capabilities            */
  usbh_ohci_hw_delay_ms,                /* Delay in ms                        */
  usbh_ohci_hw_reg_wr,                  /* Write register                     */
  usbh_ohci_hw_reg_rd,                  /* Read register                      */
  usbh_ohci_hw_pins_config,             /* Config/Unconfig pins               */
  usbh_ohci_hw_init,                    /* Init/Uninit Host Controller        */
  usbh_ohci_hw_port_power,              /* On/Off Port Power                  */
  usbh_ohci_hw_irq_en                   /* Enable/Disable interrupt           */
};


/************************** Imported Functions ********************************/

extern void USBH_OHCI_IRQHandler (void);


/************************** Auxiliary Functions *******************************/

/*------------------------- usbh_i2c_init --------------------------------------
 *
 *  Initialize or uninitialize the I2C of the USB Host Controller.
 *
 *  Parameter:  on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_i2c_init (BOOL on) {
  S32 tout;

  if (on) {
    /* Enable pins                                                            */
    LPC_IOCON->P1_28 = 1;               /* USB_SCL1                           */
    LPC_IOCON->P1_29 = 1;               /* USB_SDA1                           */

    LPC_USB->OTGClkCtrl |= (1 << 2);    /* Enable I2C                         */

    tout = 10100;
    while (!(LPC_USB->OTGClkSt & (1 << 2))) {         /* Wait I2C clock enable*/
      if (tout-- <= 100) {
        if (tout == 0)
          return (__FALSE);
        usbh_ohci_hw_delay_ms (10);                   /* Wait ~10 ms          */
      }
    }

    LPC_USB->I2C_CTL   = 1 << 8;        /* I2C reset                          */
    tout = 10100;
    while (LPC_USB->I2C_CTL & (1 << 8)) {             /* Wait reset end       */
      if (tout-- <= 100) {
        if (tout == 0)
          return (__FALSE);
        usbh_ohci_hw_delay_ms (10);                   /* Wait ~10 ms          */
      }
    }

                                        /* Set I2C clock to 100kHz            */
    LPC_USB->I2C_CLKHI = 240;           /* 48MHz /200kHz = 240 clock cycles   */
    LPC_USB->I2C_CLKLO = 240;           /* 48MHz /200kHz = 240 clock cycles   */
  } else {
    /* Return I2C to state after reset                                        */
    LPC_USB->I2C_CLKHI = 0xB9;
    LPC_USB->I2C_CLKLO = 0xB9;

    LPC_USB->OTGClkCtrl &=~(1 << 2);    /* Disable I2C                        */

    tout = 10100;
    while (LPC_USB->OTGClkSt & (1 << 2)) {            /* Wait I2C clock disabl*/
      if (tout-- <= 100) {
        if (tout == 0)
          return (__FALSE);
        usbh_ohci_hw_delay_ms (10);                   /* Wait ~10 ms          */
      }
    }

    /* Disable pins                                                           */
    LPC_IOCON->P1_28 = 0x30;
    LPC_IOCON->P1_29 = 0x30;
  }

  return (__TRUE);
}


/*------------------------- usbh_i2c_read --------------------------------------
 *
 *  Read a value of the register from the I2C chip (byte value)
 *
 *  Parameter:  i2c_adr:    Address of the I2C chip
 *              reg_adr:    Address of the register to read
 *  Return:                 0 .. 0xFF = read value, -1 = error
 *----------------------------------------------------------------------------*/

int usbh_i2c_read (U8 i2c_adr, U8 reg_adr) {
  S32 tout;

  LPC_USB->I2C_TX    = (      1 << 8) | /* START bit on transmit start        */
                       (i2c_adr << 1) | /* I2C Address                        */
                       (      0 << 0) ; /* Write request                      */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (-1);
    if (tout <= 10) usbh_ohci_hw_delay_ms (10);
  }
  LPC_USB->I2C_TX    =  reg_adr;        /* Register address to read from      */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (-1);
    if (tout <= 10) usbh_ohci_hw_delay_ms (10);
  }
  LPC_USB->I2C_TX    = (      1 << 8) | /* START bit on transmit start        */
                       (i2c_adr << 1) | /* I2C Address                        */
                       (      1 << 0) ; /* Read request                       */
  LPC_USB->I2C_TX    = (      1 << 9) | /* STOP bit on end                    */
                        0x55;           /* Dummy data transmit to receive     */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 0)) break;       /* Wait for transaction done*/
    if (tout ==  0) return (-1);
    if (tout <= 10) usbh_ohci_hw_delay_ms (10);
  }
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (!(LPC_USB->I2C_STS & (1 << 5))) break;    /* Wait for STOP condition  */
    if (tout ==  0) return (-1);
    if (tout <= 10) usbh_ohci_hw_delay_ms (10);
  }
  return (LPC_USB->I2C_RX & 0xFF);
}


/*------------------------- usbh_i2c_write -------------------------------------
 *
 *  Write a value to the register on the I2C chip (byte value)
 *
 *  Parameter:  i2c_adr:    Address of the I2C chip
 *              reg_adr:    Address of the register to write
 *              reg_val:    Value to be written
 *  Return:                 __TRUE = success, __FALSE = error
 *----------------------------------------------------------------------------*/

int usbh_i2c_write (U8 i2c_adr, U8 reg_adr, U8 reg_val) {
  S32 tout;

  LPC_USB->I2C_TX    = (      1 << 8) | /* START bit on transmit start        */
                       (i2c_adr << 1) | /* I2C Address                        */
                       (      0 << 0) ; /* Write request                      */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (__FALSE);
    if (tout <= 10) usbh_ohci_hw_delay_ms (10);
  }
  LPC_USB->I2C_TX    =  reg_adr;        /* Register address to write to       */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (LPC_USB->I2C_STS & (1 << 11)) break;      /* Wait for Tx FIFO empty   */
    if (tout ==  0) return (__FALSE);
    if (tout <= 10) usbh_ohci_hw_delay_ms (10);
  }
  LPC_USB->I2C_TX    = (      1 << 9) | /* STOP bit on end                    */
                        reg_val;        /* Register value to write            */
  for (tout=1010; tout>=0; tout--) {    /* Wait for status, max 100 ms        */
    if (!(LPC_USB->I2C_STS & (1 << 5))) break;    /* Wait for STOP condition  */
    if (tout ==  0) return (__FALSE);
    if (tout <= 10) usbh_ohci_hw_delay_ms (10);
  }
  return (__TRUE);
}


/************************** Module Functions **********************************/

/*------------------------- usbh_ohci_hw_get_capabilities ----------------------
 *
 *  Get capabilities of Host Controller Driver
 *
 *  Parameter:  cap:        Pointer to USBH_HCI_CAP structure where 
 *                          capabilities are loaded
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ohci_hw_get_capabilities (USBH_HCI_CAP *cap) {
  cap->MultiPckt = __TRUE;
  cap->MaxDataSz = USBH_OHCI_MEM_DATA_SZ;
  cap->CtrlNAKs  = 1000;
  cap->BulkNAKs  = 1000000;
}


/*------------------------- usbh_ohci_hw_delay_ms ------------------------------
 *
 *  Delay execution (in milliseconds)
 *  Calibration is done if global variable calDelay is 0
 *
 *  Parameter:  ms:         Number of milliseconds to delay execution for
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ohci_hw_delay_ms (U32 ms) {
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


/*------------------------- usbh_ohci_hw_reg_wr --------------------------------
 *
 *  Write hardware register.
 *
 *  Parameter:  reg:        Register offset
 *              val:        Register value 
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ohci_hw_reg_wr (U32 reg_ofs, U32 val) {
  *((U32 *)(USBH_OHCI_ADR + reg_ofs)) = val;
}


/*------------------------- usbh_ohci_hw_reg_rd --------------------------------
 *
 *  Read hardware register.
 *
 *  Parameter:  reg:        Register offset
 *  Return:                 Register value 
 *----------------------------------------------------------------------------*/

U32 usbh_ohci_hw_reg_rd (U32 reg_ofs) {
  return (*((U32 *)(USBH_OHCI_ADR + reg_ofs)));
}


/*------------------------- usbh_ohci_hw_pins_cfg ------------------------------
 *
 *  Configurate or unconfigurate pins used by the USB Host.
 *
 *  Parameter:  on:         __TRUE = configurate, __FALSE = unconfigurate
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_pins_config (BOOL on) {

  if (on) {
    /* Set pin functions                                                      */
    LPC_IOCON->P0_29 = 1;               /* USB_D+1                            */
    LPC_IOCON->P0_30 = 1;               /* USB_D-1                            */
    LPC_IOCON->P0_31 = 1;               /* USB_D+2                            */
                                        /* USB_D-2 is dedicated pin           */

    LPC_IOCON->P1_18 = 1;               /* USB_UP_LED1                        */
//  LPC_IOCON->P1_19 = 2;               /* !USB_PPWR1                         */
//  LPC_IOCON->P1_22 = 2;               /* USB_PWRD1                          */
    LPC_IOCON->P1_27 = 2;               /* !USB_OVRCR1                        */
    
    LPC_IOCON->P0_12 = 1 | (1 << 7);    /* !USB_PPWR2                         */
    LPC_IOCON->P0_13 = 1 | (1 << 7);    /* USB_UP_LED2                        */
    LPC_IOCON->P0_14 = 1;               /* !USB_HSTEN2                        */
    LPC_IOCON->P1_30 = 1 | (1 << 7);    /* USB_PWRD2                          */
    LPC_IOCON->P1_31 = 1 | (1 << 7);    /* !USB_OVRCR2                        */
  } else {
    /* Reset pin functions                                                    */
    LPC_IOCON->P0_12 = 0xD0;
    LPC_IOCON->P0_13 = 0xD0;
    LPC_IOCON->P0_14 = 0x30;
    LPC_IOCON->P1_30 = 0xD0;
    LPC_IOCON->P1_31 = 0xD0;

    LPC_IOCON->P1_18 = 0x30;
//  LPC_IOCON->P1_19 = 0x30;
//  LPC_IOCON->P1_22 = 0x30;
    LPC_IOCON->P1_27 = 0x30;

    LPC_IOCON->P0_29 = 0;
    LPC_IOCON->P0_30 = 0;
    LPC_IOCON->P0_31 = 0;
  }

  return (__TRUE);
}


/*------------------------- usbh_ohci_hw_init ----------------------------------
 *
 *  Initialize or uninitialize the USB Host Controller.
 *
 *  Parameter:  on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_init (BOOL on) {
  S32 tout;

  if (on) {
    usbh_ohci_hw_delay_ms (10);         /* Calibrate delay                    */

    /* Initialize memory pool for data                                        */
    if (!usbh_mem_init(0, (U32 *)&usbh_ohci_mpool, sizeof(usbh_ohci_mpool)))
      return (__FALSE);

    LPC_SC->PCONP       |=  (1UL << 31);/* Enable USB interface power/clock   */
    LPC_USB->OTGClkCtrl |=  0x19;       /* Enable Host, OTG, AHB mster clk    */

    tout = 10100;
    while (!((LPC_USB->OTGClkSt & 0x19) == 0x19)) {   /* Wait clock enable    */
      if (tout-- <= 100) {
        if (tout == 0)
          return (__FALSE);
        usbh_ohci_hw_delay_ms (10);                   /* Wait ~10 ms          */
      }
    }

    LPC_USB->StCtrl   =  0x01;          /* Both ports used as Host            */

    /* Enable USB1 Transceiver                                                */
    usbh_i2c_init(__TRUE);
    switch (usbh_i2c_read (0x2D, 0x00) | (usbh_i2c_read (0x2D, 0x01) << 8)) {
      case 0x058D:                      /* MIC2555 Transceiver                */
        /* Control Register 2 = Enable Pull-downs, Power VBUS                 */
        usbh_i2c_write (0x2D, 0x06, 0x2C);
        break;
    }

    NVIC_SetPriority (USB_IRQn, 0);     /* Set USB interrupt highest priority */
  } else {
    LPC_USB->StCtrl     &= ~0x01;       /* Deselect port function             */

    LPC_USB->OTGClkCtrl &= ~0x1D;       /* Disable Host,I2C,OTG, AHB mster clk*/

    for (tout = 100; ; tout--) {
      if ((LPC_USB->OTGClkSt & 0x1B) == 0)/* Wait for clocks disabled         */
        break;
      if (!tout) 
        return (__FALSE);
    }

    LPC_SC->PCONP       &= ~(1UL << 31);/* Disable USB interface power/clock  */
  }

  return (__TRUE);
}


/*------------------------- usbh_ohci_hw_port_power ----------------------------
 *
 *  Turn Port Power on or off with pin if not handled through OHCI.
 *
 *  Parameter:  on:         __TRUE = turn power on, __FALSE = turn power off
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_port_power (BOOL on) {

  return (__TRUE);
}


/*------------------------- usbh_ohci_hw_irq_en --------------------------------
 *
 *  USB Host OHCI Controller interrupt enable or disable.
 *
 *  Parameter:  on:         __TRUE = enable, __FALSE = disable
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_irq_en (BOOL on) {

  if (on) {
    NVIC_EnableIRQ  (USB_IRQn);         /* Enable USB interrupt               */
  } else {
    NVIC_DisableIRQ (USB_IRQn);         /* Disable USB interrupt              */
  }

  return (__TRUE);
}


/*------------------------- USB_IRQHandler -------------------------------------
 *
 *  Hardware USB Interrupt Handler Routine.
 *
 *  Parameter:
 *  Return:
 *----------------------------------------------------------------------------*/

void USB_IRQHandler (void) {
  USBH_OHCI_IRQHandler();
}
