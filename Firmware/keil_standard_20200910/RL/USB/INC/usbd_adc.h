/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_adc.h
 *      Purpose: USB Device Audio Device Class header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_ADC_H__
#define __USBD_ADC_H__


/*--------------------------- Global variables -------------------------------*/

/* USB Device Audio Class Device Global Variables */
extern        U16 USBD_ADC_VolCur;
extern const  U16 USBD_ADC_VolMin;
extern const  U16 USBD_ADC_VolMax;
extern const  U16 USBD_ADC_VolRes;
extern        U8  USBD_ADC_Mute;
extern        U32 USBD_ADC_Volume;
extern        U16 USBD_ADC_DataOut;
extern        U16 USBD_ADC_DataIn;
extern        U8  USBD_ADC_DataRun;

/* USB Device Audio Class Device library settings Variables */
extern const  U32 usbd_adc_cfg_datafreq;
extern const  U32 usbd_adc_cfg_p_s;
extern const  U32 usbd_adc_cfg_p_c;
extern const  U32 usbd_adc_cfg_b_s;
extern        S16 USBD_ADC_DataBuf      [];


/*--------------------------- Event handling routines ------------------------*/

extern void USBD_ADC_SOF_Event     (void);


#endif  /* __USBD_ADC_H__ */
