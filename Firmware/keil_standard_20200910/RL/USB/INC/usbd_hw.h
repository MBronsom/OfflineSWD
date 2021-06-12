/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_hw.h
 *      Purpose: USB Device Hardware Layer header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_HW_H__
#define __USBD_HW_H__


/* USB Hardware Functions */
extern void USBD_Init        (void);
extern void USBD_Connect     (BOOL con);
extern void USBD_Reset       (void);
extern void USBD_Suspend     (void);
extern void USBD_Resume      (void);
extern void USBD_WakeUp      (void);
extern void USBD_WakeUpCfg   (BOOL cfg);
extern void USBD_SetAddress  (U32  adr, U32 setup);
extern void USBD_Configure   (BOOL cfg);
extern void USBD_ConfigEP    (USB_ENDPOINT_DESCRIPTOR *pEPD);
extern void USBD_DirCtrlEP   (U32  dir);
extern void USBD_EnableEP    (U32  EPNum);
extern void USBD_DisableEP   (U32  EPNum);
extern void USBD_ResetEP     (U32  EPNum);
extern void USBD_SetStallEP  (U32  EPNum);
extern void USBD_ClrStallEP  (U32  EPNum);
extern void USBD_ClearEPBuf  (U32  EPNum);
extern U32  USBD_ReadEP      (U32  EPNum, U8 *pData);
extern U32  USBD_WriteEP     (U32  EPNum, U8 *pData, U32 cnt);
extern U32  USBD_GetFrame    (void);
extern U32  USBD_GetError    (void);

#endif  /* __USBD_HW_H__ */
