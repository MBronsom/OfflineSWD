/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    rl_usb.h
 *      Purpose: Main header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2014 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __RL_USB_H__
#define __RL_USB_H__

#ifdef __cplusplus
extern "C"  {
#endif

#include <stdint.h>
#include <..\..\RL\USB\INC\usb.h>

/*****************  Functions *************************************************/

/* USB Host functions exported from USB Host Core module                      */
extern BOOL  usbh_mem_init              (U8 ctrl, U32 *ptr_pool, U32 pool_sz);
extern BOOL  usbh_mem_alloc             (U8 ctrl, U8 **ptr, U32 sz);
extern BOOL  usbh_mem_free              (U8 ctrl, U8  *ptr);
extern BOOL  usbh_transfer              (U8 ctrl, USBH_EP *ptr_ep, USBH_URB *ptr_urb, U16 tout);
extern BOOL  usbh_init                  (U8 ctrl);
extern BOOL  usbh_init_all              (void);
extern BOOL  usbh_uninit                (U8 ctrl);
extern BOOL  usbh_uninit_all            (void);
extern BOOL  usbh_engine                (U8 ctrl);
extern BOOL  usbh_engine_all            (void);
extern U32   usbh_get_last_error        (U8 ctrl);
extern U8   *usbh_get_error_string      (U32 err);

/* USB Host functions exported from USB Mass Storage Class module             */
extern BOOL  usbh_msc_status            (U8 ctrl, U8 dev_idx);
extern BOOL  usbh_msc_read              (U8 ctrl, U8 dev_idx, U32  blk_adr, U8 *ptr_data, U16 blk_num);
extern BOOL  usbh_msc_write             (U8 ctrl, U8 dev_idx, U32  blk_adr, U8 *ptr_data, U16 blk_num);
extern BOOL  usbh_msc_read_config       (U8 ctrl, U8 dev_idx, U32 *tot_blk_num, U32 *blk_sz);
extern U32   usbh_msc_get_last_error    (U8 ctrl, U8 dev_idx);

/* USB Host functions exported from USB Human Interface Device Class module   */
extern BOOL  usbh_hid_status            (U8 ctrl, U8 dev_idx);
extern int   usbh_hid_data_in           (U8 ctrl, U8 dev_idx, U8  *ptr_data);
extern int   usbh_hid_data_out          (U8 ctrl, U8 dev_idx, U8  *ptr_data, U16 data_len);
extern U32   usbh_hid_get_last_error    (U8 ctrl, U8 dev_idx);
/* Overridable functions                                                      */
extern void  usbh_hid_parse_report_desc (U8 ctrl, U8 dev_idx, U8  *ptrHIDReportDesc);
extern void  usbh_hid_data_in_callback  (U8 ctrl, U8 dev_idx, U8  *ptr_data, U16 data_len);
extern int   usbh_hid_kbd_getkey        (U8 ctrl, U8 dev_idx);  
extern BOOL  usbh_hid_mouse_getdata     (U8 ctrl, U8 dev_idx, U8  *btn, S8 *x, S8 *y, S8 *wheel);

/* USB Device functions exported from USB Device Core module                  */
extern void  usbd_init                  (void);
extern void  usbd_connect               (BOOL con);
extern void  usbd_reset_core            (void);
extern BOOL  usbd_configured            (void);

/* USB Device user functions imported to USB HID Class module                 */
extern void  usbd_hid_init              (void);
extern BOOL  usbd_hid_get_report_trigger(U8 rid,   U8 *buf, int len);
extern int   usbd_hid_get_report        (U8 rtype, U8 rid, U8 *buf, U8  req);
extern void  usbd_hid_set_report        (U8 rtype, U8 rid, U8 *buf, int len, U8 req);
extern U8    usbd_hid_get_protocol      (void);
extern void  usbd_hid_set_protocol      (U8 protocol);

/* USB Device user functions imported to USB Mass Storage Class module        */
extern void  usbd_msc_init              (void);
extern void  usbd_msc_read_sect         (U32 block, U8 *buf, U32 num_of_blocks);
extern void  usbd_msc_write_sect        (U32 block, U8 *buf, U32 num_of_blocks);
extern void  usbd_msc_start_stop        (BOOL start);

/* USB Device user functions imported to USB Audio Class module               */
extern void  usbd_adc_init              (void);

/* USB Device CDC ACM class functions called automatically by USBD Core module*/
extern int32_t  USBD_CDC_ACM_Initialize                (void);
extern int32_t  USBD_CDC_ACM_Uninitialize              (void);
extern int32_t  USBD_CDC_ACM_Reset                     (void);
/* USB Device CDC ACM class user functions                                    */
extern int32_t  USBD_CDC_ACM_PortInitialize            (void);
extern int32_t  USBD_CDC_ACM_PortUninitialize          (void);
extern int32_t  USBD_CDC_ACM_PortReset                 (void);
extern int32_t  USBD_CDC_ACM_PortSetLineCoding         (CDC_LINE_CODING *line_coding);
extern int32_t  USBD_CDC_ACM_PortGetLineCoding         (CDC_LINE_CODING *line_coding);
extern int32_t  USBD_CDC_ACM_PortSetControlLineState   (uint16_t ctrl_bmp);
extern int32_t  USBD_CDC_ACM_DataSend                  (const uint8_t *buf, int32_t len);
extern int32_t  USBD_CDC_ACM_PutChar                   (const uint8_t  ch);
extern int32_t  USBD_CDC_ACM_DataRead                  (      uint8_t *buf, int32_t len);
extern int32_t  USBD_CDC_ACM_GetChar                   (void);
extern int32_t  USBD_CDC_ACM_DataAvailable             (void);
extern int32_t  USBD_CDC_ACM_Notify                    (uint16_t stat);
/* USB Device CDC ACM class overridable functions                             */
extern int32_t  USBD_CDC_ACM_SendEncapsulatedCommand   (void);
extern int32_t  USBD_CDC_ACM_GetEncapsulatedResponse   (void);
extern int32_t  USBD_CDC_ACM_SetCommFeature            (uint16_t feat);
extern int32_t  USBD_CDC_ACM_GetCommFeature            (uint16_t feat);
extern int32_t  USBD_CDC_ACM_ClearCommFeature          (uint16_t feat);
extern int32_t  USBD_CDC_ACM_SetLineCoding             (void);
extern int32_t  USBD_CDC_ACM_GetLineCoding             (void);
extern int32_t  USBD_CDC_ACM_SetControlLineState       (uint16_t ctrl_bmp);
extern int32_t  USBD_CDC_ACM_SendBreak                 (uint16_t dur);

/* USB Device user functions imported to USB Custom Class module              */
extern void  usbd_cls_init              (void);
extern void  usbd_cls_sof               (void);
extern BOOL  usbd_cls_dev_req           (BOOL setup);
extern BOOL  usbd_cls_if_req            (BOOL setup);
extern BOOL  usbd_cls_ep_req            (BOOL setup);

#ifdef __cplusplus
}
#endif

#endif  /* __RL_USB_H__ */
