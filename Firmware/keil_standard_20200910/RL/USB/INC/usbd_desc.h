/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_desc.h
 *      Purpose: USB Device Descriptors header file
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBD_DESC_H__
#define __USBD_DESC_H__

#define WBVAL(x)                          (x & 0xFF),((x >> 8) & 0xFF)
#define B3VAL(x)                          (x & 0xFF),((x >> 8) & 0xFF),((x >> 16) & 0xFF)
#define USB_DEVICE_DESC_SIZE              (sizeof(USB_DEVICE_DESCRIPTOR))
#define USB_DEVICE_QUALI_SIZE             (sizeof(USB_DEVICE_QUALIFIER_DESCRIPTOR))
#define USB_CONFIGUARTION_DESC_SIZE       (sizeof(USB_CONFIGURATION_DESCRIPTOR))
#define USB_INTERFACE_ASSOC_DESC_SIZE     (sizeof(USB_INTERFACE_ASSOCIATION_DESCRIPTOR))
#define USB_INTERFACE_DESC_SIZE           (sizeof(USB_INTERFACE_DESCRIPTOR))
#define USB_ENDPOINT_DESC_SIZE            (sizeof(USB_ENDPOINT_DESCRIPTOR))
#define USB_HID_DESC_SIZE                 (sizeof(HID_DESCRIPTOR))
#define USB_HID_REPORT_DESC_SIZE          (sizeof(USBD_HID_ReportDescriptor))

#endif  /* __USBD_DESC_H__ */
