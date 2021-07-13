/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_hid_barcode.c
 *      Purpose: Host Human Interface Device Class module example with example 
 *               for HID Keyboard emulation barcode scanner custom handling of 
 *               data (it can be inserted into HID_Kbd example and compiled into
 *               the project)
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <string.h>


      U8  data_in[8];
      U8  data_in_received;
const U8 *key = "\0\0\0\0abcdefghijklmnopqrstuvwxyz1234567890\r\x1B\x08\x09 -=[]\\";

/*------------------------- usbh_hid_parse_hid_desc ----------------------------
 *
 *  Callback function called for parsing HID report descriptor.
 *
 *  Parameter:  ctrl:                Controller index (0 .. USBH_HC_NUM-1)
 *              dev_idx:             Device instance index (0 .. USBH_HID_NUM-1)
 *              ptrHIDReportDesc:    Pointer to HID report descriptor
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_hid_parse_report_desc (U8 ctrl, U8 dev_idx, U8 *ptrHIDReportDesc) {
  /* Custom parsing of report descriptor should be done here                  */
}


/*------------------------- usbh_hid_data_in_callback --------------------------
 *
 *  Function that is called automatically upon data reception on interrupt in 
 *  endpoint
 *
 *  Parameter:  ctrl:                Controller index (0 .. USBH_HC_NUM-1)
 *              dev_idx:             Device instance index (0 .. USBH_HID_NUM-1)
 *              ptr_data:            Pointer to received data
 *              data_len:            Number of bytes received on Interrupt In 
 *                                   Endpoint
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_hid_data_in_callback (U8 ctrl, U8 dev_idx, U8 *ptr_data, U16 data_len) {
  /* Received data is stored to global variable data_in                       */

  if (data_len <= 8) {
    memcpy (data_in, ptr_data, data_len);
    data_in_received = 1;
  }
}


/*------------------------- usbh_hid_kbd_getkey --------------------------------
 *
 *  Function that retrives received data from the HID device
 *
 *  Parameter:  ctrl:                Controller index (0 .. USBH_HC_NUM-1)
 *              dev_idx:             Device instance index (0 .. USBH_HID_NUM-1)
 *  Return:                          >= 0 = Data, -1 = no new data
 *----------------------------------------------------------------------------*/

int usbh_hid_kbd_getkey (U8 ctrl, U8 dev_idx) {
  int i, dat;

  /* First unread byte from data_in is returned                               */

    if (data_in_received) {
    for (i = 2; i < 8; i++) {
      dat = data_in[i];
      if (dat) {
        dat = key[dat];
                if (data_in[0] & 0x22) {        /* If shift pressed                   */
          dat &= ~0x20;
                }
        data_in[i] = 0;
        return (dat);
            }
        }
    data_in_received = 0;
  }

  return (-1);
}
