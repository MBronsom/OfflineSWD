/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbd_cls.c
 *      Purpose: USB Device Custom Class specific module example with example 
 *               for handling custom class interface requests
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <string.h>

#define __NO_USB_LIB_C
#include "usb_config.c"

#define  NUMBER_OF_BYTES_TO_SEND 64

/* Data receive and send buffers */
U8 data_recv[64];
U8 data_send[64];


void  usbd_cls_init              (void) {
  /* Called upon USB Device stack initialization 
     used for any specific initialization of custom class */
}
void  usbd_cls_sof               (void) {
  /* Called upon Start Of Frame, used for handling of 
     custom class periodic requests */
}

BOOL usbd_cls_dev_req (BOOL setup) {
  /* Should handle any custom class device requests */
  /* See sample code for handling custom class interface requests */
  return(__FALSE);                                /* If no handling return __FALSE */
}

BOOL usbd_cls_if_req  (BOOL setup) {
  /* Should handle any custom class interface requests */
  /* Sample code which demonstrates how to use it */
  if (USBD_SetupPacket.wIndexL == 0x01) {         /* Check interface number of custom class */
    if (setup) {                                  /* Interface request is in SETUP stage (after setup packet has been received) */
    /* USBD_SetupPacket contains information of received request: bRequest, wValue, wIndex, wLength 
       which can be analyzed to determine what exactly needs to be done and to prepare response */
      switch (USBD_SetupPacket.bRequest) {
        case 0x01:                                /* Request: 0x01 - custom defined request */
          if (USBD_SetupPacket.bmRequestType.Dir == REQUEST_HOST_TO_DEVICE) {
            /* Request which expects data to be received from host */
            /* Prepare buffer for data reception */
            USBD_EP0Data.pData = data_recv;
          } else {
            /* Request which expects data to be sent to host */ 
            /* Prepare data to send */
            data_send[0] = 0xAB;
            data_send[1] = 0xCD;
            /* ... */
            USBD_EP0Data.pData = data_send;
            if (USBD_EP0Data.Count > NUMBER_OF_BYTES_TO_SEND) {
              /* If more data is requested then we have to send, we have to correct Count. 
                 Also if last packet is sized as maximum packet we have to generate
                 Zero length packet to terminate the response. */
              USBD_EP0Data.Count = NUMBER_OF_BYTES_TO_SEND;
              if (!(USBD_EP0Data.Count & (USBD_MAX_PACKET0 - 1))) USBD_ZLP = 1;
            }
            USBD_DataInStage();
          }
          return (__TRUE);
      }
    } else {                                      /* Interface request is in OUT stage (after data has been received) */
      switch (USBD_SetupPacket.bRequest) {
        case 0x01:                                /* Request: 0x01 - custom defined request */
          /* Received data available for processing
               Data buffer: data_recv
               Data length: USBD_SetupPacket.wLength
           */
          USBD_StatusInStage();                   /* send Acknowledge */
          return (__TRUE);
      }
    }
  }
  return (__FALSE);                               /* If this was not a request for our custom class interface or 
                                                     if it was a request that we do not handle */
}

BOOL usbd_cls_ep_req  (BOOL setup) {
  /* Should handle any custom class endpoint requests */
  /* See sample code for handling custom class interface requests */
  return(__FALSE);                                /* If no handling return __FALSE */
}

