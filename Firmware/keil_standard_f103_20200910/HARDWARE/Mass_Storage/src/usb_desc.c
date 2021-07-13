/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Descriptors for Mass Storage Device
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_desc.h"

const uint8_t MASS_DeviceDescriptor[MASS_SIZ_DEVICE_DESC] =  {
    0x12,   // bLength 
    0x01,   // bDescriptorType
    0x00,   // bcdUSB, version 2.00
    0x02,
    0x00,   // bDeviceClass : each interface define the device class
    0x00,   // bDeviceSubClass
    0x00,   // bDeviceProtocol
    
    0x40,   // bMaxPacketSize0 0x40 = 64
    
    0x83,   // idVendor     (0283)
    0x02,
    0x20,   // idProduct
    0x57,
    
    0x00,   // bcdDevice 2.00*/
    0x02,
    1,      // index of string Manufacturer 
    2,      // index of string descriptor of product*/
    3,      //
    0x01    //bNumConfigurations
};

const uint8_t MASS_ConfigDescriptor[MASS_SIZ_CONFIG_DESC] =  {

    0x09,   // bLength: Configuation Descriptor size
    0x02,   // bDescriptorType: Configuration
    MASS_SIZ_CONFIG_DESC,
    0x00,
    0x02,   // bNumInterfaces: 2 interface
    0x01,   // bConfigurationValue:
    0x00,   // iConfiguration:
    0xC0,   // bmAttributes:
    0x32,   // MaxPower 100 mA

    /******************** Descriptor of Mass Storage interface ********************/
    0x09,   // bLength: Interface Descriptor size
    0x04,   // bDescriptorType:
    0x00,   // bInterfaceNumber: Number of Interface
    0x00,   // bAlternateSetting: Alternate setting
    0x02,   // bNumEndpoints*/
    0x08,   // bInterfaceClass: MASS STORAGE Class
    0x06,   // bInterfaceSubClass : SCSI transparent*/
    0x50,   // nInterfaceProtocol
    4,      // iInterface:

    0x07,   // Endpoint descriptor length = 7
    0x05,   // Endpoint descriptor type
    0x81,   // Endpoint address (IN, address 1)------------------------------------------------------------------------------------
    0x02,   // Bulk endpoint type
    0x40,   // Maximum packet size (64 bytes)
    0x00,
    0x00,   // Polling interval in milliseconds

    0x07,   // Endpoint descriptor length = 7
    0x05,   // Endpoint descriptor type
    0x02,   // Endpoint address (OUT, address 2)--------------------------------------------------------------------------------------
    0x02,   // Bulk endpoint type
    0x40,   // Maximum packet size (64 bytes)
    0x00,
    0x00,   // Polling interval in milliseconds
      
            //Interface Descriptor
    0x09,   // bLength: Interface Descriptor size
    0x04,   // bDescriptorType: Interface
    0x01,   // bInterfaceNumber: Number of Interface
    0x00,   // bAlternateSetting: Alternate setting
    0x03,   // bNumEndpoints: 3 endpoints used
    0x02,   // bInterfaceClass: Communication Interface Class
    0x02,   // bInterfaceSubClass: Abstract Control Model
    0x01,   // bInterfaceProtocol: Common AT commands
    0x00,   // iInterface:
    //Header Functional Descriptor
    0x05,   // bLength: Endpoint Descriptor size
    0x24,   // bDescriptorType: CS_INTERFACE
    0x00,   // bDescriptorSubtype: Header Func Desc
    0x10,   // bcdCDC: spec release number
    0x01,
    //Call Managment Functional Descriptor
    0x05,   // bFunctionLength
    0x24,   // bDescriptorType: CS_INTERFACE
    0x01,   // bDescriptorSubtype: Call Management Func Desc
    0x00,   // bmCapabilities: D0+D1
    0x01,   // bDataInterface: 1
    //ACM Functional Descriptor
    0x04,   // bFunctionLength
    0x24,   // bDescriptorType: CS_INTERFACE
    0x02,   // bDescriptorSubtype: Abstract Control Management desc
    0x02,   // bmCapabilities
    //Union Functional Descriptor
    0x05,   // bFunctionLength
    0x24,   // bDescriptorType: CS_INTERFACE
    0x06,   // bDescriptorSubtype: Union func desc
    0x00,   // bMasterInterface: Communication class interface
    0x01,   // bSlaveInterface0: Data Class Interface
    
    //Endpoint 2 Descriptor
    0x07,   // bLength: Endpoint Descriptor size
    0x05,    // bDescriptorType: Endpoint
    0x85,   // bEndpointAddress: (IN2)
    0x03,   // bmAttributes: Interrupt
    0x08,   // wMaxPacketSize:
    0x00,
    0xFF,   // bInterval:
    
    //Endpoint 3 Descriptor
    0x07,   // bLength: Endpoint Descriptor size
    0x05,   // bDescriptorType: Endpoint
    0x06,   // bEndpointAddress: (OUT6)//端点6为输出//由PC端发数据到STM32//  
    0x02,   // bmAttributes: Bulk
    0x40,   // wMaxPacketSize:
    0x00,
    0x00,   // bInterval: ignore for Bulk transfer
    
    //Endpoint 3 Descriptor
    0x07,   // bLength: Endpoint Descriptor size
    0x05,   // bDescriptorType: Endpoint
    0x84,   // bEndpointAddress: (IN4)//端点4为输入//由STM32发数据到PC端//  
    0x02,   // bmAttributes: Bulk
    0x40,   // wMaxPacketSize:
    0x00,
    0x00    // bInterval      
};

const uint8_t MASS_StringLangID[MASS_SIZ_STRING_LANGID] =
  {
    MASS_SIZ_STRING_LANGID,
    0x03,
    0x09,
    0x04
  }
  ;      /* LangID = 0x0409: U.S. English */
const uint8_t MASS_StringVendor[MASS_SIZ_STRING_VENDOR] =
  {
    MASS_SIZ_STRING_VENDOR, /* Size of manufaturer string */
    0x03,           /* bDescriptorType = String descriptor */
    /* Manufacturer: "STMicroelectronics" */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
  };
const uint8_t MASS_StringProduct[MASS_SIZ_STRING_PRODUCT] =
  {
    MASS_SIZ_STRING_PRODUCT,
    0x03,
    /* Product name: "STM32F10x:USB Mass Storage" */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'M', 0, 'a', 0, 's', 0,
    's', 0, ' ', 0, 'S', 0, 't', 0, 'o', 0, 'r', 0, 'a', 0, 'g', 0, 'e', 0

  };

uint8_t MASS_StringSerial[MASS_SIZ_STRING_SERIAL] =
  {
    MASS_SIZ_STRING_SERIAL,
    0x03,
    /* Serial number*/
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
  };
const uint8_t MASS_StringInterface[MASS_SIZ_STRING_INTERFACE] =
  {
    MASS_SIZ_STRING_INTERFACE,
    0x03,
    /* Interface 0: "ST Mass" */
    'S', 0, 'T', 0, ' ', 0, 'M', 0, 'a', 0, 's', 0, 's', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
