/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh.h
 *      Purpose: Host Core header file
 *      Rev.:    V4.74
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __USBH_H__
#define __USBH_H__

#pragma anon_unions


/************************* Constants ******************************************/

/* Error values that USB Host functions can return                            */
typedef enum { 
  ERROR_SUCCESS = 0,                    /* No error                           */
                                        /* Errors                             */
  ERROR_USBH_HCI,                       /* Host Ctrl Instance does not exist  */
  ERROR_USBH_HCD,                       /* Host Ctrl Driver does not exist    */
  ERROR_USBH_DCI,                       /* Device Class Instance does not exst*/
  ERROR_USBH_DCD,                       /* Device Class Driver does not exist */

  ERROR_USBH_MEM_INIT,                  /* Memory initialization failed       */
  ERROR_USBH_MEM_ALLOC,                 /* Memory allocation failed           */
  ERROR_USBH_MEM_DEALLOC,               /* Memory deallocation failed         */

  ERROR_USBH_PINS_CONFIG,               /* Pins not conf/unconfigured         */
  ERROR_USBH_INIT,                      /* Controller not init/uninitialized  */
  ERROR_USBH_PORT_POWER,                /* Port power driving failed          */
  ERROR_USBH_PORT_RESET,                /* Port reset failed                  */

  ERROR_USBH_EP,                        /* Endpoint does not exist            */
  ERROR_USBH_EP_ADD,                    /* Endpoint was not added             */
  ERROR_USBH_EP_CONFIG,                 /* Endpoint was not configured        */
  ERROR_USBH_EP_REMOVE,                 /* Endpoint was not removed           */

  ERROR_USBH_URB,                       /* URB does not exist                 */
  ERROR_USBH_URB_SUBMIT,                /* URB was not submitted              */
  ERROR_USBH_URB_CANCEL,                /* URB was not cancelled              */
  ERROR_USBH_URB_TRANSFER,              /* URB transfered with error          */
  ERROR_USBH_URB_TOUT,                  /* URB has timed-out                  */

  ERROR_USBH_ENUMERATE,                 /* Device enumeration failed          */
  ERROR_USBH_UNINIT_DEVS,               /* Device uninitialization failed     */

  ERROR_USBH_MSC,                       /* MSC Device Instance does not exist */
  ERROR_USBH_MSC_SIG,                   /* MSC Class CSW Signature not correct*/
  ERROR_USBH_MSC_TAG,                   /* MSC Class different tags           */
  ERROR_USBH_MSC_RESID,                 /* MSC Class data residue not 0       */

  ERROR_USBH_HID,                       /* HID Device Instance does not exist */

  ERROR_USBH_CLS,                       /* USB Device Class does not exist    */

  ERROR_USBH_CLS_UNKNOWN,               /* Unknown Class type                 */
} USBH_ERROR;

/* USB Host Speed constants                                                   */
enum { 
  USBH_LS  = 0,                         /* Low speed                          */
  USBH_FS,                              /* Full speed                         */
  USBH_HS                               /* High speed                         */
};

/* USB Host Packet Type constants                                             */
enum {
  USBH_PACKET_OUT      =  1,            /* OUT   token     packet             */
  USBH_PACKET_IN       =  9,            /* IN    token     packet             */
  USBH_PACKET_SOF      =  5,            /* SOF   token     packet             */
  USBH_PACKET_SETUP    = 13,            /* SETUP token     packet             */
  USBH_PACKET_DATA0    =  3,            /* DATA0 data      packet             */
  USBH_PACKET_DATA1    = 11,            /* DATA1 data      packet             */
  USBH_PACKET_DATA2    =  7,            /* DATA2 data      packet             */
  USBH_PACKET_MDATA    = 15,            /* MDATA data      packet             */
  USBH_PACKET_ACK      =  2,            /* ACK   handshake packet             */
  USBH_PACKET_NAK      = 10,            /* NAK   handshake packet             */
  USBH_PACKET_STALL    = 14,            /* STALL handshake packet             */
  USBH_PACKET_NYET     =  6,            /* NYET  handshake packet             */
  USBH_PACKET_PRE      = 12,            /* PRE   special   packet (token)     */
  USBH_PACKET_ERR      = 12,            /* ERR   special   packet (handshake) */
  USBH_PACKET_SPLIT    =  8,            /* SPLIT special   packet (token)     */
  USBH_PACKET_PING     =  4,            /* PING  special   packet (token)     */
  USBH_PACKET_RESERVED =  0             /* Reserved        packet             */
};

/* USB Host Transaction Error Type                                            */
enum {
  USBH_TR_NO_ERROR     =  0,            /* No error                           */
  USBH_TR_ERROR_BSTUFF =  1,            /* Bit stuff error                    */
  USBH_TR_ERROR_PID    =  2,            /* PID check error                    */
  USBH_TR_ERROR_CRC    =  3,            /* CRC       error                    */
  USBH_TR_ERROR_BUSTT  =  4,            /* Bus Turn-around Time error         */
  USBH_TR_ERROR_FEOP   =  4,            /* False EOP error                    */
  USBH_TR_ERROR_BABBLE =  4,            /* Babble or Loss of Activity error   */
  USBH_TR_ERROR_EOF    =  5,            /* Transaction during EOF error       */
  USBH_TR_ERROR_DOVER  =  6,            /* Data Overrun error                 */
  USBH_TR_ERROR_DUNDER =  7,            /* Data Underrun error                */
  USBH_TR_ERROR_OTHER  =  8,            /* Other errors                       */
};

/************************* Macros *********************************************/

#define PREPARE_SETUP_PACKET(urb, ptr, NAKs)                                   \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  8;                                        \
  urb.PacketType                  =  USBH_PACKET_SETUP;                        \
  urb.ToggleBit                   =  0;                                        \
  urb.ToggleForce                 =  1;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_SETUP_PACKET_DATA(ptr, dir, typ, rcpnt, req, val, idx, len)    \
  ptr->bmRequestType.Dir          =  dir;                                      \
  ptr->bmRequestType.Type         =  typ;                                      \
  ptr->bmRequestType.Recipient    =  rcpnt;                                    \
  ptr->bRequest                   =  req;                                      \
  ptr->wValue                     =  U16_LE(val);                              \
  ptr->wIndex                     =  U16_LE(idx);                              \
  ptr->wLength                    =  U16_LE(len);

#define PREPARE_IN_DATA0_PACKET(urb, ptr, len, NAKs)                           \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_IN;                           \
  urb.ToggleBit                   =  0;                                        \
  urb.ToggleForce                 =  1;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_IN_DATA1_PACKET(urb, ptr, len, NAKs)                           \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_IN;                           \
  urb.ToggleBit                   =  1;                                        \
  urb.ToggleForce                 =  1;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_IN_DATAx_PACKET(urb, ptr, len, NAKs)                           \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_IN;                           \
  urb.ToggleForce                 =  0;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_OUT_DATA0_PACKET(urb, ptr, len, NAKs)                          \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_OUT;                          \
  urb.ToggleBit                   =  0;                                        \
  urb.ToggleForce                 =  1;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_OUT_DATA1_PACKET(urb, ptr, len, NAKs)                          \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_OUT;                          \
  urb.ToggleBit                   =  1;                                        \
  urb.ToggleForce                 =  1;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_OUT_DATAx_PACKET(urb, ptr, len, NAKs)                          \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_OUT;                          \
  urb.ToggleForce                 =  0;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_PING_PACKET(urb, NAKs)                                         \
  urb.DataLength                  =  0;                                        \
  urb.PacketType                  =  USBH_PACKET_PING;                         \
  urb.ToggleForce                 =  0;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_IN_DATAx_PACKET(urb, ptr, len, NAKs)                           \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_IN;                           \
  urb.ToggleForce                 =  0;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_OUT_DATAx_PACKET(urb, ptr, len, NAKs)                          \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_OUT;                          \
  urb.ToggleForce                 =  0;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_IN_DATAx_PACKET(urb, ptr, len, NAKs)                           \
  urb.ptrDataBuffer               =  ptr;                                      \
  urb.DataLength                  =  len;                                      \
  urb.PacketType                  =  USBH_PACKET_IN;                           \
  urb.ToggleForce                 =  0;                                        \
  urb.NAKRetries                  =  NAKs;

#define PREPARE_MSC_CBW(ptr_cbw, sig, tag, len, flg, lun, cb_len, cb0, cb1, cb2, cb3, cb4, cb5, cb6, cb7, cb8, cb9, cb10, cb11, cb12, cb13, cb14, cb15)\
  ptr_cbw->dSignature             =  U32_LE(sig);                              \
  ptr_cbw->dTag                   =  U32_LE(++tag);                            \
  ptr_cbw->dDataLength            =  len;                                      \
  ptr_cbw->bmFlags                =  flg;                                      \
  ptr_cbw->bLUN                   =  lun;                                      \
  ptr_cbw->bCBLength              =  cb_len;                                   \
  ptr_cbw->CB[0]                  =  cb0;                                      \
  ptr_cbw->CB[1]                  =  cb1;                                      \
  ptr_cbw->CB[2]                  =  cb2;                                      \
  ptr_cbw->CB[3]                  =  cb3;                                      \
  ptr_cbw->CB[4]                  =  cb4;                                      \
  ptr_cbw->CB[5]                  =  cb5;                                      \
  ptr_cbw->CB[6]                  =  cb6;                                      \
  ptr_cbw->CB[7]                  =  cb7;                                      \
  ptr_cbw->CB[8]                  =  cb8;                                      \
  ptr_cbw->CB[9]                  =  cb9;                                      \
  ptr_cbw->CB[10]                 =  cb10;                                     \
  ptr_cbw->CB[11]                 =  cb11;                                     \
  ptr_cbw->CB[12]                 =  cb12;                                     \
  ptr_cbw->CB[13]                 =  cb13;                                     \
  ptr_cbw->CB[14]                 =  cb14;                                     \
  ptr_cbw->CB[15]                 =  cb15;

/************************* Structures *****************************************/

typedef volatile struct {               /* USB Request Block (URB)            */
  U8          *ptrDataBuffer;           /* Pointer to Data Buffer             */
  U8          *ptrCurrentDataBuffer;    /* Pointer to Current pos in Data Buff*/
  U32          DataLength;              /* Data Length                        */
  U32          DataTransferred;         /* Data Transferred                   */
  union {
    U32        Parameters;              /* Transfer Parameters                */
    struct {
      U32      PacketType        :  4;  /* Packet Type                        */
      U32      ToggleBit         :  1;  /* Toggle Bit Value                   */
      U32      ToggleForce       :  1;  /* Toggle Bit Forced (if cleared     
                                           ToggleBit value is ignored)        */
    };
  };
  union {
    U32        Status;                  /* Status                             */
    struct {
      U32      Submitted         :  1;  /* URB Submit Status                  */
      U32      InProgress        :  1;  /* URB Processing In Progress Status  */
      U32      Cancelled         :  1;  /* URB Cancel Status                  */
      U32      Completed         :  1;  /* URB Completition Status            */
      U32      Timeout           :  1;  /* URB Timeout Status                 */
      U32      ResponsePacketType:  4;  /* Response Packet Type               */
      U32      Reserved          :  7;  /* Reserved bits to reposition other  */
      U32      Error             :  8;  /* URB Error        Status            */
    };
  };
  U32          NAKRetries;              /* Number of NAK retries              */
  U32          TimeoutCount;            /* Transaction Timeout Counter        */
  void       (*CompletedCallback)(void);/* URB Completition Callback Function */
} USBH_URB;

typedef __packed struct {               /* Endpoint settings structure        */
  U32          Handle;                  /* Handle to Endpoint                 */
  __packed union {
    U32 Para;                           /* Endpoint parameters                */
    __packed struct {
      U8       Address           : 8;   /* Endpoint communication address     */
      U8       Speed             : 8;   /* Endpoint communication speed       */
    };
  };
  USB_ENDPOINT_DESCRIPTOR Descriptor;   /* Endpoint Descriptor                */
} USBH_EP;

typedef struct {                        /* Host Controller Driver Capabilities*/
  union {
    struct {
      U32      MultiPckt;               /* Multiple data packet handling      */
    };
  };
  U32          MaxDataSz;               /* Maximum data that can be handled   */
  U32          CtrlNAKs;                /* Number of NAKs for ctrl endpoints  */
  U32          BulkNAKs;                /* Number of NAKs for bulk endpoints  */
} USBH_HCI_CAP;

typedef struct {                        /* Host Controller Instance structure */
  USBH_HCI_CAP Cap;                     /* Host Ctrl Driver Inst Capabilities */
  U32          PortCon;                 /* Port connected map                 */
  U8           LastDevAdr;              /* Last addressed device address      */
  USBH_EP      EP0;                     /* Endpoint 0                         */
  U32          LastError;               /* Last Error                         */
} USBH_HCI;

typedef struct {                        /* Mass Storage Data Structure        */
  U8           DoPing;                  /* Do Ping on next Out Packet         */
  USBH_EP      BulkInEP;                /* Bulk In Endpoint                   */
  USBH_EP      BulkOutEP;               /* Bulk Out Endpoint                  */
  U8           MaxLUN;                  /* Maximum Logical Units              */
  U32          Tag;                     /* Tag Command/Data/Status Protocol   */
} USBH_MSC;

typedef struct {                        /* HID Data Structure                 */
  USBH_URB     IntUrb;                  /* Interrupt URB                      */
  USBH_EP      IntInEP;                 /* Interrupt In Endpoint              */
  USBH_EP      IntOutEP;                /* Interrupt Out Endpoint             */
  U8           SubClass;                /* SubClass                           */
  U8           ReportDescTyp;           /* Report Descriptor Type             */
  U16          ReportDescLen;           /* Report Descriptor Length           */
  U8           ReportInPos;             /* Report Input Data Position         */
  U8           ReportInDataBuf[8];      /* Report Input Data Buffer           */
  U8           ReportInDataBufEx[8];    /* Report Input Data Buffer Last      */
  struct {
    U8         ReportInReceived  : 1;   /* New Report In Received             */
  };
} USBH_HID;

typedef struct {                        /* Device Class Instance structure    */
  U8           Protocol;                /* Class Protocol                     */
  U8           Port;                    /* Device Address                     */
  U8           Address;                 /* Device Address                     */
  U8           Speed;                   /* Device Speed                       */ 
  struct {
    U8         Config            : 1;   /* Device Configured Status           */
    U8         Init              : 1;   /* Device Initialized Status          */
  };
  U32          LastError;               /* Last Error                         */
  union {
    USBH_MSC  *ptrMSC;                  /* Pointer to MSC Device structure    */
    USBH_HID  *ptrHID;                  /* Pointer to HID Device structure    */
    void      *ptrCLS;                  /* Pointer to USB Device Class struct */
  };
} USBH_DCI;

typedef struct {                        /* Host Controller Driver structure   */
  void       (*get_capabilities) (USBH_HCI_CAP *cap); /* Get Drv Capabilities */
  void       (*delay_ms        ) (U32  ms);   /* Delay in ms                  */
  BOOL       (*pins_config     ) (BOOL on);   /* Config/Unconfig pins         */
  BOOL       (*init            ) (BOOL on);   /* Init/Uninit Host Controller  */
  BOOL       (*port_power      ) (BOOL on);   /* On/Off Port Power            */
  BOOL       (*port_reset      ) (U8   port); /* Reset Port                   */
  U32        (*get_connect     ) (void);      /* Get port conn/disconn status */
  U32        (*get_speed       ) (void);      /* Get port enumerated speed    */
  U32        (*ep_add          ) (            U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptr_epd);
  BOOL       (*ep_config       ) (U32  hndl,  U8 dev_adr, U8 ep_spd, USB_ENDPOINT_DESCRIPTOR *ptr_epd);
  BOOL       (*ep_remove       ) (U32  hndl);
  BOOL       (*urb_submit      ) (U32  hndl,  USBH_URB *ptr_urb);
  BOOL       (*urb_cancel      ) (U32  hndl,  USBH_URB *ptr_urb);
} USBH_HCD;

typedef struct {                        /* OHCI Compliant Hw Driver structure */
  U32          Ports;                   /* Ports (bits 0..15)                 */
  U16          MaxED;                   /* Maximum Endpoint Descriptors       */
  U16          MaxTD;                   /* Maximum Transfer Descriptors       */
  U16          MaxITD;                  /* Maximum Iso Transfer Descriptors   */
  U32         *PtrHCCA;                 /* Pointer to HCCA memory start       */
  U32         *PtrED;                   /* Pointer to ED memory start         */
  U32         *PtrTD;                   /* Pointer to TD memory start         */
  U32         *PtrITD;                  /* Pointer to ITD memory start        */
  U32         *PtrTDURB;                /* Pointer to TDURB memory start      */
  void       (*get_capabilities) (USBH_HCI_CAP *cap); /* Get Drv Capabilities */
  void       (*delay_ms        ) (U32  ms);   /* Delay in ms                  */
  void       (*reg_wr          ) (U32  reg_ofs, U32 val);   /* Write register */
  U32        (*reg_rd          ) (U32  reg_ofs);            /* Read register  */
  BOOL       (*pins_config     ) (BOOL on);   /* Config/Unconfig pins         */
  BOOL       (*init            ) (BOOL on);   /* Init/Uninit Host Controller  */
  BOOL       (*port_power      ) (U32  on);   /* On/Off Port Power            */
  BOOL       (*irq_en          ) (BOOL on);   /* Enable/Disable interrupt     */
} USBH_HWD_OHCI;

typedef struct {                        /* EHCI Compliant Hw Driver structure */
  U32          Ports;                   /* Ports (bits 0..15)                 */
  U16          Max_qH;                  /* Maximum Queue Heads                */
  U16          Max_qTD;                 /* Maximum Queue Transfer Descriptors */
  U16          Max_iTD;                 /* Maximum Iso Transfer Descriptors   */
  U16          Max_siTD;                /* Maximum Split Iso Transfer Descs   */
  U16          Max_FSTN;                /* Maximum Peri Frame Span Trav Nodes */
  U32         *Ptr_PFL;                 /* Pointer to Periodic Frame List mem */
  U32         *Ptr_qH;                  /* Pointer to qH memory start         */
  U32         *Ptr_qTD;                 /* Pointer to qTD memory start        */
  U32         *Ptr_iTD;                 /* Pointer to iTD memory start        */
  U32         *Ptr_siTD;                /* Pointer to siTD memory start       */
  U32         *Ptr_FSTN;                /* Pointer to FSTN memory start       */
  U32         *Ptr_qTDURB;              /* Pointer to qTDURB memory start     */
  void       (*get_capabilities) (USBH_HCI_CAP *cap); /* Get Drv Capabilities */
  void       (*delay_ms        ) (U32  ms);   /* Delay in ms                  */
  void       (*reg_wr          ) (U32  reg_ofs, U32 val);   /* Write register */
  U32        (*reg_rd          ) (U32  reg_ofs);            /* Read register  */
  BOOL       (*pins_config     ) (BOOL on);   /* Config/Unconfig pins         */
  BOOL       (*init            ) (BOOL on);   /* Init/Uninit Host Controller  */
  BOOL       (*port_power      ) (U32  on);   /* On/Off Port Power            */
  BOOL       (*irq_en          ) (BOOL on);   /* Enable/Disable interrupt     */
} USBH_HWD_EHCI;

typedef struct {                        /* Device Class Driver structure      */
  U8           ClassID;
  U8         (*config          ) (U8 ctrl, U8 port, U8 spd, U8 adr, USB_CONFIGURATION_DESCRIPTOR *ptr_cfg_desc);
  BOOL       (*unconfig        ) (U8 ctrl, U8 dev_idx);
  BOOL       (*init            ) (U8 ctrl, U8 dev_idx);
  BOOL       (*uninit          ) (U8 ctrl, U8 dev_idx);
  U32        (*get_last_error  ) (U8 ctrl, U8 dev_idx);
} USBH_DCD;


/************************* Exported Functions *********************************/

extern BOOL USBH_Send_Setup      (U8 ctrl, U8 *ptr_data);
extern BOOL USBH_Send_Data       (U8 ctrl, U8 *ptr_data, U16 data_len);
extern BOOL USBH_Rece_Data       (U8 ctrl, U8 *ptr_data, U16 data_len);

extern BOOL USBH_GetStatus       (U8 ctrl, U8  rcpnt, U8 idx, U8 *stat_dat);
extern BOOL USBH_ClearFeature    (U8 ctrl, U8  rcpnt, U8 idx, U8  feat_sel);
extern BOOL USBH_SetFeature      (U8 ctrl, U8  rcpnt, U8 idx, U8  feat_sel);
extern BOOL USBH_SetAddress      (U8 ctrl, U8  dev_adr);
extern BOOL USBH_GetDescriptor   (U8 ctrl, U8  rcpnt, U8 desc_typ, U8 desc_idx, U8 lang_id, U8 *desc_dat, U16 desc_len);
extern BOOL USBH_SetDescriptor   (U8 ctrl, U8  rcpnt, U8 desc_typ, U8 desc_idx, U8 lang_id, U8 *desc_dat, U16 desc_len);
extern BOOL USBH_GetConfiguration(U8 ctrl, U8 *cfg_dat);
extern BOOL USBH_SetConfiguration(U8 ctrl, U8  cfg_val);
extern BOOL USBH_GetInterface    (U8 ctrl, U8  idx, U8 *alt_dat);
extern BOOL USBH_SetInterface    (U8 ctrl, U8  idx, U8  alt_set);
extern BOOL USBH_SyncFrame       (U8 ctrl, U8  idx, U8 *frm_num);


#pragma no_anon_unions

#endif  /* __USBH_H__ */
