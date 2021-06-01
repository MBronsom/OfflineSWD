/*------------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *------------------------------------------------------------------------------
 *      Name:    fs_usbh_msc.c 
 *      Purpose: USB Host Mass Storage Class Interface for the FlashFS
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <rl_usb.h>

/*------------------------------------------------------------------------------
  USB-MSC FAT Driver instance definition
   usb0_drv: First USB Flash drive  [U0:]
   usb1_drv: Second USB Flash drive [U1:]
 *----------------------------------------------------------------------------*/

#define __DRV_ID0 usb0_drv
#define __DRV_ID1 usb1_drv
#define __CPUCLK  120000000

/* USB-MSC Driver Interface functions */
static BOOL Init         (U8 ctrl, U32 mode);
static BOOL UnInit       (U8 ctrl, U32 mode);
static BOOL ReadSector   (U8 ctrl, U32 sect, U8 *buf, U32 cnt);
static BOOL WriteSector  (U8 ctrl, U32 sect, U8 *buf, U32 cnt);
static BOOL ReadInfo     (U8 ctrl, Media_INFO *info);
static U32  DeviceCtrl   (U8 ctrl, U32 code, void *p);

static BOOL Init0        (U32 mode)                   { return (Init        (0, mode)); }
static BOOL UnInit0      (U32 mode)                   { return (UnInit      (0, mode)); }
static BOOL ReadSector0  (U32 sect, U8 *buf, U32 cnt) { return (ReadSector  (0, sect, buf, cnt)); }
static BOOL WriteSector0 (U32 sect, U8 *buf, U32 cnt) { return (WriteSector (0, sect, buf, cnt)); }
static BOOL ReadInfo0    (Media_INFO *info)           { return (ReadInfo    (0, info)); }
static U32  DeviceCtrl0  (U32 code, void *p)          { return (DeviceCtrl  (0, code, p)); }

static BOOL Init1        (U32 mode)                   { return (Init        (1, mode)); }
static BOOL UnInit1      (U32 mode)                   { return (UnInit      (1, mode)); }
static BOOL ReadSector1  (U32 sect, U8 *buf, U32 cnt) { return (ReadSector  (1, sect, buf, cnt)); }
static BOOL WriteSector1 (U32 sect, U8 *buf, U32 cnt) { return (WriteSector (1, sect, buf, cnt)); }
static BOOL ReadInfo1    (Media_INFO *info)           { return (ReadInfo    (1, info)); }
static U32  DeviceCtrl1  (U32 code, void *p)          { return (DeviceCtrl  (1, code, p)); }

/* USB-MSC Device Driver Control Block */
FAT_DRV __DRV_ID0 = {
  Init0,
  UnInit0,
  ReadSector0,
  WriteSector0,
  ReadInfo0,
  DeviceCtrl0
};

FAT_DRV __DRV_ID1 = {
  Init1,
  UnInit1,
  ReadSector1,
  WriteSector1,
  ReadInfo1,
  DeviceCtrl1
};


/* Local definitions */
#define WAIT_CNT(ck,us,div) (((ck/3000000)*us)/div)

/* Local variables */
static BIT media_ok[2];

/* Local Functions */
static void Delay (U32 us);

/*--------------------------- Init -------------------------------------------*/

static BOOL Init (U8 ctrl, U32 mode) {
  /* Initialize USB Host. */
  U32 cnt;

  if (mode == DM_IO) {
    /* Initialise USB hardware. */
    media_ok[ctrl] = __FALSE;
    return (usbh_init(ctrl));
  }

  if (mode == DM_MEDIA) {
    for (cnt = 0; cnt < 2500; cnt++) {
      usbh_engine(ctrl);
      if (usbh_msc_status (ctrl, 0) == __TRUE) {
        media_ok[ctrl] = __TRUE;
        return (__TRUE);
      }
      Delay (1000);
    }
  }
  return (__FALSE);
}


/*--------------------------- UnInit -----------------------------------------*/

static BOOL UnInit (U8 ctrl, U32 mode) {
  /* UnInitialize USB Host. */

  if (mode == DM_IO) {
    /* UnInitialize USB hardware. */
    return (usbh_uninit(ctrl));
  }
  if (mode == DM_MEDIA) {
    return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- ReadSector -------------------------------------*/

static BOOL ReadSector (U8 ctrl, U32 sect, U8 *buf, U32 cnt) {
  /* Read single/multiple sectors from Mass Storage Device. */

  return (usbh_msc_read(ctrl, 0, sect, buf, cnt));
}


/*--------------------------- WriteSector ------------------------------------*/

static BOOL WriteSector (U8 ctrl, U32 sect, U8 *buf, U32 cnt) {
  /* Write single/multiple sectors to Mass Storage Device. */

  return (usbh_msc_write(ctrl, 0, sect, buf, cnt));
}


/*--------------------------- ReadInfo ---------------------------------------*/

static BOOL ReadInfo (U8 ctrl, Media_INFO *info) {
  /* Read Mass Storage Device configuration. */
  U32 blen;

  if (!usbh_msc_read_config(ctrl, 0, &info->block_cnt, &blen)) { 
    /* Fail, Mass Storage Device configuration was not read. */
    return (__FALSE);
  }
  info->write_blen = info->read_blen = (U16)blen;
  return (__TRUE);
}


/*--------------------------- DeviceCtrl -------------------------------------*/

static U32 DeviceCtrl (U8 ctrl, U32 code, void *p) {
  /* Device Control system call. */

  if (code != DC_CHKMEDIA) {
    return (0);
  }
  /* Read Device Detected status. */
  if (media_ok[ctrl] == __FALSE) {
    /* Allow to initialize the media first. */
    return (M_INSERTED);
  }
  /* Allow USB Host to detect and enumerate the device. */
  usbh_engine(ctrl);
  if (usbh_msc_status(ctrl, 0) == __TRUE) {
    return (M_INSERTED);
  }
  return (0);
}


/*--------------------------- Delay ------------------------------------------*/

static void Delay (U32 us) {
  /* Approximate delay in micro seconds. */
  U32 i;

  i = WAIT_CNT(__CPUCLK, us, 12);
  while (i--);
}

/*------------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
