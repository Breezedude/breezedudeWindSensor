#pragma once
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <SdFat.h>
#include <SAMD_InternalFlash.h>
#include "ff.h"
#include "diskio.h"



DSTATUS disk_status(BYTE pdrv);
DSTATUS disk_initialize(BYTE pdrv);
DRESULT disk_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT disk_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT disk_ioctl (BYTE pdrv,BYTE cmd,	void *buff);
bool format_flash();

// Function prototypes
bool setup_flash();
void setup_usb_msc();
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize);
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize);
void msc_flush_cb (void);
bool msc_writable_callback(void);

extern Adafruit_USBD_MSC usb_msc;
#define DISK_BLOCK_SIZE 512 // Block size in bytes for tinyUSB flash drive. Should be always 512

extern InternalFlash my_internal_storage;
extern Adafruit_FlashTransport_InternalFlash flashTransport;
extern Adafruit_InternalFlash_Wrapper flash;
extern FatFileSystem fatfs;

