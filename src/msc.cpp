#include "msc.h"
#include "logging.h"
#include "tools.h"

Adafruit_USBD_MSC usb_msc;

InternalFlash my_internal_storage;
Adafruit_FlashTransport_InternalFlash flashTransport(&my_internal_storage);
Adafruit_InternalFlash_Wrapper flash(&flashTransport);
FatFileSystem fatfs;

// Flash ----------------------------------------------------------------------------------------------------------------------
#define DISK_LABEL  "Breezedude"
FATFS elmchamFatfs;

DSTATUS disk_status(BYTE pdrv){
  (void) pdrv;
	return 0;
}

DSTATUS disk_initialize(BYTE pdrv){
  (void) pdrv;
	return 0;
}

DRESULT disk_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count){
  (void) pdrv;
	return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count){
  (void) pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl (BYTE pdrv,BYTE cmd,	void *buff){
  (void) pdrv;
  switch ( cmd ){
    case CTRL_SYNC:
      flash.syncBlocks();
      return RES_OK;
    case GET_SECTOR_COUNT:
      *((DWORD*) buff) = flash.size()/512;
      return RES_OK;
    case GET_SECTOR_SIZE:
      *((WORD*) buff) = 512;
      return RES_OK;
    case GET_BLOCK_SIZE:
      *((DWORD*) buff) = 8;    // erase block size in units of sector size
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

bool format_flash(){
  //return false;
  uint8_t workbuf[1024]; // Working buffer for f_fdisk function. 4096->1024, maybe we run out of heap?
  FRESULT r = f_mkfs12("", FM_FAT, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
     log_i("Error, f_mkfs failed with error code: ", r);
    return false;
  }
  r = f_mount(&elmchamFatfs, "0:", 1);   // mount to set disk label
  if (r != FR_OK) {
     //log_i("Error, f_mount failed with error code: ", r);
    return false;
  }
   // log_i("Setting disk label to: "); log_i(DISK_LABEL); log_i("\r\n");
  r = f_setlabel(DISK_LABEL);
  if (r != FR_OK) {
     //log_i("Error, f_setlabel failed with error code: ", r);
    return false;
  }
  f_unmount("0:"); // unmount
  flash.syncBlocks(); // sync to make sure all data is written to flash
  return true;
}

int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize){
  //Serial.printf("Reading at %d with size %d\n",lba,bufsize);
  my_internal_storage.read(lba*DISK_BLOCK_SIZE, buffer, bufsize);
  return bufsize;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize){
  //Serial.printf("Writing at %d with size %d\n",lba,bufsize);
  // Erase should be done before every writing to the flash
  my_internal_storage.erase(lba*DISK_BLOCK_SIZE, bufsize);
  // Write to the flash
  my_internal_storage.write(lba*DISK_BLOCK_SIZE, buffer, bufsize);
  return bufsize;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache / sync with flash
void msc_flush_cb (void){
  my_internal_storage.flush_buffer();
}
// Invoked to check if device is writable as part of SCSI WRITE10
// Default mode is writable
bool msc_writable_callback(void){
  // true for writable, false for read-only
  if(!usb_connected){
    log_i("USB Connected: ", time());
  }
  usb_connected = true;
  return true;
}

void setup_usb_msc(){
  //log_i("USB MSC Setup\n");
    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("BrezeDu", "Mass Storage", "1.0");
   
  // Set disk size
  uint32_t DISK_BLOCK_NUM = my_internal_storage.get_flash_size()/DISK_BLOCK_SIZE;
  //log_i("DISK_BLOCK_NUM: ", DISK_BLOCK_NUM);
  usb_msc.setCapacity(DISK_BLOCK_NUM-1, DISK_BLOCK_SIZE);

  // Set callbacks
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setWritableCallback(msc_writable_callback);

  // Set Lun ready (internal flash is always ready)
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  Serial.begin(115200); // USB Serial
  //while ( !Serial ) delay(10);   // wait for native usb
  delay(10);
}

bool setup_flash(){
  setup_usb_msc();
    //if(debug_enabled()){printf("Internal flash with address %p and size %ld\r\n", my_internal_storage.get_flash_address(), my_internal_storage.get_flash_size());}
  if (flash.begin()){
    //log_i("Internal flash successfully set up\r\n");
  }else{
    log_e("Error: failed to set up the internal flash\r\n");
  }

  // The file system object from SdFat to read/write to the files in the internal flash
  int count = 0;
  while ( !fatfs.begin(&flash) ){

    if( (count == 0) && !format_flash()){
      log_e("Error: failed to FAT format flash\r\n");
      return false;
    }
    if(count == 1) {
      log_e("Error: file system not existing. failed to format. The internal flash drive should first be formated with Windows or fdisk on Linux\r\n");
      return false;
    }
    count ++;
  }

#if 0
  DEBUGSER.print("Clusters:          ");
  DEBUGSER.println(fatfs.clusterCount());
  DEBUGSER.print("Blocks x Cluster:  ");
  DEBUGSER.println(fatfs.sectorsPerCluster());
  DEBUGSER.print("Total Blocks:      ");
  DEBUGSER.println(fatfs.sectorsPerCluster() * fatfs.clusterCount());
  
  DEBUGSER.print("Volume type is: FAT");
  DEBUGSER.println(fatfs.fatType(), DEC);
  uint32_t volumesize;
  volumesize = fatfs.sectorsPerCluster();
  volumesize *= fatfs.clusterCount();

  volumesize /= 2;
  DEBUGSER.print("Volume size (Kb):  ");
  DEBUGSER.println(volumesize);

  volumesize = 0;
  volumesize = fatfs.sectorsPerCluster();    // clusters are collections of blocks
  volumesize *= fatfs.freeClusterCount();       // we'll have a lot of clusters

  volumesize /= 2;                           // SD card blocks are always 512 bytes 
  DEBUGSER.print("Free space size (Kb):  ");
  DEBUGSER.println(volumesize);

  //DEBUGSER.println("\nFiles found on the card (name, date and size in bytes): ");
  //fatfs.rootDirStart();
  //// list all files in the card with date and size
  //fatfs.ls(LS_R | LS_DATE | LS_SIZE);
#endif
  return true; 
}
