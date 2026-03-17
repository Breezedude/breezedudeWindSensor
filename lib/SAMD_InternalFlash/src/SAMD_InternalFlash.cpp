/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Written by Cristian Maglie

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "SAMD_InternalFlash.h"


// Address of the end of the sketch in the internal flash
extern "C" {
extern uint32_t __etext; // CODE END. Symbol exported from linker script
}

#define SAMD_FLASH_PAGE_SIZE (8 << NVMCTRL->PARAM.bit.PSZ) // 4096
#define FLASH_NUM_PAGES NVMCTRL->PARAM.bit.NVMP // 64
#define SAMD_FLASH_SIZE (SAMD_FLASH_PAGE_SIZE * FLASH_NUM_PAGES) // 262144
#define FLASH_BLOCK_SIZE (SAMD_FLASH_PAGE_SIZE * 16) //65536 0x10000


InternalFlash::InternalFlash(){
#define INTERNAL_FLASH_FILESYSTEM_SIZE        (80*1024)
#define INTERNAL_FLASH_FILESYSTEM_START_ADDR  (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)

  _flash_address = (uint8_t *) INTERNAL_FLASH_FILESYSTEM_START_ADDR ; // (uint8_t *)&__etext; // OK to overwrite the '0' there // 0x0002f800 = 0x00021800 + E0000, size 67584 = 0x10800
 // Use fixed address and size
 // uint16_t partialBlock = (uint32_t)_flash_address % FLASH_BLOCK_SIZE;
 // if (partialBlock) {
 //   _flash_address += FLASH_BLOCK_SIZE - partialBlock;
 // }
  // Move ahead one block. This shouldn't be necessary, but for
  // some reason certain programs are clobbering themselves.
  //_flash_address += FLASH_BLOCK_SIZE;
  _flash_size = INTERNAL_FLASH_FILESYSTEM_SIZE; //SAMD_FLASH_SIZE-(int)_flash_address;
}

void InternalFlash::write(uint32_t offset, const void *data, uint32_t size)
{
#if defined(__SAMD51__)
  // _flash_address+offset should be a multiple of 8192 for __SAMD51__
  uint32_t new_buff_addr=offset-(offset%8192);
  // If the buffer is in used and the addr is different, we flush the buffer to use it with the new addr
  if (_buff_in_used==true && new_buff_addr!=_buff_addr)
    flush_buffer();
  
  _buff_addr=new_buff_addr;
  // Initialise the buffer
  if (_buff_in_used==false)
  {
    memcpy((void *)(_buff), (const void *)(_flash_address+_buff_addr), 8192); 
    _buff_in_used=true;
  }
  // Write the data to the buffer
  uint32_t buff_offset=offset%8192;
  memcpy((void *)(_buff+buff_offset), (const void *)data, size);
  //Serial.printf("InternalFlash::write to buffer at offset %d (base addr: %d) with size %d",
  //								offset, _buff_addr, size);
  //Serial.println();
#else
  fl.write(_flash_address+offset, data, size);
  //Serial.printf("InternalFlash::write to buffer at offset %d with size %d", offset, size);
  //Serial.println();
#endif
}

void InternalFlash::erase(uint32_t offset, uint32_t size)
{
#if defined(__SAMD51__)
  // Do nothing for the __SAMD51__. Erase is done when the buffer is flushed
#else
  //volatile void *flash_address_offset = (volatile uint32_t *)(_flash_address+offset);
  fl.erase(_flash_address+offset, size);
#endif
}


void InternalFlash::read(uint32_t offset, void *data, uint32_t size)
{
  //Serial.printf("InternalFlash::read at offset %d and with size %d",offset,size);
  //Serial.println();
#if defined(__SAMD51__)
  // Check if the data which is read is located in the buffer
  uint32_t new_buff_addr=offset-(offset%8192);
  if (_buff_in_used==true && new_buff_addr==_buff_addr)
  {
    // Assume that buff_offset + size < sizeof(_buff)
    uint32_t buff_offset=offset%8192;
    memcpy(data,(void *)(_buff+buff_offset), size);
  }
  else
  {
    fl.read(_flash_address+offset, data, size);
  }
#else
  fl.read(_flash_address+offset, data, size);
#endif
}

void InternalFlash::flush_buffer()
{
// This is specific to __SAMD51__ since the writing has to be 8192 bytes long
#if defined(__SAMD51__)
  // If nothing in the buffer, we quite
  if (_buff_in_used==false)
    return;

  //Serial.printf("InternalFlash::flush_buffer");
  //Serial.println();

  // Erase first
  fl.erase(_flash_address+_buff_addr, 8192);

  // Write the buffer
  //Serial.printf("InternalFlash::write at offset %d and with size %d", _buff_addr, 8192);
  //Serial.println();
  fl.write(_flash_address+_buff_addr, (const void *)_buff, 8192);
  // Indicate the buffer is empty
  _buff_in_used=false;
#endif
}
