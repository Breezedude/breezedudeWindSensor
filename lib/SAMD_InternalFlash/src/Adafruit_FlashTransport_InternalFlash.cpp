/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 hathach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Adafruit_FlashTransport_InternalFlash.h"
#include "SAMD_InternalFlash.h"

Adafruit_FlashTransport_InternalFlash::Adafruit_FlashTransport_InternalFlash(InternalFlash *flash):_flash{flash}
{
  //Serial.println("Adafruit_FlashTransport_InternalFlash::Adafruit_FlashTransport_InternalFlash");
  _cmd_read = SFLASH_CMD_READ;
  _addr_len = 3; // work with most device if not set

  //_partition = NULL;
  memset(&_flash_device, 0, sizeof(_flash_device));
}

bool Adafruit_FlashTransport_InternalFlash::supportQuadMode(void) 
{
  //Serial.println("Adafruit_FlashTransport_InternalFlash::supportQuadMode");
  return false;
}

void Adafruit_FlashTransport_InternalFlash::begin(void)
{
  //Serial.println("Adafruit_FlashTransport_InternalFlash::begin");
  // Internal flash is always available
}

void Adafruit_FlashTransport_InternalFlash::end(void)
{
  //Serial.println("Adafruit_FlashTransport_InternalFlash::end");
  // Internal flash is always available
}

SPIFlash_Device_t *Adafruit_FlashTransport_InternalFlash::getFlashDevice(void) {
  //Serial.println("Adafruit_FlashTransport_InternalFlash::getFlashDevice");
  if (!_flash)
    return NULL;

  _flash_device.total_size = _flash->get_flash_size();
  _flash_device.is_fram = false;

  _flash_device.manufacturer_id = 0x01;
  _flash_device.memory_type = 0x01;
  _flash_device.capacity = _flash->get_flash_size();

  return &_flash_device;
}

void Adafruit_FlashTransport_InternalFlash::setClockSpeed(uint32_t write_hz,
                                                  uint32_t read_hz) 
{
  //Serial.println("Adafruit_FlashTransport_InternalFlash::setClockSpeed");
  // do nothing, just use current configured clock
}

bool Adafruit_FlashTransport_InternalFlash::runCommand(uint8_t command) 
{
  //Serial.printf("Adafruit_FlashTransport_InternalFlash::runCommand %d",command);
  //Serial.println();
  // TODO maybe SFLASH_CMD_ERASE_CHIP should erase whole partition
  // do nothing, mostly write enable
  return true;
}

bool Adafruit_FlashTransport_InternalFlash::readCommand(uint8_t command,
                                                uint8_t *response,
                                                uint32_t len) 
{
  //Serial.printf("Adafruit_FlashTransport_InternalFlash::readCommand %d",command);
  //Serial.println();
  // mostly is Read STATUS, just fill with 0x0
  memset(response, 0, len);

  return true;
}

bool Adafruit_FlashTransport_InternalFlash::writeCommand(uint8_t command,
                                                 uint8_t const *data,
                                                 uint32_t len)
{
  //Serial.printf("Adafruit_FlashTransport_InternalFlash::writeCommand %d",command);
  //Serial.println();
  //  do nothing, mostly is Write Status
  return true;
}

bool Adafruit_FlashTransport_InternalFlash::eraseCommand(uint8_t command,
                                                 uint32_t addr) 
{
  uint32_t erase_sz = (command == SFLASH_CMD_ERASE_BLOCK) ? SFLASH_BLOCK_SIZE
                                                          : SFLASH_SECTOR_SIZE;
  _flash->erase(addr,erase_sz);
  return true;
}

bool Adafruit_FlashTransport_InternalFlash::readMemory(uint32_t addr, uint8_t *data,
                                                       uint32_t len) 
{
  //Serial.printf("Adafruit_FlashTransport_InternalFlash::readMemory with addr %d and length %d",addr,len);
  //Serial.println();
  _flash->read(addr, data, len);
  return true;
}

bool Adafruit_FlashTransport_InternalFlash::writeMemory(uint32_t addr,
                                                uint8_t const *data,
                                                uint32_t len) 
{
  //Serial.printf("Adafruit_FlashTransport_InternalFlash::writeMemory with addr %d and length %d",addr,len);
  //Serial.println();
  // Internal flash requires erase before write (not only for MSC callback path).
  // For SAMD51 this is a no-op in InternalFlash::erase(), for SAMD21 it is essential.
  _flash->erase(addr, len);
  _flash->write(addr, data, len);
  return true;
}

