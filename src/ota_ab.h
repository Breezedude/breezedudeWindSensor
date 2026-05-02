#pragma once
#include <Arduino.h>

#define OTA_SLOT0_START      0x00004000UL
#define OTA_SLOT1_START      0x00020000UL
#define OTA_FLASH_END        0x0003E000UL
#define OTA_SETTINGS_ADDRESS 0x0003E000UL
#define OTA_SETTINGS_SIZE    0x00000200UL
#define OTA_VERSIONS_ADDRESS 0x0003E200UL
#define OTA_VERSIONS_SIZE    0x00000200UL
#define OTA_FLAG_ADDRESS     0x0003EF00UL
#define OTA_SLOT0_SIZE       (OTA_SLOT1_START - OTA_SLOT0_START)
#define OTA_SLOT1_SIZE       (OTA_FLASH_END - OTA_SLOT1_START)
#define OTA_FLASH_PAGE_SIZE 64U
#define OTA_FLASH_ROW_SIZE  256U

#define OTA_AB_MAGIC   0x42544455UL
#define OTA_AB_VERSION 1UL
#define OTA_AB_NUM_SLOTS 2U

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t active_slot;
  uint32_t pending_slot;
  uint32_t checksum;
} ota_ab_flags_t;

uint32_t ota_ab_checksum_words(uint32_t active_slot, uint32_t pending_slot);
bool ota_ab_read(ota_ab_flags_t &cfg);
uint8_t ota_ab_current_slot();
uint8_t ota_ab_inactive_slot();
uint32_t ota_ab_slot_address(uint8_t slot);
uint32_t ota_ab_slot_size(uint8_t slot);
bool ota_ab_request_slot(uint8_t slot);
bool ota_ab_confirm_current();
bool ota_storage_read_text(uint32_t address, char *dst, size_t dst_len);
bool ota_storage_write_text(uint32_t address, size_t capacity, const char *text);
