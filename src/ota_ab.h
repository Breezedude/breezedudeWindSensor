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

// Effective maximum OTA image size, regardless of which slot is staged into.
// The bootloader can only copy a staged image from OTA_SLOT1 into OTA_SLOT0,
// so OTA_SLOT0_SIZE (the smaller of the two) is the real upper bound.
#define OTA_MAX_IMAGE_SIZE   OTA_SLOT0_SIZE
#define OTA_FLASH_PAGE_SIZE 64U
#define OTA_FLASH_ROW_SIZE  256U

#define OTA_AB_MAGIC   0x42544455UL
#define OTA_AB_VERSION 1UL
#define OTA_AB_NUM_SLOTS 2U

#ifndef OTA_BOOT_DIAG_LOGS
#define OTA_BOOT_DIAG_LOGS 0
#endif

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t active_slot;
  uint32_t pending_slot;
  uint32_t checksum;
  // Size in bytes of the staged image in OTA_SLOT1. Appended after
  // `checksum` so the checksum formula and OTA_AB_VERSION stay unchanged;
  // must match the bootloader's OTA_AB_Flags layout (inc/uf2.h).
  uint32_t staged_size;
} ota_ab_flags_t;

typedef struct {
  uint32_t sequence;
  uint8_t requested_slot;
  uint8_t requester_current_slot;
  uint8_t reset_vector_in_slot;
  uint8_t reserved;
  uint32_t slot_base;
  uint32_t slot_size;
  uint32_t stack_ptr;
  uint32_t reset_vec;
  uint32_t rcause_before_reset;
} ota_trace_snapshot_t;

uint32_t ota_ab_checksum_words(uint32_t active_slot, uint32_t pending_slot);
bool ota_ab_read(ota_ab_flags_t &cfg);
uint8_t ota_ab_current_slot();
uint8_t ota_ab_inactive_slot();
uint32_t ota_ab_slot_address(uint8_t slot);
uint32_t ota_ab_slot_size(uint8_t slot);
bool ota_ab_request_slot(uint8_t slot, uint32_t staged_size);
bool ota_ab_confirm_current();
void log_ota_boot_diagnostics();
void log_ota_reboot_trace();
void ota_ab_log_state(const char *tag);
void ota_trace_mark_pending_reboot(uint8_t requested_slot, uint32_t slot_base, uint32_t slot_size,
                                   uint32_t stack_ptr, uint32_t reset_vec, bool reset_vector_in_slot);
bool ota_trace_consume(ota_trace_snapshot_t &snapshot);
bool ota_storage_read_text(uint32_t address, char *dst, size_t dst_len);
bool ota_storage_write_text(uint32_t address, size_t capacity, const char *text);
