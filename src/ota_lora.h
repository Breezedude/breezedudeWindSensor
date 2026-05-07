#pragma once
#include <Arduino.h>
#include "fanet.h"

#ifndef BREEZEDUDE_ENABLE_LORA_OTA
  #define BREEZEDUDE_ENABLE_LORA_OTA 1
#endif

#ifndef OTA_LORA_PROTOCOL_VERSION
  #define OTA_LORA_PROTOCOL_VERSION 3U
#endif
#ifndef OTA_LORA_MAX_CHUNK
  #define OTA_LORA_MAX_CHUNK 192U
#endif
#ifndef OTA_LORA_RX_WINDOW_MS
  #define OTA_LORA_RX_WINDOW_MS 15000UL
#endif
#ifndef OTA_LORA_SESSION_TIMEOUT_MS
  #define OTA_LORA_SESSION_TIMEOUT_MS 900000UL
#endif
#ifndef OTA_LORA_IDLE_DEBUG_MS
  #define OTA_LORA_IDLE_DEBUG_MS 350UL
#endif
#ifndef OTA_LORA_NO_UPDATE_COOLDOWN_MS
  #define OTA_LORA_NO_UPDATE_COOLDOWN_MS 1000*3600UL
#endif
#ifndef OTA_LORA_FAST_BW_KHZ
  #define OTA_LORA_FAST_BW_KHZ 500U
#endif
#ifndef OTA_LORA_FAST_SF
  #define OTA_LORA_FAST_SF 7U
#endif
#ifndef OTA_LORA_FAST_CR
  #define OTA_LORA_FAST_CR 5U
#endif
#ifndef OTA_LORA_FAST_PREAMBLE
  #define OTA_LORA_FAST_PREAMBLE 6U
#endif
#ifndef OTA_LORA_FAST_SYNCWORD
  #define OTA_LORA_FAST_SYNCWORD 0x12U
#endif

void ota_lora_begin();
void ota_lora_prepare_hwinfo(hwInfoData &info);
void ota_lora_note_hwinfo_tx_started();
uint32_t ota_lora_next_hwinfo_due_ms();
void ota_lora_defer_hwinfo_retry(uint32_t delay_ms);
bool ota_lora_on_tx_complete();
bool ota_lora_poll();
bool ota_lora_busy();
