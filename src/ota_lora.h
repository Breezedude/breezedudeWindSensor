#pragma once
#include <Arduino.h>
#include "fanet.h"

#ifndef BREEZEDUDE_ENABLE_LORA_OTA
  #define BREEZEDUDE_ENABLE_LORA_OTA 1
#endif

#ifndef OTA_LORA_PROTOCOL_VERSION
  #define OTA_LORA_PROTOCOL_VERSION 4U
#endif
#ifndef OTA_LORA_MAX_CHUNK
  #define OTA_LORA_MAX_CHUNK 192U
#endif
// How long the station listens after its OTA HWInfo for the ground station's
// answer (FANET ACK or config update). This happens once an hour, so it is worth
// keeping short. The chain it has to cover, counted from the end of the
// station's own transmission:
//   ground station receives and forwards      ~0.1 s
//   internet up to the backend               0.1 ... 1.0 s  (cellular at the top end)
//   backend waits for all ground stations      1.0 s
//   internet back down                       0.1 ... 1.0 s
//   ground station transmits the answer      0.1 ... 0.5 s  (queue and airtime)
// Typically about 1.5 s, worst case about 3.6 s; 5 s leaves margin over the
// worst case. The previous 15 s was a value from development.
#ifndef OTA_LORA_RX_WINDOW_MS
  #define OTA_LORA_RX_WINDOW_MS 5000UL
#endif
// After the ACK the station switches to fast RX mode and waits for the START
// packet. That only happens when an update really begins - rarely, and with the
// watchdog paused - so it keeps the generous value: shortening it saves nothing
// measurable and risks aborting a real update.
#ifndef OTA_LORA_START_WAIT_MS
  #define OTA_LORA_START_WAIT_MS 15000UL
#endif
#ifndef OTA_LORA_SESSION_TIMEOUT_MS
  #define OTA_LORA_SESSION_TIMEOUT_MS 900000UL
#endif
#ifndef OTA_LORA_IDLE_DEBUG_MS
  #define OTA_LORA_IDLE_DEBUG_MS 350UL
#endif
// Simplified non-verbose USB progress summary: re-print the current
// percentage at least this often, so a stalled transfer still gives
// periodic feedback even without new chunks arriving.
#ifndef OTA_LORA_SUMMARY_INTERVAL_MS
  #define OTA_LORA_SUMMARY_INTERVAL_MS 5000UL
#endif
// Proactive ack-resend: if the next chunk hasn't arrived within this many ms
// since the last accepted chunk, re-send the ack for next_seq (in case our
// previous ack got lost), up to OTA_LORA_PROACTIVE_ACK_MAX times. Spaced so
// all resends happen well within the GS's ack-timeout window, avoiding a
// full chunk retransmit.
#ifndef OTA_LORA_PROACTIVE_ACK_INTERVAL_MS
  #define OTA_LORA_PROACTIVE_ACK_INTERVAL_MS 1000UL
#endif
#ifndef OTA_LORA_PROACTIVE_ACK_MAX
  #define OTA_LORA_PROACTIVE_ACK_MAX 3U
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
// True while the station only waits for an answer to its OTA HWInfo - not during
// a running update, where the watchdog is paused and timing matters.
bool ota_lora_offer_waiting();
// Milliseconds left in that wait, 0 once the window has closed.
uint32_t ota_lora_offer_ms_left();
