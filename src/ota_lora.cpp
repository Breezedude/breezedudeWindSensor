#include "ota_lora.h"
#include "ota_ab.h"
#include "ota_sha256.h"
#include "config_update.h"
#include "radio.h"
#include "logging.h"
#include "sensors.h"
#include "tools.h"
#include "sleep.h"
#include <ArduinoUniqueID.h>
#include <uECC.h>

#if BREEZEDUDE_ENABLE_LORA_OTA

static constexpr uint8_t FANET_ACK_TYPE = 0x00u;
static constexpr uint8_t OTA_MAGIC0 = 'O';
static constexpr uint8_t OTA_MAGIC1 = 'T';

enum : uint8_t {
  OTA_OP_START  = 0x01u,
  OTA_OP_CHUNK  = 0x02u,
  OTA_OP_FINISH = 0x03u,
  OTA_OP_ACK    = 0x04u,
  OTA_OP_ABORT  = 0x05u,
  OTA_OP_CONFIG = 0x06u,  // Config update packet (higher priority than OTA)
};

enum : uint8_t {
  OTA_STATE_IDLE      = 0x00u,
  OTA_STATE_WAIT      = 0x01u,
  OTA_STATE_RX        = 0x02u,
  OTA_STATE_READY     = 0x03u,
  OTA_ERR_SIZE        = 0x82u,
  OTA_ERR_FLASH       = 0x83u,
  OTA_ERR_SEQ         = 0x84u,
  OTA_ERR_CRC         = 0x85u,
  OTA_ERR_IMAGE       = 0x86u,
  OTA_ERR_TIMEOUT     = 0x87u,
  OTA_ERR_RADIO       = 0x88u,
  OTA_ERR_SIGNATURE   = 0x89u,
};

typedef struct {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t proto;
  uint8_t op;
} __attribute__((packed)) ota_pkt_prefix_t;

typedef struct {
  ota_pkt_prefix_t prefix;
  uint32_t nonce;
  uint16_t target_version_bcd;
  uint16_t chunk_size;
  uint32_t image_size;
  uint32_t image_crc32;
  uint32_t expires;
  uint8_t image_sha256[32];
  uint8_t signature_raw[64];
} __attribute__((packed)) ota_offer_pkt_t;

typedef struct {
  ota_pkt_prefix_t prefix;
  uint16_t seq;
  uint16_t data_len;
  uint8_t payload[OTA_LORA_MAX_CHUNK];
} __attribute__((packed)) ota_chunk_pkt_t;

typedef struct {
  ota_pkt_prefix_t prefix;
  uint32_t nonce;
  uint32_t image_size;
  uint32_t image_crc32;
} __attribute__((packed)) ota_finish_pkt_t;

typedef struct {
  ota_pkt_prefix_t prefix;
  uint32_t nonce;
  uint8_t status;
  uint8_t reserved[3];
} __attribute__((packed)) ota_abort_pkt_t;

typedef struct {
  ota_pkt_prefix_t prefix;
  uint32_t nonce;
  uint16_t next_seq;
  uint8_t status;
  uint8_t accepted_chunk;
} __attribute__((packed)) ota_ack_pkt_t;

struct ota_session_t {
  bool offer_pending = false;
  bool active = false;
  bool fast_mode = false;
  uint8_t status = OTA_STATE_IDLE;
  uint8_t inactive_slot = 0;
  uint32_t nonce = 0;
  uint16_t current_version_bcd = 0;
  uint16_t target_version_bcd = 0;
  uint16_t chunk_size = 0;
  uint16_t next_seq = 0;
  uint32_t image_size = 0;
  uint32_t image_crc32 = 0;
  uint32_t expires = 0;
  uint32_t slot_base = 0;
  uint32_t slot_size = 0;
  uint32_t write_addr = 0;
  uint32_t bytes_written = 0;
  uint32_t deadline = 0;
  uint32_t crc32 = 0xFFFFFFFFu;
  uint8_t expected_sha256[32] = {0};
  ota_sha256_ctx_t sha256 = {};
  uint8_t page_buf[OTA_FLASH_ROW_SIZE * 2u] = {0xFF};
  uint16_t page_fill = 0;
  uint32_t session_start_ms = 0;
  uint32_t last_chunk_ms = 0;
  uint32_t last_wait_log_ms = 0;
  uint8_t last_progress_pct = 0;
};

static ota_session_t s_ota = {};
static bool s_ota_wdt_paused = false;
static uint32_t s_ota_chunk_led_until_ms = 0;
static uint32_t s_ota_loss_led_until_ms = 0;
static uint32_t s_ota_no_update_cooldown_until = 0;
static bool s_last_hwinfo_was_ota = false;
static uint32_t s_hwinfo_last_dynamic_ts = 0;
static uint32_t s_hwinfo_last_ota_ts = 0;
static uint32_t s_hwinfo_last_static_ts = 0;
static uint32_t s_hwinfo_retry_not_before_ms = 0;

static constexpr uint32_t OTA_HWINFO_DYNAMIC_INTERVAL_S = 10u * 60u;
static constexpr uint32_t OTA_HWINFO_DYNAMIC_LOW_BATT_INTERVAL_S = 30u * 60u;
static constexpr uint32_t OTA_HWINFO_OTA_INTERVAL_S = 60u * 60u; // 1h
static constexpr uint32_t OTA_HWINFO_STATIC_INTERVAL_S = 60u * 60u;

// Boot delays: first hwinfo sends are staggered after reset
static constexpr uint32_t OTA_HWINFO_BOOT_DELAY_DYNAMIC_MS = 1u * 60u * 1000u;   // 1 min
static constexpr uint32_t OTA_HWINFO_BOOT_DELAY_OTA_MS     = 10u * 60u * 1000u;  // 10 min
static constexpr uint32_t OTA_HWINFO_BOOT_DELAY_OTA_FAST_MS = 20u * 1000u;       // 20 sec (OTA_FAST)
static constexpr uint32_t OTA_HWINFO_BOOT_DELAY_STATIC_MS  = 10u * 60u * 1000u;  // 10 min

static uint16_t ota_version_bcd() {
  uint8_t digits[3] = {0, 0, 0};
  uint8_t idx = 0;
  for(const char *p = VERSION; *p && idx < 3; ++p) {
    if(*p >= '0' && *p <= '9') {
      digits[idx++] = (uint8_t)(*p - '0');
    }
  }
  return (uint16_t)(((digits[0] & 0x0Fu) << 8) | ((digits[1] & 0x0Fu) << 4) | (digits[2] & 0x0Fu));
}

static bool ota_no_update_cooldown_active() {
  return s_ota_no_update_cooldown_until && ((int32_t)(time() - s_ota_no_update_cooldown_until) < 0);
}

static bool ota_hwinfo_due(uint32_t now_ms, uint32_t last_ms, uint32_t interval_ms) {
  if(last_ms == 0u) {
    return true;
  }
  return (uint32_t)(now_ms - last_ms) >= interval_ms;
}

static uint32_t ota_hwinfo_due_in_ms(uint32_t now_ms, uint32_t last_ms, uint32_t interval_ms) {
  if(last_ms == 0u) {
    return 0u;
  }
  const uint32_t elapsed = (uint32_t)(now_ms - last_ms);
  if(elapsed >= interval_ms) {
    return 0u;
  }
  return interval_ms - elapsed;
}

static bool ota_hwinfo_low_battery() {
  return settings.reduce_interval_voltage > 0.1f &&
         sensor.batt_volt > 0.5f &&
         sensor.batt_volt <= settings.reduce_interval_voltage;
}

static void ota_start_no_update_cooldown() {
  s_ota_no_update_cooldown_until = time() + OTA_LORA_NO_UPDATE_COOLDOWN_MS;
  log_write("OTA no update cooldown started for ms=");
  log_write((uint32_t)OTA_LORA_NO_UPDATE_COOLDOWN_MS);
  log_write("\r\n");
}

static uint32_t ota_next_nonce() {
  uint32_t x = s_ota.nonce ^ (uint32_t)millis() ^ (uint32_t)time() ^ (uint32_t)LAST_BUILD_TIME;
  for(size_t i = 0; i < UniqueIDsize; ++i) {
    x ^= ((uint32_t)UniqueID[i]) << ((i & 3u) * 8u);
    x = (x << 5) | (x >> 27);
    x ^= 0x9E3779B9u;
  }
  if(x == 0u) {
    x = 0x13572468u;
  }
  s_ota.nonce = x;
  return x;
}

// Backend public key for OTA and config update signature verification
// ECDSA P-256 public key (64 bytes: 32-byte X coordinate + 32-byte Y coordinate)
extern const uint8_t kBackendPublicKey[64] = {
  0xF7, 0x58, 0x05, 0x9F, 0xB9, 0x17, 0x36, 0xBE,
  0x49, 0xD8, 0xF8, 0xAE, 0x3E, 0xD1, 0x62, 0xEA,
  0xD1, 0xAA, 0x54, 0xC7, 0x4D, 0x41, 0x09, 0x53,
  0xFC, 0x51, 0xEC, 0x36, 0x3A, 0x18, 0x3E, 0x2B,
  0x80, 0xE4, 0xE8, 0x73, 0x50, 0x06, 0x8A, 0x56,
  0xC4, 0x2D, 0xC2, 0x6A, 0xD2, 0x0B, 0x9C, 0x4F,
  0x07, 0x1A, 0x0D, 0x19, 0xAA, 0x43, 0xB2, 0x6D,
  0x74, 0x8C, 0x8F, 0xF6, 0x0B, 0x1E, 0x54, 0xC1,
};

static uint32_t ota_crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  for(size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for(uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return crc;
}

static String ota_hex_string(const uint8_t *data, size_t len) {
  static const char hexmap[] = "0123456789abcdef";
  String out;
  out.reserve(len * 2u);
  for(size_t i = 0; i < len; ++i) {
    out += hexmap[(data[i] >> 4) & 0x0Fu];
    out += hexmap[data[i] & 0x0Fu];
  }
  return out;
}

static String ota_make_signed_payload(const ota_offer_pkt_t &pkt) {
  String payload;
  payload.reserve(192);
  payload += String((unsigned int)FANET_VENDOR_ID);
  payload += '|';
  payload += String((unsigned int)get_fanet_id());
  payload += '|';
  payload += String((unsigned long)pkt.nonce);
  payload += '|';
  payload += String((unsigned int)s_ota.current_version_bcd);
  payload += '|';
  payload += String((unsigned int)pkt.target_version_bcd);
  payload += '|';
  payload += String((unsigned long)pkt.image_size);
  payload += '|';
  payload += String((unsigned long)pkt.image_crc32);
  payload += '|';
  payload += ota_hex_string(pkt.image_sha256, sizeof(pkt.image_sha256));
  payload += '|';
  payload += String((unsigned long)pkt.expires);
  return payload;
}

static bool ota_verify_manifest_signature(const ota_offer_pkt_t &pkt) {
  const uint32_t now_ts = (uint32_t)time();
  if(pkt.expires && now_ts && now_ts > pkt.expires) {
    log_i("OTA manifest expired\r\n");
    return false;
  }

  ota_sha256_ctx_t ctx = {};
  uint8_t digest[32] = {0};
  const String payload = ota_make_signed_payload(pkt);
  ota_sha256_init(&ctx);
  ota_sha256_update(&ctx, (const uint8_t *)payload.c_str(), payload.length());
  ota_sha256_final(&ctx, digest);

  return uECC_verify(kBackendPublicKey, digest, sizeof(digest), pkt.signature_raw, uECC_secp256r1()) == 1;
}

static void ota_flash_wait_ready() {
  while(!NVMCTRL->INTFLAG.bit.READY) {
  }
}

static inline void ota_feed_watchdog() {
  if(settings.use_wdt && !s_ota_wdt_paused) {
    wdt_reset();
  }
}

static void ota_chunk_led_pulse() {
  s_ota_chunk_led_until_ms = millis() + 12u;
  led_error(1);
}

static void ota_loss_led_pulse() {
  s_ota_loss_led_until_ms = millis() + 18u;
  led_status(1);
}

static void ota_chunk_led_update() {
  if(s_ota_chunk_led_until_ms && (int32_t)(millis() - s_ota_chunk_led_until_ms) >= 0) {
    s_ota_chunk_led_until_ms = 0;
    led_error(0);
  }
  if(s_ota_loss_led_until_ms && (int32_t)(millis() - s_ota_loss_led_until_ms) >= 0) {
    s_ota_loss_led_until_ms = 0;
    led_status(0);
  }
}

static void ota_signal_error_hold() {
  s_ota_chunk_led_until_ms = 0;
  s_ota_loss_led_until_ms = 0;
  led_status(0);
  led_error(1);
  ota_feed_watchdog();
  delay(1500);
  ota_feed_watchdog();
  led_error(0);
}

static void ota_signal_success_sequence() {
  s_ota_chunk_led_until_ms = 0;
  s_ota_loss_led_until_ms = 0;
  led_error(0);
  led_status(0);
  for(uint8_t i = 0; i < 3u; ++i) {
    led_error(1);
    ota_feed_watchdog();
    delay(600);
    led_error(0);
    led_status(1);
    ota_feed_watchdog();
    delay(600);
    led_status(0);
  }
}

static void ota_pause_watchdog() {
  if(settings.use_wdt && !s_ota_wdt_paused) {
    wdt_disable();
    s_ota_wdt_paused = true;
    log_i("OTA WDT paused\r\n");
  }
}

static void ota_resume_watchdog() {
  if(settings.use_wdt && s_ota_wdt_paused) {
    wdt_enable(2500, false);
    s_ota_wdt_paused = false;
    log_i("OTA WDT resumed\r\n");
  }
}

static bool __attribute__((unused)) ota_flash_erase_range(uint32_t start, uint32_t len) {
  if(!len) {
    return false;
  }

  uint32_t erase_start = start & ~(OTA_FLASH_ROW_SIZE - 1u);
  uint32_t erase_end = (start + len + OTA_FLASH_ROW_SIZE - 1u) & ~(OTA_FLASH_ROW_SIZE - 1u);

  NVMCTRL->CTRLB.bit.MANW = 1;
  NVMCTRL->STATUS.reg = NVMCTRL_STATUS_MASK;

  for(uint32_t addr = erase_start; addr < erase_end; addr += OTA_FLASH_ROW_SIZE) {
    ota_feed_watchdog();
    noInterrupts();
    NVMCTRL->ADDR.reg = addr / 2u;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    ota_flash_wait_ready();
    const bool failed = (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_MASK) != 0;
    interrupts();
    if(failed) {
      return false;
    }
  }
  ota_feed_watchdog();
  return true;
}

static bool ota_flash_write_page(uint32_t address, const uint8_t *data, size_t len) {
  if(len > OTA_FLASH_PAGE_SIZE) {
    return false;
  }

  uint8_t page[OTA_FLASH_PAGE_SIZE];
  uint32_t words[OTA_FLASH_PAGE_SIZE / 4u];
  memset(page, 0xFF, sizeof(page));
  memcpy(page, data, len);
  memcpy(words, page, sizeof(words));

  ota_feed_watchdog();

  // Phase 1: clear page buffer and fill it (mirrors ota_program_row in ota_ab.cpp)
  noInterrupts();
  NVMCTRL->CTRLB.bit.MANW = 1;
  NVMCTRL->STATUS.reg = NVMCTRL_STATUS_MASK;
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
  ota_flash_wait_ready();
  volatile uint32_t *dst = (volatile uint32_t *)address;
  for(size_t i = 0; i < (OTA_FLASH_PAGE_SIZE / 4u); ++i) {
    *dst++ = words[i];
  }
  interrupts();

  // Phase 2: commit page to flash. Set ADDR explicitly (SAMD21 datasheet §22.8.7.2).
  // Use a bounded wait so a stalled WP doesn't lock up the device permanently.
  noInterrupts();
  NVMCTRL->ADDR.reg = address / 2u;
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
  bool wp_ready = false;
  for(uint32_t n = 0; n < 5000000u; ++n) {
    if(NVMCTRL->INTFLAG.bit.READY) { wp_ready = true; break; }
  }
  const uint16_t nvm_status  = NVMCTRL->STATUS.reg;
  const uint8_t  nvm_intflag = NVMCTRL->INTFLAG.reg;
  const uint16_t nvm_lock    = NVMCTRL->LOCK.reg;
  interrupts();

  ota_feed_watchdog();

  if(!wp_ready) {
    log_write("OTA NVM WP timeout addr=0x");
    log_write_hex(address, 8);
    log_write(" st=0x");
    log_write_hex(nvm_status, 4);
    log_write(" if=0x");
    log_write_hex(nvm_intflag, 2);
    log_write(" lk=0x");
    log_write_hex(nvm_lock, 4);
    log_write("\r\n");
    log_flush();
    return false;
  }

  return (nvm_status & NVMCTRL_STATUS_MASK) == 0;
}

static bool ota_flash_write_row(uint32_t address, const uint8_t *data, size_t len) {
  if(len > OTA_FLASH_ROW_SIZE) {
    return false;
  }

  uint8_t row[OTA_FLASH_ROW_SIZE];
  memset(row, 0xFF, sizeof(row));
  memcpy(row, data, len);

  for(size_t off = 0; off < OTA_FLASH_ROW_SIZE; off += OTA_FLASH_PAGE_SIZE) {
    if(!ota_flash_write_page(address + off, &row[off], OTA_FLASH_PAGE_SIZE)) {
      return false;
    }
  }
  return true;
}

static bool __attribute__((unused)) ota_flush_full_rows() {
  while(s_ota.page_fill >= OTA_FLASH_ROW_SIZE) {
    if(!ota_flash_write_row(s_ota.write_addr, s_ota.page_buf, OTA_FLASH_ROW_SIZE)) {
      return false;
    }

    s_ota.write_addr += OTA_FLASH_ROW_SIZE;
    s_ota.page_fill -= OTA_FLASH_ROW_SIZE;

    if(s_ota.page_fill) {
      memmove(s_ota.page_buf, &s_ota.page_buf[OTA_FLASH_ROW_SIZE], s_ota.page_fill);
    }
    memset(&s_ota.page_buf[s_ota.page_fill], 0xFF, sizeof(s_ota.page_buf) - s_ota.page_fill);
  }
  return true;
}

static bool __attribute__((unused)) ota_flush_page() {
  if(!s_ota.page_fill) {
    return true;
  }

  bool ok = ota_flash_write_row(s_ota.write_addr, s_ota.page_buf, s_ota.page_fill);
  if(ok) {
    s_ota.write_addr += OTA_FLASH_ROW_SIZE;
    s_ota.page_fill = 0;
    memset(s_ota.page_buf, 0xFF, sizeof(s_ota.page_buf));
  }
  return ok;
}

static bool __attribute__((unused)) ota_append_image_data(const uint8_t *data, size_t len) {
  if(((size_t)s_ota.page_fill + len) > sizeof(s_ota.page_buf)) {
    return false;
  }

  memcpy(&s_ota.page_buf[s_ota.page_fill], data, len);
  s_ota.page_fill += (uint16_t)len;
  return true;
}

static bool __attribute__((unused)) ota_candidate_valid(uint32_t app_base) {
  uint32_t stack_ptr = *(uint32_t *)app_base;
  uint32_t reset_vec = *(uint32_t *)(app_base + 4u);
  bool stack_ok = (stack_ptr >= 0x20000000u) && (stack_ptr <= 0x20008000u) && ((stack_ptr & 0x3u) == 0u);
  bool reset_ok_direct = (reset_vec & 1u) && (reset_vec >= app_base) && (reset_vec < OTA_FLASH_END);
  bool reset_ok_generic = (app_base == OTA_SLOT1_START) && (reset_vec & 1u) &&
                          (reset_vec >= OTA_SLOT0_START) && (reset_vec < (OTA_SLOT0_START + OTA_SLOT0_SIZE));
  return stack_ok && (reset_ok_direct || reset_ok_generic);
}

static void ota_make_prefix(ota_pkt_prefix_t &prefix, uint8_t op) {
  prefix.magic0 = OTA_MAGIC0;
  prefix.magic1 = OTA_MAGIC1;
  prefix.proto = OTA_LORA_PROTOCOL_VERSION;
  prefix.op = op;
}

static bool ota_check_prefix(const uint8_t *data, size_t len, uint8_t op) {
  if(len < sizeof(ota_pkt_prefix_t)) {
    return false;
  }

  const ota_pkt_prefix_t *prefix = (const ota_pkt_prefix_t *)data;
  return prefix->magic0 == OTA_MAGIC0 &&
         prefix->magic1 == OTA_MAGIC1 &&
         prefix->proto == OTA_LORA_PROTOCOL_VERSION &&
         prefix->op == op;
}


static void ota_reset_state(uint8_t status) {
  s_ota.offer_pending = false;
  s_ota.active = false;
  s_ota.fast_mode = false;
  s_ota.status = status;
  s_ota.target_version_bcd = 0;
  s_ota.chunk_size = 0;
  s_ota.next_seq = 0;
  s_ota.image_size = 0;
  s_ota.image_crc32 = 0;
  s_ota.expires = 0;
  s_ota.slot_base = 0;
  s_ota.slot_size = 0;
  s_ota.write_addr = 0;
  s_ota.bytes_written = 0;
  s_ota.deadline = 0;
  s_ota.crc32 = 0xFFFFFFFFu;
  memset(s_ota.expected_sha256, 0, sizeof(s_ota.expected_sha256));
  ota_sha256_init(&s_ota.sha256);
  s_ota.page_fill = 0;
  s_ota.session_start_ms = 0;
  s_ota.last_chunk_ms = 0;
  s_ota.last_wait_log_ms = 0;
  s_ota.last_progress_pct = 0;
  s_ota_chunk_led_until_ms = 0;
  s_ota_loss_led_until_ms = 0;
  led_error(0);
  led_status(0);
  memset(s_ota.page_buf, 0xFF, sizeof(s_ota.page_buf));
}

static bool ota_apply_radio_mode(bool fast_mode) {
  float freq = settings.lora_freq;
  uint16_t bw = fast_mode ? (uint16_t)OTA_LORA_FAST_BW_KHZ : (uint16_t)settings.lora_bw;
  uint8_t sf = fast_mode ? (uint8_t)OTA_LORA_FAST_SF : (uint8_t)settings.lora_sf;
  uint8_t cr = fast_mode ? (uint8_t)OTA_LORA_FAST_CR : (uint8_t)settings.lora_cr;
  uint8_t syncword = fast_mode ? (uint8_t)OTA_LORA_FAST_SYNCWORD : (uint8_t)LORA_SYNCWORD;
  size_t preamble = fast_mode ? (size_t)OTA_LORA_FAST_PREAMBLE : 12u;

  dis_rx_sleep();
  if(radio_phy->standby() != RADIOLIB_ERR_NONE) {
    en_rx_sleep();
    return false;
  }
  if(radio_phy->setFrequency(freq) != RADIOLIB_ERR_NONE) {
    en_rx_sleep();
    return false;
  }

  int16_t state = RADIOLIB_ERR_UNKNOWN;
  if(lora_module == LORA_SX1276) {
    SX1276 *phy = static_cast<SX1276 *>(radio_phy);
    state = phy->setBandwidth((float)bw);
    if(state == RADIOLIB_ERR_NONE) { state = phy->setSpreadingFactor(sf); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setCodingRate(cr); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setSyncWord(syncword); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setPreambleLength(preamble); }
  } else if(lora_module == LORA_SX1262) {
    SX1262 *phy = static_cast<SX1262 *>(radio_phy);
    state = phy->setBandwidth((float)bw);
    if(state == RADIOLIB_ERR_NONE) { state = phy->setSpreadingFactor(sf); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setCodingRate(cr); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setSyncWord(syncword); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setPreambleLength(preamble); }
  } else if(lora_module == LORA_LLCC68) {
    LLCC68 *phy = static_cast<LLCC68 *>(radio_phy);
    state = phy->setBandwidth((float)bw);
    if(state == RADIOLIB_ERR_NONE) { state = phy->setSpreadingFactor(sf); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setCodingRate(cr); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setSyncWord(syncword); }
    if(state == RADIOLIB_ERR_NONE) { state = phy->setPreambleLength(preamble); }
  }

  if(state != RADIOLIB_ERR_NONE) {
    en_rx_sleep();
    return false;
  }

  s_ota.fast_mode = fast_mode;
  en_rx_sleep();
  return true;
}

static uint32_t ota_elapsed_ms() {
  return s_ota.session_start_ms ? (millis() - s_ota.session_start_ms) : 0u;
}

static void ota_log_progress(const char *prefix, uint32_t idle_ms = 0u) {
  const uint32_t progress = s_ota.image_size ? ((s_ota.bytes_written * 100u) / s_ota.image_size) : 100u;
  log_write(prefix);
  log_write(" seq=");
  log_write((uint32_t)(s_ota.next_seq ? (s_ota.next_seq - 1u) : 0u));
  log_write(" bytes=");
  log_write((uint32_t)s_ota.bytes_written);
  log_write("/");
  log_write((uint32_t)s_ota.image_size);
  log_write(" ");
  log_write(progress);
  log_write("% t=");
  log_write(ota_elapsed_ms());
  log_write("ms");
  if(idle_ms) {
    log_write(" idle=");
    log_write(idle_ms);
    log_write("ms");
  }
  log_write("\r\n");
}

static void ota_abort_session(uint8_t status) {
  bool restore = s_ota.fast_mode;
  log_write("OTA abort status=0x");
  log_write_hex(status, 2);
  log_write(" seq=");
  log_write((uint32_t)s_ota.next_seq);
  log_write(" bytes=");
  log_write((uint32_t)s_ota.bytes_written);
  log_write(" t=");
  log_write(ota_elapsed_ms());
  log_write("ms\r\n");
  ota_signal_error_hold();
  ota_reset_state(status);
  if(restore) {
    ota_apply_radio_mode(false);
  }
  ota_resume_watchdog();
}

static bool ota_send_ack(uint8_t status) {
  ota_ack_pkt_t ack = {};
  ota_make_prefix(ack.prefix, OTA_OP_ACK);
  ack.nonce = s_ota.nonce;
  ack.next_seq = s_ota.next_seq;
  ack.status = status;
  ack.accepted_chunk = (uint8_t)s_ota.chunk_size;

  if(status != OTA_STATE_RX || s_ota.next_seq < 3u) {
    log_write("OTA ACK tx status=0x");
    log_write_hex(status, 2);
    log_write(" next_seq=");
    log_write((uint32_t)ack.next_seq);
    log_write(" bytes=");
    log_write((uint32_t)s_ota.bytes_written);
    log_write("\r\n");
  }

  ota_feed_watchdog();
  dis_rx_sleep();
  int state = radio_phy->transmit((uint8_t *)&ack, sizeof(ack));
  if(state != RADIOLIB_ERR_NONE) {
    log_i("OTA ack failed\r\n");
    en_rx_sleep();
    return false;
  }

  if(s_ota.offer_pending || s_ota.active) {
    en_rx_sleep();
  }
  return true;
}

static bool ota_enter_update_mode() {
  s_ota.offer_pending = false;
  s_ota.active = true;
  s_ota.status = OTA_STATE_WAIT;
  // Use a short deadline while waiting for the first START packet.
  // ota_begin_session() will extend this to OTA_LORA_SESSION_TIMEOUT_MS once
  // the offer is accepted and the session is actually running.
  s_ota.deadline = time() + OTA_LORA_RX_WINDOW_MS;
  s_ota.next_seq = 0;
  s_ota.bytes_written = 0;
  s_ota.page_fill = 0;
  memset(s_ota.page_buf, 0xFF, sizeof(s_ota.page_buf));
  ota_pause_watchdog();

  if(!ota_apply_radio_mode(true)) {
    ota_reset_state(OTA_ERR_RADIO);
    ota_resume_watchdog();
    log_i("OTA radio switch failed\r\n");
    return false;
  }

  log_i("OTA fast RX mode enabled\r\n");
  return true;
}

static bool ota_begin_session(const ota_offer_pkt_t &pkt) {
  s_ota.inactive_slot = ota_ab_inactive_slot();
  s_ota.slot_base = ota_ab_slot_address(s_ota.inactive_slot);
  s_ota.slot_size = ota_ab_slot_size(s_ota.inactive_slot);
  s_ota.chunk_size = pkt.chunk_size ? min((uint16_t)OTA_LORA_MAX_CHUNK, pkt.chunk_size) : (uint16_t)OTA_LORA_MAX_CHUNK;
  s_ota.target_version_bcd = pkt.target_version_bcd;
  s_ota.image_size = pkt.image_size;
  s_ota.image_crc32 = pkt.image_crc32;
  s_ota.expires = pkt.expires;
  memcpy(s_ota.expected_sha256, pkt.image_sha256, sizeof(s_ota.expected_sha256));
  s_ota.write_addr = s_ota.slot_base;
  s_ota.bytes_written = 0;
  s_ota.next_seq = 0;
  s_ota.crc32 = 0xFFFFFFFFu;
  ota_sha256_init(&s_ota.sha256);
  s_ota.page_fill = 0;
  memset(s_ota.page_buf, 0xFF, sizeof(s_ota.page_buf));

  if(!s_ota.image_size || s_ota.image_size > s_ota.slot_size) {
    ota_send_ack(OTA_ERR_SIZE);
    ota_abort_session(OTA_ERR_SIZE);
    return false;
  }

  if(pkt.target_version_bcd <= s_ota.current_version_bcd) {
    ota_send_ack(OTA_STATE_IDLE);
    ota_abort_session(OTA_STATE_IDLE);
    return false;
  }

  if(!ota_verify_manifest_signature(pkt)) {
    log_i("OTA signature invalid on device\r\n");
    ota_send_ack(OTA_ERR_SIGNATURE);
    ota_abort_session(OTA_ERR_SIGNATURE);
    return false;
  }

  if(!ota_flash_erase_range(s_ota.slot_base, s_ota.image_size)) {
    ota_send_ack(OTA_ERR_FLASH);
    ota_abort_session(OTA_ERR_FLASH);
    return false;
  }

  s_ota.status = OTA_STATE_RX;
  s_ota.deadline = time() + OTA_LORA_SESSION_TIMEOUT_MS;

  log_write("OTA session params: slot=");
  log_write((uint32_t)s_ota.inactive_slot);
  log_write(" size=");
  log_write((uint32_t)s_ota.image_size);
  log_write(" chunk=");
  log_write((uint32_t)s_ota.chunk_size);
  log_write(" crc=0x");
  log_write_hex(s_ota.image_crc32, 8);
  log_write("\r\n");

  ota_send_ack(OTA_STATE_RX);

  log_i("OTA session started\r\n");
  return true;
}

static bool ota_process_fanet_ack(const uint8_t *data, size_t len) {
  if(!s_ota.offer_pending || len < sizeof(fanet_header)) {
    return false;
  }

  const fanet_header *header = (const fanet_header *)data;
  if(header->type != FANET_ACK_TYPE) {
    return false;
  }

  if(header->vendor != FANET_VENDOR_ID || header->address != get_fanet_id()) {
    return false;
  }

  log_i("OTA nonce RX mode end (FANET ACK received)\r\n");
  return ota_enter_update_mode();
}

// Process config update packet (higher priority than OTA firmware updates)
// Config updates have priority and will be processed first during RX window
static bool ota_process_config_update(const uint8_t *data, size_t len) {
  // Check if we're in the offer_pending state (waiting after HWInfo)
  // Config updates can only arrive during this window
  if (!s_ota.offer_pending) {
    return false;
  }

  // Check if this is a config update packet
  if (!ota_check_prefix(data, len, OTA_OP_CONFIG)) {
    return false;
  }

  // Minimum config packet: prefix(4) + payload + signature(64)
  if (len < (sizeof(ota_pkt_prefix_t) + 70)) {
    log_e("Config update packet too small\r\n");
    ota_send_ack(OTA_ERR_SIZE);
    return true; // Consumed the packet even if invalid
  }

  log_i("Config update packet received (");
  log_write((int)len);
  log_write(" bytes)\r\n");

  // Extract the actual config payload (skip the OTA prefix)
  const uint8_t *config_payload = data + sizeof(ota_pkt_prefix_t);
  size_t config_len = len - sizeof(ota_pkt_prefix_t);

  // Verify and apply config update
  // config_update_apply_packet() handles signature verification and UUID check
  int applied = config_update_apply_packet(config_payload, config_len);

  if (applied > 0) {
    log_i("Config update success: ");
    log_write(applied);
    log_write(" settings applied\r\n");
    ota_send_ack(OTA_STATE_IDLE); // ACK with "idle" = config applied successfully

    // Exit OTA mode, flush log, then reboot so new settings take effect.
    ota_reset_state(OTA_STATE_IDLE);
    log_i("Rebooting to apply new settings...\r\n");
    log_flush();
    delay(200); // let the LoRa TX finish before reset
    NVIC_SystemReset();
  } else {
    log_e("Config update failed\r\n");
    ota_send_ack(OTA_ERR_SIGNATURE); // Use signature error for config failures
    ota_abort_session(OTA_ERR_SIGNATURE);
  }

  return true; // Packet consumed
}

static bool ota_process_offer(const uint8_t *data, size_t len) {
  if(!s_ota.active || !ota_check_prefix(data, len, OTA_OP_START) || len < sizeof(ota_offer_pkt_t)) {
    return false;
  }

  const ota_offer_pkt_t *pkt = (const ota_offer_pkt_t *)data;
  if(pkt->nonce != s_ota.nonce) {
    return true;
  }

  if(s_ota.status == OTA_STATE_RX && pkt->image_size == s_ota.image_size && pkt->image_crc32 == s_ota.image_crc32) {
    ota_send_ack(OTA_STATE_RX);
    return true;
  }

  return ota_begin_session(*pkt);
}

static bool ota_process_chunk(const uint8_t *data, size_t len) {
  if(!s_ota.active || s_ota.status != OTA_STATE_RX || len < (sizeof(ota_pkt_prefix_t) + 4u) || !ota_check_prefix(data, len, OTA_OP_CHUNK)) {
    return false;
  }

  const ota_chunk_pkt_t *pkt = (const ota_chunk_pkt_t *)data;
  uint16_t data_len = pkt->data_len;
  if(data_len > OTA_LORA_MAX_CHUNK || (sizeof(ota_pkt_prefix_t) + 4u + data_len) > len) {
    ota_send_ack(OTA_ERR_SIZE);
    return true;
  }

  ota_chunk_led_pulse();

  if(pkt->seq != s_ota.next_seq) {
    ota_loss_led_pulse();
    log_write("OTA unexpected chunk seq=");
    log_write((uint32_t)pkt->seq);
    log_write(" expected=");
    log_write((uint32_t)s_ota.next_seq);
    log_write("\r\n");
    if(s_ota.next_seq > 0u && pkt->seq == (uint16_t)(s_ota.next_seq - 1u)) {
      ota_send_ack(OTA_STATE_RX);
      return true;
    }
    ota_send_ack(OTA_ERR_SEQ);
    return true;
  }

  if((s_ota.bytes_written + data_len) > s_ota.image_size) {
    ota_send_ack(OTA_ERR_SIZE);
    ota_abort_session(OTA_ERR_SIZE);
    return true;
  }

  if(pkt->seq < 3u) {
    log_write("OTA chunk rx seq=");
    log_write((uint32_t)pkt->seq);
    log_write(" len=");
    log_write((uint32_t)data_len);
    log_write(" bytes_before=");
    log_write((uint32_t)s_ota.bytes_written);
    log_write("\r\n");
  }

  if(!ota_append_image_data(pkt->payload, data_len)) {
    ota_send_ack(OTA_ERR_FLASH);
    ota_abort_session(OTA_ERR_FLASH);
    return true;
  }

  ota_feed_watchdog();
  s_ota.crc32 = ota_crc32_update(s_ota.crc32, pkt->payload, data_len);
  ota_sha256_update(&s_ota.sha256, pkt->payload, data_len);
  s_ota.bytes_written += data_len;
  s_ota.next_seq++;
  s_ota.deadline = time() + OTA_LORA_SESSION_TIMEOUT_MS;
  if(s_ota.session_start_ms == 0u) {
    s_ota.session_start_ms = millis();
  }
  s_ota.last_chunk_ms = millis();

  const uint32_t progress = s_ota.image_size ? ((s_ota.bytes_written * 100u) / s_ota.image_size) : 100u;
  if(pkt->seq < 3u || progress >= (uint32_t)(s_ota.last_progress_pct + 5u) || s_ota.bytes_written == s_ota.image_size) {
    ota_log_progress("OTA progress");
    s_ota.last_progress_pct = (uint8_t)progress;
    s_ota.last_wait_log_ms = s_ota.last_chunk_ms;
  }

  dis_rx_sleep();
  const uint32_t flushStartMs = millis();
  if(!ota_flush_full_rows()) {
    ota_send_ack(OTA_ERR_FLASH);
    ota_abort_session(OTA_ERR_FLASH);
    return true;
  }
  const uint32_t flushMs = millis() - flushStartMs;
  if(flushMs >= 4u) {
    log_write("OTA flash flush ms=");
    log_write(flushMs);
    log_write(" after_seq=");
    log_write((uint32_t)pkt->seq);
    log_write("\r\n");
  }

  ota_send_ack(OTA_STATE_RX);
  return true;
}

static bool ota_process_abort(const uint8_t *data, size_t len) {
  if((!s_ota.active && !s_ota.offer_pending) || len < sizeof(ota_abort_pkt_t) || !ota_check_prefix(data, len, OTA_OP_ABORT)) {
    return false;
  }

  const ota_abort_pkt_t *pkt = (const ota_abort_pkt_t *)data;
  if(pkt->nonce != s_ota.nonce) {
    return true;
  }

  // In multi-GS deployments a station that does not serve the update may still
  // emit an "idle/no-update" abort for the same nonce. Ignore this only while we
  // have already entered active update mode (after FANET ACK) and are still
  // waiting for the first START packet. During offer_pending (before FANET ACK)
  // an IDLE abort means "no update available" and must be honoured.
  if(s_ota.active && pkt->status == OTA_STATE_IDLE && s_ota.status == OTA_STATE_WAIT && s_ota.next_seq == 0u && s_ota.bytes_written == 0u) {
    log_i("OTA abort idle ignored while waiting for start\r\n");
    return true;
  }

  log_write("OTA abort packet received status=0x");
  log_write_hex(pkt->status, 2);
  log_write("\r\n");
  if(pkt->status == OTA_STATE_IDLE) {
    ota_start_no_update_cooldown();
  }
  ota_abort_session(pkt->status ? pkt->status : OTA_STATE_IDLE);
  return true;
}

static bool ota_process_finish(const uint8_t *data, size_t len) {
  if(!s_ota.active || s_ota.status != OTA_STATE_RX || len < sizeof(ota_finish_pkt_t) || !ota_check_prefix(data, len, OTA_OP_FINISH)) {
    return false;
  }

  const ota_finish_pkt_t *pkt = (const ota_finish_pkt_t *)data;
  if(pkt->nonce != s_ota.nonce || pkt->image_size != s_ota.image_size || pkt->image_crc32 != s_ota.image_crc32) {
    ota_send_ack(OTA_ERR_CRC);
    ota_abort_session(OTA_ERR_CRC);
    return true;
  }

  if(!ota_flush_page()) {
    ota_send_ack(OTA_ERR_FLASH);
    ota_abort_session(OTA_ERR_FLASH);
    return true;
  }

  if(s_ota.bytes_written != s_ota.image_size) {
    ota_send_ack(OTA_ERR_SIZE);
    ota_abort_session(OTA_ERR_SIZE);
    return true;
  }

  uint32_t crc = s_ota.crc32 ^ 0xFFFFFFFFu;
  if(crc != s_ota.image_crc32) {
    ota_send_ack(OTA_ERR_CRC);
    ota_abort_session(OTA_ERR_CRC);
    return true;
  }

  uint8_t image_hash[32] = {0};
  ota_sha256_final(&s_ota.sha256, image_hash);
  if(memcmp(image_hash, s_ota.expected_sha256, sizeof(image_hash)) != 0) {
    log_i("OTA SHA256 mismatch on device\r\n");
    ota_send_ack(OTA_ERR_SIGNATURE);
    ota_abort_session(OTA_ERR_SIGNATURE);
    return true;
  }

  if(!ota_candidate_valid(s_ota.slot_base)) {
    const uint32_t stack_ptr = *(uint32_t *)s_ota.slot_base;
    const uint32_t reset_vec = *(uint32_t *)(s_ota.slot_base + 4u);
#if OTA_BOOT_DIAG_LOGS
    log_write("OTA image invalid base=0x");
    log_write_hex(s_ota.slot_base, 8);
    log_write(" sp=0x");
    log_write_hex(stack_ptr, 8);
    log_write(" rv=0x");
    log_write_hex(reset_vec, 8);
    log_write("\r\n");
#endif
    ota_send_ack(OTA_ERR_IMAGE);
    ota_abort_session(OTA_ERR_IMAGE);
    return true;
  }

  const uint32_t stack_ptr = *(uint32_t *)s_ota.slot_base;
  const uint32_t reset_vec = *(uint32_t *)(s_ota.slot_base + 4u);
  const bool reset_in_slot = (reset_vec & 1u) && (reset_vec >= s_ota.slot_base) && (reset_vec < (s_ota.slot_base + s_ota.slot_size));
#if OTA_BOOT_DIAG_LOGS
  log_write("OTA candidate OK slot=");
  log_write((uint32_t)s_ota.inactive_slot);
  log_write(" base=0x");
  log_write_hex(s_ota.slot_base, 8);
  log_write(" sp=0x");
  log_write_hex(stack_ptr, 8);
  log_write(" rv=0x");
  log_write_hex(reset_vec, 8);
  log_write(" rv_in_slot=");
  log_write(reset_in_slot ? "1" : "0");
  log_write(" slot_end=0x");
  log_write_hex(s_ota.slot_base + s_ota.slot_size, 8);
  log_write("\r\n");

  if(!reset_in_slot) {
    log_i("OTA warning: reset vector is outside target slot; bootloader may reject pending slot\r\n");
  }
#endif

  ota_ab_log_state("finish-pre-request");

  if(!ota_ab_request_slot(s_ota.inactive_slot)) {
    log_write("OTA request slot failed target=");
    log_write((uint32_t)s_ota.inactive_slot);
    log_write("\r\n");
    ota_ab_log_state("finish-request-failed");
    ota_send_ack(OTA_ERR_FLASH);
    ota_abort_session(OTA_ERR_FLASH);
    return true;
  }

  ota_ab_log_state("finish-post-request");

  ota_trace_mark_pending_reboot(s_ota.inactive_slot, s_ota.slot_base, s_ota.slot_size,
                                stack_ptr, reset_vec, reset_in_slot);

  ota_send_ack(OTA_STATE_READY);
  log_i("OTA ready, rebooting\r\n");
  ota_signal_success_sequence();
  log_flush();
  delay(40);
  NVIC_SystemReset();
  return true;
}

void ota_lora_begin() {
  s_ota.current_version_bcd = ota_version_bcd();
  s_ota_wdt_paused = false;

  // Stagger first hwinfo packets after boot using fake past timestamps.
  // Setting last = now - interval + delay makes the first TX fire after 'delay' ms.
  const uint32_t now = (uint32_t)time();
  const uint32_t ota_boot_delay = settings.ota_fast ? OTA_HWINFO_BOOT_DELAY_OTA_FAST_MS : OTA_HWINFO_BOOT_DELAY_OTA_MS;
  s_hwinfo_last_dynamic_ts = now - (OTA_HWINFO_DYNAMIC_INTERVAL_S * 1000u) + OTA_HWINFO_BOOT_DELAY_DYNAMIC_MS;
  if(!s_hwinfo_last_dynamic_ts) s_hwinfo_last_dynamic_ts = 1u;
  s_hwinfo_last_ota_ts     = now - (OTA_HWINFO_OTA_INTERVAL_S     * 1000u) + ota_boot_delay;
  if(!s_hwinfo_last_ota_ts) s_hwinfo_last_ota_ts = 1u;
  s_hwinfo_last_static_ts  = now - (OTA_HWINFO_STATIC_INTERVAL_S  * 1000u) + OTA_HWINFO_BOOT_DELAY_STATIC_MS;
  if(!s_hwinfo_last_static_ts) s_hwinfo_last_static_ts = 1u;

  s_hwinfo_retry_not_before_ms = 0;
  ota_reset_state(OTA_STATE_IDLE);
}

void ota_lora_prepare_hwinfo(hwInfoData &info) {
  s_ota.current_version_bcd = ota_version_bcd();
  memset(&info.debug_dynamic, 0, sizeof(info.debug_dynamic));
  memset(&info.debug_ota, 0, sizeof(info.debug_ota));
  memset(&info.debug_static, 0, sizeof(info.debug_static));
  s_last_hwinfo_was_ota = false;
  info.debug_type = 0u;

  const uint32_t now_ms = (uint32_t)time();
  const uint32_t dynamic_interval_ms = (ota_hwinfo_low_battery() ? OTA_HWINFO_DYNAMIC_LOW_BATT_INTERVAL_S
                                                                  : OTA_HWINFO_DYNAMIC_INTERVAL_S) * 1000u;
  const bool dynamic_due = ota_hwinfo_due(now_ms, s_hwinfo_last_dynamic_ts, dynamic_interval_ms);
  const bool ota_due = settings.ota_enable &&
                       !ota_no_update_cooldown_active() &&
                       ota_hwinfo_due(now_ms, s_hwinfo_last_ota_ts, OTA_HWINFO_OTA_INTERVAL_S * 1000u);
  const bool static_due = ota_hwinfo_due(now_ms, s_hwinfo_last_static_ts, OTA_HWINFO_STATIC_INTERVAL_S * 1000u);

  if(dynamic_due) {
    info.debug_type = HWINFO_DEBUG_DYNAMIC;
    info.debug_dynamic.vbatt_mv = (uint16_t)constrain(lroundf(sensor.batt_volt * 1000.0f), 0L, 65535L);
    info.debug_dynamic.batt_perc = (uint8_t)constrain(sensor.batt_perc, 0, 100);
    info.debug_dynamic.pv_state = (sensor.pv_charging ? 0x01u : 0x00u) | (sensor.pv_done ? 0x02u : 0x00u);
    info.debug_dynamic.rx_count = fanet_rx_counter;
    info.debug_dynamic.forward_count = fanet_forward_counter;
    info.debug_dynamic.lbt_counter = settings.lbt_counter;
    info.debug_dynamic.reserved = 0u;
    s_hwinfo_last_dynamic_ts = now_ms;
  } else if(ota_due) {
    info.debug_type = HWINFO_DEBUG_OTA;
    info.debug_ota.version_bcd = s_ota.current_version_bcd;
    info.debug_ota.nonce = ota_next_nonce();
    info.debug_ota.slot_capacity_kb = (uint16_t)(ota_ab_slot_size(ota_ab_inactive_slot()) / 1024u);
    info.debug_ota.ota_proto = OTA_LORA_PROTOCOL_VERSION;
    info.debug_ota.ota_state = ota_ab_current_slot() & 0x01u;
    s_last_hwinfo_was_ota = true;
    s_hwinfo_last_ota_ts = now_ms;
  } else if(static_due) {
    info.debug_type = HWINFO_DEBUG_STATIC;
    info.debug_static.sensor_type = settings.sensor_type;
    info.debug_static.use_baro = settings.use_baro ? 1u : 0u;
    info.debug_static.uv_triggered = settings.uv_triggered ? 1u : 0u;
    info.debug_static.lbt = settings.lora_lbt ? 1u : 0u;
    info.debug_static.lora_rssi_threshold = (uint8_t)constrain(-settings.lora_rssi_threshold, 0, 255);
    info.debug_static.sensor_integ_s = (uint8_t)constrain((int)(settings.sensor_integration_time / 1000u), 0, 255);
    info.debug_static.reduce_interval_voltage = (uint8_t)constrain((int)lroundf((settings.reduce_interval_voltage * 100.0f) - 200.0f), 0, 255);
    s_hwinfo_last_static_ts = now_ms;
  }
}

uint32_t ota_lora_next_hwinfo_due_ms() {
  const uint32_t now_ms = (uint32_t)time();

  const uint32_t dynamic_interval_ms = (ota_hwinfo_low_battery() ? OTA_HWINFO_DYNAMIC_LOW_BATT_INTERVAL_S
                                                                  : OTA_HWINFO_DYNAMIC_INTERVAL_S) * 1000u;
  uint32_t due_ms = ota_hwinfo_due_in_ms(now_ms, s_hwinfo_last_dynamic_ts, dynamic_interval_ms);

  if(settings.ota_enable && !ota_no_update_cooldown_active()) {
    due_ms = min(due_ms, ota_hwinfo_due_in_ms(now_ms, s_hwinfo_last_ota_ts, OTA_HWINFO_OTA_INTERVAL_S * 1000u));
  }

  due_ms = min(due_ms, ota_hwinfo_due_in_ms(now_ms, s_hwinfo_last_static_ts, OTA_HWINFO_STATIC_INTERVAL_S * 1000u));

  if(s_hwinfo_retry_not_before_ms && (int32_t)(now_ms - s_hwinfo_retry_not_before_ms) < 0) {
    const uint32_t retry_wait_ms = (uint32_t)(s_hwinfo_retry_not_before_ms - now_ms);
    due_ms = max(due_ms, retry_wait_ms);
  }

  return due_ms;
}

void ota_lora_defer_hwinfo_retry(uint32_t delay_ms) {
  s_hwinfo_retry_not_before_ms = (uint32_t)time() + delay_ms;
}

void ota_lora_note_hwinfo_tx_started() {
  s_hwinfo_retry_not_before_ms = 0;
  if(!radio_phy || s_ota.active || !s_last_hwinfo_was_ota || ota_no_update_cooldown_active() || !settings.ota_enable) {
    return;
  }
  s_last_hwinfo_was_ota = false;
  s_ota.offer_pending = true;
  s_ota.status = OTA_STATE_WAIT;
}

bool ota_lora_on_tx_complete() {
  if(!s_ota.offer_pending) {
    return false;
  }
  s_ota.deadline = time() + OTA_LORA_RX_WINDOW_MS;
  log_i("OTA nonce RX mode start\r\n");
  en_rx_sleep();
  return true;
}

bool ota_lora_busy() {
  return s_ota.offer_pending || s_ota.active;
}

bool ota_lora_poll() {
  ota_chunk_led_update();
  if(!ota_lora_busy()) {
    return false;
  }

  if(time() > s_ota.deadline) {
    const bool nonce_wait_timeout = s_ota.offer_pending && !s_ota.active;
    uint8_t status = s_ota.active ? OTA_ERR_TIMEOUT : OTA_STATE_IDLE;
    if(nonce_wait_timeout) {
      log_i("OTA nonce RX mode end (timeout)\r\n");
    }
    ota_abort_session(status);
    if(settings.lora_smart_rcv) {
      en_rx_sleep();
    } else {
      radio_sleep();
    }
    return false;
  }

  if(!loraReceivedFlag) {
    if(s_ota.active && s_ota.status == OTA_STATE_RX && s_ota.bytes_written > 0u && s_ota.last_chunk_ms > 0u) {
      const uint32_t nowMs = millis();
      const uint32_t idleMs = nowMs - s_ota.last_chunk_ms;
      if(idleMs >= OTA_LORA_IDLE_DEBUG_MS && (nowMs - s_ota.last_wait_log_ms) >= OTA_LORA_IDLE_DEBUG_MS) {
        s_ota.last_wait_log_ms = nowMs;
        ota_loss_led_pulse();
        ota_log_progress("OTA wait", idleMs);
      }
    }
    return false;
  }

  loraReceivedFlag = false;
  int numBytes = radio_phy->getPacketLength();
  if(numBytes <= 0 || numBytes > 255) {
    en_rx_sleep();
    return true;
  }

  uint8_t packet[255] = {0};
  int state = radio_phy->readData(packet, numBytes);
  if(state != RADIOLIB_ERR_NONE) {
    en_rx_sleep();
    return true;
  }

  if(ota_process_fanet_ack(packet, (size_t)numBytes)) {
    return true;
  }
  // Config updates have higher priority than OTA firmware updates
  if(ota_process_config_update(packet, (size_t)numBytes)) {
    return true;
  }
  if(ota_process_offer(packet, (size_t)numBytes)) {
    return true;
  }
  if(ota_process_chunk(packet, (size_t)numBytes)) {
    return true;
  }
  if(ota_process_abort(packet, (size_t)numBytes)) {
    return true;
  }
  if(ota_process_finish(packet, (size_t)numBytes)) {
    return true;
  }

  if(s_ota.offer_pending || s_ota.active) {
    en_rx_sleep();
    return true;
  }
  return false;
}

#else

void ota_lora_begin() {}
void ota_lora_prepare_hwinfo(hwInfoData &info) { (void)info; }
void ota_lora_note_hwinfo_tx_started() {}
bool ota_lora_on_tx_complete() { return false; }
bool ota_lora_poll() { return false; }
bool ota_lora_busy() { return false; }

#endif
