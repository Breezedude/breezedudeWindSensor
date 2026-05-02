#include "ota_ab.h"
#include "logging.h"

static bool ota_wait_ready() {
  // Guard against rare NVM controller stalls so startup cannot hang forever.
  const uint32_t max_spins = 1000000UL;
  uint32_t spins = 0;
  while (!NVMCTRL->INTFLAG.bit.READY) {
    if (++spins >= max_spins) {
      return false;
    }
  }
  return true;
}

uint32_t ota_ab_checksum_words(uint32_t active_slot, uint32_t pending_slot) {
  return OTA_AB_MAGIC ^ OTA_AB_VERSION ^ active_slot ^ pending_slot ^ 0xA5A55A5AUL;
}

static bool ota_ab_valid(const ota_ab_flags_t &cfg) {
  return cfg.magic == OTA_AB_MAGIC && cfg.version == OTA_AB_VERSION &&
         cfg.active_slot < OTA_AB_NUM_SLOTS && cfg.pending_slot < OTA_AB_NUM_SLOTS &&
         cfg.checksum == ota_ab_checksum_words(cfg.active_slot, cfg.pending_slot);
}

bool ota_ab_read(ota_ab_flags_t &cfg) {
  const ota_ab_flags_t *stored = (const ota_ab_flags_t *)OTA_FLAG_ADDRESS;
  cfg = *stored;
  return ota_ab_valid(cfg);
}

uint8_t ota_ab_current_slot() {
  return (SCB->VTOR >= OTA_SLOT1_START) ? 1U : 0U;
}

uint8_t ota_ab_inactive_slot() {
  return ota_ab_current_slot() ? 0U : 1U;
}

uint32_t ota_ab_slot_address(uint8_t slot) {
  return slot ? OTA_SLOT1_START : OTA_SLOT0_START;
}

uint32_t ota_ab_slot_size(uint8_t slot) {
  return slot ? OTA_SLOT1_SIZE : OTA_SLOT0_SIZE;
}

static bool ota_program_row(uint32_t address, const uint32_t *row_words) {
  // Erase the row — interrupts restored immediately after so UART/DMA can proceed.
  noInterrupts();
  NVMCTRL->CTRLB.bit.MANW = 1;
  if (!ota_wait_ready()) {
    interrupts();
    return false;
  }
  NVMCTRL->STATUS.reg = NVMCTRL_STATUS_MASK;
  NVMCTRL->ADDR.reg = address / 2;
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
  if (!ota_wait_ready()) {
    interrupts();
    return false;
  }
  interrupts();

  // Write each page.  Split into two noInterrupts phases so we can log between
  // PBC+fill and WP — this pinpoints which half hangs if we get another freeze.
  const size_t wpp = OTA_FLASH_PAGE_SIZE / 4;
  for (size_t page = 0; page < (OTA_FLASH_ROW_SIZE / OTA_FLASH_PAGE_SIZE); ++page) {
    const uint32_t page_addr = address + page * OTA_FLASH_PAGE_SIZE;

    // Phase 1 — clear page buffer and fill it.
    noInterrupts();
    NVMCTRL->CTRLB.bit.MANW = 1;
    NVMCTRL->STATUS.reg = NVMCTRL_STATUS_MASK;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
    if (!ota_wait_ready()) {
      interrupts();
      return false;
    }
    volatile uint32_t *dst = (volatile uint32_t *)page_addr;
    for (size_t i = 0; i < wpp; ++i) {
      *dst++ = row_words[page * wpp + i];
    }
    interrupts();

    // Phase 2 — commit page to flash.
    noInterrupts();
    NVMCTRL->ADDR.reg = page_addr / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
    if (!ota_wait_ready()) {
      interrupts();
      return false;
    }
    interrupts();
  }
  return true;
}

static bool ota_ab_write(const ota_ab_flags_t &in_cfg) {
  ota_ab_flags_t cfg = in_cfg;
  cfg.magic = OTA_AB_MAGIC;
  cfg.version = OTA_AB_VERSION;
  cfg.checksum = ota_ab_checksum_words(cfg.active_slot, cfg.pending_slot);

  uint32_t row[OTA_FLASH_ROW_SIZE / 4];
  for (size_t i = 0; i < (OTA_FLASH_ROW_SIZE / 4); ++i) {
    row[i] = 0xFFFFFFFFu;
  }
  memcpy(row, &cfg, sizeof(cfg));

  return ota_program_row(OTA_FLAG_ADDRESS, row);
}

bool ota_storage_read_text(uint32_t address, char *dst, size_t dst_len) {
  if (!dst || dst_len < 2 || address < OTA_FLASH_END || address >= 0x00040000UL) {
    return false;
  }

  size_t n = 0;
  bool any = false;
  const uint8_t *src = (const uint8_t *)address;
  while (n < (dst_len - 1)) {
    uint8_t c = src[n];
    if (c == 0x00 || c == 0xFF) {
      break;
    }
    dst[n++] = (char)c;
    any = true;
  }
  dst[n] = '\0';
  return any;
}

bool ota_storage_write_text(uint32_t address, size_t capacity, const char *text) {
  if (!text || !capacity || (address & (OTA_FLASH_ROW_SIZE - 1u)) != 0u) {
    return false;
  }
  if (address < OTA_FLASH_END || (address + capacity) > 0x00040000UL) {
    return false;
  }

  size_t text_len = strlen(text);
  if (text_len >= capacity) {
    text_len = capacity - 1u;
  }

  // Static to avoid 512-byte stack hit (called from deep call chains like config update).
  static uint8_t row_bytes[OTA_FLASH_ROW_SIZE];
  static uint32_t row_words[OTA_FLASH_ROW_SIZE / 4];

  for (size_t off = 0; off < capacity; off += OTA_FLASH_ROW_SIZE) {
    size_t chunk = capacity - off;
    if (chunk > OTA_FLASH_ROW_SIZE) {
      chunk = OTA_FLASH_ROW_SIZE;
    }

    memset(row_bytes, 0xFF, sizeof(row_bytes));
    if (off < text_len) {
      size_t copy_len = text_len - off;
      if (copy_len > chunk) {
        copy_len = chunk;
      }
      memcpy(row_bytes, text + off, copy_len);
    }
    memcpy(row_words, row_bytes, sizeof(row_words));
    if (!ota_program_row(address + off, row_words)) {
      log_e("SW:row failed\r\n"); log_flush();
      return false;
    }
  }

  return true;
}

bool ota_ab_request_slot(uint8_t slot) {
  if (slot >= OTA_AB_NUM_SLOTS) {
    return false;
  }

  ota_ab_flags_t cfg = {};
  cfg.active_slot = ota_ab_current_slot();
  cfg.pending_slot = slot;
  return ota_ab_write(cfg);
}

bool ota_ab_confirm_current() {
  ota_ab_flags_t cfg = {};
  cfg.active_slot = ota_ab_current_slot();
  cfg.pending_slot = cfg.active_slot;
  return ota_ab_write(cfg);
}
