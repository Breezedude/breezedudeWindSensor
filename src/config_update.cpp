#include "config_update.h"
#include "tools.h"
#include "ota_sha256.h"
#include "ota_lora.h"
#include "logging.h"
#include <ArduinoUniqueID.h>
#include <uECC.h>

#if BREEZEDUDE_ENABLE_LORA_OTA

// Backend public key for signature verification (same as OTA)
// This is the ECDSA P-256 public key corresponding to the backend's private key
extern const uint8_t kBackendPublicKey[64];

void config_update_get_uuid_suffix(uint8_t suffix[CONFIG_UPDATE_UUID_SUFFIX_LEN]) {
  // SAMD21 has 16-byte unique ID, we take the last 4 bytes
  if (UniqueIDsize >= CONFIG_UPDATE_UUID_SUFFIX_LEN) {
    for (size_t i = 0; i < CONFIG_UPDATE_UUID_SUFFIX_LEN; ++i) {
      suffix[i] = UniqueID[UniqueIDsize - CONFIG_UPDATE_UUID_SUFFIX_LEN + i];
    }
  } else {
    // Fallback if ID is shorter than expected
    memset(suffix, 0, CONFIG_UPDATE_UUID_SUFFIX_LEN);
    for (size_t i = 0; i < UniqueIDsize; ++i) {
      suffix[i] = UniqueID[i];
    }
  }
}

String config_update_get_full_uuid_hex() {
  String uuid_hex = "";
  for (size_t i = 0; i < UniqueIDsize; ++i) {
    if (UniqueID[i] < 0x10) {
      uuid_hex += "0";
    }
    uuid_hex += String(UniqueID[i], HEX);
  }
  uuid_hex.toUpperCase();
  return uuid_hex;
}

bool config_update_verify_packet(const uint8_t *packet, size_t packet_len) {
  // Minimum packet: Type(1) + UUID(4) + NumSettings(1) + Signature(64) = 70 bytes
  if (!packet || packet_len < 70) {
    log_e("Config update packet too small\r\n");
    return false;
  }

  // Check packet type
  if (packet[0] != CONFIG_UPDATE_PKT_TYPE) {
    log_e("Config update invalid type\r\n");
    return false;
  }

  // Verify UUID suffix matches this device.
  // All-zero suffix = broadcast mode (backend does not know UUID yet) -> accept.
  const uint8_t *packet_uuid_suffix = &packet[1];
  bool broadcast_uuid = true;
  for (size_t i = 0; i < CONFIG_UPDATE_UUID_SUFFIX_LEN; ++i) {
    if (packet_uuid_suffix[i] != 0x00) { broadcast_uuid = false; break; }
  }
  if (!broadcast_uuid) {
    uint8_t expected_suffix[CONFIG_UPDATE_UUID_SUFFIX_LEN];
    config_update_get_uuid_suffix(expected_suffix);
    if (memcmp(packet_uuid_suffix, expected_suffix, CONFIG_UPDATE_UUID_SUFFIX_LEN) != 0) {
      log_e("Config update UUID mismatch\r\n");
      return false;
    }
  }

  // Signature is last 64 bytes
  const size_t payload_len = packet_len - CONFIG_UPDATE_SIGNATURE_LEN;
  const uint8_t *signature = &packet[payload_len];

  // Compute SHA256 digest of payload (everything except signature)
  ota_sha256_ctx_t ctx = {};
  uint8_t digest[32] = {0};
  ota_sha256_init(&ctx);
  ota_sha256_update(&ctx, packet, payload_len);
  ota_sha256_final(&ctx, digest);

  // Verify signature using backend public key
  int verify_result = uECC_verify(kBackendPublicKey, digest, sizeof(digest), 
                                   signature, uECC_secp256r1());
  
  if (verify_result != 1) {
    log_e("Config update signature verification failed\r\n");
    return false;
  }

  log_i("Config update signature verified\r\n");
  return true;
}

int config_update_parse_packet(const uint8_t *packet, size_t packet_len,
                                 ConfigSetting *settings, size_t max_settings) {
  if (!packet || !settings || packet_len < 70) {
    return -1;
  }

  size_t pos = 1; // Skip type byte

  // Skip UUID suffix (already verified)
  pos += CONFIG_UPDATE_UUID_SUFFIX_LEN;

  // Read number of settings
  uint8_t num_settings = packet[pos++];
  if (num_settings == 0 || num_settings > max_settings) {
    log_e("Config update: invalid num_settings=");
    log_write(num_settings);
    log_write("\r\n");
    return -1;
  }

  // Parse each setting
  for (uint8_t i = 0; i < num_settings; ++i) {
    if (pos >= packet_len - CONFIG_UPDATE_SIGNATURE_LEN) {
      log_e("Config update: packet truncated at setting ");
      log_write((uint32_t)i);
      log_write("\r\n");
      return -1;
    }

    // Read key length and key
    uint8_t key_len = packet[pos++];
    if (key_len == 0 || key_len > CONFIG_UPDATE_MAX_KEY_LEN) {
      log_e("Config update: invalid key_len=");
      log_write((uint32_t)key_len);
      log_write("\r\n");
      return -1;
    }

    if (pos + key_len >= packet_len - CONFIG_UPDATE_SIGNATURE_LEN) {
      log_e("Config update: key extends beyond packet\r\n");
      return -1;
    }

    String key = "";
    for (uint8_t j = 0; j < key_len; ++j) {
      key += (char)packet[pos++];
    }

    // Read value length and value
    if (pos >= packet_len - CONFIG_UPDATE_SIGNATURE_LEN) {
      log_e("Config update: no value_len for key ");
      log_write(key.c_str());
      log_write("\r\n");
      return -1;
    }

    uint8_t value_len = packet[pos++];
    if (value_len > CONFIG_UPDATE_MAX_VALUE_LEN) {
      log_e("Config update: invalid value_len=");
      log_write((uint32_t)value_len);
      log_write("\r\n");
      return -1;
    }

    if (pos + value_len > packet_len - CONFIG_UPDATE_SIGNATURE_LEN) {
      log_e("Config update: value extends beyond packet\r\n");
      return -1;
    }

    String value = "";
    for (uint8_t j = 0; j < value_len; ++j) {
      value += (char)packet[pos++];
    }

    settings[i].key = key;
    settings[i].value = value;

    if (debug_enabled()) {
      log_i("Config update parsed: ");
      log_i(key.c_str());
      log_i(" = ");
      log_i(value.c_str());
      log_i("\r\n");
    }
  }

  return num_settings;
}

int config_update_apply_packet(const uint8_t *packet, size_t packet_len) {
  // First verify signature
  if (!config_update_verify_packet(packet, packet_len)) {
    log_e("Config update verify failed\r\n");
    return -1;
  }

  // Parse settings — static to keep off stack (String members + keys/values arrays
  // would add ~260 bytes to an already deep call chain, risking stack overflow on SAMD21).
  static ConfigSetting settings[CONFIG_UPDATE_MAX_SETTINGS];

  int num_settings = config_update_parse_packet(packet, packet_len, settings,
                                                  CONFIG_UPDATE_MAX_SETTINGS);
  if (num_settings <= 0) {
    log_e("Config update parse failed\r\n");
    return -1;
  }

  // Write all parsed settings directly to settings.txt in flash.
  // apply_setting() is intentionally NOT called: unknown keys would be rejected there,
  // and config updates may legitimately add new keys. Settings take effect on next boot.
  static const char* keys[CONFIG_UPDATE_MAX_SETTINGS];
  static const char* values[CONFIG_UPDATE_MAX_SETTINGS];

  for (int i = 0; i < num_settings; ++i) {
    log_i("Config update parsed: ");
    log_write(settings[i].key.c_str());
    log_write(" = ");
    log_write(settings[i].value.c_str());
    log_write("\r\n");
    keys[i]   = settings[i].key.c_str();
    values[i] = settings[i].value.c_str();
  }

  if (!update_multiple_settings_in_flash(keys, values, (size_t)num_settings)) {
    log_e("Config update: failed to save settings to flash\r\n");
    return -1;
  }

  log_i("Config update: settings saved to flash (");
  log_write(num_settings);
  log_write(" changes)\r\n");

  return num_settings;
}

#endif // BREEZEDUDE_ENABLE_LORA_OTA
