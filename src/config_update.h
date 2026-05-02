#pragma once

#include <Arduino.h>

// Config update packet structure (received from GS after OTA nonce window)
// Packet format:
//   [Type: 0x0C = Config Update] (1 byte)
//   [UUID_Suffix: 4 bytes - last 4 bytes of MCU UUID]
//   [Num_Settings: 1 byte]
//   For each setting:
//     [Key_Len: 1 byte]
//     [Key: N bytes]
//     [Value_Len: 1 byte]
//     [Value: M bytes]
//   [Signature: 64 bytes - ECDSA P-256 raw signature over all preceding bytes]

#define CONFIG_UPDATE_PKT_TYPE 0x0C
#define CONFIG_UPDATE_MAX_SETTINGS 8
#define CONFIG_UPDATE_MAX_KEY_LEN 50
#define CONFIG_UPDATE_MAX_VALUE_LEN 100
#define CONFIG_UPDATE_UUID_SUFFIX_LEN 4
#define CONFIG_UPDATE_SIGNATURE_LEN 64

// Get the last 4 bytes of the SAMD21 unique ID (for signature verification)
void config_update_get_uuid_suffix(uint8_t suffix[CONFIG_UPDATE_UUID_SUFFIX_LEN]);

// Get full 16-byte UUID as hex string (for backend registration)
String config_update_get_full_uuid_hex();

// Verify config update packet signature
// Returns true if signature is valid and UUID suffix matches this device
bool config_update_verify_packet(const uint8_t *packet, size_t packet_len);

// Parse and apply config update packet
// Returns number of settings successfully applied, or -1 on error
int config_update_apply_packet(const uint8_t *packet, size_t packet_len);

// Structure for a single config setting (used internally)
struct ConfigSetting {
  String key;
  String value;
};

// Parse config update packet into settings array
// Returns number of settings parsed, or -1 on error
int config_update_parse_packet(const uint8_t *packet, size_t packet_len,
                                 ConfigSetting *settings, size_t max_settings);
