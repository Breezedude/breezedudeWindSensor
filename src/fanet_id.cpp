#include "fanet_id.h"

uint16_t fanet_id_legacy(const uint8_t *uuid) {
  return (uint16_t)(uuid[0] + (uuid[1] << 8));
}

uint16_t fanet_id_from_uuid(const uint8_t *uuid, size_t len) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= uuid[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    }
  }
  return (crc == 0x0000u || crc == 0xFFFFu) ? 0x0001u : crc;
}
