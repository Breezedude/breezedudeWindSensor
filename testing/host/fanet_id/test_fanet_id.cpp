// Host test for the real src/fanet_id.cpp - the FANET address derived from the MCU
// UUID. An address that changes is a new station as far as the backend can tell,
// so the values are pinned here.
#include <cstdio>

#include "fanet_id.h"

static int g_fail = 0;
static void check(bool cond, const char *what) {
  printf("  %-66s %s\n", what, cond ? "ok" : "FAIL");
  if (!cond) g_fail++;
}

int main() {
  const uint8_t uuid[16] = {0x70, 0xA4, 0x5B, 0x23, 0x50, 0x30, 0x4A, 0x46,
                            0x43, 0x2E, 0x31, 0x20, 0xFF, 0x16, 0x1F, 0x15};

  printf("\nthe two ways\n");
  check(fanet_id_legacy(uuid) == 0xA470, "up to 2.4: the first two bytes, unchanged");
  check(fanet_id_from_uuid(uuid, sizeof(uuid)) == 0xEE0F, "from 2.5: CRC-16/MODBUS over all 16 bytes");

  printf("\nit is the standard checksum\n");
  const uint8_t text[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  check(fanet_id_from_uuid(text, sizeof(text)) == 0x4B37, "the check value of CRC-16/MODBUS");

  printf("\nevery byte counts\n");
  uint8_t other[16];
  for (int i = 0; i < 16; i++) other[i] = uuid[i];
  other[15] ^= 0x01;
  check(fanet_id_from_uuid(other, sizeof(other)) != 0xEE0F, "the last byte changes the address");
  check(fanet_id_legacy(other) == 0xA470, "  ... which the first two bytes would not notice");

  printf("\nthe values an address must not have\n");
  const uint8_t gives_zero[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0x91, 0xCB};
  const uint8_t gives_ones[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0xC6, 0xDE};
  check(fanet_id_from_uuid(gives_zero, 16) == 0x0001, "a checksum of 0x0000 is not used");
  check(fanet_id_from_uuid(gives_ones, 16) == 0x0001, "a checksum of 0xFFFF is not used");

  printf("\n%s\n", g_fail ? "SOME CHECKS FAILED" : "all checks passed");
  return g_fail ? 1 : 0;
}
