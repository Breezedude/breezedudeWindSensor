// Host test for pack_hwinfo() in src/fanet.cpp (copied in next to the stubs
// by run.bat so its quoted includes resolve to them).
//
// The byte layout is the contract with four decoders - groundstation
// frontend, backend, gsmap, device management - so this test pins it byte by
// byte against doc/hwinfo-protocol.md. A struct that silently gains padding
// or a field that moves shows up here before it shows up as garbage on a map.
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "fanet.h"

#include "fanet.cpp"

static int g_fail = 0;
static void check(bool cond, const char *what) {
  printf("  %-60s %s\n", what, cond ? "ok" : "FAIL");
  if (!cond) g_fail++;
}

static void dump(const uint8_t *b, size_t n) {
  printf("      ");
  for (size_t i = 0; i < n; i++) printf("%02X ", b[i]);
  printf("(%u bytes)\n", (unsigned)n);
}

static hwInfoData base() {
  hwInfoData d;
  memset(&d, 0, sizeof(d));
  d.vid = 0xBD;
  d.fanet_id = 0x1BFD;
  d.bSubtypeBuild = true;
  d.device_type = FANET_BD_DEVICE_TRANSMITTER;
  d.develop_mode = false;
  d.build_day = 17;
  d.build_month = 10;
  d.build_year = 2026;
  d.bUptime = true;
  d.uptime_min = 0x1234;
  return d;
}

// Common prefix: header, byte0, subtype, build date, uptime, debug type.
static const size_t PREFIX = 4 + 1 + 1 + 2 + 2 + 1;

static bool prefix_ok(const uint8_t *b, uint8_t debug_type) {
  // Header: type 0x0A, no forward, no ext header; vendor; address LE.
  if (b[0] != 0x0A || b[1] != 0xBD || b[2] != 0xFD || b[3] != 0x1B) return false;
  if (b[4] != 0x50) return false;                  // bit6 subtype+build, bit4 uptime
  if (b[5] != FANET_BD_DEVICE_TRANSMITTER) return false;
  // build date: day 17 | month 10 << 5 | (2026-2019) << 9 = 0x0F51
  if (b[6] != 0x51 || b[7] != 0x0F) return false;
  if (b[8] != 0x34 || b[9] != 0x12) return false;  // uptime LE
  return b[10] == debug_type;
}

int main() {
  uint8_t buf[64];

  printf("struct sizes\n");
  check(sizeof(hwinfo_debug_t1) == 10, "t1 dynamic is 10 bytes");
  check(sizeof(hwinfo_debug_t2) == 10, "t2 ota is 10 bytes");
  check(sizeof(hwinfo_debug_t3) == 7,  "t3 static is 7 bytes");
  check(sizeof(hwinfo_debug_t4) == 3,  "t4 repeater header is 3 bytes");
  check(sizeof(hwinfo_repeater_station_t) == 5, "t4 station entry is 5 bytes");

  printf("\ntype 1 dynamic\n");
  hwInfoData d = base();
  d.debug_type = HWINFO_DEBUG_DYNAMIC;
  d.debug_dynamic.vbatt_mv = 4123;
  d.debug_dynamic.batt_perc = 98;
  d.debug_dynamic.flags = 0x01 | (HWINFO_AWAKE_BELOW_5_PCT << 2);
  d.debug_dynamic.rx_count = 0x0203;
  d.debug_dynamic.forward_count = 0x0405;
  d.debug_dynamic.lbt_counter = 7;
  d.debug_dynamic.resets = 5 | (HWINFO_RESET_WDT << 5);
  size_t n = pack_hwinfo(&d, buf);
  dump(buf, n);
  check(n == PREFIX + 10, "length is prefix + 10");
  check(prefix_ok(buf, 1), "prefix as documented");
  check(buf[11] == 0x1B && buf[12] == 0x10, "vbatt_mv little-endian");
  check(buf[13] == 98, "batt_perc");
  check(buf[14] == 0x05, "flags: charging + awake class 1");
  check(buf[15] == 0x03 && buf[16] == 0x02, "rx_count little-endian");
  check(buf[17] == 0x05 && buf[18] == 0x04, "forward_count little-endian");
  check(buf[19] == 7, "lbt_counter");
  check(buf[20] == 0x25, "resets: count 5, cause WDT");

  printf("\ntype 3 static\n");
  d = base();
  d.debug_type = HWINFO_DEBUG_STATIC;
  d.debug_static.sensor_type = 4;     // DAVIS6410
  d.debug_static.use_baro = 1;
  d.debug_static.uv_triggered = 0;
  d.debug_static.lbt = 1;
  d.debug_static.lora_rssi_threshold = 104;
  d.debug_static.sensor_integ_s = 12;
  d.debug_static.reduce_interval_voltage = 141;
  d.debug_static.hw_rev = 0x01 | 0x20 | (0x02 << 6);   // 2.5 map, overruled, raw pins 10
  d.debug_static.bl_version = (4 << 12) | (2 << 6) | 1; // v4.2.1
  n = pack_hwinfo(&d, buf);
  dump(buf, n);
  check(n == PREFIX + 7, "length is prefix + 7");
  check(prefix_ok(buf, 3), "prefix as documented");
  check(buf[11] == 0xA4, "bitfield byte: sensor 4, baro, lbt");
  check(buf[12] == 104 && buf[13] == 12 && buf[14] == 141, "thresholds and integration");
  check(buf[15] == 0xA1, "hw_rev byte");
  check(buf[16] == 0x81 && buf[17] == 0x40, "bl_version 4.2.1 little-endian = 0x4081");

  printf("\ntype 4 repeater\n");
  d = base();
  d.debug_type = HWINFO_DEBUG_REPEATER;
  d.debug_repeater.flags = HWINFO_REPEATER_SCHED | 0x04 | (2 << 3);
  d.debug_repeater.rx_duty = 17;
  d.debug_repeater.forwarded = 9;
  d.repeater_station_count = 2;
  d.repeater_stations[0].vendor = 0xBD;
  d.repeater_stations[0].address = 0x1BFD;
  d.repeater_stations[0].period_s = 40;
  d.repeater_stations[0].status = 3 | (1 << 3) | (3 << 5);   // synced, 1 miss, hw synced
  d.repeater_stations[1].vendor = 0x11;
  d.repeater_stations[1].address = 0x2233;
  d.repeater_stations[1].period_s = 0;
  d.repeater_stations[1].status = 1;                         // learn
  n = pack_hwinfo(&d, buf);
  dump(buf, n);
  check(n == PREFIX + 3 + 2 * 5, "length is prefix + 3 + 5 per station");
  check(prefix_ok(buf, 4), "prefix as documented");
  check(buf[11] == 0x17, "flags: sched, scanning, 2 stations");
  check(buf[12] == 17 && buf[13] == 9, "rx_duty, forwarded");
  check(buf[14] == 0xBD && buf[15] == 0xFD && buf[16] == 0x1B, "station 0 address");
  check(buf[17] == 40 && buf[18] == 0x6B, "station 0 period and status");
  check(buf[19] == 0x11 && buf[20] == 0x33 && buf[21] == 0x22, "station 1 address");
  check(buf[22] == 0 && buf[23] == 0x01, "station 1 period unknown, learning");

  d.repeater_station_count = 0;
  n = pack_hwinfo(&d, buf);
  check(n == PREFIX + 3, "no stations: header only");

  d.repeater_station_count = 9;   // more than the array holds
  n = pack_hwinfo(&d, buf);
  check(n == PREFIX + 3 + HWINFO_REPEATER_MAX_STATIONS * 5, "station count is capped");

  printf("\n%s\n", g_fail ? "FAILURES" : "all checks passed");
  return g_fail != 0;
}
