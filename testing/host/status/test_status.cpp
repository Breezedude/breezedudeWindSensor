// Host test for the real src/status.cpp (copied in next to the stubs by
// run.bat so its quoted includes resolve to them).
//
// The point of this test is the buffer: STATUS is the contract a test rig and
// the web tooling parse, and a truncated JSON document is worse than none.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <string>

#include "tools.h"
#include "sensors.h"
#include "power.h"
#include "ota_ab.h"
#include "logging.h"

// ---- stub state ----
Settings settings;
RuntimeState state;
bool settings_ok = true;
Sensors sensor;
bool undervoltage = false;
bool deep_undervoltage = false;
bool reduced_interval = false;
LORA_MODULE lora_module = LORA_LLCC68;
CHIP_BARO chip_baro = BARO_HP203B;

static Pm pm_instance;
Pm *PM = &pm_instance;

static uint32_t g_now = 1234567;
uint32_t time() { return g_now; }
uint16_t get_fanet_id() { return 0x1234; }

static const char *g_bl = "v4.1.5-0-gd7161e0";
const char *get_bootloader_version() { return g_bl; }
uint8_t ota_ab_current_slot() { return 1; }

static std::string g_out;
void log_i(const char *msg) { g_out += msg; }
void log_flush() {}

// newlib's sbrk does not exist on the host; free_ram() only needs it to return
// something stable so the value lands in the document.
static char fake_heap[4096];
extern "C" char *sbrk(int incr) { (void)incr; return fake_heap; }

#include "status.cpp"

// ---- minimal JSON validity check ----
// Not a full parser: verifies balanced braces, quoted keys, and that no value
// got cut off mid-token.
static bool looks_like_json(const std::string &s, std::string &why) {
  if (s.size() < 2 || s.front() != '{' || s.back() != '}') {
    why = "not brace delimited";
    return false;
  }
  int depth = 0;
  bool in_str = false, escaped = false;
  for (char c : s) {
    if (escaped) { escaped = false; continue; }
    if (in_str) {
      if (c == '\\') escaped = true;
      else if (c == '"') in_str = false;
      continue;
    }
    if (c == '"') in_str = true;
    else if (c == '{') depth++;
    else if (c == '}') depth--;
    if (depth < 0) { why = "unbalanced braces"; return false; }
  }
  if (in_str) { why = "unterminated string"; return false; }
  if (depth != 0) { why = "unbalanced braces"; return false; }
  if (s.find(",,") != std::string::npos) { why = "empty element"; return false; }
  return true;
}

static bool has_key(const std::string &s, const char *key) {
  return s.find(std::string("\"") + key + "\":") != std::string::npos;
}

static int g_fail = 0;
static void check(bool cond, const char *what) {
  printf("  %-56s %s\n", what, cond ? "ok" : "FAIL");
  if (!cond) g_fail++;
}

int main() {
  char buf[768]; // same size status_print_json() uses

  printf("typical device\n");
  snprintf(settings.station_name, sizeof(settings.station_name), "Teststation (456m)");
  settings.sensor_type = s_WS85_UART;
  sensor.batt_volt = 3.812f;
  sensor.batt_perc = 78;
  sensor.baro_pressure = 981.23f;
  sensor.temperature = -4.5f;
  sensor.wind_speed = 12.3f;
  sensor.wind_gust = 18.7f;
  sensor.wind_heading = 271;
  sensor.humidity = 62;
  sensor.last_data = g_now - 1200;
  sensor.last_baro_reading = g_now - 800;
  state.lbt_counter = 3;
  PM->RCAUSE.reg = PM_RCAUSE_POR;

  size_t n = status_build_json(buf, sizeof(buf));
  std::string out(buf);
  std::string why;
  check(n > 0, "fits the buffer");
  check(looks_like_json(out, why), "is valid JSON");
  if (!why.empty()) printf("      -> %s\n", why.c_str());
  for (const char *k : {"fw","build","bl","id","name","sensor","lora","baro",
                        "settings_ok","uptime_s","rcause","reset","vbat_mv","soc",
                        "uv","deep_uv","uv_seen","reduced","wind_age_ms","baro_age_ms",
                        "wind_dkmh","gust_dkmh","dir","temp_dc","hum","baro_pa",
                        "lbt_cnt","slot","free_ram"}) {
    if (!has_key(out, k)) { printf("  missing key: %s\n", k); g_fail++; }
  }
  check(out.find("\"vbat_mv\":3812") != std::string::npos, "voltage as integer mV");
  check(out.find("\"baro_pa\":98123") != std::string::npos, "pressure as integer Pa");
  check(out.find("\"temp_dc\":-45") != std::string::npos, "negative temperature survives");
  check(out.find("\"uptime_s\":1234") != std::string::npos, "uptime in seconds");
  check(out.find("\"reset\":\"POR\"") != std::string::npos, "reset cause decoded");
  check(out.find("\"wind_age_ms\":1200") != std::string::npos, "data age computed");
  printf("      %u bytes used of %u\n", (unsigned)n, (unsigned)sizeof(buf));

  printf("\nworst case lengths\n");
  // Longest name the firmware accepts, every character needing an escape, plus
  // the longest bootloader string get_bootloader_version() can return.
  memset(settings.station_name, '"', sizeof(settings.station_name) - 1);
  settings.station_name[sizeof(settings.station_name) - 1] = '\0';
  g_bl = "0123456789012345678901234567890123456";  // 37 chars, buffer is 40
  sensor.batt_volt = 4.199f;
  sensor.baro_pressure = 1099.99f;
  sensor.temperature = -99.9f;
  sensor.wind_speed = 999.9f;
  sensor.wind_gust = 999.9f;
  sensor.wind_heading = 359;
  sensor.humidity = 100;
  g_now = 4294967295u;             // just before the wrap
  sensor.last_data = 1;
  sensor.last_baro_reading = 1;
  state.lbt_counter = 255;
  PM->RCAUSE.reg = 0x00;           // unknown

  n = status_build_json(buf, sizeof(buf));
  out.assign(buf);
  why.clear();
  printf("      %u bytes used of %u\n", (unsigned)n, (unsigned)sizeof(buf));
  check(n > 0, "worst case still fits the buffer");
  check(looks_like_json(out, why), "worst case is valid JSON");
  if (!why.empty()) printf("      -> %s\n", why.c_str());
  check(out.find("\\\"") != std::string::npos, "quotes in the name are escaped");
  check(out.find("\"reset\":\"unknown\"") != std::string::npos, "unknown reset cause handled");

  printf("\ntruncation behaviour\n");
  char tiny[64];
  n = status_build_json(tiny, sizeof(tiny));
  out.assign(tiny);
  why.clear();
  check(n == 0, "reports failure when it does not fit");
  check(looks_like_json(out, why), "  ... and still emits parseable JSON");
  check(out.find("error") != std::string::npos, "  ... carrying an error field");

  n = status_build_json(nullptr, 100);
  check(n == 0, "null buffer refused");

  printf("\ncontrol characters\n");
  snprintf(settings.station_name, sizeof(settings.station_name), "Bad\tName");
  settings.station_name[3] = '\n';   // a raw newline would break line framing
  n = status_build_json(buf, sizeof(buf));
  out.assign(buf);
  check(out.find('\n') == std::string::npos, "newline stripped from the name");
  why.clear();
  check(looks_like_json(out, why), "  ... document still valid");

  printf("\n%s\n", g_fail ? "FAILURES" : "all checks passed");
  return g_fail != 0;
}
