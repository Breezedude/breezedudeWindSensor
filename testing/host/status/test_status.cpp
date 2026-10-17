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
static uint8_t g_resets = 0, g_reset_last = 0;
uint8_t reset_history_count() { return g_resets; }
uint8_t reset_history_last_cause() { return g_reset_last; }
static uint32_t g_awake_ms = 0, g_awake_total = 0, g_rx_ms = 0, g_rx_total = 0;
void power_duty_get(uint32_t *a, uint32_t *t) { *a = g_awake_ms; *t = g_awake_total; }
void radio_rx_duty_get(uint32_t *r, uint32_t *t) { *r = g_rx_ms; *t = g_rx_total; }
static bool g_noise = false;
static int16_t g_noise_min = 0, g_noise_avg = 0, g_noise_max = 0;
static uint32_t g_noise_n = 0;
bool noise_stats(int16_t *mn, int16_t *av, int16_t *mx, uint32_t *n) {
  if (!g_noise) return false;
  *mn = g_noise_min; *av = g_noise_avg; *mx = g_noise_max; *n = g_noise_n;
  return true;
}
static const char *g_hw = "2.4 or earlier";
const char *hw_name() { return g_hw; }

static std::string g_out;
void log_i(const char *msg) { g_out += msg; }
void log_raw(const char *msg) { g_out += msg; }
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
  char buf[896]; // same size status_print_json() uses

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
  g_awake_ms = 1234; g_awake_total = 600000;   // 2 per mille
  g_rx_ms = 0; g_rx_total = 0;                 // never measured

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
                        "lbt_cnt","slot","free_ram","resets","reset_last","hw",
                        "awake_pm","rx_pm","noise_dbm","noise_n"}) {
    if (!has_key(out, k)) { printf("  missing key: %s\n", k); g_fail++; }
  }
  check(out.find("\"vbat_mv\":3812") != std::string::npos, "voltage as integer mV");
  check(out.find("\"baro_pa\":98123") != std::string::npos, "pressure as integer Pa");
  check(out.find("\"temp_dc\":-45") != std::string::npos, "negative temperature survives");
  check(out.find("\"uptime_s\":1234") != std::string::npos, "uptime in seconds");
  check(out.find("\"reset\":\"POR\"") != std::string::npos, "reset cause decoded");
  check(out.find("\"wind_age_ms\":1200") != std::string::npos, "data age computed");
  check(out.find("\"awake_pm\":2,") != std::string::npos, "awake duty in per mille");
  check(out.find("\"rx_pm\":0,") != std::string::npos, "rx duty zero without a measurement");
  check(out.find("\"noise_dbm\":null") != std::string::npos, "noise null without samples");
  check(out.find("\"resets\":0,") != std::string::npos, "no resets after power-on");
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
  g_resets = 255; g_reset_last = 0x00;
  g_hw = "2.4 or earlier";
  g_awake_ms = 4000000000u; g_awake_total = 4000000000u;
  g_rx_ms = 4000000000u; g_rx_total = 4000000000u;
  g_noise = true; g_noise_min = -150; g_noise_avg = -100; g_noise_max = -20; g_noise_n = 4294967295u;

  n = status_build_json(buf, sizeof(buf));
  out.assign(buf);
  why.clear();
  printf("      %u bytes used of %u\n", (unsigned)n, (unsigned)sizeof(buf));
  check(n > 0, "worst case still fits the buffer");
  check(looks_like_json(out, why), "worst case is valid JSON");
  if (!why.empty()) printf("      -> %s\n", why.c_str());
  check(out.find("\\\"") != std::string::npos, "quotes in the name are escaped");
  check(out.find("\"reset\":\"unknown\"") != std::string::npos, "unknown reset cause handled");
  check(out.find("\"awake_pm\":1000,") != std::string::npos, "awake duty saturates at 1000 per mille");
  check(out.find("\"noise_dbm\":[-150,-100,-20]") != std::string::npos, "noise as a JSON array");

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
