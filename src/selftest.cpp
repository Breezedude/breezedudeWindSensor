#include "selftest.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "config_update.h"
#include "defines.h"
#include "hw.h"
#include "logging.h"
#include "power.h"
#include "radio.h"
#include "sensors.h"
#include "sleep.h"
#include "status.h"
#include "tools.h"

extern CHIP_BARO chip_baro;
extern LORA_MODULE lora_module;
void read_serial_cmd();

// What counts as a working board. A pressure for anywhere up to 3000 m, and a
// temperature for a room. A battery below 3.2 V is not broken, but it should not
// leave the bench like that. While it charges it reads above its open voltage.
#define ST_BARO_MIN_PA     70000L
#define ST_BARO_MAX_PA    110000L
#define ST_TEMP_MIN_DC      -100L
#define ST_TEMP_MAX_DC       600L
#define ST_BATT_MIN_MV      3200L
#define ST_BATT_MAX_MV      4350L
// With no antenna the chip reads its own noise floor, around -120 dBm. Above the
// upper limit something on the board or next to it drowns out every station.
#define ST_NOISE_MIN_DBM    -135
#define ST_NOISE_MAX_DBM     -70
// A packet of this size is on air for more than 20 ms. TX done much sooner means
// the flag did not come from a transmission.
#define ST_TX_MIN_MS           5u
#define ST_TX_TIMEOUT_MS    2000u
// The sensor sends about every 9 s, and needs a moment after switching on.
#define ST_WS85_TIMEOUT_MS 25000u
// How long the host has to be gone before the station restarts.
#define ST_HOST_GONE_MS     3000u
// Without a host the test was started on the debug UART and nothing ends it.
#define ST_HOLD_MAX_MS    600000u
// The page repeats its request until it sees the first line. One that was still
// waiting in the buffer while the test ran is not a request for another test.
#define ST_REPEAT_GUARD_MS  2000u

enum StState : uint8_t { ST_IDLE, ST_PENDING, ST_HOLD };
static StState st_state = ST_IDLE;
static bool st_with_ws85 = true;
static bool st_had_host = false;
static uint8_t st_failed = 0;
static uint32_t st_end_ms = 0;
static uint32_t st_host_seen_ms = 0;

static void st_print(const char *json){
  log_raw("ST ");
  log_raw(json);
  log_raw("\r\n");
  log_flush();
}

static void st_result(bool ok, const char *json){
  if(!ok && st_failed < 255){ st_failed++; }
  st_print(json);
}

static void st_blink(){
  static int8_t shown = -1;
  const int8_t phase = (int8_t)((millis() / 200u) & 1u);
  if(phase == shown){ return; }
  shown = phase;
  led_status(phase != 0);
  led_error(phase == 0);
}

// Everything that waits goes through here, so that the LEDs keep blinking and USB
// keeps being served. delay() is what runs the TinyUSB task.
static void st_wait(uint32_t ms){
  const uint32_t start = millis();
  do {
    st_blink();
    handle_usb_link_watchdog();
    delay(ms < 5u ? ms : 5u);
  } while((millis() - start) < ms);
}

static void st_show_verdict(bool passed){
  led_status(passed);
  led_error(!passed);
}

static void test_hw(){
  const bool ok = !hw_id_overruled() && !hw_id_unknown();
  char line[80];
  snprintf(line, sizeof(line), "{\"t\":\"hw\",\"ok\":%d,\"name\":\"%s\",\"code\":%u}",
           ok ? 1 : 0, hw_name(), (unsigned)hw_id_code());
  st_result(ok, line);
}

// With the solder jumper closed USB feeds the charger like a solar panel in full
// sun. Its pins are read often, because a charger that is just starting up or
// topping off does not hold them still.
static void test_charge(){
  uint8_t charging = 0, done = 0;
  const uint8_t samples = 15;
  for(uint8_t i = 0; i < samples; i++){
    get_solar_charger_state();
    if(sensor.pv_charging){ charging++; }
    if(sensor.pv_done){ done++; }
    st_wait(100);
  }
  const bool ok = charging || done;
  char line[80];
  snprintf(line, sizeof(line), "{\"t\":\"charge\",\"ok\":%d,\"charging\":%u,\"done\":%u,\"n\":%u}",
           ok ? 1 : 0, (unsigned)charging, (unsigned)done, (unsigned)samples);
  st_result(ok, line);
}

static void test_batt(){
  // The value is an average over the last five readings, and read_batt_perc()
  // takes one every 100 ms at most.
  for(uint8_t i = 0; i < 5; i++){
    read_batt_perc();
    st_wait(120);
  }
  const long mv = (long)(sensor.batt_volt * 1000.0f + 0.5f);
  const bool ok = mv >= ST_BATT_MIN_MV && mv <= ST_BATT_MAX_MV;
  char line[64];
  snprintf(line, sizeof(line), "{\"t\":\"batt\",\"ok\":%d,\"mv\":%ld,\"soc\":%d}",
           ok ? 1 : 0, mv, sensor.batt_perc);
  st_result(ok, line);
}

static void test_baro(){
  settings.use_baro = true;
  sensor.last_baro_reading = 0;
  bool have_reading = false;
  if(init_baro()){
    // read_baro() starts the next conversion itself once it has a reading.
    baro_start_reading();
    for(uint8_t i = 0; i < 5 && !have_reading; i++){
      st_wait(80);
      read_baro();
      have_reading = sensor.last_baro_reading != 0;
    }
  }
  const long p_pa = (long)(sensor.baro_pressure * 100.0f);
  const long t_dc = (long)(sensor.baro_temp * 10.0f);
  const bool ok = have_reading &&
                  p_pa >= ST_BARO_MIN_PA && p_pa <= ST_BARO_MAX_PA &&
                  t_dc >= ST_TEMP_MIN_DC && t_dc <= ST_TEMP_MAX_DC;
  char line[96];
  snprintf(line, sizeof(line), "{\"t\":\"baro\",\"ok\":%d,\"chip\":\"%s\",\"read\":%d,\"p_pa\":%ld,\"t_dc\":%ld}",
           ok ? 1 : 0, baro_name(), have_reading ? 1 : 0, p_pa, t_dc);
  st_result(ok, line);
}

static void test_lora_tx(){
  // Nobody should act on this packet. A type nothing uses, and without the forward
  // bit: as a name or a weather packet it would end up on the map.
  uint8_t pkt[4 + 8] = {0x3E, FANET_VENDOR_ID};
  const uint16_t id = get_fanet_id();
  pkt[2] = (uint8_t)(id & 0xFFu);
  pkt[3] = (uint8_t)(id >> 8);
  memcpy(pkt + 4, "SELFTEST", 8);

  dis_rx_sleep();
  transmittedFlag = false;
  radio_phy->setPacketSentAction(set_fanet_send_flag);
  const uint32_t start = millis();
  const int code = radio_phy->startTransmit(pkt, sizeof(pkt));
  while(code == RADIOLIB_ERR_NONE && !transmittedFlag && (millis() - start) < ST_TX_TIMEOUT_MS){
    st_wait(1);
  }
  const uint32_t ms = millis() - start;
  const bool done = transmittedFlag;
  if(code == RADIOLIB_ERR_NONE){ radio_phy->finishTransmit(); }
  transmittedFlag = false;

  const bool ok = code == RADIOLIB_ERR_NONE && done && ms >= ST_TX_MIN_MS;
  char line[64];
  snprintf(line, sizeof(line), "{\"t\":\"lora_tx\",\"ok\":%d,\"code\":%d,\"done\":%d,\"ms\":%lu}",
           ok ? 1 : 0, code, done ? 1 : 0, (unsigned long)ms);
  st_result(ok, line);
}

static void test_lora_rx(){
  en_rx_sleep();
  st_wait(30);
  char line[80];
  if(lora_module == LORA_SX1262 || lora_module == LORA_LLCC68){
    const uint8_t samples = 16;
    int32_t sum = 0;
    int16_t lo = 0, hi = 0;
    for(uint8_t i = 0; i < samples; i++){
      const int16_t rssi = (int16_t)static_cast<SX126x*>(radio_phy)->getRSSI(false);
      if(!i || rssi < lo){ lo = rssi; }
      if(!i || rssi > hi){ hi = rssi; }
      sum += rssi;
      st_wait(15);
    }
    const int avg = (int)(sum / samples);
    const bool ok = avg >= ST_NOISE_MIN_DBM && avg <= ST_NOISE_MAX_DBM;
    snprintf(line, sizeof(line), "{\"t\":\"lora_rx\",\"ok\":%d,\"min\":%d,\"avg\":%d,\"max\":%d}",
             ok ? 1 : 0, (int)lo, avg, (int)hi);
    st_result(ok, line);
  } else {
    // The SX1276 of the alpha boards has no reading of the channel in this firmware.
    st_result(true, "{\"t\":\"lora_rx\",\"ok\":1,\"none\":1}");
  }
  dis_rx_sleep();
}

static void test_lora(){
  if(!radio_phy){ radio_init(); }
  const bool found = radio_phy != nullptr;
  char line[64];
  snprintf(line, sizeof(line), "{\"t\":\"lora_spi\",\"ok\":%d,\"module\":\"%s\"}",
           found ? 1 : 0, lora_module_name());
  st_result(found, line);
  if(!found){
    st_result(false, "{\"t\":\"lora_tx\",\"ok\":0,\"none\":1}");
    st_result(false, "{\"t\":\"lora_rx\",\"ok\":0,\"none\":1}");
    return;
  }
  test_lora_tx();
  test_lora_rx();
}

static void test_ws85(){
  const auto type = settings.sensor_type;
  settings.sensor_type = s_WS85_UART;
  set_sensor_power_override(1);
  ws85uart.begin(1);
  ws85uart.set_baud_115200();

  const uint32_t start = millis();
  bool ok = false;
  while(!ok && (millis() - start) < ST_WS85_TIMEOUT_MS){
    st_blink();
    handle_usb_link_watchdog();
    ok = read_ws85_uart() == RESP_COMPLETE;
    yield();
  }
  const uint32_t ms = millis() - start;

  set_sensor_power_override(0);
  set_sensor_power_override(-1);
  settings.sensor_type = type;

  char line[72];
  snprintf(line, sizeof(line), "{\"t\":\"ws85\",\"ok\":%d,\"ms\":%lu,\"t_dc\":%ld}",
           ok ? 1 : 0, (unsigned long)ms, ok ? (long)(sensor.temperature * 10.0f) : 0L);
  st_result(ok, line);
}

static void selftest_run(){
  // The watchdog stays off: the page notices a board that hangs, and the test is
  // over at the next REBOOT or when the cable comes out. Nothing below has to fit
  // into its period, and a reset in the middle would read as a passed step.
  wdt_disable();
  settings.use_wdt = false;
  st_failed = 0;
  st_had_host = usb_host_connected();
  st_host_seen_ms = millis();

  String uuid = config_update_get_full_uuid_hex();
  char line[200];
  snprintf(line, sizeof(line),
           "{\"begin\":1,\"fw\":\"%s\",\"bl\":\"%s\",\"hw\":\"%s\",\"id\":\"%02X%04X\",\"uuid\":\"%s\",\"settings_ok\":%d,\"ws85\":%d}",
           VERSION, get_bootloader_version(), hw_name(), FANET_VENDOR_ID, get_fanet_id(),
           uuid.c_str(), settings_ok ? 1 : 0, st_with_ws85 ? 1 : 0);
  st_print(line);

  test_hw();
  test_charge();
  test_batt();
  test_baro();
  test_lora();
  if(st_with_ws85){ test_ws85(); }

  const bool passed = st_failed == 0;
  st_show_verdict(passed);
  snprintf(line, sizeof(line), "{\"end\":1,\"ok\":%d,\"failed\":%u}", passed ? 1 : 0, (unsigned)st_failed);
  st_print(line);
  st_end_ms = millis();
}

void selftest_request(const char *arg){
  if(strcmp(arg, "FAIL") == 0){
    if(st_state == ST_HOLD){ st_show_verdict(false); }
    return;
  }
  if(st_state == ST_PENDING){ return; }
  if(st_state == ST_HOLD && (millis() - st_end_ms) < ST_REPEAT_GUARD_MS){ return; }
  st_with_ws85 = strcmp(arg, "NOWS85") != 0;
  st_state = ST_PENDING;
}

bool selftest_poll(){
  if(st_state == ST_IDLE){ return false; }
  if(st_state == ST_PENDING){
    st_state = ST_HOLD;
    selftest_run();
  }
  read_serial_cmd();
  handle_usb_link_watchdog();
  const uint32_t now = millis();
  if(usb_host_connected()){
    st_host_seen_ms = now;
  }
  const bool over = st_had_host ? (now - st_host_seen_ms) >= ST_HOST_GONE_MS
                                : (now - st_end_ms) >= ST_HOLD_MAX_MS;
  if(over){
    NVIC_SystemReset();
  }
  return true;
}
