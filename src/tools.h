#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoUniqueID.h>
#include "defines.h"
#include "hw.h"

typedef enum {
  s_invalid,
  s_WS80,
  s_WS85,
  s_WS85_UART,
  s_DAVIS6410,
  s_WINDNERD
} Sensor;

// Room for the configured name plus the " (1234m)" altitude suffix that
// setup() appends.  A fixed array keeps the configuration off the heap: the
// station is expected to run for months without a reset.  Longer names are
// truncated with a warning (see the NAME handler in tools.cpp).
#define STATION_NAME_LEN 64

typedef struct {
  char station_name[STATION_NAME_LEN] = {0};
  float pos_lon;
  float pos_lat;
  float altitude = -1;
  int heading_offset = 0;
  uint32_t wind_age = WIND_AGE; 
  uint32_t gust_age = GUST_AGE;

  bool use_wdt = true; // use watchdog reset
  int div_cpu_fast = 1; // fast CPU clock 1= 48Mhz/1
  int div_cpu_slow = 1; // if > 1, cpu clocks down by 48/n Mhz

  // Sensor selction
  float reduce_interval_voltage = 3.38; // below this voltage the send inverval will be reduced to save energy
  Sensor sensor_type = s_invalid;
  uint32_t sensor_integration_time = 12000; // ms to sleep while counting pulses. Resolution of gust detection
  bool use_baro = false;
  bool use_gps = false;
  bool use_imu = false;

  bool testmode = false; // send without weather station to check lora coverage
  uint32_t gps_baud = 9600;

  float lora_freq = LORA_FREQ_EU;
  int lora_bw = LORA_BW_EU;
  int lora_sf = 7;
  int lora_cr = 5;
  bool lora_lbt = false;
  int  lora_rssi_threshold = -104;  // RSSI threshold in dBm for LBT (lower = more sensitive)
  bool lora_smart_rcv = false;
  bool forward_data = false;    // set forward bit on all outgoing FANET packets (relay-friendly station)
  bool repeater = false;        // dedicated repeater mode: always-on RX, forward all packets with fwd bit
  bool repeater_sched = false;  // on top of it: listen only around the packets that are expected
  uint32_t repeater_scan_interval = 3600000; // ms from one scan window to the next
  uint32_t repeater_scan_duration = 360000;  // ms a scan window lasts, 0 switches scanning off

  uint32_t broadcast_interval_weather = BROADCAST_INTERVAL;
  uint32_t broadcast_interval_name = 1000*60*5; // 5 min
  bool ota_enable = true;                      // allow OTA updates (set OTA_ENABLE=0 to disable)
  bool ota_fast = false;                        // send OTA hwinfo after 20s instead of 10min (debug)

  // debugging
  bool verbose_usb = false;
  // Let the debug UART wake the station from standby, so the console stays
  // reachable instead of only for the ~2 s after a reset. Off by default: it
  // costs standby current, and the RX line sits behind a Schottky and a 10 kOhm
  // pull-up since the phantom-supply fix, so a stray edge could wake it for
  // nothing. A station on a mast has no console attached anyway.
  bool uart_wake = false;
  bool use_protection_board = true; // if true, wait 160us after powering on sensor to allow protection board to switch on power
} Settings;

extern Settings settings;

// Volatile runtime state.  This used to live inside Settings, which mixed three
// unrelated things: values parsed from the config file, flags the firmware sets
// while running, and one-shot debug switches.  Reloading the configuration
// (settings = Settings()) silently cleared the runtime flags as a side effect.
typedef struct {
  // set by the firmware while running
  uint8_t lbt_counter = 0;      // LBT retries since boot
  bool uv_triggered = false;    // undervoltage seen; clears once the battery is full again
  bool use_pulse_counter = false; // TC pulse counter configured in setup()
  bool use_rtc_counter = false;   // RTC counter configured in setup()

  // debug switches, set over the USB console
  bool skip_lora = false;         // skip lora module init
  bool analog_test_mode = false;  // power the davis sensor and stream raw ADC values
} RuntimeState;

extern RuntimeState state;

// True once parse_file() has accepted a complete configuration.
extern bool settings_ok;
extern uint32_t sleeptime_cum;
extern volatile bool usb_detach_event;
extern volatile bool usb_ignore_detach_event;

extern const char* wakeup_source_string[];

void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode, bool en_rtc);
bool is_wsxx();

// Sensor-class predicates.  These three combinations were written out inline at
// nine different places, so adding a sensor meant finding every one of them.
//
//   sensor_uses_uart()        - sensor is powered and streams over SENSOR_UART
//   sensor_uses_raw_uart()    - ... and is read with a plain SENSOR_UART.begin()
//                               (WS85_UART goes through the WS85WindSensor driver)
//   sensor_tracks_last_data() - ... and updates sensor.last_data, so the
//                               freshness of the wind data can be checked
static inline bool sensor_uses_uart(){
  return is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD;
}
static inline bool sensor_uses_raw_uart(){
  return is_wsxx() || settings.sensor_type == s_WINDNERD;
}
static inline bool sensor_tracks_last_data(){
  return is_wsxx() || settings.sensor_type == s_WS85_UART;
}
bool process_line(char * in, int len, bool (*cb)(char*, char*));
// Clocks a stuck I2C slave free before Wire.begin(); true when it had to.
bool i2c_bus_recover();
// Resets since power-on, kept across resets in .noinit RAM. Call at boot.
void reset_history_note();
uint8_t reset_history_count();
uint8_t reset_history_last_cause();
const char *reset_history_text();
// The byte hwinfo_debug_t1::resets carries: count mod 32, last cause as HWINFO_RESET_*.
uint8_t reset_history_code();
uint8_t reset_cause_code(uint8_t rcause);
// Bootloader version packed as hwinfo_debug_t3::bl_version, 0 when unknown.
uint16_t get_bootloader_version_code();
uint32_t time();

// Wrap-safe deadline test.  time() is a free-running uint32_t millisecond
// counter that wraps every ~49.7 days, so an absolute comparison such as
// (time() > deadline) flips to the wrong answer around the wrap.  Comparing
// the *signed* difference stays correct as long as the deadline is less than
// ~24.8 days away, which holds for every deadline in this firmware.
static inline bool time_reached(uint32_t deadline){
  return (int32_t)(time() - deadline) >= 0;
}
uint16_t get_fanet_id();
void pinDisable( uint32_t ulPin);
bool led_status(bool s);
bool led_error(bool s);
void i2c_scan();
bool apply_setting(char* settingName,  char* settingValue);
// True for a known setting that may be written to settings.txt. Console
// commands (REBOOT, FORMAT, ...) return false: persisting one would make
// parse_file() replay it on every boot.
bool setting_key_persistable(const char *key);
void print_settings();
// Replaces settings.txt in flash with text, watchdog-guarded.
bool write_settings_flash(const char *text);
bool update_settings_in_flash(const char* key, const char* value);
bool update_multiple_settings_in_flash(const char** keys, const char** values, size_t count);
bool zone_not_eu();
float parse_decimal_fast(const char *s);
const char* get_bootloader_version();
bool create_versionfile(const char *filename);
void handle_usb_link_watchdog();
bool parse_file(char * filename);
void read_serial_cmd();
// True while a debug UART command line has started but not ended yet.
bool console_line_pending();
void log_v_hex_dump(const uint8_t *data, size_t len);
void log_reset_cause();
bool format_flash();
bool setup_flash();
