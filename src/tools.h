#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoUniqueID.h>
#include "defines.h"

typedef enum {
  HW_unknown,
  HW_1_3,
  HW_2_0
} HW_Version;

typedef enum {
  s_invalid,
  s_WS80,
  s_WS85,
  s_WS85_UART,
  s_DAVIS6410,
  s_WINDNERD
} Sensor;

typedef struct {
  String station_name ="";
  float pos_lon;
  float pos_lat;
  float altitude = -1;
  int heading_offset = 0;
  uint32_t wind_age = WIND_AGE; 
  uint32_t gust_age = GUST_AGE;

  bool use_wdt = true; // use watchdog reset
  int div_cpu_fast = 1; // fast CPU clock 1= 48Mhz/1
  int div_cpu_slow = 1; // if > 1, cpu clocks down by 48/n Mhz

  bool use_pulse_counter;
  bool use_rtc_counter;

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
  uint8_t lbt_counter=0; // 8bit counter for LBT retries, resets on boot
  int  lora_rssi_threshold = -104;  // RSSI threshold in dBm for LBT (lower = more sensitive)
  bool lora_smart_rcv = false;

  uint32_t broadcast_interval_weather = BROADCAST_INTERVAL;
  uint32_t broadcast_interval_info = 60*60*1000; // 1h
  uint32_t broadcast_interval_name = 1000*60*5; // 5 min
  bool ota_enable = true;                      // allow OTA updates (set OTA_ENABLE=0 to disable)
  bool ota_fast = false;                        // send OTA hwinfo after 20s instead of 10min (debug)

  // debug states
  bool uv_triggered = false; // flag to indicate if undervoltage has been triggered, resets when voltage is at 100% again

  // debugging
  bool verbose_usb = false;
  bool skip_lora = false; // skip lora module init
  bool analog_test_mode = false; // for davis sensor testing, set to 1 to power on sensor and output adc values, 0 to power off
} Settings;

extern Settings settings;
extern uint32_t sleeptime_cum;
extern volatile bool usb_detach_event;
extern volatile bool usb_ignore_detach_event;

extern const char* wakeup_source_string[];

void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode, bool en_rtc);
bool is_wsxx();
bool process_line(char * in, int len, bool (*cb)(char*, char*));
uint32_t time();
uint16_t get_fanet_id();
void pinDisable( uint32_t ulPin);
bool led_status(bool s);
bool led_error(bool s);
void i2c_scan();
bool apply_setting(char* settingName,  char* settingValue);
void print_settings();
bool update_settings_in_flash(const char* key, const char* value);
bool update_multiple_settings_in_flash(const char** keys, const char** values, size_t count);
bool zone_not_eu();
float parse_decimal_fast(const char *s);
String get_bootloader_version();
bool create_versionfile(const char *filename);
void handle_usb_link_watchdog();
bool parse_file(char * filename);
void read_serial_cmd();
void log_v_hex_dump(const uint8_t *data, size_t len);
void log_reset_cause();
bool format_flash();
bool setup_flash();
