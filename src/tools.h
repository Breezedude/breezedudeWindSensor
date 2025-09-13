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
} Sensor;

typedef struct {
String station_name ="";
float pos_lon;
float pos_lat;
float altitude = -1;
int heading_offset = 0;
uint32_t wind_age = WIND_AGE; 
uint32_t gust_age = GUST_AGE;

bool use_wdt = false; // use watchdog reset
int div_cpu_fast = 1; // fast CPU clock 1= 48Mhz/1
int div_cpu_slow = 1; // if > 1, cpu clocks down by 48/n Mhz


// Sensor selction
float reduce_interval_voltage = 3.5; // below this voltage the send inverval will be reduced to save energy
Sensor sensor_type = s_invalid;
uint32_t sensor_integration_time = 12000; // ms to sleep while counting pulses. Resolution of gust detection
bool is_baro = false;
bool is_gps = false;

bool testmode = false; // send without weather station to check lora coverage
uint32_t gps_baud = 9600;

float lora_freq = LORA_FREQ_EU;
int lora_bw = LORA_BW_EU;
int lora_sf = 7;
int lora_cr = 5;

uint32_t broadcast_interval_weather = BROADCAST_INTERVAL;
uint32_t broadcast_interval_info = 0;
uint32_t broadcast_interval_name = 1000*60*5; // 5 min

// debugging
bool test_with_usb = false;
bool no_sleep = false;
bool test_heater = false;
bool forward_serial_while_usb = false;
bool skip_lora = false; // skip lora module init
} Settings;

extern Settings settings;

extern HW_Version bd_hw_version;  // Declare the variable (extern)
extern uint32_t sleeptime_cum;
extern uint32_t sleep_offset;

extern const char* wakeup_source_string [4];

bool is_wsxx();
uint32_t time();
uint16_t get_fanet_id();
void pinDisable( uint32_t ulPin);
bool led_status(bool s);
bool led_error(bool s);
void i2c_scan();
bool apply_setting(char* settingName,  char* settingValue);
void print_settings();
bool zone_not_eu();
