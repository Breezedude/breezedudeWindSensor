#include "tools.h"
#include "defines.h"
#include "logging.h"
#include "hist.h"
#include "ws85uart.h"

#include <Wire.h>

#ifndef BREEZEDUDE_BARO_BMP280
  #define BREEZEDUDE_BARO_BMP280 0
#endif
#ifndef BREEZEDUDE_BARO_SPL06
  #define BREEZEDUDE_BARO_SPL06 1
#endif
#ifndef BREEZEDUDE_BARO_BMP3XX
  #define BREEZEDUDE_BARO_BMP3XX 0
#endif
#ifndef BREEZEDUDE_BARO_HP203B
  #define BREEZEDUDE_BARO_HP203B 1
#endif
#ifndef BREEZEDUDE_ENABLE_GPS
  #define BREEZEDUDE_ENABLE_GPS 1
#endif

#if BREEZEDUDE_BARO_SPL06
#include <SPL06.h>
#endif
#if BREEZEDUDE_BARO_BMP3XX
#include <Adafruit_BMP3XX.h>
#endif
#if BREEZEDUDE_BARO_BMP280
#include <BMP280.h>
#endif
#if BREEZEDUDE_BARO_HP203B
#include <HP203B.h> // https://github.com/ncdcommunity/Arduino_Library_HP203B_Barometer_Altimeter_Sensor/tree/master
#endif
#include <qmc5883p.h>
#if BREEZEDUDE_ENABLE_GPS
#include <TinyGPS++.h>
#endif

extern WS85WindSensor ws85uart;
#if BREEZEDUDE_ENABLE_GPS
extern TinyGPSPlus tinyGps;
#endif

// I2C Barometer
#if BREEZEDUDE_BARO_BMP280
extern BMP280 bmp280; // Bosch BMP280
#endif
#if BREEZEDUDE_BARO_SPL06
extern SPL06 spl; // Goertek SPL06-001
#endif
#if BREEZEDUDE_BARO_HP203B
extern HP203B hp; // HP203B 0x76 or 0x77
#endif
#if BREEZEDUDE_BARO_BMP3XX
extern Adafruit_BMP3XX bmp3xx;
#endif
extern QMC5883P qmc5883p; // GY-271



extern CHIP_BARO chip_baro;
extern CHIP_IMU chip_imu;


typedef struct {

    // Measurement values
    float baro_pressure = 0;
    float baro_temp = 0; // temp from bmp280, inside case/on pcb
    int wind_dir_raw = 0;
    int wind_heading = 0;
    float wind_speed = 0;
    float wind_gust = 0;
    float temperature = 0;
    int humidity = 0;
    int light_lux = 0;
    float uv_level = 0;
    float wsxx_vcc = 0;
    float cap_voltage = 0; // WS85 supercap voltage
    float batt_volt = 0;
    int batt_perc = 0;

    // Timing
    uint32_t next_baro_reading = 0;
    uint32_t last_baro_reading = 0;
    uint32_t last_data = 0;


    // GPS
    uint32_t last_gps_valid = 0;
    float gps_lat = 0;
    float gps_lon = 0;
    float gps_alt = 0;

    // PV
    bool pv_charging = false; // currently charging, state from pv charger
    bool pv_done = false; // battery fully charged, state from pv charger

} Sensors;

extern Sensors sensor;



bool init_imu();
void get_imu();
void switch_sensor_power(bool state);
// Whether the sensor currently has supply. Arming SERCOM0 for a UART sensor
// without one is worse than pointless - see switch_sensor_power().
bool sensor_power_is_on();
// Test override for the sensor supply: -1 automatic, 0 forced off, 1 forced on.
// Runtime only - it is deliberately not stored, so a reset undoes it.
void set_sensor_power_override(int8_t mode);
int8_t get_sensor_power_override();
void read_batt_perc();
// One sample of the battery divider on a given ADC pin, volts; the spread of
// the seven readings in *spread, and in *clipped whether any of them sat at
// full scale. For hw_detect(), which has to find the pin.
float batt_sample(uint8_t pin, float *spread, bool *clipped);
void battery_update_power_state(float v);
uint8_t battery_soc_percent(float v);
float position_lat();
float position_lon();
float position_alt();
void baro_start_reading();
void forward_analog_test_serial();
bool set_value(char* key,  char* value);
int read_wsxx();
// How long the station may sleep without listening to a WS80/WS85 text
// sensor after a complete block: until shortly before the next block is due,
// from the block period measured by read_wsxx(). 0 when that is already now.
uint32_t wsxx_sleep_before_next_block_ms();
int read_ws85_uart();

bool init_baro();
void read_baro();
// True while a barometer conversion runs and its result is not due yet.
bool baro_conversion_running();
// A pressure reading from this or the previous send cycle is in hand. What the
// weather packet and the log go by - never settings.use_baro alone.
bool baro_fresh();
// "HP203B", "HP203B, no reading", "not found, retrying", "off": for STATUS,
// the boot log and print_settings().
const char *baro_state_text();

int read_windnerd();
int read_wind_dir();
// Raw direction count to degrees, without the heading offset.
int dir_raw_to_degrees(int d);
// Continues an open compact log line with what the last sensor window brought.
void log_sensor_window_fragment();

void calc_pulse_sensor(uint32_t pulses, uint32_t dmillis);
void get_solar_charger_state();
void read_gps();