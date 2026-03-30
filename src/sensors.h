#include "tools.h"
#include "defines.h"
#include "logging.h"
#include "hist.h"
#include "ws85uart.h"

#include <Wire.h>
#include <SPL06.h>
#include <Adafruit_BMP3XX.h>
#include <BMP280.h>
#include <HP203B.h> // https://github.com/ncdcommunity/Arduino_Library_HP203B_Barometer_Altimeter_Sensor/tree/master
#include <qmc5883p.h>
#include <TinyGPS++.h>

extern WS85WindSensor ws85uart;
extern TinyGPSPlus tinyGps;


// I2C Barometer
extern BMP280 bmp280; // Bosch BMP280
extern SPL06 spl; // Goertek SPL06-001
extern HP203B hp; // HP203B 0x76 or 0x77
extern Adafruit_BMP3XX bmp3xx;
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

    // PV
    bool pv_charging = false; // currently charging, state from pv charger
    bool pv_done = false; // battery fully charged, state from pv charger

} Sensors;

extern Sensors sensor;



bool init_imu();
void get_imu();
void baro_start_reading();
void forward_sensor_serial();
bool set_value(char* key,  char* value);
bool parse_wsdat(char* input, int len);
int read_wsxx();
int read_ws85_uart();

bool init_baro();
void read_baro();

int read_windnerd();
int read_wind_dir();

void calc_pulse_sensor(uint32_t pulses, uint32_t dmillis);
void get_solar_charger_state();
void read_gps();