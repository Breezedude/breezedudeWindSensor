// Host stub: the measurement fields status.cpp reports.
#pragma once
#include <Arduino.h>

typedef struct {
  float baro_pressure;
  float temperature;
  float wind_speed;
  float wind_gust;
  int   wind_heading;
  int   humidity;
  float batt_volt;
  int   batt_perc;
  uint32_t last_data;
  uint32_t last_baro_reading;
} Sensors;

extern Sensors sensor;
