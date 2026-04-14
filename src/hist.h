#pragma once
#include <Arduino.h>
#include "logging.h"
#include "defines.h"

extern uint32_t time();


// Gust History, for data transmission
typedef struct g{
  bool valid = false;
  uint32_t time;
  uint32_t gust;
  uint32_t wind;
  int dir_raw;
} WindSample;



// Data history
typedef struct h{
  bool set = false;
  int8_t wind;
  int8_t temp;
  int8_t humd;
  int8_t light;
  int8_t batt;
  bool pv_charging;
  bool pv_done;
} History;


extern History history[HISTORY_LEN];
extern int hist_count;
extern WindSample wind_history[WIND_HIST_LEN]; // gust ringbuffer
extern uint8_t wind_hist_pos; // current position in ringbuffer


void check_wind_hist_bin();
void add_wind_history_wind(float val_wind);
void add_wind_history_gust(float val_gust);
void add_wind_history_dir(int val_dir);
WindSample get_wind_from_hist(uint32_t age);
void insert_sorted(uint32_t* arr, int arrlen, uint32_t value);
float get_gust_from_hist(uint32_t age);
void save_history(float wind_speed, float temperature, int humidity, int light_lux, float batt_volt, bool pv_charging, bool pv_done);
uint32_t history_sum_light(int len);
uint32_t history_sum_wind(int len);
