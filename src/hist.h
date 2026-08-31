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


extern WindSample wind_history[WIND_HIST_LEN]; // gust ringbuffer
extern uint8_t wind_hist_pos; // current position in ringbuffer


void check_wind_hist_bin();
void add_wind_history_wind(float val_wind);
void add_wind_history_gust(float val_gust);
void add_wind_history_dir(int val_dir);
WindSample get_wind_from_hist(uint32_t age);
float get_gust_from_hist(uint32_t age);
