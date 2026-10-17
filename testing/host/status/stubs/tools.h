// Host stub: the parts of tools.h that status.cpp reads.
#pragma once
#include <Arduino.h>
#include "defines.h"

#define STATION_NAME_LEN 64

typedef enum {
  s_invalid, s_WS80, s_WS85, s_WS85_UART, s_DAVIS6410, s_WINDNERD
} Sensor;

typedef struct {
  char station_name[STATION_NAME_LEN];
  Sensor sensor_type;
} Settings;

typedef struct {
  uint8_t lbt_counter;
  bool uv_triggered;
} RuntimeState;

extern Settings settings;
extern RuntimeState state;
extern bool settings_ok;

uint32_t time();
uint16_t get_fanet_id();
const char* get_bootloader_version();
uint8_t reset_history_count();
uint8_t reset_history_last_cause();
