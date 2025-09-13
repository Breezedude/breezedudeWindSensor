#pragma once
#include <Arduino.h>
#include "display.h"


extern Uart Serial2;
#define DEBUGSER Serial2
extern int div_cpu;
extern bool usb_connected;

void log_i(const char * msg);
void log_i(const char * msg, uint32_t num);
void log_i(const char * msg, int32_t num);
void log_i(const char * msg, int num);
void log_i(const char * msg, float num);
void log_e(const char * msg);
void log_flush();
void log_ser_begin();
void log_enable_debug();
void log_disable_debug();
void log_set_debug(bool en);
bool debug_enabled();
void log_set_error(bool en);