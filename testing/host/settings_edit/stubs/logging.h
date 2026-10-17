// Host stub for logging.h - captures output into a buffer the test inspects.
#pragma once
#include <Arduino.h>
void log_i(const char * msg);
void log_raw(const char * msg);
void log_write(uint32_t num);
void log_flush();
