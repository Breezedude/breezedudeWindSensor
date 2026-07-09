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
void log_v(const char * msg);
void log_v(const char * msg, uint32_t num);
void log_v(const char * msg, int32_t num);
void log_v(const char * msg, int num);
void log_v(const char * msg, float num);
void log_write(const char * msg);
void log_write(uint32_t num);
void log_write(int32_t num);
void log_write(int num);
void log_write(float num);
void log_write_hex(uint32_t num, uint8_t width = 0);
// Like log_write(), but USB output is gated on settings.verbose_usb (DEBUGSER
// output is unaffected and always written when debug_en is set).
void log_write_v(const char * msg);
void log_write_v(uint32_t num);
void log_write_v(int32_t num);
void log_write_v(int num);
void log_write_v(float num);
void log_write_hex_v(uint32_t num, uint8_t width = 0);
// Simplified status output for non-verbose USB users (e.g. "OTA: 42%").
// Only written when USB is connected and verbose_usb is off - DEBUGSER
// already gets the full detail via log_write_v above.
void log_s(const char * msg);
void log_s(const char * msg, uint32_t num, const char * suffix = "");
void log_e(const char * msg);
void log_flush();
void log_ser_begin();
void log_enable_debug();
void log_disable_debug();
void log_set_debug(bool en);
bool debug_enabled();
void log_set_error(bool en);