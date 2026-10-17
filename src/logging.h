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
// The detailed running log, printed only while USB_VERBOSE is set - on the
// debug UART as well as on USB. log_write_v() continues such a line.
void log_write_v(const char * msg);
void log_write_v(uint32_t num);
void log_write_v(int32_t num);
void log_write_v(int num);
void log_write_v(float num);
void log_write_hex_v(uint32_t num, uint8_t width = 0);
// The compact running log, printed only while USB_VERBOSE is off: one line per
// wakeup and per transmission. log_write() continues such a line.
// Only written when USB is connected and verbose_usb is off - DEBUGSER
// already gets the full detail via log_write_v above.
void log_s(const char * msg);
void log_s(const char * msg, uint32_t num, const char * suffix = "");
// Every line is marked with its level: E| endangers regular operation,
// W| a fault that does not weigh heavily, I| normal, D| extra detail.
// The marker is written once per line, not once per call, so a message built
// from several log_i()/log_write() fragments still carries exactly one.
// Warnings share the errors_en gate with log_e(): both are worth having on the
// debug UART when the ordinary chatter is switched off.
// Extra detail - same reach as log_i(), only marked D| instead of I|. For
// numbers and internals: chunk counters, raw samples, sleep durations.
void log_d(const char * msg);
void log_d(const char * msg, uint32_t num);
void log_d(const char * msg, int32_t num);
void log_d(const char * msg, int num);
void log_d(const char * msg, float num);
// Machine readable payload: written without a line marker, so a DUMP stays a
// verbatim copy of the settings file. Gated like log_i().
void log_raw(const char * msg);
void log_w(const char * msg);
void log_w(const char * msg, uint32_t num);
void log_w(const char * msg, int32_t num);
void log_w(const char * msg, int num);
void log_w(const char * msg, float num);
void log_e(const char * msg);
void log_flush();
void log_ser_begin();
void log_enable_debug();
void log_disable_debug();
void log_set_debug(bool en);
bool debug_enabled();
void log_set_error(bool en);