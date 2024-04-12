#ifndef SLEEP_H
#define SLEEP_H

#include <Arduino.h>

void pinDisable(uint32_t pin);
void configGCLK6(bool en_rtc);
void sleep(bool light);

int wdt_enable(int maxPeriodMS, bool isForSleep);
uint32_t rtc_sleep_cfg(uint32_t milliseconds);
void wdt_disable();
void wdt_reset();
bool set_cpu_div(int divisor);

void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode, bool en_rtc);
void setup_PM(bool en_counter);

void setup_pulse_counter();
uint32_t read_pulse_counter();
void reset_pulse_counter();
void stop_pulse_counter();

void setup_rtc_time_counter();
uint32_t read_time_counter();
void reset_time_counter();

#endif