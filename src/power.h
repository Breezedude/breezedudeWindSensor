#pragma once
#include <Arduino.h>

// Deep sleep, wakeup and the undervoltage/solar recovery behaviour.
// Split out of main.cpp, which mixed power management, TX scheduling and the
// Arduino entry points in one 1100 line file.

#define WDT_PERIOD 2500 // ms for wdt to wait to reset mcu if not reset in time

extern int div_cpu;                 // current CPU clock divider
extern bool first_sleep;            // first sleep after reset, USB peripheral could still be on
extern uint32_t sleep_allowed;      // time() from which entering deepsleep is allowed
extern uint32_t loopcounter;        // loop() iterations since the last wakeup
extern bool reduced_interval;       // reduced send interval active
extern bool undervoltage;
extern bool deep_undervoltage;      // 3 consecutive readings < 2.9V; extended sleep for solar charging
extern volatile int wakeup_source;
extern volatile uint32_t pulsecount; // pulses from the reed switch wind sensor

// Enter deepsleep for the configured window. Returns the time actually slept.
uint32_t sleep(bool enable_uart_interrupt);

// Re-arm peripherals after a sleep window and refresh battery/charger state.
void wakeup();

// Full sleep cycle: work out how long to sleep, shut things down, sleep,
// service the sensor and any LoRa traffic, then wake up again.
void go_sleep();

bool usb_host_connected();
