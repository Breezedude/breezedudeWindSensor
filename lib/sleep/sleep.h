#ifndef SLEEP_H
#define SLEEP_H

#include <Arduino.h>

extern Uart Serial2;

void SERCOM1_Handler();
void mark_uart_wakeup();
void mark_console_wakeup();
// Diagnostics for UART_WAKE, see the note above SERCOM1_Handler in sleep.cpp
// for how to read them together.
extern volatile uint32_t sercom1_irq_count;
extern volatile uint32_t sercom1_rxs_count;
extern volatile uint32_t sercom1_rxc_count;
extern volatile uint32_t sercom1_err_count;
extern volatile uint32_t sercom1_last_status;
#ifdef WS85_TRACE
// WS85_TRACE debug build only: what SERCOM0 did while the sensor UART was armed,
// see SERCOM0_Handler() in sleep.cpp and ws85_trace_report() in src/sensors.cpp.
#define WS85_TRACE_RX_TIMES 40  // byte arrival times kept per wake cycle
extern volatile uint32_t sercom0_rxs_count;
extern volatile uint32_t sercom0_rxc_count;
extern volatile uint32_t sercom0_err_count;
extern volatile uint32_t sercom0_rx_us[WS85_TRACE_RX_TIMES];
extern volatile uint8_t  sercom0_rx_n;
#endif
void enable_sercom0_int();
void disable_sercom0_int();
// The debug UART, set up once to keep receiving while the station sleeps.
//
// Must be called after every DEBUGSER.begin(): Uart::begin() runs a SWRST that
// wipes RUNSTDBY, SFDE and BAUD. Passing false leaves the UART exactly as the
// Arduino core configured it, which is the zero-cost default.
void configure_debug_uart_standby(bool enable);
void pinDisable(uint32_t pin);
void configGCLK6(bool en_rtc);
void deepsleep(bool light);

int wdt_enable(int maxPeriodMS, bool isForSleep);
uint32_t rtc_sleep_cfg(uint32_t milliseconds);
void wdt_disable();
void wdt_reset();
bool set_cpu_div(int divisor);

void setup_BOD33_interrupt();

void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode, bool en_rtc);
void setup_PM(bool en_pulse_counter, bool en_rtc_counter);

void setup_pulse_counter();
uint16_t read_pulse_counter();
uint16_t pulse_count_since_arm();
void reset_pulse_counter();
void arm_pulse_counter();
void stop_pulse_counter();

void setup_rtc_time_counter();
uint32_t read_time_counter();
uint32_t elapsed_time_counter();
void reset_time_counter();

#endif