// Host stub for the Arduino API src/ws85uart.h uses, with a UART whose receive
// side the test fills.
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>

#define F(s) (s)

uint32_t micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

// On the target these come from logging.h and tools.h, included before
// ws85uart.h by the file that uses it.
bool debug_enabled();
void log_i(const char *msg);
void log_i(const char *msg, uint32_t value);
void log_v(const char *msg);

class Stream {
public:
  void print(const char *) {}
  void print(float, int) {}
  void print(unsigned int) {}
  void println(const char *) {}
  void flush() {}
};

class HardwareSerial : public Stream {
public:
  std::deque<uint8_t> rx;   // what the receiver holds, oldest byte first

  void begin(unsigned long) {}
  void end() {}
  int available() { return (int)rx.size(); }
  int read() {
    if (rx.empty()) {
      return -1;
    }
    const uint8_t b = rx.front();
    rx.pop_front();
    return b;
  }
  size_t write(uint8_t) { return 1; }
  size_t write(const uint8_t *, size_t n) { return n; }
};
