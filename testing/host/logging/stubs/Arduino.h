// Host stub for the firmware's Arduino.h - just enough of Print/Stream/String
// for logging.cpp. Every stream keeps what was written to it, which is the
// whole point: the test reads those buffers back.
//
// The integer types are declared here rather than taken from <stdint.h>, and
// <string> is avoided, so that nothing drags the host's own definitions in.
// On the ARM toolchain int32_t is `long`, so log_i(const char*, int32_t) and
// log_i(const char*, int) are two different functions; where int32_t is `int`
// they collapse into one and logging.cpp stops compiling. Matching the target
// keeps the test honest about which overloads exist.
#pragma once
#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
typedef long int32_t;

class String {
public:
  char buf[96];
  String() { buf[0] = '\0'; }
  String(const char *p) { snprintf(buf, sizeof(buf), "%s", p ? p : ""); }
  String(int v) { snprintf(buf, sizeof(buf), "%d", v); }
  String(unsigned v) { snprintf(buf, sizeof(buf), "%u", v); }
  String(long v) { snprintf(buf, sizeof(buf), "%ld", v); }
  String(unsigned long v) { snprintf(buf, sizeof(buf), "%lu", v); }
  String(float v) { snprintf(buf, sizeof(buf), "%.2f", (double)v); }
  String(double v) { snprintf(buf, sizeof(buf), "%.2f", v); }
  String operator+(const String &o) const {
    String r;
    snprintf(r.buf, sizeof(r.buf), "%s%s", buf, o.buf);
    return r;
  }
  const char *c_str() const { return buf; }
};

class Print {
public:
  char out[4096];
  size_t len;
  Print() { clear(); }
  void clear() { len = 0; out[0] = '\0'; }

  void raw(const char *s, size_t n) {
    if (len + n >= sizeof(out)) { n = sizeof(out) - len - 1; }
    memcpy(out + len, s, n);
    len += n;
    out[len] = '\0';
  }
  void print(const char *v) { if (v) raw(v, strlen(v)); }
  void print(char v) { raw(&v, 1); }
  void print(int v) { char b[24]; raw(b, snprintf(b, sizeof(b), "%d", v)); }
  void print(unsigned v) { char b[24]; raw(b, snprintf(b, sizeof(b), "%u", v)); }
  void print(long v) { char b[24]; raw(b, snprintf(b, sizeof(b), "%ld", v)); }
  void print(unsigned long v) { char b[24]; raw(b, snprintf(b, sizeof(b), "%lu", v)); }
  void print(float v) { char b[32]; raw(b, snprintf(b, sizeof(b), "%.2f", (double)v)); }
  void print(double v) { char b[32]; raw(b, snprintf(b, sizeof(b), "%.2f", v)); }
  template <typename T> void println(T v) { print(v); raw("\r\n", 2); }
  void write(const uint8_t *p, size_t n) { raw((const char *)p, n); }
  void flush() {}
};

class Stream : public Print {};

class Uart : public Stream {
public:
  void begin(long) {}
};

class Serial_ : public Stream {};

extern Serial_ Serial;

inline void yield() {}
