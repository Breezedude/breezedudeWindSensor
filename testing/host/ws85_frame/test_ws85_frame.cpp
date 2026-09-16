// Host test for the real src/ws85uart.h (copied in next to the stub by run.bat).
//
// After a standby wakeup the station's receiver delivers the tail of a 33 byte
// frame: its first bytes are lost while the receiver starts up, and now and then
// a byte decoded out of step comes in front of the rest. The bursts marked as
// recorded are what the rig showed with the WS85_TRACE build (seen=[...] in the
// W85 q= lines); the others are made from them.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "Arduino.h"

// ---- stub state ----
static uint32_t g_micros = 1000;
uint32_t micros() { return g_micros; }
void delay(unsigned long) {}
void delayMicroseconds(unsigned int) {}
bool debug_enabled() { return true; }

static std::string g_log;
void log_i(const char *msg) { g_log += msg; }
void log_i(const char *msg, uint32_t value) {
  g_log += msg;
  g_log += std::to_string(value);
  g_log += "\n";
}
void log_v(const char *) {}

#include "ws85uart.h"

typedef std::vector<uint8_t> Bytes;

static Bytes hex(const char *s) {
  Bytes out;
  for (;;) {
    char *end;
    const unsigned long v = strtoul(s, &end, 16);
    if (end == s) {
      return out;
    }
    out.push_back((uint8_t)v);
    s = end;
  }
}

static Bytes tail(const Bytes &b, size_t lost) { return Bytes(b.begin() + lost, b.end()); }

static Bytes cat(const Bytes &a, const Bytes &b) {
  Bytes r = a;
  r.insert(r.end(), b.begin(), b.end());
  return r;
}

// Modbus CRC-16 appended low byte first, for frames made up here.
static Bytes with_crc(Bytes f) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : f) {
    crc ^= b;
    for (int k = 0; k < 8; k++) {
      crc = (uint16_t)((crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1));
    }
  }
  f.push_back((uint8_t)(crc & 0xFF));
  f.push_back((uint8_t)(crc >> 8));
  return f;
}

struct Outcome {
  bool valid = false;
  bool before_quiet = false;   // taken before the line had been quiet for the timeout
  WS85Measurement m{};
  std::string log;
};

// One burst after a wakeup: a byte every 87 us at 115200 baud, and a poll after
// each, as read_ws85_uart() does with its 500 us timeout.
static Outcome receive(WS85WindSensor &ws, HardwareSerial &uart, const Bytes &burst) {
  Outcome o;
  g_log.clear();
  ws.lastByte = g_micros;
  for (uint8_t b : burst) {
    uart.rx.push_back(b);
    g_micros += 87;
    ws.poll(500);
    if (ws.available()) {
      o.before_quiet = true;
      break;
    }
  }
  if (!ws.available()) {
    g_micros += 600;
    ws.poll(500);
  }
  o.valid = ws.available();
  o.m = ws.get();
  o.log = g_log;
  uart.rx.clear();
  g_micros += 8000000;   // the next frame is 8.3 s away
  return o;
}

static int g_fail = 0;
static void check(bool cond, const char *what) {
  printf("  %-64s %s\n", what, cond ? "ok" : "FAIL");
  if (!cond) g_fail++;
}

static bool close_to(float a, float b) { return std::fabs(a - b) < 0.01f; }

// Recorded: 26.5 degrees, calm, wind from 315 degrees.
static const char *FRAME_A =
    "85 03 1C 00 85 00 04 00 85 00 00 43 D5 FF FF FF FF 02 99 FF FF 00 00 00 00 01 3B 00 00 FF FF CD BF";
// The example in ws85uart.h: 26.7 degrees, 1.1 m/s, gusts 1.3 m/s, from 7 degrees.
static const char *FRAME_D =
    "85 03 1C 00 85 00 04 00 85 00 00 2A C8 FF FF FF FF 02 9B FF FF 00 0B 00 0D 00 07 00 00 FF FF 8B 87";

int main() {
  HardwareSerial uart;
  WS85WindSensor ws(&uart);
  ws.begin();
  const Bytes a = hex(FRAME_A);
  const Bytes d = hex(FRAME_D);
  Outcome o;

  printf("complete frames\n");
  o = receive(ws, uart, a);
  check(o.valid, "gives a measurement");
  check(o.before_quiet, "  ... as soon as the 33rd byte is in");
  check(close_to(o.m.temperatureC, 26.5f) && o.m.windDirection == 315, "  ... 26.5 degrees, from 315");
  o = receive(ws, uart, d);
  check(o.valid && close_to(o.m.windSpeed, 3.96f) && close_to(o.m.gustSpeed, 4.68f) && o.m.windDirection == 7,
        "3.96 km/h, gusts 4.68 km/h, from 7 degrees");

  printf("\nleading bytes lost to the wakeup\n");
  o = receive(ws, uart, hex("1C 00 85 00 04 00 85 00 00 43 D5 FF FF FF FF 02 99 FF FF 00 00 00 00 01 3B 00 00 FF FF CD BF"));
  check(o.valid && o.m.windDirection == 315, "recorded seen=31[1C 00 85 ...]: 2 put back");
  o = receive(ws, uart, hex("00 85 00 04 00 85 00 00 43 D5 FF FF FF FF 02 84 FF FF 00 00 00 00 00 00 00 00 FF FF 04 76"));
  check(o.valid && close_to(o.m.temperatureC, 24.4f), "recorded seen=30[00 85 00 ...]: 3 put back");
  o = receive(ws, uart, hex("85 00 04 00 85 00 00 43 D5 FF FF FF FF 02 99 FF FF 00 00 00 00 01 3B 00 00 FF FF CD BF"));
  check(o.valid && o.m.windDirection == 315, "recorded seen=29[85 00 04 ...]: 4 put back");
  o = receive(ws, uart, tail(d, 9));
  check(o.valid && o.m.windDirection == 7, "all 9 head bytes put back");
  o = receive(ws, uart, tail(d, 10));
  check(!o.valid, "10 missing are one too many");
  check(o.log.find("only got") != std::string::npos, "  ... reported as too short");

  printf("\na byte decoded out of step in front of the rest\n");
  o = receive(ws, uart, hex("E0 1C 00 85 00 04 00 85 00 00 43 D5 FF FF FF FF 02 99 FF FF 00 00 00 00 01 3B 00 00 FF FF CD BF"));
  check(o.valid && o.m.windDirection == 315, "recorded seen=32[E0 1C 00 85 ...]");
  check(o.log.find("CRC error") == std::string::npos, "  ... without a CRC error on the way");
  o = receive(ws, uart, cat(hex("E0"), tail(a, 4)));
  check(o.valid, "in front of 4 lost bytes");
  o = receive(ws, uart, cat(hex("E0 7F"), tail(a, 3)));
  check(o.valid, "two of them in front of 3 lost bytes");
  o = receive(ws, uart, cat(hex("E0"), d));
  check(o.valid && o.m.windDirection == 7, "in front of a complete frame");
  check(o.before_quiet, "  ... taken as soon as its last byte is in");

  printf("\nwhat must not become a measurement\n");
  o = receive(ws, uart, hex("1C 00 85 00 04 00 85 00 00 FF 02 82 FF FF 00 00 00 00 00 13 00 00 FF FF 88 73"));
  check(!o.valid, "recorded bytes lost in the middle (watchdog stall)");
  check(o.log.find("CRC error") != std::string::npos, "  ... reported as a CRC error");
  Bytes broken = a;
  broken[20] ^= 0x10;
  o = receive(ws, uart, broken);
  check(!o.valid, "a wrong byte behind the head");
  check(o.log.find("CRC error") != std::string::npos, "  ... reported as a CRC error");
  o = receive(ws, uart, hex("86 00 43 D5 05 DC 06 85 03 83 95 25"));
  check(!o.valid && o.log.find("only got") != std::string::npos, "recorded 12 byte fragment after power-up");

  printf("\na sensor with another model code\n");
  Bytes other = with_crc(Bytes(a.begin(), a.begin() + 31));
  other[4] = 0x90;
  other = with_crc(Bytes(other.begin(), other.begin() + 31));
  o = receive(ws, uart, other);
  check(o.valid, "complete frame taken as it came");
  o = receive(ws, uart, tail(other, 2));
  check(o.valid, "2 lost: only address and function put back");

  printf("\nfrom one burst to the next\n");
  receive(ws, uart, tail(a, 12));
  o = receive(ws, uart, tail(a, 2));
  check(o.valid, "a burst that failed does not spoil the next");

  printf("\n%s\n", g_fail ? "FAILURES" : "all checks passed");
  return g_fail != 0;
}
