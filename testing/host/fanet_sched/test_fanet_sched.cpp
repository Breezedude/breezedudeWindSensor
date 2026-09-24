// Host test for the real src/fanet_sched.cpp (copied in next to the stub by
// run.bat, so its include of logging.h resolves to the stub).
//
// The module is pure arithmetic on the millisecond clock: it never reads a
// clock itself, the caller passes the time in. That is what makes the whole
// state machine - learning a sender's rhythm, opening windows, giving up,
// starting over - testable here, including the wrap of the 32 bit counter that
// happens every 49.7 days.
#include <cstdio>
#include <cstring>
#include <string>

#include "fanet_sched.h"

// ---- stub ----
static std::string g_log;
void log_i(const char *msg) { g_log += msg; }

// ---- helpers ----
static int g_fail = 0;
static void check(bool cond, const char *what) {
  printf("  %-66s %s\n", what, cond ? "ok" : "FAIL");
  if (!cond) g_fail++;
}

static const uint32_t ID_A = 0x11BD01;
static const uint32_t ID_B = 0x11BD02;

static bool info_of(uint32_t id, SchedInfo *out) {
  SchedInfo buf[SCHED_MAX_STATIONS];
  const uint8_t n = sched_snapshot(buf, SCHED_MAX_STATIONS);
  for (uint8_t i = 0; i < n; i++) {
    if (buf[i].id == id) {
      *out = buf[i];
      return true;
    }
  }
  return false;
}

static uint8_t station_count() {
  SchedInfo buf[SCHED_MAX_STATIONS];
  return sched_snapshot(buf, SCHED_MAX_STATIONS);
}

// Three packets at the given period put a station into SYNCED; returns the time
// of the last one.
static uint32_t sync_station(uint32_t id, uint32_t t0, uint32_t period) {
  sched_note_weather(id, t0);
  sched_note_weather(id, t0 + period);
  sched_note_weather(id, t0 + 2 * period);
  return t0 + 2 * period;
}

static void fresh(uint32_t scan_interval_ms, uint32_t scan_duration_ms) {
  sched_reset();
  sched_configure(scan_interval_ms, scan_duration_ms);
  g_log.clear();
}

int main() {
  const uint32_t T = 40000;   // 40 s, the usual weather interval
  SchedInfo s;

  printf("learning a rhythm\n");
  fresh(3600000, 0);          // scanning off, so only the stations decide
  sched_note_weather(ID_A, 100000);
  check(info_of(ID_A, &s) && s.state == SCHED_LEARN, "first packet: station is known, period unknown");
  check(sched_rx_wanted(100001), "  ... and the receiver stays on");
  sched_note_weather(ID_A, 100000 + T);
  check(info_of(ID_A, &s) && s.state == SCHED_CHECK && s.period == T, "second packet: period measured, not trusted yet");
  sched_note_weather(ID_A, 100000 + 2 * T);
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.period == T, "third packet confirms it: synced");

  printf("\nwindows around the expected packet\n");
  const uint32_t next = s.next;
  const uint32_t guard = s.guard;
  check(!sched_rx_wanted(next - guard - 1000), "quiet well before the window");
  check(sched_rx_wanted(next - guard + 10), "on just before the packet is due");
  check(sched_rx_wanted(next + 20), "on at the expected moment");
  check(!sched_rx_wanted(next + guard + 500), "off again after the window");
  check(info_of(ID_A, &s) && s.misses == 1, "  ... that empty window counted as a miss");

  printf("\na punctual sender narrows the window\n");
  fresh(3600000, 0);
  uint32_t t = sync_station(ID_A, 100000, T);
  for (int i = 0; i < 6; i++) {
    info_of(ID_A, &s);
    t = s.next + 15;                 // 15 ms late every time
    sched_rx_wanted(s.next - s.guard + 1);
    sched_note_weather(ID_A, t);
  }
  info_of(ID_A, &s);
  check(s.state == SCHED_SYNCED && s.misses == 0, "six packets in a row, all caught");
  check(s.guard <= 300, "guard has come down to the punctual case");
  check(s.guard >= 150, "  ... but not below the floor");

  printf("\none miss is tolerated, two are not\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  sched_rx_wanted(s.next + s.guard + 200);        // window passes, nothing arrives
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.misses == 1, "first empty window: schedule kept");
  check(s.guard >= 1000, "  ... window widened for the next attempt");
  check(!sched_rx_wanted(s.next - s.guard - 100), "  ... still sleeping in between");
  sched_rx_wanted(s.next + s.guard + 200);        // second one too
  check(info_of(ID_A, &s) && s.state == SCHED_RESYNC, "second empty window: back to listening");
  check(sched_rx_wanted(s.next + s.guard + 5000), "  ... receiver on regardless of the clock");

  printf("\nfinding the sender again\n");
  uint32_t t_back = s.next + 30;                  // it turns up in the window it was due in
  sched_note_weather(ID_A, t_back);
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.misses == 0, "a packet that fits the rhythm resyncs it");
  check(!sched_rx_wanted(t_back + 1000), "  ... and the receiver sleeps again");

  printf("\na sender that stays away frees its slot\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  sched_rx_wanted(s.next + s.guard + 200);
  sched_rx_wanted(s.next + 2 * T);                // two misses -> resync
  check(info_of(ID_A, &s) && s.state == SCHED_RESYNC, "resyncing");
  sched_rx_wanted(s.next + 10 * T);               // silent for far longer than the patience
  check(station_count() == 0, "slot given back");
  check(sched_rx_wanted(s.next + 11 * T), "  ... with nothing left to plan for, it listens again");

  printf("\nnothing known yet, but a scan window\n");
  fresh(300000, 60000);
  check(sched_rx_wanted(1000), "the scan right after boot");
  check(!sched_rx_wanted(70000), "between scans with an empty table: asleep");
  check(sched_next_deadline(70000) == 300000 + 1000 - 70000, "  ... until the next scan");

  printf("\nthe sender changes its interval\n");
  fresh(3600000, 0);
  t = sync_station(ID_A, 100000, T);
  sched_note_weather(ID_A, t + 3 * T + 700);      // battery low: three times the interval
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED, "a gap of three periods still fits the rhythm");
  sched_note_weather(ID_A, t + 3 * T + 700 + 7000);   // now something entirely different
  check(info_of(ID_A, &s) && s.state == SCHED_CHECK && s.period == 7000, "an interval that does not fit is learned anew");

  printf("\nfive stations, and one too many\n");
  fresh(3600000, 0);
  for (uint32_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    sync_station(0x11BE00 + i, 100000 + 1000 * i, T);
  }
  check(station_count() == SCHED_MAX_STATIONS, "table full");
  sched_note_weather(0x11BEFF, 100000 + 3 * T);
  check(station_count() == SCHED_MAX_STATIONS, "  ... stays full");
  check(info_of(0x11BEFF, &s), "  ... the newcomer got in");
  check(!info_of(0x11BE00, &s), "  ... the one silent longest had to go");

  printf("\nscan window\n");
  fresh(3600000, 360000);      // every hour for six minutes
  check(sched_rx_wanted(1000), "scanning right after boot");
  check(sched_scanning(), "  ... and it says so");
  check(!sched_rx_wanted(1000 + 360000 + 10), "over after six minutes");
  check(sched_next_deadline(1000 + 360000 + 10) > 3000000, "next scan is about an hour away");
  check(sched_rx_wanted(1000 + 3600000 + 10), "and it comes back");

  printf("\nwhat the caller is told to wake up for\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  const uint32_t quiet = s.next - s.guard - 5000;
  check(sched_next_deadline(quiet) == 5000, "sleep until the window opens");
  const uint32_t inside = s.next - s.guard + 100;
  check(sched_next_deadline(inside) == 2 * s.guard + 60 - 100, "inside the window: wake when it closes");

  printf("\nthe millisecond counter wraps\n");
  fresh(3600000, 0);
  const uint32_t late = 0xFFFFFF00u - 2 * T;      // sync across the wrap
  t = sync_station(ID_B, late, T);
  check(info_of(ID_B, &s) && s.state == SCHED_SYNCED, "synced across the wrap");
  check(!sched_rx_wanted(s.next - s.guard - 1000), "quiet before the window, on the far side of zero");
  check(sched_rx_wanted(s.next + 10), "window opens after the wrap");
  sched_note_weather(ID_B, s.next + 10);
  check(info_of(ID_B, &s) && s.misses == 0 && s.state == SCHED_SYNCED, "and the packet is counted");

  printf("\n%s\n", g_fail ? "FAILURES" : "all checks passed");
  return g_fail != 0;
}
