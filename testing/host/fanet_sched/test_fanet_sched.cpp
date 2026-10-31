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

// Keeps a synced station's weather rhythm alive up to t_end: a packet arrives
// exactly when it is expected, every time. Returns the last moment fed.
static uint32_t feed_weather(uint32_t id, uint32_t t_end) {
  SchedInfo s;
  uint32_t last = 0;
  while (info_of(id, &s) && (int32_t)(s.next - t_end) <= 0) {
    last = s.next;
    sched_note_weather(id, last);
  }
  return last;
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
  check(sched_rx_wanted(next + SCHED_TAIL_MS - 100), "still on for a sender held back by its cooldown");
  check(!sched_rx_wanted(next + SCHED_TAIL_MS + 1), "off again once even that is over");
  check(info_of(ID_A, &s) && s.misses == 1, "  ... that empty window counted as a miss");

  printf("\na late packet keeps the rhythm, it does not restart it\n");
  fresh(3600000, 0);
  uint32_t t = sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  const uint32_t period_before = s.period;
  const uint32_t due = s.next;
  t = due + 3500;                    // held back by the cooldown after a name packet
  sched_note_weather(ID_A, t);
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED, "a packet 3.5 s late is still the same rhythm");
  check(s.period == period_before, "  ... the period is left alone");
  check(s.next == due + period_before, "  ... the prediction stays on the sender's grid");
  // The sender keeps its slot (tx_grid.cpp): the next packet is on time again.
  check(sched_rx_wanted(due + period_before), "  ... so the window is open when the next one comes on time");
  sched_note_weather(ID_A, due + period_before);
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.misses == 0, "  ... and it is caught");

  printf("\na grid sender with a late packet every third time\n");
  fresh(3600000, 0);
  t = sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  uint32_t slot = s.next;
  uint32_t caught = 0, missed = 0;
  for (int i = 0; i < 30; i++) {
    const uint32_t arrival = slot + ((i % 3 == 0) ? 2500u : 20u);
    if (sched_rx_wanted(arrival)) { sched_note_weather(ID_A, arrival); caught++; }
    else missed++;
    slot += T;
  }
  info_of(ID_A, &s);
  check(caught == 30 && missed == 0, "thirty packets, every one caught");
  check(s.period > T - 100 && s.period < T + 100, "  ... and the period is still the sender's");

  printf("\na sender whose grid moved for good\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  slot = s.next + 4000;              // restarted: every packet 4 s later from now on
  for (int i = 0; i < 3; i++) {
    sched_rx_wanted(slot);
    sched_note_weather(ID_A, slot);
    slot += T;
  }
  info_of(ID_A, &s);
  check(s.state == SCHED_SYNCED && s.misses == 0, "late by the same three times: still caught");
  check(s.next > slot - 200 && s.next < slot + 200, "  ... and the prediction has moved to the new grid");

  printf("\ntwo clocks drifting apart\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  slot = s.next;
  caught = 0; missed = 0;
  for (int i = 0; i < 60; i++) {
    slot += T + 40;                  // the sender's 40 s are 40.04 s here
    if (sched_rx_wanted(slot)) { sched_note_weather(ID_A, slot); caught++; }
    else missed++;
  }
  info_of(ID_A, &s);
  check(missed == 0, "sixty packets 1 per mille slow, none missed");
  check(s.period > T + 20 && s.period < T + 60, "  ... the period has followed the drift");

  printf("\na punctual sender narrows the window\n");
  fresh(3600000, 0);
  t = sync_station(ID_A, 100000, T);
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
  sched_rx_wanted(s.next + SCHED_TAIL_MS + 1);    // window passes, nothing arrives
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.misses == 1, "first empty window: schedule kept");
  check(s.guard >= 1000, "  ... window widened for the next attempt");
  check(!sched_rx_wanted(s.next - s.guard - 100), "  ... still sleeping in between");
  sched_rx_wanted(s.next + SCHED_TAIL_MS + 1);    // second one too
  check(info_of(ID_A, &s) && s.state == SCHED_RESYNC, "second empty window: back to listening");
  check(sched_rx_wanted(s.next + 5000), "  ... receiver on regardless of the clock");

  printf("\nfinding the sender again\n");
  uint32_t t_back = s.next + 30;                  // it turns up in the window it was due in
  sched_note_weather(ID_A, t_back);
  check(info_of(ID_A, &s) && s.state == SCHED_RESYNC, "a packet after empty windows: the gap proves nothing yet");
  check(sched_rx_wanted(t_back + 1000), "  ... so the receiver stays on to measure the next one");
  t_back += T;
  sched_note_weather(ID_A, t_back);
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.misses == 0 && s.period == T, "the next one a period later: the rhythm was right all along");
  check(!sched_rx_wanted(t_back + 1000), "  ... and the receiver sleeps again");

  printf("\nwindows that keep missing do not teach a longer interval\n");
  fresh(3600000, 0);
  t = sync_station(ID_A, 100000, T);
  for (int i = 0; i < 4; i++) {
    t += 3 * T;                                   // two windows empty, through no fault of the sender
    sched_note_weather(ID_A, t);
    t += T;                                       // heard, the receiver was on
    sched_note_weather(ID_A, t);
    check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.period == T, "  the period stays the sender's");
  }

  printf("\na sender that stays away frees its slot\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  sched_rx_wanted(s.next + SCHED_TAIL_MS + 1);
  info_of(ID_A, &s);
  uint32_t probe = s.next + SCHED_TAIL_MS + 1;
  sched_rx_wanted(probe);                         // two misses -> resync
  check(info_of(ID_A, &s) && s.state == SCHED_RESYNC, "resyncing");
  sched_rx_wanted(probe + 4 * T);                 // silent for far longer than the patience
  check(station_count() == 0, "slot given back");
  check(sched_rx_wanted(probe + 5 * T), "  ... with nothing left to plan for, it listens again");

  printf("\nnothing known yet, but a scan window\n");
  fresh(300000, 60000);
  check(sched_rx_wanted(1000), "the scan right after boot");
  check(!sched_rx_wanted(70000), "between scans with an empty table: asleep");
  check(sched_next_deadline(70000) == 300000 + 1000 - 70000, "  ... until the next scan");

  printf("\nthe sender changes its interval\n");
  fresh(3600000, 0);
  t = sync_station(ID_A, 100000, T);
  t += 3 * T + 700;                               // battery low: three times the interval
  sched_note_weather(ID_A, t);
  check(info_of(ID_A, &s) && s.state == SCHED_RESYNC && s.period == T,
        "one gap of three periods proves nothing: listen to the next one");
  t += 3 * T + 700;
  sched_note_weather(ID_A, t);
  check(info_of(ID_A, &s) && s.state == SCHED_SYNCED && s.period == 3 * T + 700,
        "the same gap with the receiver on all the way is the new interval");
  t += 7000;                                      // now something entirely different
  sched_note_weather(ID_A, t);
  check(info_of(ID_A, &s) && s.state == SCHED_CHECK && s.period == 7000, "an interval that fits nothing is learned anew");

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

  printf("\nthe HWInfo rhythm of a station\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);     // weather at 100000, 140000, 180000
  uint32_t t_hw = 180000 + T / 2;    // HWInfo sits between two weather packets
  sched_note_hwinfo(ID_A, t_hw);
  check(info_of(ID_A, &s) && s.hw_state == SCHED_CHECK && s.hw_period == SCHED_HW_SEED_MS,
        "one HWInfo is enough: the protocol interval is taken on trust");
  check(!sched_rx_wanted(t_hw + 100), "  ... and the receiver goes straight back to sleep");
  const uint32_t hw_due = s.hw_next;
  const uint32_t hw_guard = s.hw_guard;
  check(hw_guard > 2000, "the assumed interval gets a wide window");
  feed_weather(ID_A, hw_due - hw_guard - 2000);
  check(!sched_rx_wanted(hw_due - hw_guard - 2000), "asleep right up to that window");
  check(sched_rx_wanted(hw_due - hw_guard + 100), "on early enough for two clocks that disagree");
  check(sched_rx_wanted(hw_due), "on when the HWInfo is due");
  sched_note_hwinfo(ID_A, hw_due + 20);
  check(info_of(ID_A, &s) && s.hw_state == SCHED_SYNCED, "the second packet confirms the rhythm");
  check(s.hw_period == SCHED_HW_SEED_MS + 20, "  ... the measured gap replaces the assumption");
  check(s.hw_guard <= 500, "  ... and the window narrows to the measured case");
  check(!sched_rx_wanted(hw_due + 2000), "  ... it closes on the packet, like any window");

  printf("\na clock two per cent off, and an hourly packet in between\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  t_hw = 180000 + T / 2;
  sched_note_hwinfo(ID_A, t_hw);
  feed_weather(ID_A, t_hw + 350000);
  sched_note_hwinfo(ID_A, t_hw + 350000);         // an hourly report: rounds to one period, outside the window
  info_of(ID_A, &s);
  check(s.hw_state == SCHED_CHECK && s.hw_next == t_hw + SCHED_HW_SEED_MS, "an off-rhythm report leaves the expectation alone");
  feed_weather(ID_A, t_hw + 612000);
  check(sched_rx_wanted(t_hw + 612000), "twelve seconds after the assumed ten minutes the window is still open");
  sched_note_hwinfo(ID_A, t_hw + 612000);
  info_of(ID_A, &s);
  check(s.hw_state == SCHED_SYNCED && s.hw_period == 612000, "  ... and the gap measured is from the rhythm's own last packet");
  check(!sched_rx_wanted(t_hw + 613000), "  ... the window closes on it");

  printf("\nthe second HWInfo lost: measured over two periods\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  t_hw = 180000 + T / 2;
  sched_note_hwinfo(ID_A, t_hw);
  feed_weather(ID_A, t_hw + 1212000);             // one per cent off
  info_of(ID_A, &s);
  check(s.hw_state == SCHED_CHECK && s.hw_next == t_hw + 2 * SCHED_HW_SEED_MS, "one lost report does not give the rhythm up");
  check(sched_rx_wanted(t_hw + 1212000), "the window for the next one is open");
  sched_note_hwinfo(ID_A, t_hw + 1212000);
  info_of(ID_A, &s);
  check(s.hw_state == SCHED_SYNCED && s.hw_period == 606000, "  ... and the period is half the gap");

  printf("\nan unknown station's HWInfo is not followed\n");
  sched_note_hwinfo(ID_B, hw_due + 3000);
  check(!info_of(ID_B, &s), "no slot for a station that never sent weather");

  printf("\na HWInfo rhythm is dropped, not resynced\n");
  info_of(ID_A, &s);
  probe = s.hw_next + SCHED_TAIL_MS + 1;
  feed_weather(ID_A, probe);
  sched_rx_wanted(probe);
  check(info_of(ID_A, &s) && s.hw_state == SCHED_SYNCED, "first empty HWInfo window: rhythm kept");
  check(s.hw_guard > 500, "  ... window widened for the next attempt");
  probe = s.hw_next + SCHED_TAIL_MS + 1;
  feed_weather(ID_A, probe);
  sched_rx_wanted(probe);
  check(info_of(ID_A, &s) && s.hw_state == SCHED_FREE, "second: dropped, the next scan can find it again");
  check(s.state == SCHED_SYNCED, "  ... while the weather rhythm carries on untouched");

  printf("\nwhat the caller is told to wake up for\n");
  fresh(3600000, 0);
  sync_station(ID_A, 100000, T);
  info_of(ID_A, &s);
  const uint32_t quiet = s.next - s.guard - 5000;
  check(sched_next_deadline(quiet) == 5000, "sleep until the window opens");
  const uint32_t inside = s.next - s.guard + 100;
  check(sched_next_deadline(inside) == s.guard + SCHED_TAIL_MS - 100, "inside the window: wake when it closes");

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
