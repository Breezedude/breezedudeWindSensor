#pragma once
#include <stdint.h>

// When a repeater only has to pass on weather packets, it does not have to
// listen the whole time. A station sends on its own clock - last send plus its
// interval - and that rhythm turns out to be stable to a few milliseconds over
// one period, because the repeater measures it with a clock that drifts the
// same way. So the plan is: learn each sender's period, wake shortly before the
// next packet is due, listen just long enough, and sleep in between.
//
// What this costs in coverage: outside its windows the repeater is deaf. It is
// therefore a mode of its own, REPEATER_SCHED, on top of the classic repeater,
// which keeps listening continuously.
//
// The scan window is what keeps it from going blind: every REPEATER_SCAN_
// INTERVAL the receiver stays on for REPEATER_SCAN_DURATION, long enough to
// find stations that appeared since, and to catch the name packets that are
// only sent every few minutes.
//
// Everything here works on the millisecond clock of time(), which survives
// standby. The caller passes it in rather than the module reading it, so the
// whole state machine can be exercised on a host.

#define SCHED_MAX_STATIONS 5
#define SCHED_NO_DEADLINE  0xFFFFFFFFu

enum SchedState : uint8_t {
  SCHED_FREE = 0,   // slot unused
  SCHED_LEARN,      // heard once, period unknown
  SCHED_CHECK,      // one period measured, waiting for it to be confirmed
  SCHED_SYNCED,     // period known, windows are being used
  SCHED_RESYNC      // window missed twice, listening continuously again
};

struct SchedInfo {
  uint32_t id;       // FANET sender, (vendor << 16) | address
  uint32_t period;   // ms between packets, 0 while unknown
  uint32_t next;     // when the next packet is expected
  uint32_t guard;    // ms the window opens before and closes after that
  uint16_t hits;     // packets received from this station
  uint8_t  misses;   // windows in a row that stayed empty
  uint8_t  state;    // SchedState
};

// Scan window parameters. A duration of 0 switches scanning off.
void sched_configure(uint32_t scan_interval_ms, uint32_t scan_duration_ms);

// Every weather packet that carries the forward bit, with the time the station
// woke up for it.
void sched_note_weather(uint32_t sender_id, uint32_t t_rx);

// Should the receiver be on at this moment?
bool sched_rx_wanted(uint32_t now);

// Milliseconds until the plan changes - a window or the scan opening or
// closing. SCHED_NO_DEADLINE when nothing is pending.
uint32_t sched_next_deadline(uint32_t now);

// One line per station, for the log.
void sched_log_status(uint32_t now);

// For the host test and for diagnostics.
uint8_t sched_snapshot(SchedInfo *out, uint8_t max);
bool sched_scanning(void);
void sched_reset(void);
