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
// HWInfo (type 0x0A) is followed as a second rhythm per station, so a station's
// hardware and firmware report gets forwarded too and not only when it happens
// to fall into a scan. Two differences to the weather rhythm, both because it
// comes only every ten minutes:
//
//   - It is not measured before it is used. Measuring needs two packets in a
//     row, which a repeater that sleeps between its windows would only ever get
//     inside a scan window longer than the interval itself. The interval is not
//     a secret though - ota_lora.cpp sends HWInfo every ten minutes - so one
//     packet is enough to start, with a wide guard because an assumed period is
//     only as good as the two clocks agree. The first measured gap replaces it
//     and the guard collapses.
//   - It is never resynced by listening: after two empty windows the rhythm is
//     dropped and left to the next scan. Ten minutes of listening would cost
//     more than the packet is worth.
//
// OTA packets are not part of this. They never carry the forward bit, so the
// repeater does not pass them on - this mode forwards what a station reports
// about itself, it does not carry an update session.
#define SCHED_HW_SEED_MS       600000u  // ota_lora.cpp: OTA_HWINFO_DYNAMIC_INTERVAL_S
#define SCHED_HW_SEED_GUARD_MS  15000u  // 2.5 % of clock offset over ten minutes; measured: 2 %, both ways possible
//
// Everything here works on the millisecond clock of time(), which survives
// standby. The caller passes it in rather than the module reading it, so the
// whole state machine can be exercised on a host.

#define SCHED_MAX_STATIONS 5
#define SCHED_NO_DEADLINE  0xFFFFFFFFu

// How long a window stays open past the moment a packet is expected. A station
// never sends early - its next send is its last send plus its interval - but it
// is regularly late: scheduler.cpp holds every transmission back until
// fanet_cooldown (4 s) has passed since the previous one, so a weather packet
// that follows a name or HWInfo packet slides by that much, and listen-before-
// talk can add more. Measured on the rig: a rhythm of 39.28 s with one packet
// 3.5 s late, right after that station had sent its name.
//
// The tail is nearly free, because a window is not closed by the clock but by
// the packet: the moment it arrives, the next one is expected a full period
// later and the receiver goes back to sleep. Waiting nine seconds only costs
// anything when the packet really is late or lost - which is exactly when it is
// worth waiting.
#define SCHED_TAIL_MS      9000u

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
  uint32_t hw_period;// the same for the HWInfo rhythm, 0 while unknown
  uint32_t hw_next;
  uint32_t hw_guard;
  uint8_t  hw_state; // SchedState, but never SCHED_RESYNC
};

// Scan window parameters. A duration of 0 switches scanning off.
void sched_configure(uint32_t scan_interval_ms, uint32_t scan_duration_ms);

// Every weather packet that carries the forward bit, with the time the station
// woke up for it. A weather packet is what puts a station in the table.
void sched_note_weather(uint32_t sender_id, uint32_t t_rx);

// Every HWInfo packet that carries the forward bit. Only stations already in
// the table are followed: a station that never sends weather has nothing this
// mode would forward on a schedule.
void sched_note_hwinfo(uint32_t sender_id, uint32_t t_rx);

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
// True once since the last call when a station was dropped to make room.
bool sched_dropped_take(void);
void sched_reset(void);
