#include "fanet_sched.h"

#include <stdio.h>
#include <string.h>

#include "logging.h"

// The window opens this long before the packet is expected. Only the repeater's
// own timing has to fit in here, so it stays small; it follows the prediction
// error and doubles after an empty window.
#define SCHED_GUARD_MIN_MS  150u
#define SCHED_GUARD_MAX_MS  2000u
#define SCHED_GUARD_START_MS 500u
// Anything outside this is not a broadcast interval and is not learned.
#define SCHED_PERIOD_MIN_MS 5000u
#define SCHED_PERIOD_MAX_MS 600000u
// Empty windows in a row before the receiver stays on to find the sender again.
#define SCHED_MISS_LIMIT    2u
// How long that lasts before the station is given up and its slot freed.
#define SCHED_RESYNC_PERIODS 3u
// A station heard once but never again is forgotten after this.
#define SCHED_LEARN_TIMEOUT_MS 900000u
// A sender may skip this many periods and still be recognised by its rhythm.
#define SCHED_SKIP_MAX      4u
// HWInfo is sent every ten minutes (ota_lora.cpp), so its rhythm is learned
// within these bounds. Two empty windows drop it - see sched_note_hwinfo().
#define SCHED_HW_PERIOD_MIN_MS 15000u
#define SCHED_HW_PERIOD_MAX_MS 3600000u
#define SCHED_HW_MISS_LIMIT 2u
// The HWInfo window starts out wide enough for an assumed interval, so it needs
// more room than a weather window ever does.
#define SCHED_HW_GUARD_MAX_MS 12000u

struct Station {
  uint32_t id;
  uint32_t period;
  uint32_t last_rx;
  uint32_t next;
  uint32_t guard;
  uint32_t err_avg;    // running average of how early a packet arrived, ms
  uint32_t deadline;   // learn timeout, or when to give up resyncing
  uint16_t hits;
  uint8_t  misses;
  uint8_t  state;
  uint8_t  skip_k;     // how many periods the last gap spanned, to spot a changed interval
  // The HWInfo rhythm of the same station, followed separately.
  uint32_t hw_period;
  uint32_t hw_last;
  uint32_t hw_next;
  uint32_t hw_guard;
  uint32_t hw_err_avg;
  uint16_t hw_hits;
  uint8_t  hw_misses;
  uint8_t  hw_state;
};

static Station tab[SCHED_MAX_STATIONS];
static uint32_t scan_interval_ms = 3600000u;
static uint32_t scan_duration_ms = 360000u;
static uint32_t scan_next = 0;     // due right after boot, so a fresh station learns at once
static uint32_t scan_until = 0;
static bool scanning = false;

// time() is a 32 bit millisecond counter and wraps after 49.7 days, so every
// comparison is made on the difference.
static inline bool reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

static inline uint32_t clamp_guard(uint32_t g) {
  if (g < SCHED_GUARD_MIN_MS) return SCHED_GUARD_MIN_MS;
  if (g > SCHED_GUARD_MAX_MS) return SCHED_GUARD_MAX_MS;
  return g;
}

static inline bool plausible(uint32_t gap) {
  return gap >= SCHED_PERIOD_MIN_MS && gap <= SCHED_PERIOD_MAX_MS;
}

static inline bool hw_plausible(uint32_t gap) {
  return gap >= SCHED_HW_PERIOD_MIN_MS && gap <= SCHED_HW_PERIOD_MAX_MS;
}

static inline uint32_t clamp_hw_guard(uint32_t g) {
  if (g < SCHED_GUARD_MIN_MS) return SCHED_GUARD_MIN_MS;
  if (g > SCHED_HW_GUARD_MAX_MS) return SCHED_HW_GUARD_MAX_MS;
  return g;
}

// Does a measured gap still belong to the expected rhythm? Asymmetric, for the
// same reason the window is: a sender that was held back by its cooldown comes
// late, so a gap may exceed the expectation by the tail. Coming early means the
// interval really did change - the battery scale factor does that - and then
// the period is learned again. The early side keeps 5 %, at least 1 s, for the
// jitter of two clocks measuring each other.
static bool matches(uint32_t gap, uint32_t expected) {
  const uint32_t tol = (expected / 20u) > 1000u ? (expected / 20u) : 1000u;
  if (gap + tol < expected) return false;
  return gap <= expected + SCHED_TAIL_MS;
}

static uint32_t window_open(const Station *s)  { return s->next - s->guard; }
static uint32_t window_close(const Station *s) { return s->next + SCHED_TAIL_MS; }
static uint32_t hw_window_open(const Station *s)  { return s->hw_next - s->hw_guard; }
static uint32_t hw_window_close(const Station *s) { return s->hw_next + SCHED_TAIL_MS; }

static void say(const char *what, const Station *s) {
  char line[96];
  snprintf(line, sizeof(line), "SCHED %s 0x%06lX T=%lums g=%lums hit=%u miss=%u\r\n",
           what, (unsigned long)s->id, (unsigned long)s->period, (unsigned long)s->guard,
           (unsigned)s->hits, (unsigned)s->misses);
  log_i(line);
}

static void say_hw(const char *what, const Station *s) {
  char line[96];
  snprintf(line, sizeof(line), "SCHED hw %s 0x%06lX T=%lums g=%lums hit=%u miss=%u\r\n",
           what, (unsigned long)s->id, (unsigned long)s->hw_period, (unsigned long)s->hw_guard,
           (unsigned)s->hw_hits, (unsigned)s->hw_misses);
  log_i(line);
}

static Station *find(uint32_t id) {
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    if (tab[i].state != SCHED_FREE && tab[i].id == id) return &tab[i];
  }
  return nullptr;
}

// A free slot, or the station that has been silent longest - a sender that
// stopped should not keep a newcomer out.
static Station *slot_for(void) {
  Station *worst = nullptr;
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    if (tab[i].state == SCHED_FREE) return &tab[i];
    if (!worst || (int32_t)(tab[i].last_rx - worst->last_rx) < 0) worst = &tab[i];
  }
  if (worst) say("drops", worst);
  return worst;
}

static void forget(Station *s) {
  say("gone", s);
  memset(s, 0, sizeof(*s));
}

static void tick(uint32_t now) {
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    Station *s = &tab[i];
    if (s->state == SCHED_SYNCED) {
      uint8_t steps = 0;
      while (reached(now, window_close(s))) {
        s->misses++;
        s->next += s->period;
        s->guard = clamp_guard(s->guard * 2u);
        // Two empty windows, or a jump in time that skipped several, mean the
        // prediction is no longer worth anything.
        if (s->misses >= SCHED_MISS_LIMIT || ++steps > SCHED_SKIP_MAX) {
          s->state = SCHED_RESYNC;
          s->deadline = now + SCHED_RESYNC_PERIODS * s->period;
          say("resync", s);
          break;
        }
      }
    } else if (s->state != SCHED_FREE && reached(now, s->deadline)) {
      forget(s);
      continue;
    }

    // The HWInfo rhythm of a station is dropped rather than resynced: it comes
    // every ten minutes, and listening that long to find it again would cost
    // more than the packet is worth. The next scan window picks it up.
    if (s->hw_state == SCHED_CHECK || s->hw_state == SCHED_SYNCED) {
      uint8_t steps = 0;
      while (reached(now, hw_window_close(s))) {
        s->hw_misses++;
        s->hw_next += s->hw_period;
        s->hw_guard = clamp_hw_guard(s->hw_guard * 2u);
        if (s->hw_misses >= SCHED_HW_MISS_LIMIT || ++steps > SCHED_SKIP_MAX) {
          s->hw_state = SCHED_FREE;
          say_hw("lost", s);
          break;
        }
      }
    }
  }

  if (!scan_duration_ms) return;
  if (scanning && reached(now, scan_until)) {
    scanning = false;
    log_i("SCHED scan done\r\n");
  }
  if (!scanning && reached(now, scan_next)) {
    scanning = true;
    scan_until = now + scan_duration_ms;
    scan_next = now + scan_interval_ms;   // the interval counts from one scan to the next
    log_i("SCHED scan start\r\n");
    sched_log_status(now);
  }
}

void sched_configure(uint32_t interval_ms, uint32_t duration_ms) {
  scan_interval_ms = interval_ms ? interval_ms : 3600000u;
  scan_duration_ms = duration_ms;
}

void sched_reset(void) {
  memset(tab, 0, sizeof(tab));
  scanning = false;
  scan_next = 0;
  scan_until = 0;
}

void sched_note_weather(uint32_t id, uint32_t t_rx) {
  tick(t_rx);

  Station *s = find(id);
  if (!s) {
    s = slot_for();
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->id = id;
    s->state = SCHED_LEARN;
    s->guard = SCHED_GUARD_START_MS;
    s->last_rx = t_rx;
    s->hits = 1;
    s->deadline = t_rx + SCHED_LEARN_TIMEOUT_MS;
    say("new", s);
    return;
  }

  const uint32_t gap = t_rx - s->last_rx;
  s->last_rx = t_rx;
  s->hits++;
  s->deadline = t_rx + SCHED_LEARN_TIMEOUT_MS;

  if (s->state == SCHED_LEARN) {
    if (plausible(gap)) {
      s->period = gap;
      s->state = SCHED_CHECK;
    }
  } else if (s->state == SCHED_CHECK) {
    if (matches(gap, s->period)) {
      // Take the shorter of the two: a gap stretched by the cooldown is longer
      // than the interval, never shorter, so the smaller measurement is the one
      // closer to the truth.
      if (gap < s->period) { s->period = gap; }
      s->state = SCHED_SYNCED;
      s->misses = 0;
      s->err_avg = 0;
      s->guard = SCHED_GUARD_START_MS;
      say("synced", s);
    } else if (plausible(gap)) {
      s->period = gap;   // the first measurement was an outlier, start from this one
    }
  } else {
    // Synced or resyncing: does the gap fit the rhythm, with skipped periods
    // allowed? If it does not, the sender changed its interval and the period
    // is learned again.
    const uint32_t k = s->period ? (gap + s->period / 2u) / s->period : 0u;
    if (k >= 1u && k <= SCHED_SKIP_MAX && matches(gap, k * s->period)) {
      // Only earliness widens the guard. Lateness is covered by the tail, and
      // feeding it in here would open the window earlier and earlier for a
      // station that is merely waiting out its own cooldown.
      const int32_t err = (int32_t)(s->next - t_rx);
      const uint32_t early = err > 0 ? (uint32_t)err : 0u;
      s->err_avg = (3u * s->err_avg + early) / 4u;
      s->guard = clamp_guard(4u * s->err_avg + 50u);
      if (s->state != SCHED_SYNCED) say("back", s);
      s->state = SCHED_SYNCED;
      s->misses = 0;
      // A gap of k periods is a lost packet the first time. Twice in a row with
      // the same k it is the sender's new interval - scheduler.cpp multiplies
      // the broadcast interval when the battery runs low, by exactly three.
      // Without this the repeater keeps the old rhythm, wakes for k-1 windows
      // per cycle that can never have anything in them, and resyncs after every
      // second of them: measured on the rig at 40 % of the time with the
      // receiver on, against a per cent once the interval is taken over.
      if (k >= 2u && s->skip_k == (uint8_t)k) {
        s->period = gap;
        s->err_avg = 0;
        s->guard = SCHED_GUARD_START_MS;
        say("interval", s);
      }
      s->skip_k = (uint8_t)k;
    } else if (plausible(gap)) {
      s->period = gap;
      s->state = SCHED_CHECK;
      s->guard = SCHED_GUARD_START_MS;
      s->misses = 0;
      s->skip_k = 0;
      say("relearn", s);
    }
  }
  // Every packet re-anchors the prediction on the moment it actually arrived.
  // A station that was held back by its cooldown has moved its whole rhythm by
  // that much, so this is what keeps the window on the sender.
  s->next = t_rx + s->period;
}

void sched_note_hwinfo(uint32_t id, uint32_t t_rx) {
  tick(t_rx);

  Station *s = find(id);
  if (!s) return;   // only stations whose weather we already follow

  const uint32_t gap = t_rx - s->hw_last;
  const uint8_t was = s->hw_state;
  s->hw_last = t_rx;
  s->hw_hits++;

  if (was != SCHED_FREE) {
    const uint32_t k = s->hw_period ? (gap + s->hw_period / 2u) / s->hw_period : 0u;
    if (was == SCHED_CHECK && k == 1u && hw_plausible(gap)) {
      // The second packet, and the first real measurement. It was taken with
      // our own clock, so the offset between the two oscillators is inside the
      // number - up to a per cent, six seconds over ten minutes - and the
      // window can close down to the width the weather rhythm uses.
      s->hw_period = gap;
      s->hw_err_avg = 0;
      s->hw_guard = SCHED_GUARD_START_MS;
      s->hw_state = SCHED_SYNCED;
      s->hw_misses = 0;
      s->hw_next = t_rx + s->hw_period;
      say_hw("synced", s);
      return;
    }
    if (k >= 1u && k <= SCHED_SKIP_MAX && matches(gap, k * s->hw_period)) {
      const int32_t err = (int32_t)(s->hw_next - t_rx);
      const uint32_t early = err > 0 ? (uint32_t)err : 0u;
      s->hw_err_avg = (3u * s->hw_err_avg + early) / 4u;
      s->hw_guard = clamp_hw_guard(4u * s->hw_err_avg + 50u);
      s->hw_state = SCHED_SYNCED;
      s->hw_misses = 0;
      s->hw_next = t_rx + s->hw_period;
      return;
    }
    // Off the rhythm: ota_lora.cpp runs three timers, and the hourly ones put a
    // packet between the ten minute ones. It is forwarded like any other, but
    // it must not drag the prediction off the rhythm that holds.
    return;
  }

  // First HWInfo of this station. One packet says nothing about the interval,
  // but the protocol does, so the ten minutes from ota_lora.cpp are taken on
  // trust and the window opens wide. See fanet_sched.h for why this is not
  // measured first like the weather rhythm.
  s->hw_period = SCHED_HW_SEED_MS;
  s->hw_state = SCHED_CHECK;
  s->hw_guard = SCHED_HW_SEED_GUARD_MS;
  s->hw_misses = 0;
  s->hw_next = t_rx + SCHED_HW_SEED_MS;
  say_hw("expect", s);
}

static bool nothing_known(void) {
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    if (tab[i].state != SCHED_FREE) return false;
  }
  return true;
}

bool sched_rx_wanted(uint32_t now) {
  tick(now);
  if (scanning) return true;
  // Nothing learned and no scan window to learn in: then there is nothing to
  // plan around and the repeater listens, as it would without this mode.
  if (!scan_duration_ms && nothing_known()) return true;
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    const Station *s = &tab[i];
    if (s->hw_state == SCHED_CHECK || s->hw_state == SCHED_SYNCED) {
      if (reached(now, hw_window_open(s)) && !reached(now, hw_window_close(s))) return true;
    }
    switch (s->state) {
      case SCHED_LEARN:
      case SCHED_CHECK:
      case SCHED_RESYNC:
        return true;   // the rhythm is not known, only listening finds it
      case SCHED_SYNCED:
        if (reached(now, window_open(s)) && !reached(now, window_close(s))) return true;
        break;
      default:
        break;
    }
  }
  return false;
}

static void consider(uint32_t now, uint32_t at, uint32_t *best) {
  uint32_t d = (int32_t)(at - now) < 0 ? 0u : at - now;
  if (d < *best) *best = d;
}

uint32_t sched_next_deadline(uint32_t now) {
  tick(now);
  uint32_t best = SCHED_NO_DEADLINE;
  if (scan_duration_ms) consider(now, scanning ? scan_until : scan_next, &best);
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    const Station *s = &tab[i];
    if (s->hw_state == SCHED_CHECK || s->hw_state == SCHED_SYNCED) {
      consider(now, reached(now, hw_window_open(s)) ? hw_window_close(s) : hw_window_open(s), &best);
    }
    if (s->state == SCHED_SYNCED) {
      consider(now, reached(now, window_open(s)) ? window_close(s) : window_open(s), &best);
    } else if (s->state != SCHED_FREE) {
      consider(now, s->deadline, &best);
    }
  }
  return best;
}

void sched_log_status(uint32_t now) {
  char line[128];
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS; i++) {
    const Station *s = &tab[i];
    if (s->state == SCHED_FREE) continue;
    const int32_t due = (int32_t)(s->next - now);
    const int32_t hw_due = (int32_t)(s->hw_next - now);
    snprintf(line, sizeof(line),
             "SCHED 0x%06lX st=%u T=%lums g=%lums in=%ldms hit=%u miss=%u hw=%u/%lums/%ldms\r\n",
             (unsigned long)s->id, (unsigned)s->state, (unsigned long)s->period,
             (unsigned long)s->guard, (long)due, (unsigned)s->hits, (unsigned)s->misses,
             (unsigned)s->hw_state, (unsigned long)s->hw_period,
             s->hw_state == SCHED_FREE ? 0L : (long)hw_due);
    log_i(line);
  }
}

uint8_t sched_snapshot(SchedInfo *out, uint8_t max) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < SCHED_MAX_STATIONS && n < max; i++) {
    const Station *s = &tab[i];
    if (s->state == SCHED_FREE) continue;
    out[n].id = s->id;
    out[n].period = s->period;
    out[n].next = s->next;
    out[n].guard = s->guard;
    out[n].hits = s->hits;
    out[n].misses = s->misses;
    out[n].state = s->state;
    out[n].hw_period = s->hw_period;
    out[n].hw_next = s->hw_next;
    out[n].hw_guard = s->hw_guard;
    out[n].hw_state = s->hw_state;
    n++;
  }
  return n;
}

bool sched_scanning(void) { return scanning; }
