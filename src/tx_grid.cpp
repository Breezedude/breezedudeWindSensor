#include "tx_grid.h"

static uint32_t interval_ms = 0;
static uint32_t slot = 0;       // when the next packet belongs
static uint32_t attempt = 0;    // when to try next, never before the slot
static uint32_t last_sent = 0;
static bool     running = false;

// The clock wraps after 49.7 days, so every comparison is made on the
// difference and never on the absolute values.
static inline bool reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

void tx_grid_reset(void) {
  interval_ms = 0;
  slot = 0;
  attempt = 0;
  last_sent = 0;
  running = false;
}

void tx_grid_set_interval(uint32_t now, uint32_t iv) {
  if (!iv) {
    interval_ms = 0;
    return;
  }
  if (running && iv == interval_ms) {
    return;
  }
  // Either the first slot of all, or the interval changed under us. Both are
  // measured from the last packet that went out, so the rhythm continues from
  // where it is rather than restarting.
  interval_ms = iv;
  slot = (running ? last_sent : now) + iv;
  attempt = slot;
  running = true;
}

uint32_t tx_grid_due_in(uint32_t now) {
  if (!interval_ms) {
    return TX_GRID_NEVER;
  }
  return reached(now, attempt) ? 0u : (attempt - now);
}

void tx_grid_sent(uint32_t now) {
  last_sent = now;
  if (!interval_ms) {
    return;
  }
  slot += interval_ms;
  // The slot has gone by already: the station slept through it in
  // undervoltage, or the interval was shortened. Start a fresh grid from here
  // instead of firing off every packet that was missed.
  if (reached(now, slot)) {
    slot = now + interval_ms;
  }
  attempt = slot;
}

void tx_grid_retry(uint32_t now, uint32_t delay_ms) {
  attempt = now + delay_ms;
}

uint32_t tx_grid_slot(void) {
  return slot;
}
