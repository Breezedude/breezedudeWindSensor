// Host test for the real src/tx_grid.cpp - the fixed send grid of the weather
// packet.
//
// The module never reads a clock, the caller passes the time in, so every case
// that matters can be played through here: a punctual sender, one packet held
// back by the cooldown, a retry that must not move the rhythm, an interval that
// changes with the battery voltage, a slot slept through entirely, and the wrap
// of the 32 bit millisecond counter every 49.7 days.
#include <cstdio>

#include "tx_grid.h"

static int g_fail = 0;
static void check(bool cond, const char *what) {
  printf("  %-66s %s\n", what, cond ? "ok" : "FAIL");
  if (!cond) g_fail++;
}

static const uint32_t T = 40000;   // the usual weather interval

// Runs the grid forward to the next slot and sends there, punctually.
static uint32_t send_on_time(uint32_t now) {
  tx_grid_set_interval(now, T);
  const uint32_t due = now + tx_grid_due_in(now);
  tx_grid_set_interval(due, T);
  tx_grid_sent(due);
  return due;
}

int main() {
  printf("the first slot\n");
  tx_grid_reset();
  tx_grid_set_interval(100000, T);
  check(tx_grid_due_in(100000) == T, "one interval after the grid starts");
  check(tx_grid_due_in(100000 + T) == 0, "due when it arrives");
  check(tx_grid_due_in(100000 + T + 5000) == 0, "and stays due until it is sent");

  printf("\na punctual sender does not drift\n");
  tx_grid_reset();
  uint32_t t = 100000;
  tx_grid_set_interval(t, T);
  for (int i = 0; i < 20; i++) {
    t = send_on_time(t);
  }
  check(t == 100000 + 20 * T, "twenty packets, still exactly on the grid");

  printf("\na packet held back keeps the rhythm\n");
  tx_grid_reset();
  tx_grid_set_interval(100000, T);
  tx_grid_sent(100000 + T);                 // first packet, on time
  const uint32_t slot = tx_grid_slot();
  check(slot == 100000 + 2 * T, "the next slot is one interval on");
  // The cooldown behind a name packet holds this one back by 3.5 s, exactly the
  // case measured on the rig.
  tx_grid_sent(slot + 3500);
  check(tx_grid_slot() == 100000 + 3 * T, "a packet 3.5 s late does not move the grid");
  check(tx_grid_due_in(slot + 3500) == T - 3500, "  ... so the next one comes early enough to be on time");

  printf("\na retry moves the attempt, not the grid\n");
  tx_grid_reset();
  tx_grid_set_interval(100000, T);
  const uint32_t due = 100000 + T;
  tx_grid_retry(due, 1500);                 // channel busy
  check(tx_grid_due_in(due) == 1500, "try again in 1.5 s");
  check(tx_grid_slot() == due, "  ... the slot itself has not moved");
  tx_grid_retry(due + 1500, 10000);         // still no wind data
  check(tx_grid_due_in(due + 1500) == 10000, "and again in 10 s");
  check(tx_grid_slot() == due, "  ... still the same slot");
  tx_grid_sent(due + 11500);
  check(tx_grid_slot() == due + T, "once it goes out, the grid carries on from the slot");

  printf("\nthe interval changes with the battery\n");
  tx_grid_reset();
  tx_grid_set_interval(100000, T);
  tx_grid_sent(100000 + T);                 // last packet at 140000
  tx_grid_set_interval(140000 + 1000, 3 * T);   // voltage dropped: three times the interval
  check(tx_grid_slot() == 140000 + 3 * T, "the new interval counts from the last packet");
  check(tx_grid_due_in(140000 + 1000) == 3 * T - 1000, "  ... and the wait is what is left of it");
  // The voltage recovers well into the long interval, where the short one has
  // long since come and gone.
  tx_grid_set_interval(140000 + 100000, T);
  check(tx_grid_slot() == 140000 + T, "back to the short interval, also from the last packet");
  check(tx_grid_due_in(140000 + 100000) == 0, "  ... that slot has passed, so it goes out at once");
  tx_grid_sent(140000 + 100000);
  check(tx_grid_slot() == 140000 + 100000 + T, "  ... and the grid starts again from that packet");

  printf("\na slot slept through\n");
  tx_grid_reset();
  tx_grid_set_interval(100000, T);
  tx_grid_sent(100000 + T);
  // Half an hour of undervoltage sleep: the grid is far behind.
  const uint32_t wake = 100000 + T + 1800000;
  tx_grid_sent(wake);
  check(tx_grid_slot() == wake + T, "the grid starts again from here");
  check(tx_grid_due_in(wake) == T, "  ... no burst of the packets that were missed");

  printf("\nthe interval can be switched off\n");
  tx_grid_reset();
  tx_grid_set_interval(100000, 0);
  check(tx_grid_due_in(100000) == TX_GRID_NEVER, "nothing is due while the interval is 0");

  printf("\nthe millisecond counter wraps\n");
  tx_grid_reset();
  const uint32_t late = 0xFFFFFF00u;          // 256 ms before the wrap
  tx_grid_set_interval(late, T);
  check(tx_grid_due_in(late) == T, "a slot on the far side of zero");
  const uint32_t after = late + T;            // wrapped
  check(tx_grid_due_in(after) == 0, "due after the wrap");
  tx_grid_sent(after);
  check(tx_grid_slot() == after + T, "and the grid carries on across it");
  check(tx_grid_due_in(after + 1000) == T - 1000, "  ... with the wait still counted right");

  printf("\n%s\n", g_fail ? "FAILURES" : "all checks passed");
  return g_fail != 0;
}
