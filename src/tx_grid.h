#pragma once
#include <stdint.h>

// A fixed send grid for the one message that has to keep a rhythm: the weather
// packet.
//
// The station used to schedule it as "last send plus interval", with the anchor
// taken at the moment the packet actually left the radio. Every delay was
// therefore carried forward for good. Delays are not rare: scheduler.cpp holds
// each of a station's own transmissions apart by fanet_cooldown, four seconds,
// so a weather packet that came due right behind a name packet went out four
// seconds late - and stayed four seconds late for ever after. Listen-before-talk
// and wind data that is not ready yet do the same.
//
// For a station that people read, that is invisible. For a repeater that sleeps
// between the packets it forwards it is the difference between a window of a
// few hundred milliseconds and re-learning the sender's rhythm every few
// minutes: measured on the rig, 39.28 s, 39.28 s, 42.8 s, and a rhythm shifted
// by 3.5 s from then on.
//
// So the slot is kept, not the delay: the grid moves on by exactly one interval
// per packet sent, and a retry moves only the attempt. A packet that was late is
// followed by one that is on time again.
//
// Everything works on the millisecond clock of time(), which survives standby
// and wraps every 49.7 days. The caller passes it in rather than the module
// reading it, so the whole thing can be exercised on a host.

#define TX_GRID_NEVER 0xFFFFFFFFu

// The interval in use, already multiplied by broadcast_scale_factor. Call it
// whenever the value may have changed - the factor follows the battery voltage
// and the setting can be changed over the console. A changed interval rebuilds
// the pending slot from the last packet sent, so a station that recovers from
// low voltage does not stay quiet for the rest of the old, longer interval.
// An interval of 0 disables the grid.
void tx_grid_set_interval(uint32_t now, uint32_t interval_ms);

// Milliseconds until the next attempt is due, 0 if it is due now, TX_GRID_NEVER
// while the grid is disabled.
uint32_t tx_grid_due_in(uint32_t now);

// A packet went out. The grid moves on by one interval - not by "one interval
// from now", which is what let the rhythm walk.
void tx_grid_sent(uint32_t now);

// The packet could not go out: the channel was busy, or the wind data was not
// ready. Try again after delay_ms; the slot itself does not move.
void tx_grid_retry(uint32_t now, uint32_t delay_ms);

// The slot the next packet belongs in, for diagnostics and tests.
uint32_t tx_grid_slot(void);

void tx_grid_reset(void);
