#pragma once
#include <stdint.h>

// Packets waiting to be forwarded. A forward goes out 0.5-2 s after the packet
// arrived, and with room for only one, anything another station sent in that
// time was not forwarded at all.
//
// Like tx_grid and fanet_sched this module never reads a clock, the caller passes
// the time in, so it can be exercised on a host.
#define FWD_QUEUE_SLOTS       4
#define FWD_QUEUE_PACKET_MAX  255
// A busy channel defers a forward again and again. Weather this old is no use to
// anybody, and the slot is needed for the next packet.
#define FWD_QUEUE_MAX_AGE_MS  30000u

struct FwdPacket {
  uint32_t queued_at;
  uint32_t send_at;
  uint16_t len;      // 0: slot free
  uint8_t  data[FWD_QUEUE_PACKET_MAX];
};

void fwd_queue_reset(void);
// False when the packet does not fit: no slot left, or too long.
bool fwd_queue_add(const uint8_t *data, int len, uint32_t now, uint32_t delay_ms);
// Frees what has been waiting too long and returns how many that were.
uint8_t fwd_queue_expire(uint32_t now);
// The longest waiting packet that is due, nullptr if none is. It stays in the
// queue until fwd_queue_release() or fwd_queue_defer().
FwdPacket *fwd_queue_due(uint32_t now);
static inline void fwd_queue_defer(FwdPacket *p, uint32_t now, uint32_t delay_ms) { p->send_at = now + delay_ms; }
static inline void fwd_queue_release(FwdPacket *p) { p->len = 0; }
// Milliseconds until the next packet is due, at least 1 while one waits, so a
// caller can tell "due now" from "nothing waiting", which is 0.
uint32_t fwd_queue_delay_ms(uint32_t now);
