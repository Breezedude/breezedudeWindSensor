#include "fwd_queue.h"

#include <string.h>

static FwdPacket slots[FWD_QUEUE_SLOTS];

void fwd_queue_reset(void) {
  for (uint8_t i = 0; i < FWD_QUEUE_SLOTS; i++) slots[i].len = 0;
}

bool fwd_queue_add(const uint8_t *data, int len, uint32_t now, uint32_t delay_ms) {
  if (len <= 0 || len > FWD_QUEUE_PACKET_MAX) return false;
  for (uint8_t i = 0; i < FWD_QUEUE_SLOTS; i++) {
    FwdPacket *p = &slots[i];
    if (p->len) continue;
    memcpy(p->data, data, (size_t)len);
    p->len = (uint16_t)len;
    p->queued_at = now;
    p->send_at = now + delay_ms;
    return true;
  }
  return false;
}

uint8_t fwd_queue_expire(uint32_t now) {
  uint8_t dropped = 0;
  for (uint8_t i = 0; i < FWD_QUEUE_SLOTS; i++) {
    if (slots[i].len && (int32_t)(now - slots[i].queued_at) >= (int32_t)FWD_QUEUE_MAX_AGE_MS) {
      slots[i].len = 0;
      dropped++;
    }
  }
  return dropped;
}

FwdPacket *fwd_queue_due(uint32_t now) {
  FwdPacket *oldest = nullptr;
  for (uint8_t i = 0; i < FWD_QUEUE_SLOTS; i++) {
    FwdPacket *p = &slots[i];
    if (!p->len || (int32_t)(now - p->send_at) < 0) continue;
    if (!oldest || (int32_t)(p->queued_at - oldest->queued_at) < 0) oldest = p;
  }
  return oldest;
}

uint32_t fwd_queue_delay_ms(uint32_t now) {
  uint32_t best = 0;
  for (uint8_t i = 0; i < FWD_QUEUE_SLOTS; i++) {
    if (!slots[i].len) continue;
    const int32_t left = (int32_t)(slots[i].send_at - now);
    const uint32_t d = left > 1 ? (uint32_t)left : 1u;
    if (!best || d < best) best = d;
  }
  return best;
}
