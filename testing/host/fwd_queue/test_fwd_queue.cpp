// Host test for the real src/fwd_queue.cpp - the packets waiting to be forwarded:
// a second station's packet during the delay of the first, a full queue, a channel
// that stays busy, and the wrap of the 32 bit millisecond counter.
#include <cstdio>
#include <cstring>

#include "fwd_queue.h"

static int g_fail = 0;
static void check(bool cond, const char *what) {
  printf("  %-66s %s\n", what, cond ? "ok" : "FAIL");
  if (!cond) g_fail++;
}

static void add(uint8_t mark, uint32_t now, uint32_t delay_ms) {
  uint8_t pkt[18];
  memset(pkt, mark, sizeof(pkt));
  fwd_queue_add(pkt, sizeof(pkt), now, delay_ms);
}

int main() {
  printf("\nnothing waiting\n");
  fwd_queue_reset();
  check(fwd_queue_delay_ms(1000) == 0, "no delay to wake for");
  check(fwd_queue_due(1000) == nullptr, "nothing due");

  printf("\na second station during the delay of the first\n");
  add(0xA1, 1000, 1500);
  add(0xB2, 1200, 600);
  check(fwd_queue_delay_ms(1300) == 500, "the sooner of the two is what to wake for");
  check(fwd_queue_due(1300) == nullptr, "neither is due yet");
  FwdPacket *p = fwd_queue_due(1800);
  check(p && p->data[0] == 0xB2 && p->len == 18, "the second one is due first");
  fwd_queue_release(p);
  check(fwd_queue_due(1800) == nullptr, "  ... and gone once it is sent");
  check(fwd_queue_delay_ms(1800) == 700, "the first one is still waiting");
  p = fwd_queue_due(2500);
  check(p && p->data[0] == 0xA1, "  ... and goes out at its own time");
  fwd_queue_release(p);
  check(fwd_queue_delay_ms(2500) == 0, "nothing left");

  printf("\nboth due: the one that has waited longer\n");
  fwd_queue_reset();
  add(0xA1, 1000, 900);
  add(0xB2, 1100, 500);
  p = fwd_queue_due(2000);
  check(p && p->data[0] == 0xA1, "the older one first");
  check(fwd_queue_delay_ms(2000) == 1, "due counts as 1 ms, not as nothing waiting");

  printf("\na full queue\n");
  fwd_queue_reset();
  for (uint8_t i = 0; i < FWD_QUEUE_SLOTS; i++) add((uint8_t)(0x10 + i), 1000, 500);
  uint8_t pkt[18] = {0x77};
  check(!fwd_queue_add(pkt, sizeof(pkt), 1100, 500), "the next packet is refused");
  p = fwd_queue_due(1500);
  fwd_queue_release(p);
  check(fwd_queue_add(pkt, sizeof(pkt), 1600, 500), "after a send there is room for it");

  printf("\nwhat does not fit\n");
  fwd_queue_reset();
  uint8_t big[FWD_QUEUE_PACKET_MAX + 1] = {0};
  check(!fwd_queue_add(big, sizeof(big), 1000, 0), "too long");
  check(!fwd_queue_add(big, 0, 1000, 0), "empty");
  check(fwd_queue_add(big, FWD_QUEUE_PACKET_MAX, 1000, 0), "the longest there is");

  printf("\na channel that stays busy\n");
  fwd_queue_reset();
  add(0xA1, 1000, 500);
  add(0xB2, 20000, 500);
  p = fwd_queue_due(1500);
  fwd_queue_defer(p, 1500, 2000);
  check(fwd_queue_due(1600) == nullptr, "deferred, not due");
  p = fwd_queue_due(3500);
  check(p && p->data[0] == 0xA1, "due again after the deferral");
  check(fwd_queue_expire(1000 + FWD_QUEUE_MAX_AGE_MS - 1) == 0, "still fresh just before the limit");
  check(fwd_queue_expire(1000 + FWD_QUEUE_MAX_AGE_MS) == 1, "too old: dropped");
  p = fwd_queue_due(1000 + FWD_QUEUE_MAX_AGE_MS);
  check(p && p->data[0] == 0xB2, "  ... the younger one stays");

  printf("\nthe counter wraps\n");
  fwd_queue_reset();
  const uint32_t wrap = 0xFFFFFF00u;
  add(0xA1, wrap, 0x200);
  check(fwd_queue_delay_ms(wrap + 0x100) == 0x100, "delay across the wrap");
  check(fwd_queue_due(wrap + 0x100) == nullptr, "not due before the wrap");
  check(fwd_queue_due(wrap + 0x200) != nullptr, "due after it");
  check(fwd_queue_expire(wrap + 0x200) == 0, "not expired by the wrap");

  printf("\n%s\n", g_fail ? "SOME CHECKS FAILED" : "all checks passed");
  return g_fail ? 1 : 0;
}
