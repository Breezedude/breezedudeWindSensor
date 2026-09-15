#ifdef WAKE_PROFILE
#include "wake_profile.h"
#include "logging.h"

static uint32_t marks[WP_COUNT];
static uint32_t last_cycle[WP_COUNT];
static bool last_cycle_ready = false;

void wake_mark(WakeMark m){
  if(m == WP_WAKE){
    memset(marks, 0, sizeof(marks));
  }
  marks[m] = micros();
  if(m == WP_DEEP && marks[WP_WAKE]){
    memcpy(last_cycle, marks, sizeof(last_cycle));
    last_cycle_ready = true;
  }
}

void wake_profile_report(unsigned cpu_divider){
  if(!last_cycle_ready){
    return;
  }
  last_cycle_ready = false;
  // Short names in the PROF line, one per WakeMark; WP_WAKE is time zero.
  static const char *const names[WP_COUNT] = {
    "", "slp", "pc", "dir", "wu", "loop", "tts", "log", "rtc", "rad", "arm", "enq", "fl", "pre"
  };
  char line[220];
  int n = snprintf(line, sizeof(line), "PROF d%u", cpu_divider);
  uint32_t prev = last_cycle[WP_WAKE];
  for(int i = 1; i < WP_COUNT && n < (int)sizeof(line) - 24; i++){
    if(!last_cycle[i]){
      n += snprintf(line + n, sizeof(line) - n, " %s-", names[i]); // mark not reached in this cycle
      continue;
    }
    n += snprintf(line + n, sizeof(line) - n, " %s%lu", names[i], (unsigned long)((last_cycle[i] - prev) / 100));
    prev = last_cycle[i];
  }
  snprintf(line + n, sizeof(line) - n, " tot%lu\r\n",
           (unsigned long)((last_cycle[WP_DEEP] - last_cycle[WP_WAKE]) / 100));
  log_d(line);
}
#endif
