#pragma once
// Wake-phase profiler, compiled in only for a diagnostic build: add -DWAKE_PROFILE
// to the build flags, or set PLATFORMIO_BUILD_FLAGS=-DWAKE_PROFILE for one build.
// Without the flag both functions are empty inlines and the release image does
// not change. The output is a log_d() line, so it needs DEBUG=1.
//
// wake_mark() stores micros() at fixed points of a wake cycle, in the order of
// the enum. A cycle runs from one return from standby (WP_WAKE) to the next
// entry into standby (WP_DEEP). Its line is printed during the cycle that
// follows, from sleep() just before the log flush, so the printing does not land
// in the phases it reports. A DAVIS wake with GCLK6 at 8192 Hz, recorded on the
// rig:
//
//   PROF d1 slp6 pc1 dir10 wu2 loop0 tts0 log1 rtc3 rad16 arm1 enq19 fl162 pre0 tot228
//
// Each value is the time since the previous mark the cycle reached, in 0.1 ms,
// named after the mark it ends at (see the enum). A name with "-" instead of a
// value is a mark the cycle did not pass: a UART sensor cycle has no pulse
// counter, and a wakeup inside a DAVIS window goes straight back to sleep. tot
// runs from WP_WAKE to WP_DEEP. d is the CPU clock divider: micros() runs on the
// CPU clock, which is divided by div_cpu after the first sleep.
#include <Arduino.h>

enum WakeMark : uint8_t {
  WP_WAKE,      //        deepsleep() returned, pins restored - time zero
  WP_SLEEP_RET, // "slp"  end of sleep(): sensor UART back, time counter read, WDT on
  WP_PULSE,     // "pc"   DAVIS: pulse counter read
  WP_DIR,       // "dir"  DAVIS: calc_pulse_sensor(), including the direction ADC
  WP_WAKEUP,    // "wu"   end of go_sleep(): wakeup()
  WP_GO,        // "loop" go_sleep() entered: loop() in between
  WP_TTS,       // "tts"  time to sleep calculated
  WP_LOG,       // "log"  "will sleep" logged - only queued, sleep() flushes
  WP_RTC,       // "rtc"  rtc_sleep_cfg()
  WP_RADIO,     // "rad"  radio_sleep()
  WP_ARM,       // "arm"  DAVIS: arm_pulse_counter()
  WP_FLUSH0,    // "enq"  sleep() up to its log_flush(): WDT off, sensor UART armed,
                //        this profile line queued. A UART sensor cycle passes no
                //        mark in between, so reading the sensor lands here too.
  WP_FLUSH1,    // "fl"   log_flush() done: the UART has drained
  WP_DEEP,      // "pre"  pins quiet, about to enter standby
  WP_COUNT
};

#ifdef WAKE_PROFILE
void wake_mark(WakeMark m);
void wake_profile_report(unsigned cpu_divider);
#else
inline void wake_mark(WakeMark) {}
inline void wake_profile_report(unsigned) {}
#endif
