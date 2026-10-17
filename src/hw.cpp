#include "hw.h"

#include <Arduino.h>

#include "defines.h"
#include "tools.h"   // pinDisable()

static HW_Version version = HW_LEGACY;
static uint8_t id_code = 0;
static bool unknown = false;

void hw_detect(void) {
  // A revision that wants to be known ties its pin to ground through 4.7k;
  // against the internal pull-up of 20-60 kOhm that is a solid low. An open
  // pad reads high.
  pinMode(PIN_HW_ID0, INPUT_PULLUP);
  pinMode(PIN_HW_ID1, INPUT_PULLUP);
  delayMicroseconds(100);   // the pull-up is weak, give the line a moment
  const uint8_t id0 = digitalRead(PIN_HW_ID0) == LOW ? 1u : 0u;
  const uint8_t id1 = digitalRead(PIN_HW_ID1) == LOW ? 1u : 0u;
  // Back to the lowest-power state the moment the answer is in: input buffer
  // off, no pull. Left pulled up, the 4.7k would draw around 70 uA for the
  // life of the station - more than the whole station sleeps on.
  pinDisable(PIN_HW_ID0);
  pinDisable(PIN_HW_ID1);

  id_code = (uint8_t)((id1 << 1) | id0);
  switch (id_code) {
    case 0:  version = HW_LEGACY; break;
    case 1:  version = HW_2_5;    break;
    default:
      // A code this firmware does not know, so a later revision. The newest
      // pin map it has is the best guess; the boot log says that it guessed.
      version = HW_2_5;
      unknown = true;
      break;
  }
}

HW_Version hw_version(void) { return version; }
uint8_t hw_id_code(void)    { return id_code; }
bool hw_id_unknown(void)    { return unknown; }

const char *hw_name(void) {
  switch (version) {
    case HW_2_5: return "2.5";
    default:     return "2.4 or earlier";
  }
}

uint8_t hw_pin_v_read(void) {
  return version == HW_2_5 ? PIN_V_READ_HW25 : PIN_V_READ_HW24;
}

uint8_t hw_pin_led_error(void) {
  return version == HW_2_5 ? PIN_ERRORLED_HW25 : PIN_ERRORLED_HW24;
}
