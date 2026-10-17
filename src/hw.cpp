#include "hw.h"

#include <Arduino.h>

#include "defines.h"
#include "sensors.h"   // batt_sample()
#include "tools.h"     // pinDisable()

static HW_Version version = HW_LEGACY;
static uint8_t id_code = 0;
static bool unknown = false;
static bool overruled = false;
static float v24 = 0, v25 = 0;

// A battery divider that is really there reads a battery: somewhere between a
// deeply discharged cell and a full one, and steady across the samples. An
// open ADC pin reads whatever charge sits on it, and not twice the same.
static bool looks_like_a_battery(float v, float spread) {
  return v >= 2.0f && v <= 4.6f && spread < 0.15f;
}

void hw_detect(void) {
  // A revision that wants to be known ties its pin to ground through 4.7k;
  // against the internal pull-up of 20-60 kOhm that is a solid low. An open
  // pad reads high - once whatever capacitance hangs on it has charged
  // through that weak pull-up, so give it time and read more than once.
  pinMode(PIN_HW_ID0, INPUT_PULLUP);
  pinMode(PIN_HW_ID1, INPUT_PULLUP);
  delay(10);
  uint8_t low0 = 0, low1 = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (digitalRead(PIN_HW_ID0) == LOW) low0++;
    if (digitalRead(PIN_HW_ID1) == LOW) low1++;
    delayMicroseconds(200);
  }
  // Back to the lowest-power state the moment the answer is in: input buffer
  // off, no pull. Left pulled up, the 4.7k would draw around 70 uA for the
  // life of the station - more than the whole station sleeps on. On a board
  // that uses the pad for something else this is also the state it was in
  // before, nothing here ever drove it.
  pinDisable(PIN_HW_ID0);
  pinDisable(PIN_HW_ID1);

  const uint8_t id0 = (low0 == 3) ? 1u : 0u;   // all three reads, or it does not count
  const uint8_t id1 = (low1 == 3) ? 1u : 0u;
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

  // Now ask the hardware itself. The divider answers on exactly one of the
  // two pins; if that is not the one the ID pin points at, the ID pin is
  // wrong - an older board with something of its own on PA27, a 2.5 with the
  // resistor missing - and the divider decides. Both open, both plausible,
  // or a battery too flat to tell: the ID pin stands.
  float spread24 = 0, spread25 = 0;
  v24 = batt_sample(PIN_V_READ_HW24, &spread24);
  v25 = batt_sample(PIN_V_READ_HW25, &spread25);
  const bool bat24 = looks_like_a_battery(v24, spread24);
  const bool bat25 = looks_like_a_battery(v25, spread25);
  if (version == HW_2_5 && bat24 && !bat25) {
    version = HW_LEGACY;
    overruled = true;
  } else if (version == HW_LEGACY && bat25 && !bat24) {
    version = HW_2_5;
    overruled = true;
  }
}

static bool forced = false;

void hw_force(int revision) {
  if (revision == 24)      { version = HW_LEGACY; forced = true; }
  else if (revision == 25) { version = HW_2_5;    forced = true; }
}
bool hw_forced(void) { return forced; }

HW_Version hw_version(void) { return version; }
uint8_t hw_id_code(void)    { return id_code; }
bool hw_id_unknown(void)    { return unknown; }
bool hw_id_overruled(void)  { return overruled; }
float hw_divider_v24(void)  { return v24; }
float hw_divider_v25(void)  { return v25; }

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
