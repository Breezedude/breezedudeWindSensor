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
static float spread24 = 0, spread25 = 0;
static bool clip24 = false, clip25 = false;

// A battery divider that is really there reads a battery: somewhere between a
// deeply discharged cell and a full one, and steady across the samples. An
// open ADC pin reads whatever charge sits on it, and not twice the same.
//
// A pin at full scale is no evidence either way. The divider (100k/330k
// against the 1.0 V reference) clips at 4.19 V, so a full battery can get
// there - but so does any pin that carries more than 1 V, and that reads as
// steady as a battery. The first 2.5 boards showed exactly that on the old
// divider pin PB08: 4.19 V without a spread, which outvoted their correct ID
// pin and put them on the 2.4 pin map.
static bool looks_like_a_battery(float v, float spread, bool clipped) {
  return !clipped && v >= 2.0f && v <= 4.6f && spread < 0.15f;
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
  v24 = batt_sample(PIN_V_READ_HW24, &spread24, &clip24);
  v25 = batt_sample(PIN_V_READ_HW25, &spread25, &clip25);
  const bool bat24 = looks_like_a_battery(v24, spread24, clip24);
  const bool bat25 = looks_like_a_battery(v25, spread25, clip25);
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
uint8_t hw_rev_code(void) {
  return (uint8_t)(((uint8_t)version & 0x0Fu) | (forced ? 0x10u : 0u) | (overruled ? 0x20u : 0u) |
                   ((id_code & 0x03u) << 6));
}
uint8_t hw_id_code(void)    { return id_code; }
bool hw_id_unknown(void)    { return unknown; }
bool hw_id_overruled(void)  { return overruled; }
float hw_divider_v(void)    { return version == HW_2_5 ? v25 : v24; }

// "4.08 V (spread 210 mV)" or "4.19 V (full scale)". Integer formatting: the
// build has no float printf.
static int pin_text(char *out, size_t len, float v, float spread, bool clipped) {
  const unsigned mv = (unsigned)(v * 1000.0f + 0.5f);
  if (clipped) {
    return snprintf(out, len, "%u.%02u V (full scale)", mv / 1000u, (mv % 1000u) / 10u);
  }
  const unsigned spread_mv = (unsigned)(spread * 1000.0f + 0.5f);
  return snprintf(out, len, "%u.%02u V (spread %u mV)", mv / 1000u, (mv % 1000u) / 10u, spread_mv);
}

const char *hw_divider_text(void) {
  static char text[80];
  int n = snprintf(text, sizeof(text), "2.4 pin ");
  n += pin_text(text + n, sizeof(text) - n, v24, spread24, clip24);
  n += snprintf(text + n, sizeof(text) - n, ", 2.5 pin ");
  pin_text(text + n, sizeof(text) - n, v25, spread25, clip25);
  return text;
}

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
