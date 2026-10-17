#pragma once
#include <stdint.h>

// Which board the firmware is running on. Hardware 2.5 moved two pins - the
// battery divider to PB02 and the red LED to PA16 - and is the first revision
// that can be told apart electrically: PA27 carries a 4.7k pull-down. Earlier
// boards leave that pad open, so against the internal pull-up they read high.
// Nothing before 2.5 is distinguished any further; those boards all share the
// old pin map. PA28 is wired the same way and reserved for the next revision
// that needs telling apart.
typedef enum : uint8_t {
  HW_LEGACY = 0,   // up to 2.4: no ID resistor, both ID pins read high
  HW_2_5    = 1,   // PA27 pulled down
} HW_Version;

// Reads the ID pins and leaves them in their lowest-power state. Must run
// before anything drives the red LED or reads the battery - first thing in
// setup(). Until it has run the pin map is the legacy one.
void hw_detect(void);

HW_Version hw_version(void);
const char *hw_name(void);     // for the boot log
uint8_t hw_id_code(void);      // the raw two bits, PA28 high, PA27 low
bool hw_id_unknown(void);      // a code this firmware has no pin map for

// The pins that differ between revisions.
uint8_t hw_pin_v_read(void);
uint8_t hw_pin_led_error(void);
