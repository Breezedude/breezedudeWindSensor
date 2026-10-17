#pragma once
#include <stdint.h>

// Which board the firmware is running on. Hardware 2.5 moved two pins - the
// battery divider to PB02 and the red LED to PA16 - and is the first revision
// that can be told apart electrically: PA27 carries a 4.7k pull-down. Earlier
// boards leave that pad open, so against the internal pull-up they read high.
// Nothing before 2.5 is distinguished any further; those boards all share the
// old pin map. PA28 is wired the same way and reserved for the next revision
// that needs telling apart.
//
// The ID pin is not trusted on its own. An older board may have put something
// of its own on that pad - alpha 1.4 has it on an SPI chip select - and a pin
// map chosen wrongly reads the battery from an open pin: a dead battery, as far
// as the firmware can tell, undervoltage sleep, USB switched off. So the battery
// divider is sampled on both candidate pins and where it actually answers
// decides; the ID pin only gets to break a tie.
typedef enum : uint8_t {
  HW_LEGACY = 0,   // up to 2.4: no ID resistor, both ID pins read high
  HW_2_5    = 1,   // PA27 pulled down
} HW_Version;

// Reads the ID pins and leaves them in their lowest-power state, then samples
// the battery divider on both pins. Must run before anything drives the red
// LED or reads the battery - first thing in setup(). Until it has run the pin
// map is the legacy one.
void hw_detect(void);

// HW_REVISION=24 or =25 in settings.txt pins the map regardless of what the
// pins say - the way out for a board that fools the detection. Anything else
// leaves it automatic. Applied when the settings are read, which is after
// hw_detect() and before anything uses the pins.
void hw_force(int revision);
bool hw_forced(void);

HW_Version hw_version(void);
const char *hw_name(void);     // for the boot log
uint8_t hw_id_code(void);      // the raw two bits, PA28 high, PA27 low
bool hw_id_unknown(void);      // a code this firmware has no pin map for
bool hw_id_overruled(void);    // the divider contradicted the ID pin and won
float hw_divider_v24(void);    // what the 2.4 divider pin read at boot, V
float hw_divider_v25(void);    // what the 2.5 divider pin read at boot, V

// The pins that differ between revisions.
uint8_t hw_pin_v_read(void);
uint8_t hw_pin_led_error(void);
