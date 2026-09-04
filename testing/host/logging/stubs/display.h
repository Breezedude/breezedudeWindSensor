// Host stub: the firmware's display.h. Present but empty - the classification
// tests do not care about the display, and display_present() being a runtime
// value here (not a constant false) means the String paths get compiled,
// which is exactly what should be checked for compile errors.
#pragma once
#include <Arduino.h>

bool display_present();
void display_add_line(String txt);
