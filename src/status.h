#pragma once
#include <Arduino.h>

// One-line machine readable device status, emitted by the STATUS console
// command on both consoles (USB CDC and the debug UART).
//
// This exists so tools stop scraping the boot log with regular expressions.
// breezedude-analyze.js currently recovers version, FANET id, sensor type and
// station name from eight separate log-line patterns, and a test rig would
// have had to reimplement the same guesswork.
//
// All numbers are integers - newlib-nano leaves printf float support out, and
// scaled integers parse more reliably anyway. Units are in the key names.

// Builds the JSON document into out. Returns the length written, or 0 if it
// did not fit (in which case out is left as a valid, truncation-flagged
// document rather than half a line of JSON).
size_t status_build_json(char *out, size_t out_len);

// Builds and prints it, followed by CRLF.
void status_print_json();
// The same with awake_pm taken from the given window instead of the one since
// the last HWInfo report.
void status_print_json_window(uint32_t awake_ms, uint32_t total_ms);
// Prints the document every STATUS_INTERVAL minutes; call from the main loop.
void status_periodic();
