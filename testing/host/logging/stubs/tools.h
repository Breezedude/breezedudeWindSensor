// Host stub: only the settings field logging.cpp reads.
#pragma once
#include <Arduino.h>

struct HostSettings {
  bool verbose_usb;
  bool uart_wake = false;   // read by log_ser_begin()
};
extern HostSettings settings;
