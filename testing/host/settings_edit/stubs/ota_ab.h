// Host stub: the settings region is a plain array here instead of mapped flash.
#pragma once
#include <Arduino.h>
extern unsigned char fake_settings_flash[768];
#define OTA_SETTINGS_ADDRESS ((uintptr_t)fake_settings_flash)
#define OTA_SETTINGS_SIZE    ((size_t)768)
