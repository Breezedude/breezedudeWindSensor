// Host stub: only what settings_edit.cpp uses from the real tools.h.
#pragma once
#include <Arduino.h>
uint32_t time();
bool setting_key_persistable(const char *key);
bool write_settings_flash(const char *text);
bool update_multiple_settings_in_flash(const char** keys, const char** values, size_t count);
