#pragma once
#include <Arduino.h>

// Edit settings.txt in bootloader flash over the USB CDC console, so a host
// tool can change individual parameters without uploading a whole file.
//
// Protocol (one command per line, replies are single lines with a fixed
// prefix so a host tool can parse them):
//
//   DUMP              -> "DUMP BEGIN" / raw settings.txt / "DUMP END"
//   SET KEY=VALUE     -> stage one change   "SET OK KEY (2/16)"
//   SETLIST           -> list staged changes
//   SETCLEAR          -> discard staged changes
//   SAVE              -> write all staged changes to flash in one erase cycle
//
// Changes are staged rather than written immediately because every write
// erases and rewrites the settings rows; editing eight parameters one at a
// time would mean eight erase cycles instead of one.
//
// Values take effect on the next boot, matching the LoRa config-update path in
// config_update.cpp. Send REBOOT after SAVE to apply them.
//
// Everything after the first '=' is taken verbatim, including spaces and
// further '=' characters, so this does not go through process_line().

// Handles one console line. Returns true if the line was a settings-editor
// command and has been consumed; false if the caller should keep processing it.
bool settings_edit_handle_line(const char *line, int len);
