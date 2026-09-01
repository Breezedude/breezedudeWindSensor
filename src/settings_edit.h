#pragma once
#include <Arduino.h>

// Edit settings.txt in bootloader flash over the USB CDC console, so a host
// tool can change parameters without uploading a file through the USB mass
// storage device (which is not reliable on every host OS).
//
// Protocol: one command per line. Replies start with a fixed prefix so a host
// tool can parse them; errors are always "<CMD> ERR <reason>".
//
//   DUMP              -> "DUMP BEGIN" / raw settings.txt / "DUMP END"
//
//   -- patch individual parameters --
//   SET KEY=VALUE     -> stage one change   "SET OK NAME (2/16 staged)"
//   SETLIST           -> list staged changes
//   SETCLEAR          -> discard staged changes
//   SAVE              -> write all staged changes in one erase cycle
//
//   -- replace the whole file (fallback when a file write is not possible) --
//   FILE BEGIN        -> start collecting; every following line is content
//   ...lines...
//   FILE END          -> write the collected text as the new settings.txt
//   FILE ABORT        -> discard
//
// Changes are staged rather than written per SET because every write erases and
// reprograms the settings rows; editing eight parameters one at a time would
// cost eight erase cycles instead of one.
//
// Values take effect on the next boot, matching the LoRa config-update path in
// config_update.cpp. Send REBOOT after SAVE or FILE END to apply them.
//
// Everything after the first '=' in a SET is taken verbatim, including spaces
// and further '=' characters, so this does not go through process_line().

// Handles one console line. Returns true if the line was consumed by the
// settings editor; false if the caller should keep processing it. While a FILE
// transfer is active every line is consumed as content except FILE END/ABORT.
bool settings_edit_handle_line(const char *line, int len);
