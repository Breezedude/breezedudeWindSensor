#include "settings_edit.h"

#include <string.h>

#include "tools.h"
#include "logging.h"
#include "ota_ab.h"
#include "ota_lora.h"

// update_multiple_settings_in_flash() accepts at most 16 keys per write.
#define SETTINGS_EDIT_MAX_ENTRIES 16
#define SETTINGS_EDIT_LINE_MAX 128
#define SETTINGS_EDIT_KEY_MAX 32
#define SETTINGS_EDIT_VALUE_MAX 96
// A FILE transfer that stops halfway would otherwise swallow every later
// command, so it is dropped once nothing has arrived for this long.
#define SETTINGS_EDIT_FILE_TIMEOUT_MS 30000UL

// One buffer for both modes - they are mutually exclusive. In EDIT_STAGE it
// holds packed "KEY\0VALUE\0" entries, in EDIT_FILE the raw settings.txt text.
// Sized to the flash region so a full file always fits.
static char     edit_buf[OTA_SETTINGS_SIZE];
static uint16_t edit_len = 0;

typedef enum : uint8_t {
  EDIT_IDLE,
  EDIT_STAGE,
  EDIT_FILE,
} EditMode;

static EditMode edit_mode = EDIT_IDLE;
static uint32_t file_last_line_ms = 0;

static uint8_t  stage_count = 0;
static uint16_t stage_key_off[SETTINGS_EDIT_MAX_ENTRIES];
static uint16_t stage_val_off[SETTINGS_EDIT_MAX_ENTRIES];

static const char* staged_key(uint8_t i)   { return edit_buf + stage_key_off[i]; }
static const char* staged_value(uint8_t i) { return edit_buf + stage_val_off[i]; }

static void edit_reset(){
  edit_len = 0;
  stage_count = 0;
  edit_mode = EDIT_IDLE;
}

// Staging --------------------------------------------------------------------

// Drops entry i and closes the gap, keeping the remaining offsets valid.
static void stage_remove(uint8_t i){
  const uint16_t entry_start = stage_key_off[i];
  const uint16_t entry_end   = (uint16_t)(stage_val_off[i] + strlen(staged_value(i)) + 1u);
  const uint16_t gap = (uint16_t)(entry_end - entry_start);

  memmove(edit_buf + entry_start, edit_buf + entry_end, (size_t)(edit_len - entry_end));
  edit_len = (uint16_t)(edit_len - gap);

  for(uint8_t j = i; j + 1u < stage_count; j++){
    stage_key_off[j] = (uint16_t)(stage_key_off[j + 1] - gap);
    stage_val_off[j] = (uint16_t)(stage_val_off[j + 1] - gap);
  }
  stage_count--;
}

static int stage_find(const char *key){
  for(uint8_t i = 0; i < stage_count; i++){
    if(strcmp(staged_key(i), key) == 0){
      return (int)i;
    }
  }
  return -1;
}

// Adds or replaces one staged key/value. Returns nullptr on success, otherwise
// a short reason for the reply line.
static const char* stage_set(const char *key, const char *value){
  const int existing = stage_find(key);
  if(existing >= 0){
    stage_remove((uint8_t)existing); // re-setting a key replaces it, never duplicates it
  }

  if(stage_count >= SETTINGS_EDIT_MAX_ENTRIES){
    return "stage full";
  }
  const size_t key_len = strlen(key);
  const size_t val_len = strlen(value);
  const size_t need = key_len + 1u + val_len + 1u;
  if((size_t)edit_len + need > sizeof(edit_buf)){
    return "stage buffer full";
  }

  stage_key_off[stage_count] = edit_len;
  memcpy(edit_buf + edit_len, key, key_len + 1u);
  edit_len = (uint16_t)(edit_len + key_len + 1u);

  stage_val_off[stage_count] = edit_len;
  memcpy(edit_buf + edit_len, value, val_len + 1u);
  edit_len = (uint16_t)(edit_len + val_len + 1u);

  stage_count++;
  edit_mode = EDIT_STAGE;
  return nullptr;
}

// Console replies ------------------------------------------------------------

static void reply_err(const char *cmd, const char *reason, const char *detail = nullptr){
  log_i(cmd);
  log_i(" ERR ");
  log_i(reason);
  if(detail){
    log_i(": ");
    log_i(detail);
  }
  log_i("\r\n");
  log_flush();
}

// Commands -------------------------------------------------------------------

static void cmd_dump(){
  log_i("DUMP BEGIN\r\n");
  // The settings region is memory mapped, so it can be streamed out in small
  // chunks instead of buffering the whole block in RAM.
  const uint8_t *src = (const uint8_t *)OTA_SETTINGS_ADDRESS;
  char chunk[65];
  size_t pos = 0;
  while(pos < OTA_SETTINGS_SIZE){
    size_t n = 0;
    while(n < sizeof(chunk) - 1u && pos < OTA_SETTINGS_SIZE){
      const uint8_t c = src[pos];
      if(c == 0x00u || c == 0xFFu){ // end of stored text / erased flash
        pos = OTA_SETTINGS_SIZE;
        break;
      }
      chunk[n++] = (char)c;
      pos++;
    }
    if(n == 0u){
      break;
    }
    chunk[n] = '\0';
    log_i(chunk);
  }
  log_i("\r\nDUMP END\r\n");
  log_flush();
}

// Characters that process_line() cannot carry through on the next boot, so a
// value containing one would silently come back different from what was sent:
//   '#' '!' '>'  end the line, truncating the value
//   '?'          toggles literal mode and is swallowed
//   >= 127       are skipped outright (so no umlauts in a station name)
// Returns the offending character, or 0 if the value round-trips cleanly.
static char value_unstorable_char(const char *value){
  for(const char *p = value; *p != '\0'; ++p){
    const unsigned char c = (unsigned char)*p;
    if(c == '#' || c == '!' || c == '>' || c == '?' || c >= 127u){
      return *p;
    }
  }
  return 0;
}

static void cmd_set(const char *arg){
  const char *eq = strchr(arg, '=');
  if(eq == nullptr){
    reply_err("SET", "expected KEY=VALUE");
    return;
  }

  // Key: everything before '=', trailing spaces trimmed.
  size_t key_len = (size_t)(eq - arg);
  while(key_len > 0u && (arg[key_len - 1u] == ' ' || arg[key_len - 1u] == '\t')){
    key_len--;
  }
  if(key_len == 0u || key_len >= SETTINGS_EDIT_KEY_MAX){
    reply_err("SET", "bad key");
    return;
  }
  char key[SETTINGS_EDIT_KEY_MAX];
  memcpy(key, arg, key_len);
  key[key_len] = '\0';

  // Value: everything after '=' verbatim, only leading spaces trimmed so
  // "SET NAME = Foo" works. Trailing whitespace was already stripped from the
  // line. Spaces and further '=' inside the value are preserved.
  const char *value = eq + 1;
  while(*value == ' ' || *value == '\t'){
    value++;
  }
  if(*value == '\0'){
    reply_err("SET", "empty value", key);
    return;
  }
  if(strlen(value) >= SETTINGS_EDIT_VALUE_MAX){
    reply_err("SET", "value too long", key);
    return;
  }

  if(!setting_key_persistable(key)){
    // Either unknown, or a console command such as REBOOT/FORMAT. Storing one
    // of those would make parse_file() replay it on every boot.
    reply_err("SET", "unknown or non-persistable key", key);
    return;
  }

  const char bad = value_unstorable_char(value);
  if(bad != 0){
    const char detail[4] = { '\'', bad, '\'', '\0' };
    reply_err("SET", "character cannot be stored", detail);
    return;
  }

  // A value containing spaces needs process_line()'s literal mode switched on,
  // otherwise the next boot reads "Wendelstein Nord" back as "WendelsteinNord".
  // The '?' is consumed by the parser and is not part of the value.
  char stored[SETTINGS_EDIT_VALUE_MAX + 1];
  if(strpbrk(value, " \t") != nullptr){
    stored[0] = '?';
    snprintf(stored + 1, sizeof(stored) - 1, "%s", value);
    value = stored;
  }

  const char *fail = stage_set(key, value);
  if(fail){
    reply_err("SET", fail, key);
    return;
  }

  log_i("SET OK ");
  log_i(key);
  log_i(" (");
  log_write((uint32_t)stage_count);
  log_i("/");
  log_write((uint32_t)SETTINGS_EDIT_MAX_ENTRIES);
  log_i(" staged)\r\n");
  log_flush();
}

static void cmd_setlist(){
  log_i("STAGED ");
  log_write((uint32_t)stage_count);
  log_i("\r\n");
  for(uint8_t i = 0; i < stage_count; i++){
    log_i("  ");
    log_i(staged_key(i));
    log_i("=");
    log_i(staged_value(i));
    log_i("\r\n");
  }
  log_flush();
}

static void cmd_setclear(){
  edit_reset();
  log_i("SET CLEARED\r\n");
  log_flush();
}

static void cmd_save(){
  if(stage_count == 0u){
    reply_err("SAVE", "nothing staged");
    return;
  }
  if(ota_lora_busy()){
    // The write erases and reprograms flash rows for several hundred
    // milliseconds, long enough to miss OTA chunks. Staged changes are kept,
    // so the host can retry once the transfer has finished.
    reply_err("SAVE", "OTA in progress, retry later");
    return;
  }

  const char *keys[SETTINGS_EDIT_MAX_ENTRIES];
  const char *values[SETTINGS_EDIT_MAX_ENTRIES];
  for(uint8_t i = 0; i < stage_count; i++){
    keys[i]   = staged_key(i);
    values[i] = staged_value(i);
  }

  const uint8_t written = stage_count;
  if(!update_multiple_settings_in_flash(keys, values, written)){
    reply_err("SAVE", "flash write failed");
    return;
  }

  edit_reset();
  log_i("SAVE OK ");
  log_write((uint32_t)written);
  log_i(" changes, REBOOT to apply\r\n");
  log_flush();
}

// Whole-file transfer --------------------------------------------------------

// True if text contains a line starting with "<key>=".
static bool has_key_line(const char *text, const char *key){
  const size_t key_len = strlen(key);
  const char *p = text;
  bool at_line_start = true;
  while(*p != '\0'){
    if(at_line_start && strncmp(p, key, key_len) == 0 && p[key_len] == '='){
      return true;
    }
    at_line_start = (*p == '\n');
    p++;
  }
  return false;
}

static void cmd_file_begin(){
  if(stage_count != 0u){
    reply_err("FILE", "staged changes pending, SAVE or SETCLEAR first");
    return;
  }
  edit_len = 0;
  edit_mode = EDIT_FILE;
  file_last_line_ms = time();
  log_i("FILE BEGIN OK, send lines, end with FILE END\r\n");
  log_flush();
}

static void cmd_file_abort(){
  edit_reset();
  log_i("FILE ABORTED\r\n");
  log_flush();
}

// Appends one received content line, normalised to CRLF like the rest of the
// settings writer. Returns false if the buffer is full (transfer is dropped).
static bool file_append_line(const char *line){
  const size_t line_len = strlen(line);
  const size_t need = line_len + 2u; // "\r\n"
  if((size_t)edit_len + need >= sizeof(edit_buf)){
    return false;
  }
  memcpy(edit_buf + edit_len, line, line_len);
  edit_len = (uint16_t)(edit_len + line_len);
  edit_buf[edit_len++] = '\r';
  edit_buf[edit_len++] = '\n';
  file_last_line_ms = time();
  return true;
}

static void cmd_file_end(){
  if(edit_mode != EDIT_FILE){
    reply_err("FILE", "no transfer active");
    return;
  }
  if(edit_len == 0u){
    edit_reset();
    reply_err("FILE", "no content received");
    return;
  }
  if(ota_lora_busy()){
    reply_err("FILE", "OTA in progress, retry later"); // buffer is kept
    return;
  }

  edit_buf[edit_len] = '\0';

  // Report, but do not refuse: a configuration missing a mandatory key is
  // still worth storing, because the alternative is being unable to write
  // anything at all over the console.
  const bool has_name = has_key_line(edit_buf, "NAME");
  const bool has_lat  = has_key_line(edit_buf, "LAT");
  const bool has_lon  = has_key_line(edit_buf, "LON");
  if(!has_name || !has_lat || !has_lon){
    log_i("FILE WARN missing:");
    if(!has_name){ log_i(" NAME"); }
    if(!has_lat) { log_i(" LAT"); }
    if(!has_lon) { log_i(" LON"); }
    log_i("\r\n");
  }

  const uint16_t bytes = edit_len;
  if(!write_settings_flash(edit_buf)){
    reply_err("FILE", "flash write failed"); // buffer is kept, host can retry
    return;
  }

  edit_reset();
  log_i("FILE OK ");
  log_write((uint32_t)bytes);
  log_i(" bytes, REBOOT to apply\r\n");
  log_flush();
}

// Dispatch -------------------------------------------------------------------

// Matches a command word at the start of line. On success returns a pointer to
// the argument (leading spaces skipped, may point at '\0').
static const char* match_cmd(const char *line, const char *cmd){
  const size_t n = strlen(cmd);
  if(strncmp(line, cmd, n) != 0){
    return nullptr;
  }
  const char *rest = line + n;
  if(*rest != '\0' && *rest != ' ' && *rest != '\t'){
    return nullptr; // e.g. "SETTINGS" must not match "SET"
  }
  while(*rest == ' ' || *rest == '\t'){
    rest++;
  }
  return rest;
}

bool settings_edit_handle_line(const char *line, int len){
  if(line == nullptr || len <= 0){
    return false;
  }
  if(len >= SETTINGS_EDIT_LINE_MAX){
    len = SETTINGS_EDIT_LINE_MAX - 1;
  }

  // Work on a trimmed copy: the raw line still carries \r and possibly padding.
  char buf[SETTINGS_EDIT_LINE_MAX];
  memcpy(buf, line, (size_t)len);
  buf[len] = '\0';

  int end = len;
  while(end > 0 && (buf[end - 1] == '\r' || buf[end - 1] == '\n' ||
                    buf[end - 1] == ' '  || buf[end - 1] == '\t')){
    buf[--end] = '\0';
  }

  // A transfer whose host went away must not swallow commands forever.
  if(edit_mode == EDIT_FILE && (time() - file_last_line_ms) > SETTINGS_EDIT_FILE_TIMEOUT_MS){
    edit_reset();
    log_i("FILE ABORTED (timeout)\r\n");
    log_flush();
  }

  if(edit_mode == EDIT_FILE){
    // Everything is content now, except the two commands that end the transfer.
    const char *arg = match_cmd(buf, "FILE");
    if(arg != nullptr){
      if(strcmp(arg, "END") == 0)   { cmd_file_end();   return true; }
      if(strcmp(arg, "ABORT") == 0) { cmd_file_abort(); return true; }
    }
    if(!file_append_line(buf)){
      edit_reset();
      reply_err("FILE", "too large, transfer dropped");
    }
    return true;
  }

  const char *start = buf;
  while(*start == ' ' || *start == '\t'){
    start++;
  }
  if(*start == '\0'){
    return false;
  }

  // Longest first, so the list reads correctly on its own; match_cmd() would
  // also reject "SETLIST" for "SET" because the next character is not a space.
  const char *arg = nullptr;
  if((arg = match_cmd(start, "SETCLEAR")) != nullptr) { cmd_setclear(); return true; }
  if((arg = match_cmd(start, "SETLIST")) != nullptr)  { cmd_setlist();  return true; }
  if((arg = match_cmd(start, "SET")) != nullptr)      { cmd_set(arg);   return true; }
  if((arg = match_cmd(start, "SAVE")) != nullptr)     { cmd_save();     return true; }
  if((arg = match_cmd(start, "DUMP")) != nullptr)     { cmd_dump();     return true; }
  if((arg = match_cmd(start, "FILE")) != nullptr){
    if(strcmp(arg, "BEGIN") == 0) { cmd_file_begin(); return true; }
    if(strcmp(arg, "ABORT") == 0) { cmd_file_abort(); return true; }
    if(strcmp(arg, "END") == 0)   { cmd_file_end();   return true; }
    reply_err("FILE", "expected BEGIN, END or ABORT");
    return true;
  }

  return false; // not ours, let process_line()/apply_setting() have it
}
