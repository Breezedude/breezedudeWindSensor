// Host test for the real src/settings_edit.cpp (copied in next to these stubs
// by run.sh so its quoted includes resolve to them).
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <string>
#include <vector>

// ---- captured console output ----
static std::string g_out;
void log_i(const char *msg){ g_out += msg; }
void log_write(uint32_t n){ char b[16]; snprintf(b, sizeof(b), "%u", (unsigned)n); g_out += b; }
void log_flush(){}

// ---- fake settings flash ----
unsigned char fake_settings_flash[768];

// ---- captured flash write ----
static std::vector<std::pair<std::string,std::string>> g_written;
static bool g_write_should_fail = false;
static int  g_write_calls = 0;
static bool g_ota_busy = false;

bool ota_lora_busy(){ return g_ota_busy; }

bool update_multiple_settings_in_flash(const char** keys, const char** values, size_t count){
  g_write_calls++;
  if(g_write_should_fail) return false;
  g_written.clear();
  for(size_t i = 0; i < count; i++) g_written.push_back({keys[i], values[i]});
  return true;
}

// ---- the firmware's persistable-key table, mirrored for the test ----
static const char *kCfg[] = {
  "NAME","LON","LAT","ALT","BROADCAST_INTERVAL_WEATHER","HEADING_OFFSET","WDT","DEBUG"
};
static const char *kCmd[] = { "REBOOT","FORMAT","RESET","DELAY","SKIP_LORA" };

bool setting_key_persistable(const char *key){
  for(const char *k : kCfg) if(strcmp(k, key) == 0) return true;
  (void)kCmd;
  return false;
}

#include "settings_edit.cpp"

// ---- harness ----
static int g_fail = 0;

static bool feed(const char *line){
  g_out.clear();
  return settings_edit_handle_line(line, (int)strlen(line));
}

static void check(bool cond, const char *what){
  printf("  %-58s %s\n", what, cond ? "ok" : "FAIL");
  if(!cond) g_fail++;
}

static bool out_has(const char *needle){
  return g_out.find(needle) != std::string::npos;
}

static bool staged_is(std::vector<std::pair<std::string,std::string>> want){
  if(g_written.size() != want.size()) return false;
  for(size_t i = 0; i < want.size(); i++){
    if(g_written[i] != want[i]) return false;
  }
  return true;
}

int main(){
  printf("dispatch\n");
  check(feed("SET NAME=Foo"),            "SET is consumed");
  check(feed("SAVE"),                    "SAVE is consumed");
  check(feed("DUMP"),                    "DUMP is consumed");
  check(feed("SETLIST"),                 "SETLIST is consumed");
  check(feed("SETCLEAR"),                "SETCLEAR is consumed");
  check(!feed("REBOOT=1"),               "REBOOT=1 falls through to apply_setting");
  check(!feed("SETTINGS=1"),             "SETTINGS=1 does not match SET");
  check(!feed("SAVEPOINT=1"),            "SAVEPOINT=1 does not match SAVE");
  check(!feed(""),                       "empty line falls through");
  check(!feed("NAME=Foo"),               "plain KEY=VALUE still falls through");

  printf("\nvalue parsing\n");
  feed("SETCLEAR");
  feed("SET NAME=Wendelstein Nord");
  check(out_has("SET OK NAME"),          "spaces inside a value are preserved (accepted)");
  feed("SETLIST");
  check(out_has("NAME=Wendelstein Nord"),"  ... and stored verbatim");

  feed("SETCLEAR");
  feed("SET NAME = Foo Bar ");
  feed("SETLIST");
  check(out_has("NAME=Foo Bar"),         "spaces around '=' trimmed, inner/trailing handled");

  feed("SETCLEAR");
  feed("SET NAME=A=B");
  feed("SETLIST");
  check(out_has("NAME=A=B"),             "second '=' stays in the value");

  feed("SETCLEAR");
  feed("SET NAME=Foo\r");
  feed("SETLIST");
  // SETLIST terminates each line with \r\n itself, so the value kept the CR
  // only if we see it twice.
  check(out_has("NAME=Foo\r\n") && !out_has("NAME=Foo\r\r\n"), "trailing CR stripped");

  printf("\nvalidation\n");
  feed("SETCLEAR");
  check(feed("SET REBOOT=1")   && out_has("SET ERR unknown or non-persistable key: REBOOT"),
        "console command refused");
  check(feed("SET FORMAT=1")   && out_has("non-persistable"), "FORMAT refused");
  check(feed("SET NOPE=1")     && out_has("non-persistable"), "unknown key refused");
  check(feed("SET NAME")       && out_has("SET ERR expected KEY=VALUE"), "missing '=' refused");
  check(feed("SET NAME=")      && out_has("SET ERR empty value"), "empty value refused");
  check(feed("SET =1")         && out_has("SET ERR bad key"), "empty key refused");
  feed("SETLIST");
  check(out_has("STAGED 0"),   "no rejected command was staged");

  printf("\nstaging\n");
  feed("SETCLEAR");
  feed("SET NAME=First");
  feed("SET LAT=47.7");
  feed("SET NAME=Second");           // replaces, must not duplicate
  feed("SETLIST");
  check(out_has("STAGED 2"),         "re-setting a key replaces instead of duplicating");
  check(out_has("NAME=Second"),      "  ... with the new value");
  check(out_has("LAT=47.7"),         "  ... and the other entry intact");

  g_written.clear();
  feed("SAVE");
  check(out_has("SAVE OK 2 changes"),"SAVE reports the staged count");
  check(staged_is({{"NAME","Second"},{"LAT","47.7"}}) ||
        staged_is({{"LAT","47.7"},{"NAME","Second"}}),
        "SAVE hands both pairs to flash exactly once each");
  feed("SETLIST");
  check(out_has("STAGED 0"),         "stage cleared after a successful SAVE");

  printf("\nremoval offset arithmetic\n");
  feed("SETCLEAR");
  feed("SET NAME=AAAAAAAAAAAAAAAA");  // long first entry
  feed("SET LAT=1.5");
  feed("SET ALT=100");
  feed("SET NAME=B");                 // removes the long entry from the front
  g_written.clear();
  feed("SAVE");
  check(staged_is({{"LAT","1.5"},{"ALT","100"},{"NAME","B"}}),
        "removing a front entry keeps the later ones readable");

  feed("SETCLEAR");
  feed("SET NAME=x");
  feed("SET LAT=2.5");
  feed("SET LAT=3.5");                // remove the last entry, then re-add
  g_written.clear();
  feed("SAVE");
  check(staged_is({{"NAME","x"},{"LAT","3.5"}}),
        "removing the last entry leaves the buffer consistent");

  printf("\nlimits\n");
  feed("SETCLEAR");
  int accepted = 0;
  for(int i = 0; i < 20; i++){
    // cycle through the known keys so each SET is a distinct persistable key
    const char *k = kCfg[i % (int)(sizeof(kCfg)/sizeof(kCfg[0]))];
    char line[64];
    snprintf(line, sizeof(line), "SET %s=%d", k, i);
    feed(line);
    if(out_has("SET OK")) accepted++;
  }
  feed("SETLIST");
  check(out_has("STAGED 8"), "distinct keys collapse to 8, no duplicates, no overflow");

  feed("SETCLEAR");
  char longval[128];
  memset(longval, 'x', sizeof(longval)); longval[sizeof(longval)-1] = '\0';
  char line[200];
  snprintf(line, sizeof(line), "SET NAME=%s", longval);
  feed(line);
  check(out_has("SET ERR value too long"), "over-long value refused");

  // A value right at the station-name limit must still go through.
  feed("SETCLEAR");
  char maxval[64];
  memset(maxval, 'y', sizeof(maxval)); maxval[sizeof(maxval)-1] = '\0';
  snprintf(line, sizeof(line), "SET NAME=%s", maxval);
  feed(line);
  check(out_has("SET OK NAME"), "63 char value accepted");

  printf("\nSAVE failure path\n");
  feed("SETCLEAR");
  feed("SET NAME=Keep");
  g_write_should_fail = true;
  feed("SAVE");
  check(out_has("SAVE ERR flash write failed"), "flash failure reported");
  g_write_should_fail = false;
  feed("SETLIST");
  check(out_has("STAGED 1") && out_has("NAME=Keep"),
        "staged changes survive a failed SAVE so it can be retried");

  feed("SETCLEAR");
  feed("SAVE");
  check(out_has("SAVE ERR nothing staged"), "SAVE with empty stage refused");
  int calls_before = g_write_calls;
  feed("SAVE");
  check(g_write_calls == calls_before, "  ... and does not touch flash");

  printf("\nOTA guard\n");
  feed("SETCLEAR");
  feed("SET NAME=Later");
  g_ota_busy = true;
  calls_before = g_write_calls;
  feed("SAVE");
  check(out_has("SAVE ERR OTA in progress"), "SAVE refused while an OTA runs");
  check(g_write_calls == calls_before,       "  ... and no flash write is attempted");
  feed("SETLIST");
  check(out_has("STAGED 1") && out_has("NAME=Later"), "  ... staged changes are kept");
  g_ota_busy = false;
  g_written.clear();
  feed("SAVE");
  check(out_has("SAVE OK 1 changes") && staged_is({{"NAME","Later"}}),
        "  ... and go through once the OTA is done");

  printf("\nDUMP\n");
  const char *cfg = "NAME=Teststation\r\nLAT=45.6\r\n";
  memset(fake_settings_flash, 0xFF, sizeof(fake_settings_flash));
  memcpy(fake_settings_flash, cfg, strlen(cfg));
  feed("DUMP");
  check(out_has("DUMP BEGIN") && out_has("NAME=Teststation") &&
        out_has("LAT=45.6") && out_has("DUMP END"), "DUMP streams the stored text");
  check(g_out.find("\xFF") == std::string::npos, "  ... and stops at erased flash");

  memset(fake_settings_flash, 0xFF, sizeof(fake_settings_flash));
  feed("DUMP");
  check(out_has("DUMP BEGIN") && out_has("DUMP END"), "DUMP on a blank device still terminates");

  printf("\n%s\n", g_fail ? "FAILURES" : "all checks passed");
  return g_fail != 0;
}
