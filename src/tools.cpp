#include "tools.h"
#include "defines.h"
#include "logging.h"
#include "sleep.h"
#include "ota_ab.h"
#include "ota_lora.h"
#include "config_update.h"
#include <Adafruit_TinyUSB.h>

extern LORA_MODULE lora_module;
extern CHIP_BARO chip_baro;
extern volatile int wakeup_source;

uint32_t sleeptime_cum = 0; // cumulative time spend in sleepmode, used for time() calculation
Settings settings;
RuntimeState state;
volatile bool usb_detach_event = false;
volatile bool usb_ignore_detach_event = false;
static uint32_t usb_last_activity_ms = 0;

bool format_flash(){
  bool ok = ota_storage_write_text(OTA_SETTINGS_ADDRESS, OTA_SETTINGS_SIZE, "");
  if(ok){
    ota_storage_write_text(OTA_VERSIONS_ADDRESS, OTA_VERSIONS_SIZE, "");
    log_i("Bootloader settings storage erased\r\n");
  } else {
    log_e("Error: failed to erase bootloader settings storage\r\n");
  }
  return ok;
}

bool setup_flash(){
  char probe[8] = {0};
  if(ota_storage_read_text(OTA_SETTINGS_ADDRESS, probe, sizeof(probe))){
    log_v("Bootloader settings storage ready\r\n");
  } else {
    log_i("Bootloader settings storage empty, waiting for user config\r\n");
  }
  return true;
}

void log_reset_cause(){
  const uint8_t cause = PM->RCAUSE.reg;
  log_write("Reset cause: 0x");
  log_write_hex(cause, 2);
  log_write(" [");

  bool any = false;
  if(cause & PM_RCAUSE_POR)   { log_write("POR "); any = true; }
  if(cause & PM_RCAUSE_BOD12) { log_write("BOD12 "); any = true; }
  if(cause & PM_RCAUSE_BOD33) { log_write("BOD33 "); any = true; }
  if(cause & PM_RCAUSE_EXT)   { log_write("EXT "); any = true; }
  if(cause & PM_RCAUSE_WDT)   { log_write("WDT "); any = true; }
  if(cause & PM_RCAUSE_SYST)  { log_write("SYST "); any = true; }
  if(!any) {
    log_write("unknown");
  }
  log_write("]\r\n");
}

bool parse_file(char * filename){
  #define LINEBUFFERSIZE 512
  bool ret = false;
  bool parsed = false;
  led_status(1);

  // Reset to struct defaults before each parse attempt.
  // Optional parameters keep their built-in defaults, while mandatory fields
  // must be supplied by the user config below.
  settings = Settings();

  if(strcmp(filename, SETTINGSFILE) == 0){
    char linebuffer[LINEBUFFERSIZE] = {0};
    char rawbuf[LINEBUFFERSIZE] = {0};
    const char *text = nullptr;
    int co = 0;
    auto parse_settings_line = [&](int line_length) {
      if(line_length <= 0){
        return;
      }
      linebuffer[line_length] = '\0';
      process_line(linebuffer, line_length, &apply_setting);
    };

    if(ota_storage_read_text(OTA_SETTINGS_ADDRESS, rawbuf, sizeof(rawbuf))){
      log_v("Reading settings from bootloader storage\r\n");
      text = rawbuf;
    } else {
      log_e("Bootloader settings empty, waiting for user config\r\n");
      text = nullptr;
    }

    if(text){
      for(size_t i = 0; text[i] != '\0'; ++i){
        linebuffer[co] = text[i];
        if(linebuffer[co] == '\n'){
          parse_settings_line(co);
          co = -1;
        }
        co++;
        if(co >= LINEBUFFERSIZE){
          log_e("Bootloader settings buffer error\r\n");
          return false;
        }
      }
      if(co > 0){
        parse_settings_line(co);
      }
      parsed = true;
    }
  }

  if(parsed){
    ret = true;

    if(settings.station_name[0] == '\0'){
      ret = false;
      log_e("Missing mandatory setting: NAME\r\n");
    }

    if(settings.pos_lat == 0 || settings.pos_lon == 0){
      ret = false;
      log_e("Missing or invalid mandatory location (LAT/LON)\r\n");
    }

    if(settings.sensor_type == s_invalid && !settings.repeater){
      ret = false;
      log_e("Missing mandatory setting: sensor type\r\n");
    }

    led_status(0);
  }
  if(!ret){
    led_status(0);
    led_error(1);
  }
  log_flush();
  return ret;
}

void read_serial_cmd(){
  #define CMDBUFFERSIZE 127
  static char buffer [CMDBUFFERSIZE];
  static int co = 0;
  bool ok = false;

  while (Serial.available()){
    usb_connected = true;
    usb_last_activity_ms = millis();
    // Drop an over-long line instead of writing past the buffer: input longer
    // than CMDBUFFERSIZE without a newline used to corrupt memory here.
    if(co >= (CMDBUFFERSIZE - 1)){
      log_e("Serial cmd too long, discarded\r\n");
      co = 0;
    }
    buffer[co] = Serial.read();
    if(buffer[co] == '\n'){
      ok = process_line(buffer, co, &apply_setting); // Line complete
      co=-1; // against +1 below
    }
    co++;
  }
  if(ok){
    // Apply new mppt voltage
  }
}

void log_v_hex_dump(const uint8_t *data, size_t len){
  if(!settings.verbose_usb || !usb_connected){
    return;
  }

  size_t i = 0;
  while(i < len){
    size_t chunk = min((size_t)24, len - i);
    for(size_t j = 0; j < chunk; j++){
      log_write_hex(data[i + j], 2);
      log_i(" ");
    }
    log_i("\r\n");
    i += chunk;
  }
}

extern "C" void tud_mount_cb(void) {
  usb_connected = true;
  usb_last_activity_ms = millis();
  usb_detach_event = false;
  usb_ignore_detach_event = false;
  wakeup_source = WAKEUP_USB;
}

extern "C" void tud_cdc_rx_cb(uint8_t itf) {
  (void)itf;
  wakeup_source = WAKEUP_USB;
}

extern "C" void tud_umount_cb(void) {
  usb_connected = false;
  if(!usb_ignore_detach_event) {
    usb_detach_event = true;
  }
}

static bool usb_fsm_disconnected(){
  // SAMD21 device FSM: OFF (L3) means disconnected/disabled.
  return USB->DEVICE.FSMSTATUS.bit.FSMSTATE == 0x01;
}

static bool usb_frame_changed(){
  static uint8_t last_frame = 0;
  uint8_t cur = USB->DEVICE.FNUM.bit.FNUM;
  bool changed = (cur != last_frame);
  last_frame = cur;
  return changed;
}

void handle_usb_link_watchdog(){
  static bool usb_prev_connected = false;
  const uint32_t now = millis();

  if(ota_lora_busy()) {
    usb_detach_event = false;
    return;
  }

  const bool dtr = Serial.dtr();
  const bool fsm_disconnected = usb_fsm_disconnected();
  const bool frame_progress = usb_frame_changed();

  if(frame_progress || dtr) {
    usb_last_activity_ms = now;
  }

  const bool recent_activity = usb_last_activity_ms && ((now - usb_last_activity_ms) <= 1500UL);
  bool connected = (dtr || recent_activity) && !fsm_disconnected;

  if(usb_detach_event && !usb_ignore_detach_event){
    connected = false;
  }

  usb_connected = connected;

  if(connected != usb_prev_connected){
    if(connected){
      log_i("USB connected\r\n");
      usb_detach_event = false;
      usb_ignore_detach_event = false;
    } else {
      log_i("USB disconnected\r\n");
      usb_detach_event = false;
      usb_last_activity_ms = 0;
    }
    usb_prev_connected = connected;
  }
}


void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode, bool en_rtc) {
	EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
	if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI){
    return;
  }
	attachInterrupt(pin, callback, mode);
	configGCLK6(en_rtc);
	// Enable wakeup capability on pin in case being used during sleep
	EIC->WAKEUP.reg |= (1 << in);
}

// Helper ----------------------------------------------------------------------------------------------------------------------

const char* wakeup_source_string[] = {"NONE", "RTC", "EIC", "WDT", "UART", "LORA", "USB"};

const char* get_bootloader_version(){
  static char version[40] = "unknown";
  static bool cached = false;
  if(cached){
    return version;
  }
  cached = true;

  constexpr uintptr_t BOOT_FLASH_START = 0x00000000UL;
  constexpr uintptr_t BOOT_FLASH_END   = 0x00004000UL;
  static const char prefix[] = "UF2 Bootloader ";
  constexpr size_t prefix_len = sizeof(prefix) - 1u;

  const uint8_t *flash_ptr = reinterpret_cast<const uint8_t *>(BOOT_FLASH_START);
  const size_t scan_len = (size_t)(BOOT_FLASH_END - BOOT_FLASH_START);

  for(size_t i = 0; i + prefix_len < scan_len; ++i){
    if(memcmp(flash_ptr + i, prefix, prefix_len) != 0){
      continue;
    }

    char buf[sizeof(version)];
    size_t n = 0;
    const char *s = reinterpret_cast<const char *>(flash_ptr + i + prefix_len);
    while(n < sizeof(buf) - 1u){
      char c = s[n];
      if(c == '\0' || c == '\r' || c == '\n' || (uint8_t)c == 0xFFu){
        break;
      }
      buf[n++] = c;
    }
    buf[n] = '\0';

    if(n > 0u){
      memcpy(version, buf, n + 1u);
    }
    break;
  }

  return version;
}

// Copies the bootloader version into dst, cut off at the "-0-g<sha>" git
// suffix and at the first space.
static void get_bootloader_version_for_version_text(char *dst, size_t dst_len){
#ifdef BOOTLOADER_VERSION_TXT_OVERRIDE
  snprintf(dst, dst_len, "%s", BOOTLOADER_VERSION_TXT_OVERRIDE);
#else
  snprintf(dst, dst_len, "%s", get_bootloader_version());
  char *git_suffix = strstr(dst, "-0-g");
  if(git_suffix != nullptr && git_suffix != dst){
    *git_suffix = '\0';
  }
  char *feature_suffix = strchr(dst, ' ');
  if(feature_suffix != nullptr && feature_suffix != dst){
    *feature_suffix = '\0';
  }
#endif
}

float parse_decimal_fast(const char *s){
  if(!s){
    return 0.0f;
  }

  while(*s == ' ' || *s == '\t'){
    s++;
  }

  bool neg = false;
  if(*s == '-' || *s == '+'){
    neg = (*s == '-');
    s++;
  }

  uint32_t int_part = 0;
  while(*s >= '0' && *s <= '9'){
    int_part = int_part * 10u + (uint32_t)(*s - '0');
    s++;
  }

  uint32_t frac_part = 0;
  uint32_t frac_scale = 1;
  if(*s == '.' || *s == ','){
    s++;
    while(*s >= '0' && *s <= '9'){
      if(frac_scale < 1000000u){
        frac_part = frac_part * 10u + (uint32_t)(*s - '0');
        frac_scale *= 10u;
      }
      s++;
    }
  }

  float value = (float)int_part + ((float)frac_part / (float)frac_scale);
  return neg ? -value : value;
}

// if a WS80 or WS85 sensor with "DEBUG hack" is connected
bool is_wsxx(){
  return settings.sensor_type == s_WS85 || settings.sensor_type == s_WS80;
}

// process line in 'key=value' format and hand to callback function
bool process_line(char * in, int len, bool (*cb)(char*, char*)){
  #define BUFFLEN 127
  char name [BUFFLEN];
  char value [BUFFLEN];
  memset(name,'\0',BUFFLEN);
  memset(value,'\0',BUFFLEN);
  bool literal = false; // 
  char* ptr = name; //start with name filed
  int c = 0; // counter
  int oc= 0; // output counter
  bool cont = true; //continue flag
  int max_in_len = min(len, BUFFLEN - 1);

  while (cont && (c < max_in_len) && (oc < (BUFFLEN-1))){ // limit to buffer size
    if(in[c] < 127){
      switch (in[c]) {
        case '\r': if(c != 0) {cont = false;} break;
        case '\n': cont = false; break;
        //case '-': cont = false; break; // catches negative temps
        case '>': cont = false; break;
        case '!':  cont = false; break;
        case '#':  cont = false; break;
        case '?':  literal = true; break; // enable literal mode
        case '=':  if(oc && (ptr == name) ){ *(ptr+oc) = '\0'; ptr = value; oc = 0;} break; // switch to value
        case ' ':  if(!literal) {break;} // avoid removing whitespaces from name
        default:   *(ptr+oc) = in[c]; oc++; break; // copy char
      }
    }
    c++;
  }
  if(oc && (ptr == value)){ // value is set
    return cb(name, value);
  }
  
  return false;
}

// Settings ----------------------------------------------------------------------------------------------------------------------
static void cmd_print_uuid() {
  String uuid = config_update_get_full_uuid_hex();
  char line[48];
  snprintf(line, sizeof(line), "UUID=%s\r\n", uuid.c_str());
  log_i(line);
  log_flush();
}

// Settings are dispatched through a table instead of a 55-deep strcmp chain.
// Order no longer matters, adding a key is a one-line entry, and the key list
// is data other code can walk.
//
// The handlers are named functions rather than lambdas on purpose: with
// -std=gnu++11 the lambda-to-function-pointer conversion is not constexpr, so a
// table of lambdas cannot be statically initialised and ends up in RAM, filled
// in by startup code.
typedef bool (*SettingApplyFn)(const char *value);

typedef struct {
  const char    *key;
  SettingApplyFn apply;
} SettingEntry;

// Seconds in the config file, milliseconds internally.
static uint32_t seconds_to_ms(const char *v){ return (uint32_t)atoi(v) * 1000u; }

// Most handlers are a single assignment; SETTING_FN keeps them to one line each
// while still producing an ordinary named function.
#define SETTING_FN(fn_name, statement) \
  static bool fn_name(const char *v){ (void)v; statement; return true; }

// --- station identity and position ---
static bool set_name(const char *v){
  if(strlen(v) >= sizeof(settings.station_name)){
    log_e("NAME too long, truncated\r\n");
  }
  snprintf(settings.station_name, sizeof(settings.station_name), "%s", v);
  return true;
}
SETTING_FN(set_lon,       settings.pos_lon  = parse_decimal_fast(v))
SETTING_FN(set_lat,       settings.pos_lat  = parse_decimal_fast(v))
SETTING_FN(set_alt,       settings.altitude = parse_decimal_fast(v))

// --- radio ---
SETTING_FN(set_lora_freq, settings.lora_freq = parse_decimal_fast(v))
SETTING_FN(set_lora_bw,   settings.lora_bw   = atoi(v))
SETTING_FN(set_lbt,       settings.lora_lbt  = atoi(v))
SETTING_FN(set_lbt_rssi,  settings.lora_rssi_threshold = atoi(v))
SETTING_FN(set_smart_fwd, settings.lora_smart_rcv = atoi(v))
SETTING_FN(set_fwd_data,  settings.forward_data   = atoi(v))
SETTING_FN(set_repeater,  settings.repeater       = atoi(v))

// --- measurement / averaging ---
SETTING_FN(set_heading_offset, settings.heading_offset = atoi(v))
SETTING_FN(set_gust_age,       settings.gust_age = seconds_to_ms(v))
SETTING_FN(set_wind_age,       settings.wind_age = seconds_to_ms(v))
SETTING_FN(set_redu_volt,      settings.reduce_interval_voltage = parse_decimal_fast(v))

// --- broadcast intervals, in seconds; 0 disables ---
SETTING_FN(set_bc_weather, settings.broadcast_interval_weather = seconds_to_ms(v))
SETTING_FN(set_bc_name,    settings.broadcast_interval_name    = seconds_to_ms(v))

// --- sensor selection ---
SETTING_FN(set_baro,        settings.use_baro = atoi(v))
SETTING_FN(set_davis6410,   if(atoi(v)){ settings.sensor_type = s_DAVIS6410; })
SETTING_FN(set_ws80,        if(atoi(v)){ settings.sensor_type = s_WS80; })       // WSXX auto detection
SETTING_FN(set_ws85,        if(atoi(v)){ settings.sensor_type = s_WS85; })
SETTING_FN(set_ws85_uart,   if(atoi(v)){ settings.sensor_type = s_WS85_UART; })
SETTING_FN(set_windnerd,    if(atoi(v)){ settings.sensor_type = s_WINDNERD; })
SETTING_FN(set_gps,         settings.use_gps  = atoi(v))
SETTING_FN(set_gps_baud,    settings.gps_baud = atoi(v))
SETTING_FN(set_imu,         settings.use_imu  = atoi(v))

// --- Davis 6410 specific ---
SETTING_FN(set_integration_time, settings.sensor_integration_time = seconds_to_ms(v))
SETTING_FN(set_protection_board, settings.use_protection_board    = atoi(v))

// --- logging / misc ---
SETTING_FN(set_debug,       log_set_debug(atoi(v)))
SETTING_FN(set_errors,      log_set_error(atoi(v)))
SETTING_FN(set_usb_verbose, settings.verbose_usb   = atoi(v))
SETTING_FN(set_testmode,    settings.testmode      = atoi(v))
SETTING_FN(set_wdt,         settings.use_wdt       = atoi(v))
SETTING_FN(set_div_cpu_slow,settings.div_cpu_slow  = atoi(v))

// --- OTA ---
SETTING_FN(set_ota_enable,  settings.ota_enable = atoi(v) != 0)
SETTING_FN(set_ota_fast,    settings.ota_fast   = atoi(v))

// --- console commands ---
SETTING_FN(cmd_get_uuid,    cmd_print_uuid())
SETTING_FN(cmd_sleep,       usb_connected = false)
SETTING_FN(cmd_skip_lora,   state.skip_lora = true)
SETTING_FN(cmd_delay,       delay(atoi(v)))                    // for WDT testing
SETTING_FN(cmd_analog_test, state.analog_test_mode = atoi(v))
// RESET is a full reset: re-entering setup() appended the altitude to the
// station name a second time and re-initialised the radio underneath us.
SETTING_FN(cmd_reboot,      NVIC_SystemReset())

#undef SETTING_FN

static bool cmd_ota_confirm(const char *v){
  (void)v;
  return ota_ab_confirm_current();
}

static bool cmd_ota_boot_slot(const char *v){
  // Manual debug trigger, not a real OTA transfer: no verified image size is
  // known here, so pass 0 (unknown) and let the bootloader fall back to its
  // scan-based size detection.
  if(ota_ab_request_slot((uint8_t)atoi(v), 0u)) {
    NVIC_SystemReset();
  }
  return true;
}

static bool cmd_format(const char *v){
  (void)v;
  if(format_flash()){
    NVIC_SystemReset();
  } else {
    log_i("Error resetting bootloader settings\r\n");
  }
  return true;
}

static const SettingEntry setting_table[] = {
  {"NAME",                       set_name},
  {"LON",                        set_lon},
  {"LAT",                        set_lat},
  {"ALT",                        set_alt},

  {"LORA_FREQ",                  set_lora_freq},
  {"LORA_BW",                    set_lora_bw},
  {"LBT",                        set_lbt},
  {"LBT_RSSI_THRESHOLD",         set_lbt_rssi},
  {"FANET_SMART_FORWARD",        set_smart_fwd},
  {"FORWARD_DATA",               set_fwd_data},
  {"REPEATER",                   set_repeater},

  {"HEADING_OFFSET",             set_heading_offset},
  {"GUST_AGE",                   set_gust_age},
  {"WIND_AGE",                   set_wind_age},
  {"REDU_INTERV_VOLT",           set_redu_volt},

  {"BROADCAST_INTERVAL_WEATHER", set_bc_weather},
  {"BROADCAST_INTERVAL_NAME",    set_bc_name},

  {"SENSOR_BARO",                set_baro},
  {"SENSOR_DAVIS6410",           set_davis6410},
  {"SENSOR_WSXX",                set_ws80},   // auto detection
  {"SENSOR_WS80",                set_ws80},   // kept for old settings files
  {"SENSOR_WS85",                set_ws85},
  {"SENSOR_WS85_UART",           set_ws85_uart},
  {"SENSOR_WINDNERD",            set_windnerd},
  {"SENSOR_GPS",                 set_gps},
  {"GPS_BAUD",                   set_gps_baud},
  {"SENSOR_IMU",                 set_imu},

  {"SENSOR_INTEGRATION_TIME",    set_integration_time},
  {"SENSOR_PROTECTION_BOARD",    set_protection_board},

  {"DEBUG",                      set_debug},
  {"ERRORS",                     set_errors},
  {"USB_VERBOSE",                set_usb_verbose},
  {"TESTMODE",                   set_testmode},
  {"WDT",                        set_wdt},
  {"DIV_CPU_SLOW",               set_div_cpu_slow},

  {"OTA_ENABLE",                 set_ota_enable},
  {"OTA_FAST",                   set_ota_fast},
  {"OTA_CONFIRM",                cmd_ota_confirm},
  {"OTA_BOOT_SLOT",              cmd_ota_boot_slot},

  {"GET_UUID",                   cmd_get_uuid},
  {"SLEEP",                      cmd_sleep},
  {"FORMAT",                     cmd_format},
  {"RESET",                      cmd_reboot},
  {"REBOOT",                     cmd_reboot},
  {"SKIP_LORA",                  cmd_skip_lora},
  {"DELAY",                      cmd_delay},
  {"ANALOG_TEST",                cmd_analog_test},
};

bool apply_setting(char* settingName,  char* settingValue){
  //if(debug_enabled()){printf("Setting: %s = %s\r\n",settingName, settingValue); log_flush();}
  for(const SettingEntry &e : setting_table){
    if(strcmp(settingName, e.key) == 0){
      return e.apply(settingValue);
    }
  }
  return false;
}

void print_settings(){
  if(debug_enabled()){
    if(!settings.use_baro){log_i("Baro: OFF\r\n");}
    log_i("Name: "); log_i(settings.station_name); log_i("\r\n");
    //log_i("Lon: ", pos_lon);
    //log_i("Lat: ", pos_lat);
    //log_i("Alt: ", altitude);
    log_flush();
    log_i("Heading offset: ", settings.heading_offset); 
    log_i("Broadcast interval weather [s]: ", settings.broadcast_interval_weather/1000);
    log_i("Broadcast interval name [s]: ", settings.broadcast_interval_name/1000);
    log_i("LBT: "); log_i(settings.lora_lbt ? "ON\r\n" : "OFF\r\n");
    if(settings.repeater) { log_i("Repeater mode: ON\r\n"); }
    if(settings.forward_data) { log_i("Forward data: ON\r\n"); }
    if(is_wsxx()){log_i("Sensor: WSXX Auto detect\r\n");}
    if(settings.sensor_type == s_DAVIS6410){log_i("Sensor: DAVIS 6410\r\n");}
    if(settings.sensor_type == s_WS85_UART){log_i("Sensor: WS85 UART\r\n");}
    if(settings.sensor_type == s_WINDNERD){log_i("Sensor: Windnerd\r\n");}
    log_flush();
  }
}

// Update specific settings in the existing settings.txt from flash
// Preserves all other content (comments, formatting, other parameters)
bool update_settings_in_flash(const char* key, const char* value) {
  const char* keys[1]   = { key };
  const char* values[1] = { value };
  return update_multiple_settings_in_flash(keys, values, 1);
}

// Update multiple settings in flash atomically
// Uses static buffers to avoid heap fragmentation / large stack allocations.
bool update_multiple_settings_in_flash(const char** keys, const char** values, size_t count) {
  if (count == 0) {
    return true;
  }
  if (count > 16) {
    log_e("Too many settings to update at once\r\n");
    return false;
  }

  // Static buffers: stored in BSS, never on stack or heap.
  static char existing[OTA_SETTINGS_SIZE];
  static char output[OTA_SETTINGS_SIZE];

  memset(existing, 0, sizeof(existing));
  memset(output,   0, sizeof(output));

  if (!ota_storage_read_text(OTA_SETTINGS_ADDRESS, existing, sizeof(existing))) {
    log_e("Settings flash empty, refusing update\r\n");
    return false;
  } else {
    size_t existing_len = strlen(existing);
    if (existing_len < 100u) {
      // Settings appear truncated. Refuse partial rewrite until user uploads a valid file.
      log_e("Settings truncated (");
      char lbuf[8]; snprintf(lbuf, sizeof(lbuf), "%u", (unsigned)existing_len);
      log_e(lbuf);
      log_e(" bytes), refusing update\r\n");
      return false;
    } else {
      if (debug_enabled()) {
        char lbuf[8]; snprintf(lbuf, sizeof(lbuf), "%u", (unsigned)existing_len);
        log_i("Settings read: ");
        log_i(lbuf);
        log_i(" bytes\r\n");
      }
    }
  }

  bool keys_found[16] = {false};
  size_t out_pos = 0;

  log_i("SF:loop\r\n"); log_flush();

  const char* src = existing;
  while (*src != '\0') {
    const char* line_start = src;

    // Find end of printable line content (stop before \r / \n / \0)
    const char* line_end = src;
    while (*line_end != '\0' && *line_end != '\r' && *line_end != '\n') {
      line_end++;
    }
    size_t line_len = (size_t)(line_end - line_start);

    // Check if this line matches one of the keys to replace
    bool replaced = false;
    for (size_t i = 0; i < count; ++i) {
      size_t key_len = strlen(keys[i]);
      if (line_len > key_len &&
          line_start[key_len] == '=' &&
          strncmp(line_start, keys[i], key_len) == 0) {
        // Write replacement: key=value\r\n
        size_t val_len = strlen(values[i]);
        if (out_pos + key_len + 1u + val_len + 2u + 1u > sizeof(output)) {
          log_e("SF:overflow replace\r\n");
          return false;
        }
        memcpy(output + out_pos, keys[i], key_len); out_pos += key_len;
        output[out_pos++] = '=';
        memcpy(output + out_pos, values[i], val_len); out_pos += val_len;
        output[out_pos++] = '\r';
        output[out_pos++] = '\n';
        keys_found[i] = true;
        replaced = true;
        break;
      }
    }

    if (!replaced) {
      if (out_pos + line_len + 1u > sizeof(output)) {
        log_e("SF:overflow copy\r\n");
        return false;
      }
      memcpy(output + out_pos, line_start, line_len);
      out_pos += line_len;
    }

    // Advance past line ending — always consume the terminator byte(s),
    // regardless of whether the line was replaced.
    if (*line_end == '\0') {
      break;
    } else if (*line_end == '\r') {
      if (!replaced) {
        if (out_pos + 1u >= sizeof(output)) { log_e("SF:overflow eol\r\n"); return false; }
        output[out_pos++] = '\r';
      }
      line_end++;
      if (*line_end == '\n') {
        if (!replaced) {
          if (out_pos + 1u >= sizeof(output)) { log_e("SF:overflow eol\r\n"); return false; }
          output[out_pos++] = '\n';
        }
        line_end++; // always advance past \n
      }
    } else { // '\n'
      if (!replaced) {
        if (out_pos + 1u >= sizeof(output)) { log_e("SF:overflow eol\r\n"); return false; }
        output[out_pos++] = '\n';
      }
      line_end++;
    }
    src = line_end;
  }

  log_i("SF:append\r\n"); log_flush();

  // Append keys that were not found in the existing file
  for (size_t i = 0; i < count; ++i) {
    if (!keys_found[i]) {
      // Ensure the file ends with a newline before appending
      if (out_pos > 0 && output[out_pos - 1] != '\n') {
        if (out_pos + 2u >= sizeof(output)) { log_e("SF:overflow nl\r\n"); return false; }
        output[out_pos++] = '\r';
        output[out_pos++] = '\n';
      }
      size_t key_len = strlen(keys[i]);
      size_t val_len = strlen(values[i]);
      if (out_pos + key_len + 1u + val_len + 2u + 1u > sizeof(output)) {
        log_e("SF:overflow append\r\n");
        return false;
      }
      memcpy(output + out_pos, keys[i], key_len); out_pos += key_len;
      output[out_pos++] = '=';
      memcpy(output + out_pos, values[i], val_len); out_pos += val_len;
      output[out_pos++] = '\r';
      output[out_pos++] = '\n';
    }
  }

  output[out_pos] = '\0';

  {
    char lbuf[24];
    snprintf(lbuf, sizeof(lbuf), "SF:write %u bytes\r\n", (unsigned)out_pos);
    log_i(lbuf); log_flush();
  }

  // Disable WDT for the duration of flash erase+write (can take >500ms for 2 rows).
  // Re-enable immediately after so normal watchdog protection resumes.
  if (settings.use_wdt) {
    wdt_disable();
  }

  bool write_ok = ota_storage_write_text(OTA_SETTINGS_ADDRESS, OTA_SETTINGS_SIZE, output);

  if (settings.use_wdt) {
    wdt_enable(2500, false);
  }

  if (!write_ok) {
    log_e("SF:write failed\r\n");
    return false;
  }

  log_i("SF:done\r\n"); log_flush();
  return true;
}



// get current millis since reset, including time spend in deepsleep. Missing time waiting for UART ws80 sensor
uint32_t time(){
  return millis() + sleeptime_cum;
}

uint16_t get_fanet_id(){
  return UniqueID[0] + ((UniqueID[1])<<8);
}

// sets gpio in low power unconnected floating state
void pinDisable( uint32_t ulPin){
  EPortType port = g_APinDescription[ulPin].ulPort;
  uint32_t pin = g_APinDescription[ulPin].ulPin;
  uint32_t pinMask = (1ul << pin);
  // Set pin to reset value
  PORT->Group[port].PINCFG[pin].reg = (uint8_t) (PORT_PINCFG_RESETVALUE);
  PORT->Group[port].DIRCLR.reg = pinMask;
}



bool led_status(bool s){
static bool state = false;
bool ret = false;
    if(s){
      pinMode(PIN_STATUSLED,OUTPUT);
      digitalWrite(PIN_STATUSLED, 1);
      if(state){ret = true;} // was on before
      state = true;
    } else {
      state = false;
      digitalWrite(PIN_STATUSLED, 0);
      pinDisable(PIN_STATUSLED);
    }
  return ret;
}


// sets red LED pin to output and turns it on
bool led_error(bool s){
static bool state = false;
bool ret = false;
  if(s){
    pinMode(PIN_ERRORLED,OUTPUT);
    digitalWrite(PIN_ERRORLED, 1);
    if(state){ret = true;} // was on before
    state = true;
  } else {
    state = false;;
    digitalWrite(PIN_ERRORLED, 0);
    pinDisable(PIN_ERRORLED);
  }
  return ret;
}



void i2c_scan(){

  #if 0
  if(!debug_enabled()){
    return;
  }

  byte error, address;
  int nDevices;
  nDevices = 0;
  for(address = 1; address < 127; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0){
      DEBUGSER.print("I2C device found at address 0x");
      if (address<16)
        DEBUGSER.print("0");
      DEBUGSER.println(address,HEX);
      nDevices++;
    }
    else if (error==4){
      DEBUGSER.print("Unknown error at address 0x");
      if (address<16)
        DEBUGSER.print("0");
      DEBUGSER.println(address,HEX);
    }    
  }
  if (nDevices == 0){DEBUGSER.println("No I2C devices found\n");}

#endif
}



bool zone_not_eu(){
  return ((settings.pos_lon < -30.0f && settings.pos_lon > -180.0f) ||          // North/South America
    (settings.pos_lon > 110.0f && settings.pos_lon <= 180.0f) ||                // Japan, Australia, NZ
    (settings.pos_lon >= 75.0f && settings.pos_lon <= 110.0f && settings.pos_lat > 15.0f)  // China
    );
}

// Renders version.txt into out. Returns the length written (excluding the
// terminator), or 0 if it did not fit.
static size_t build_version_text(char *out, size_t out_len){
  char bootloader_version[40];
  get_bootloader_version_for_version_text(bootloader_version, sizeof(bootloader_version));

  const char *lora_name = "UNKNOWN";
  if(lora_module == LORA_SX1276) { lora_name = "SX1276"; }
  if(lora_module == LORA_SX1262) { lora_name = "SX1262"; }
  if(lora_module == LORA_LLCC68) { lora_name = "LLCC68"; }

  const char *baro_name = "UNKNOWN";
  if(chip_baro == BARO_BMP280) { baro_name = "BMP280"; }
  if(chip_baro == BARO_BMP3xx) { baro_name = "BMP3xx"; }
  if(chip_baro == BARO_SPL06) { baro_name = "SPL06"; }
  if(chip_baro == BARO_HP203B) { baro_name = "HP203B"; }

  int n = snprintf(out, out_len,
    "Firmware: %s\r\n"
    "FW Build: %s %s\r\n"
    "Bootloader: %s\r\n"
    "FANET ID: %02X%04X\r\n"
    "LoRa Module: %s\r\n"
    "Barometer: %s\r\n"
    "Chip-ID: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
    VERSION,
    __DATE__, __TIME__,
    bootloader_version,
    FANET_VENDOR_ID, get_fanet_id(),
    lora_name,
    baro_name,
    UniqueID[0],  UniqueID[1],  UniqueID[2],  UniqueID[3],
    UniqueID[4],  UniqueID[5],  UniqueID[6],  UniqueID[7],
    UniqueID[8],  UniqueID[9],  UniqueID[10], UniqueID[11],
    UniqueID[12], UniqueID[13], UniqueID[14], UniqueID[15]);

  if(n <= 0 || (size_t)n >= out_len){
    log_e("Version text does not fit\r\n");
    return 0;
  }
  return (size_t)n;
}

bool create_versionfile(const char *filename){
  (void)filename;
  // One buffer, built once: the old code rendered the text twice, once to
  // compare and once to write, and did it through Arduino String.
  char text[OTA_VERSIONS_SIZE];
  if(build_version_text(text, sizeof(text)) == 0u){
    return false;
  }

  char existing[OTA_VERSIONS_SIZE] = {0};
  if(ota_storage_read_text(OTA_VERSIONS_ADDRESS, existing, sizeof(existing)) &&
     strcmp(existing, text) == 0){
    return true; // unchanged
  }

  bool ok = ota_storage_write_text(OTA_VERSIONS_ADDRESS, OTA_VERSIONS_SIZE, text);
  if(ok){
    log_i("Version file updated\r\n");
  } else {
    log_e("Version file write failed\r\n");
  }
  return ok;
}