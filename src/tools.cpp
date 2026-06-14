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

    if(settings.station_name.length() == 0){
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

String get_bootloader_version(){
  static bool cached = false;
  static String version = "unknown";
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

    char buf[40];
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
      version = String(buf);
    }
    break;
  }

  return version;
}

static String get_bootloader_version_for_version_text(){
#ifdef BOOTLOADER_VERSION_TXT_OVERRIDE
  return String(BOOTLOADER_VERSION_TXT_OVERRIDE);
#else
  String version = get_bootloader_version();
  int git_suffix = version.indexOf("-0-g");
  if(git_suffix > 0){
    version.remove(git_suffix);
  }
  int feature_suffix = version.indexOf(' ');
  if(feature_suffix > 0){
    version.remove(feature_suffix);
  }
  return version;
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

bool apply_setting(char* settingName,  char* settingValue){
  //if(debug_enabled()){printf("Setting: %s = %s\r\n",settingName, settingValue); log_flush();}
  
  if(strcmp(settingName,"NAME")==0){settings.station_name = settingValue; return 1;}
  if(strcmp(settingName,"LON")==0) {settings.pos_lon = parse_decimal_fast(settingValue); return 1;}
  if(strcmp(settingName,"LAT")==0) {settings.pos_lat = parse_decimal_fast(settingValue); return 1;}
  if(strcmp(settingName,"ALT")==0) {settings.altitude = parse_decimal_fast(settingValue); return 1;}
  if(strcmp(settingName,"LORA_FREQ")==0){settings.lora_freq = parse_decimal_fast(settingValue); return 1;} 
  if(strcmp(settingName,"LORA_BW")==0)  {settings.lora_bw = atoi(settingValue); return 1;}

  if(strcmp(settingName,"REDU_INTERV_VOLT")==0) { settings.reduce_interval_voltage = parse_decimal_fast(settingValue); return 1;}

  if(strcmp(settingName,"HEADING_OFFSET")==0) {settings.heading_offset = atoi(settingValue); return 1;}
  if(strcmp(settingName,"GUST_AGE")==0) {settings.gust_age = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"WIND_AGE")==0) {settings.wind_age = (uint32_t)atoi(settingValue)*1000; return 1;}
  
// Broadcast intervals in seconds, 0 = disable
  if(strcmp(settingName,"BROADCAST_INTERVAL_WEATHER")==0) {settings.broadcast_interval_weather = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_NAME")==0)    {settings.broadcast_interval_name = (uint32_t)atoi(settingValue)*1000; return 1;}

  if(strcmp(settingName,"SENSOR_BARO")==0) {settings.use_baro = atoi(settingValue); return 1;}
  if(strcmp(settingName,"SENSOR_DAVIS6410")==0) { if(atoi(settingValue)){settings.sensor_type = s_DAVIS6410;} return 1;}
  if(strcmp(settingName,"SENSOR_WSXX")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS80;} return 1;} // auto detection
  if(strcmp(settingName,"SENSOR_WS80")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS80;} return 1;} // keep for comatibility with old settings file
  if(strcmp(settingName,"SENSOR_WS85")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS85;} return 1;}
  if(strcmp(settingName,"SENSOR_WS85_UART")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS85_UART;} return 1;}
  if(strcmp(settingName,"SENSOR_WINDNERD")==0) { if(atoi(settingValue)){settings.sensor_type = s_WINDNERD;} return 1;}
  

// Davis 6410 specific
  if(strcmp(settingName,"SENSOR_INTEGRATION_TIME")==0) {settings.sensor_integration_time = atoi(settingValue)*1000; return 1;}

  if(strcmp(settingName,"SENSOR_GPS")==0) {settings.use_gps = atoi(settingValue); return 1;}
  if(strcmp(settingName,"GPS_BAUD")==0) {settings.gps_baud = atoi(settingValue); return 1;}
  if(strcmp(settingName,"SENSOR_IMU")==0) {settings.use_imu = atoi(settingValue); return 1;}

  if(strcmp(settingName,"DEBUG")==0) {log_set_debug(atoi(settingValue)); return 1;}
  if(strcmp(settingName,"ERRORS")==0) {log_set_error(atoi(settingValue)); return 1;}
  if(strcmp(settingName,"TESTMODE")==0) {settings.testmode = atoi(settingValue); return 1;}
  if(strcmp(settingName,"OTA_FAST")==0) {settings.ota_fast = atoi(settingValue); return 1;}
  if(strcmp(settingName,"WDT")==0) {settings.use_wdt = atoi(settingValue); return 1;}
  if(strcmp(settingName,"DIV_CPU_SLOW")==0) {settings.div_cpu_slow = atoi(settingValue); return 1;}
  if(strcmp(settingName,"USB_VERBOSE")==0) {settings.verbose_usb = atoi(settingValue); return 1;}


// Test commands
  if(strcmp(settingName,"LBT")==0) {settings.lora_lbt = atoi(settingValue); return 1;}
  if(strcmp(settingName,"LBT_RSSI_THRESHOLD")==0) {settings.lora_rssi_threshold = atoi(settingValue); return 1;}
  if(strcmp(settingName,"FANET_SMART_FORWARD")==0) {settings.lora_smart_rcv = atoi(settingValue); return 1;}
  if(strcmp(settingName,"FORWARD_DATA")==0) {settings.forward_data = atoi(settingValue); return 1;}
  if(strcmp(settingName,"REPEATER")==0) {settings.repeater = atoi(settingValue); return 1;}

  if(strcmp(settingName,"OTA_ENABLE")==0) {settings.ota_enable = atoi(settingValue) != 0; return 1;}
  if(strcmp(settingName,"OTA_CONFIRM")==0) {return ota_ab_confirm_current() ? 1 : 0;}
  if(strcmp(settingName,"GET_UUID")==0) {cmd_print_uuid(); return 1;}
  if(strcmp(settingName,"OTA_BOOT_SLOT")==0) {
    uint8_t slot = (uint8_t)atoi(settingValue);
    // Manual debug trigger, not a real OTA transfer: no verified image size
    // is known here, so pass 0 (unknown) and let the bootloader fall back
    // to its scan-based size detection.
    if(ota_ab_request_slot(slot, 0u)) {
      NVIC_SystemReset();
    }
    return 1;
  }

  if(strcmp(settingName,"SLEEP")==0) {usb_connected =false; return 1;}
  if(strcmp(settingName,"FORMAT")==0) {if(format_flash()){NVIC_SystemReset();} else {log_i("Error resetting bootloader settings\r\n");} return 1;}
  if(strcmp(settingName,"RESET")==0) {setup(); return 1;}
  if(strcmp(settingName,"SKIP_LORA")==0) {settings.skip_lora = true; return 1;}
  if(strcmp(settingName,"DELAY")==0) {delay(atoi(settingValue)); return 1;} // delay for WDT testing
  if(strcmp(settingName,"REBOOT")==0) {NVIC_SystemReset(); return 1;}

  if(strcmp(settingName,"ANALOG_TEST")==0) {settings.analog_test_mode = atoi(settingValue); return 1;} // power on/off davis sensor for testing

  return 0;
}

void print_settings(){
  if(debug_enabled()){
    if(!settings.use_baro){log_i("Baro: OFF\r\n");}
    log_i("Name: "); log_i(settings.station_name.c_str()); log_i("\r\n");
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
  return ((settings.pos_lon < -30.0 && settings.pos_lon > -180.0) ||            // North/South America
    (settings.pos_lon > 110.0 && settings.pos_lon <= 180.0) ||                  // Japan, Australia, NZ
    (settings.pos_lon >= 75.0 && settings.pos_lon <= 110.0 && settings.pos_lat > 15.0)  // China
    );
}

static String build_version_text(){
  String bootloader_version = get_bootloader_version_for_version_text();

  const char *lora_name = "UNKNOWN";
  if(lora_module == LORA_SX1276) { lora_name = "SX1276"; }
  if(lora_module == LORA_SX1262) { lora_name = "SX1262"; }
  if(lora_module == LORA_LLCC68) { lora_name = "LLCC68"; }

  const char *baro_name = "UNKNOWN";
  if(chip_baro == BARO_BMP280) { baro_name = "BMP280"; }
  if(chip_baro == BARO_BMP3xx) { baro_name = "BMP3xx"; }
  if(chip_baro == BARO_SPL06) { baro_name = "SPL06"; }
  if(chip_baro == BARO_HP203B) { baro_name = "HP203B"; }

  char line[96];
  String rawText;
  rawText.reserve(256);
  rawText += "Firmware: "; rawText += VERSION; rawText += "\r\n";
  int n = snprintf(line, sizeof(line), "FW Build: %s %s\r\n", __DATE__, __TIME__);
  if(n > 0) { rawText += line; }
  rawText += "Bootloader: "; rawText += bootloader_version; rawText += "\r\n";
  n = snprintf(line, sizeof(line), "FANET ID: %02X%04X\r\n", FANET_VENDOR_ID, get_fanet_id());
  if(n > 0) { rawText += line; }
  n = snprintf(line, sizeof(line), "LoRa Module: %s\r\n", lora_name);
  if(n > 0) { rawText += line; }
  n = snprintf(line, sizeof(line), "Barometer: %s\r\n", baro_name);
  if(n > 0) { rawText += line; }
  n = snprintf(line, sizeof(line),
    "Chip-ID: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
    UniqueID[0],  UniqueID[1],  UniqueID[2],  UniqueID[3],
    UniqueID[4],  UniqueID[5],  UniqueID[6],  UniqueID[7],
    UniqueID[8],  UniqueID[9],  UniqueID[10], UniqueID[11],
    UniqueID[12], UniqueID[13], UniqueID[14], UniqueID[15]);
  if(n > 0) { rawText += line; }

  return rawText;
}

static bool versionfile_needs_rewrite(const char *path){
  (void)path;
  char existing[OTA_VERSIONS_SIZE] = {0};
  if(!ota_storage_read_text(OTA_VERSIONS_ADDRESS, existing, sizeof(existing))){
    return true; // flash empty → write needed
  }
  return build_version_text() != String(existing);
}

bool create_versionfile(const char *filename){
  (void)filename;
  if(!versionfile_needs_rewrite(filename)){
    return true;
  }
  String text = build_version_text();
  bool ok = ota_storage_write_text(OTA_VERSIONS_ADDRESS, OTA_VERSIONS_SIZE, text.c_str());
  if(ok){
    log_i("Version file updated\r\n");
  } else {
    log_e("Version file write failed\r\n");
  }
  return ok;
}