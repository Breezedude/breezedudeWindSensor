#include "tools.h"
#include "defines.h"
#include "logging.h"
#include "sleep.h"
#include "msc.h"

extern LORA_MODULE lora_module;
extern CHIP_BARO chip_baro;

uint32_t sleeptime_cum = 0; // cumulative time spend in sleepmode, used for time() calculation
Settings settings;


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

const char* wakeup_source_string[] = {"NONE", "RTC", "EIC", "WDT", "UART", "LORA"};

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
bool apply_setting(char* settingName,  char* settingValue){
  //if(debug_enabled()){printf("Setting: %s = %s\r\n",settingName, settingValue); log_flush();}
  
  if(strcmp(settingName,"NAME")==0){settings.station_name = settingValue; return 1;}
  if(strcmp(settingName,"LON")==0) {settings.pos_lon = atof(settingValue); return 1;}
  if(strcmp(settingName,"LAT")==0) {settings.pos_lat = atof(settingValue); return 1;}
  if(strcmp(settingName,"ALT")==0) {settings.altitude = atof(settingValue); return 1;}
  if(strcmp(settingName,"LORA_FREQ")==0){settings.lora_freq = atof(settingValue); return 1;} 
  if(strcmp(settingName,"LORA_BW")==0)  {settings.lora_bw = atoi(settingValue); return 1;}

  if(strcmp(settingName,"REDU_INTERV_VOLT")==0) { settings.reduce_interval_voltage = atof(settingValue); return 1;}

  if(strcmp(settingName,"HEADING_OFFSET")==0) {settings.heading_offset = atoi(settingValue); return 1;}
  if(strcmp(settingName,"GUST_AGE")==0) {settings.gust_age = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"WIND_AGE")==0) {settings.wind_age = (uint32_t)atoi(settingValue)*1000; return 1;}
  
// Broadcast intervals in seconds, 0 = disable
  if(strcmp(settingName,"BROADCAST_INTERVAL_WEATHER")==0) {settings.broadcast_interval_weather = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_NAME")==0)    {settings.broadcast_interval_name = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_INFO")==0)    {settings.broadcast_interval_info = (uint32_t)atoi(settingValue)*1000; return 1;}

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
  if(strcmp(settingName,"TEST_USB")==0) {
    settings.test_with_usb = atoi(settingValue);
    // Disable MSC while in USB test mode — host-side polling makes the system
    // very laggy when loop() is not running. Re-enable if mode is turned off.
    usb_msc.setUnitReady(!settings.test_with_usb);
    return 1;
  }
  if(strcmp(settingName,"TESTMODE")==0) {settings.testmode = atoi(settingValue); return 1;}
  if(strcmp(settingName,"WDT")==0) {settings.use_wdt = atoi(settingValue); return 1;}
  if(strcmp(settingName,"DIV_CPU_SLOW")==0) {settings.div_cpu_slow = atoi(settingValue); return 1;}
  if(strcmp(settingName,"FORWARD_UART")==0) {settings.forward_serial_while_usb = atoi(settingValue); return 1;}
  if(strcmp(settingName,"USB_VERBOSE")==0) {settings.verbose_usb = atoi(settingValue); return 1;}


// Test commands
  if(strcmp(settingName,"FANET_RECEIVE")==0) {settings.lora_smart_rcv = atoi(settingValue); return 1;}
  if(strcmp(settingName,"SLEEP")==0) {usb_connected =false; return 1;}
  if(strcmp(settingName,"FORMAT")==0) {if(format_flash()){NVIC_SystemReset();} else {log_i("Error Formating Flash\r\n");} return 1;}
  if(strcmp(settingName,"RESET")==0) {setup(); return 1;}
  if(strcmp(settingName,"SKIP_LORA")==0) {settings.skip_lora = true; return 1;}
  if(strcmp(settingName,"DELAY")==0) {delay(atoi(settingValue)); return 1;} // delay for WDT testing
  if(strcmp(settingName,"REBOOT")==0) {NVIC_SystemReset(); return 1;}
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
    if(is_wsxx()){log_i("Sensor: WSXX Auto detect\r\n");}
    if(settings.sensor_type == s_DAVIS6410){log_i("Sensor: DAVIS 6410\r\n");}
    if(settings.sensor_type == s_WS85_UART){log_i("Sensor: WS85 UART\r\n");}
    if(settings.sensor_type == s_WINDNERD){log_i("Sensor: Windnerd\r\n");}
    log_flush();
  }
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
}

bool zone_not_eu(){
  return ((settings.pos_lon < -30.0 && settings.pos_lon > -180.0) ||            // North/South America
    (settings.pos_lon > 110.0 && settings.pos_lon <= 180.0) ||                  // Japan, Australia, NZ
    (settings.pos_lon >= 75.0 && settings.pos_lon <= 110.0 && settings.pos_lat > 15.0)  // China
    );
}

static bool sync_flash_fs(){
  bool ok = flash.syncBlocks();
  if(!ok){
    log_i("flash.syncBlocks failed\n");
    return false;
  }
  my_internal_storage.flush_buffer();
  return true;
}

static void parse_versionfile_line(const char *line, String &file_version){
  if(strncmp(line, "Version: ", 9) == 0){
    file_version = String(line + 9);
  }
}

static bool versionfile_needs_rewrite(const char *path){
  if(!fatfs.exists(path)){
    log_i("Version file missing\n");
    return true;
  }

  File f = fatfs.open(path, FILE_READ);
  if(!f){
    log_i("Version file open failed, rewrite\n");
    return true;
  }

  String file_version = "";
  char line[128];
  int idx = 0;

  while(f.available()){
    int c = f.read();
    if(c < 0){
      break;
    }
    if(c == '\r'){
      continue;
    }
    if(c == '\n'){
      line[idx] = '\0';
      parse_versionfile_line(line, file_version);
      idx = 0;
      continue;
    }
    if(idx < (int)sizeof(line) - 1){
      line[idx++] = (char)c;
    }
  }
  if(idx > 0){
    line[idx] = '\0';
    parse_versionfile_line(line, file_version);
  }
  f.close();

  if(file_version.length() == 0){
    log_i("Version file has no version line, rewrite\n");
    return true;
  }
  if(file_version != String(VERSION)){
    log_i("Version changed, rewrite version file\n");
    return true;
  }
  return false;
}

bool create_versionfile(const char *filename){
  const char *path = (filename && filename[0]) ? filename : "version.txt";
  const char *name = path[0] == '/' ? path + 1 : path;

  FatFile f;
  FatFile root;
  if (!fatfs.begin(&flash)) {
    log_i("fs start fail\r\n");
    return false;
  }

  if(!versionfile_needs_rewrite(path)){
    //log_i("Version file up-to-date\n");
    return true;
  }

  if(!root.open("/")){
    log_v("open root failed\n");
    return false;
  }

  bool opened = f.open(&root, name, O_WRONLY | O_CREAT | O_TRUNC);
  root.close();
  if(!opened){
    log_v("open file error\r\n");
    return false;
  }

  char line[96];
  int n = snprintf(line, sizeof(line), "Version: %s\r\n", VERSION);
  if(n > 0) { f.write((const uint8_t*)line, n); }
  n = snprintf(line, sizeof(line), "FW Build: %s %s\r\n", __DATE__, __TIME__);
  if(n > 0) { f.write((const uint8_t*)line, n); }
  n = snprintf(line, sizeof(line), "FANET ID: %02X%04X\r\n", FANET_VENDOR_ID, get_fanet_id());
  if(n > 0) { f.write((const uint8_t*)line, n); }

  const char *lora_name = "UNKNOWN";
  if(lora_module == LORA_SX1276) { lora_name = "SX1276"; }
  if(lora_module == LORA_SX1262) { lora_name = "SX1262"; }
  if(lora_module == LORA_LLCC68) { lora_name = "LLCC68"; }
  n = snprintf(line, sizeof(line), "LoRa Module: %s\r\n", lora_name);
  if(n > 0) { f.write((const uint8_t*)line, n); }

  const char *baro_name = "UNKNOWN";
  if(chip_baro == BARO_BMP280) { baro_name = "BMP280"; }
  if(chip_baro == BARO_BMP3xx) { baro_name = "BMP3xx"; }
  if(chip_baro == BARO_SPL06) { baro_name = "SPL06"; }
  if(chip_baro == BARO_HP203B) { baro_name = "HP203B"; }
  n = snprintf(line, sizeof(line), "Barometer: %s\r\n", baro_name);
  if(n > 0) { f.write((const uint8_t*)line, n); }

  if(!f.sync()){
    log_v("file sync failed\n");
  }
  if(!f.close()){
    log_v("file close failed\n");
    return false;
  }

  if(!sync_flash_fs()){
    return false;
  }

  if(!fatfs.exists(path)){
    log_v("Version file entry missing after write\r\n");
    return false;
  }

  //log_v("Version file success\r\n");
  return true;
}