#include "tools.h"
#include "defines.h"
#include "logging.h"
#include "msc.h"

uint32_t sleeptime_cum = 0; // cumulative time spend in sleepmode, used for time() calculation
uint32_t sleep_offset =0; //time passed since last increment of time(). used for ws80 deepsleep reading time offset from rtc

HW_Version bd_hw_version = HW_unknown;
Settings settings;


// Helper ----------------------------------------------------------------------------------------------------------------------

const char* wakeup_source_string [4] = {"NONE", "RTC", "EIC", "WDT"};

// if a WS80 or WS85 sensor with "DEBUG hack" is connected
bool is_wsxx(){
  return settings.sensor_type == s_WS85 || settings.sensor_type == s_WS80;
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

#ifdef HAS_HEATER
  //if(strcmp(settingName,"HEATER")==0) {settings.is_heater = atoi(settingValue); return 1;}
  //if(strcmp(settingName,"V_HEATER")==0) {settings.heater_voltage  = atof(settingValue); return 1;}
  //if(strcmp(settingName,"V_MPPT")==0) { settings.mppt_voltage = atof(settingValue); return 1;}
#endif

  if(strcmp(settingName,"REDU_INTERV_VOLT")==0) { settings.reduce_interval_voltage = atof(settingValue); return 1;}

  if(strcmp(settingName,"HEADING_OFFSET")==0) {settings.heading_offset = atoi(settingValue); return 1;}
  if(strcmp(settingName,"GUST_AGE")==0) {settings.gust_age = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"WIND_AGE")==0) {settings.wind_age = (uint32_t)atoi(settingValue)*1000; return 1;}
  
// Broadcast intervals in seconds, 0 = disable
  if(strcmp(settingName,"BROADCAST_INTERVAL_WEATHER")==0) {settings.broadcast_interval_weather = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_NAME")==0)    {settings.broadcast_interval_name = (uint32_t)atoi(settingValue)*1000; return 1;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_INFO")==0)    {settings.broadcast_interval_info = (uint32_t)atoi(settingValue)*1000; return 1;}

  if(strcmp(settingName,"SENSOR_BARO")==0) {settings.is_baro = atoi(settingValue); return 1;}
  if(strcmp(settingName,"SENSOR_DAVIS6410")==0) { if(atoi(settingValue)){settings.sensor_type = s_DAVIS6410;} return 1;}
  if(strcmp(settingName,"SENSOR_WSXX")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS80;} return 1;} // auto detection
  if(strcmp(settingName,"SENSOR_WS80")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS80;} return 1;} // keep for comatibility with old settings file
  if(strcmp(settingName,"SENSOR_WS85")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS85;} return 1;}
  if(strcmp(settingName,"SENSOR_WS85_UART")==0) { if(atoi(settingValue)){settings.sensor_type = s_WS85_UART;} return 1;}

// Davis 6410 specific
  if(strcmp(settingName,"SENSOR_INTEGRATION_TIME")==0) {settings.sensor_integration_time = atoi(settingValue); return 1;}

  if(strcmp(settingName,"SENSOR_GPS")==0) {settings.is_gps = atoi(settingValue); return 1;}
  if(strcmp(settingName,"GPS_BAUD")==0) {settings.gps_baud = atoi(settingValue); return 1;}

  if(strcmp(settingName,"DEBUG")==0) {log_set_debug(atoi(settingValue)); return 1;}
  if(strcmp(settingName,"ERRORS")==0) {log_set_error(atoi(settingValue)); return 1;}
  if(strcmp(settingName,"INSOMNIA")==0) {settings.no_sleep = atoi(settingValue); return 1;}
  if(strcmp(settingName,"TEST_USB")==0) {settings.test_with_usb = atoi(settingValue); return 1;}
  if(strcmp(settingName,"TESTMODE")==0) {settings.testmode = atoi(settingValue); return 1;}
  if(strcmp(settingName,"WDT")==0) {settings.use_wdt = atoi(settingValue); return 1;}
  if(strcmp(settingName,"DIV_CPU_SLOW")==0) {settings.div_cpu_slow = atoi(settingValue); return 1;}
  if(strcmp(settingName,"FORWARD_UART")==0) {settings.forward_serial_while_usb = atoi(settingValue); return 1;}

// Test commands
  if(strcmp(settingName,"TEST_HEATER")==0) {settings.test_heater = atoi(settingValue); return 1;}
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
    log_i("Name: "); log_i(settings.station_name.c_str()); log_i("\r\n");
    //log_i("Lon: ", pos_lon);
    //log_i("Lat: ", pos_lat);
    //log_i("Alt: ", altitude);
    log_flush();
//  #ifdef HAS_HEATER
//    if(bd_hw_version == HW_1_3){
//      log_i("Heater voltage: ", settings.heater_voltage);
//      log_i("MPPT voltage: ", settings.mppt_voltage);
//    }
//  #endif
    log_i("Heading offset: ", settings.heading_offset); 
    log_i("Broadcast interval weather [s]: ", settings.broadcast_interval_weather/1000);
    if(is_wsxx()){log_i("Sensor: WSXX Auto detect\n");}
    if(settings.sensor_type == s_DAVIS6410){log_i("Sensor: DAVIS 6410\n");}
    log_flush();
  }
}



// get current millis since reset, including time spend in deepsleep. Missing time waiting for UART ws80 sensor
uint32_t time(){
  return millis() + sleeptime_cum + sleep_offset;
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
  if( bd_hw_version >= HW_2_0){
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
  } else {
    ret = led_status(s);
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

      if(address == 0x2F){bd_hw_version = HW_1_3;} // only HW1.3 has digipot
 
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
  return ((settings.pos_lon < -30.0 && settings.pos_lon > -180.0) ||                  // North/South America
    (settings.pos_lon > 110.0 && settings.pos_lon <= 180.0) ||                  // Japan, Australia, NZ
    (settings.pos_lon >= 75.0 && settings.pos_lon <= 110.0 && settings.pos_lat > 15.0)  // China
    );
}