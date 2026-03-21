#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <array>


#include <LibPrintf.h>

#include <SdFat.h>
#include <SAMD_InternalFlash.h>
#include "wiring_private.h" // pinPeripheral() function


#include "defines.h"
#include "hist.h"
#include "sensors.h"
#include "logging.h"
#include "sleep.h"
#include "fanet.h"
#include "display.h"
#include "msc.h"
#include "tools.h"
#include "radio.h"


// Todo list:
// * Read bootloader version
// * detect sensor frozen?

// https://github.com/adafruit/Adafruit_TinyUSB_Arduin
// https://github.com/adafruit/ArduinoCore-samd
// https://github.com/Mollayo/SAMD_InternalFlash
// https://github.com/Microsoft/uf2
// https://github.com/adafruit/uf2-samdx1

//Modifications:
// SAMD_InternalFlash.cpp:  
// use last 40kb of flash as FAT12 disk for settings file
//    _flash_address = (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)


// Serial2 for Degugging on HW > 1.3
Uart Serial2(&sercom1, PIN_SERCOM1_RX, PIN_SERCOM1_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0 ) ;

// WS85 with UART
WS85WindSensor ws85uart(&Serial1);

// Pulsecounter
volatile uint32_t pulsecount =0; // pulses from wind speed sensor using reed switch

// WDT and CPU Clock
#define WDT_PERIOD 2500 // ms for wdt to wait to reset mcu if not reset in time

int div_cpu = 1; // current div
bool first_sleep = true; // first sleep after reset, USB perephial could still be on
bool usb_connected = false;
uint32_t next_tx_time = 0;

uint32_t sleep_allowed = 0; // time() when is is ok so eenter deepsleep
bool reduced_interval = false; // reduced interval active
bool undervoltage = false;

// ### Variables for storing settings from file, may be overewritten #####

bool use_mcp4652 = true; // used on first version of PCB (<=1.3) to set MPPT and DCDC voltage

bool settings_ok = false;


// Message timing

uint32_t last_msg_weather = 0;
uint32_t last_msg_name = 0;
uint32_t last_msg_info = 0;

uint32_t broadcast_scale_factor = 1; // multiplier for broadcast values. is set to 2..5 if battery is low


uint32_t last_fnet_send = 0; // last package send
uint32_t fanet_cooldown = 4000;
uint32_t loopcounter = 0;

int wakeup_source = WAKEUP_NONE;

void switch_WS_power (bool state){
  static bool current_power_state = false;
  if(state && !current_power_state){ // turn on
    current_power_state = true;
    pinMode(PIN_PS_WS,OUTPUT);
    digitalWrite(PIN_PS_WS, 0);
  }
  if(!state && current_power_state){ // turn off
    current_power_state = false;
    digitalWrite(PIN_PS_WS, 1);
  }
}

uint8_t voltageToSOCNonLinear(float v) {
    if(v < 0.8) {led_error(1); log_i("V_Batt read error: ", v); return 0;} // bad reading
    if(v < 3.4) {switch_WS_power(0); return 0;}
    if(v > 3.5) {switch_WS_power(1); undervoltage = false;}
    if(v > 4.15){return 100;}

    float soc = powf((v - 3.4f) / (4.15f - 3.4f), 1.5f);
    return (uint8_t)(soc * 100.0f + 0.5f);
}

// Trigger ADC and calc battery value in percent and volts
void read_batt_perc(){
  static uint32_t last_battery_reading=0;
  // only sample if last reading is older than 100ms
  if(time()- last_battery_reading > 100){
    last_battery_reading = time();
    analogReference(AR_INTERNAL1V0);
    analogReadResolution(10);
    pinMode(PIN_V_READ, INPUT);
    //delayMicroseconds(10);
    float val=0;
    digitalWrite(PIN_V_READ_TRIGGER,0);
    delayMicroseconds(10);
    for( int i= 0; i< 4; i++){
      val += (float) analogRead(PIN_V_READ);
    }
    digitalWrite(PIN_V_READ_TRIGGER,1);

    //pinDisable(PIN_V_READ_TRIGGER); //disable at sleep begin
    val /=4;
    pinDisable(PIN_V_READ);
    val *= 0.0040925; // 100k/330k 1.0V Vref
    sensor.batt_volt = val;
  
  sensor.batt_perc = voltageToSOCNonLinear(sensor.batt_volt);
  log_i("V_Bat: ", sensor.batt_volt);
  log_i("Bat_perc: ", sensor.batt_perc);
  }
}

// Sleep ----------------------------------------------------------------------------------------------------------------------

// dummy function
void wakeup_EIC(){
  wakeup_source = WAKEUP_EIC;
}

uint32_t sleep(bool enable_uart_interrupt){

  // Disable Debug uart
  //if(debug_enabled()){
  //  DEBUGSER.end();
  //}

  // disable wdt during sleep
  if(settings.use_wdt) {
    wdt_disable();
  }

  if(enable_uart_interrupt){
    if(is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD){
      enable_sercom0_int();
      //log_i("Enable UART Interrupt\r\n");
    }
  }

  reset_time_counter(); // start counting sleeptime from zero
  
  deepsleep(false); // no light sleep
  uint32_t p = micros();



  if(is_wsxx() || settings.sensor_type == s_WINDNERD){
    disable_sercom0_int();
    p = micros()-p;
    //DEBUGSER.println(p);
    SENSOR_UART.begin(115200*div_cpu);
  }

  if(settings.sensor_type == s_WS85_UART) {
    disable_sercom0_int();
    ws85uart.begin(); // required to get data
  }

  uint32_t t = read_time_counter();

  sleeptime_cum += t; // add slept time to global time counter. Should be done directly after wakeup but we need every tick to get UART working first

  // Disable Debug uart again
  //if(debug_enabled()){
  //  DEBUGSER.begin(115200*div_cpu);
  //}
  // re-enable wdt after sleep
  if(settings.use_wdt) {
    wdt_enable(WDT_PERIOD, false);
  }
  return t;
}

// called after sleep
void wakeup(){
  // USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE; // Re-enable USB, no need, not working?
  loopcounter = 0;
  log_i("\r\n########\r\n");
  log_i("Wakeup: ", time()); 
  log_i("Wakeup_source: "); log_i(wakeup_source_string[wakeup_source]);log_i("\r\n");
  wakeup_source = WAKEUP_NONE;

  pinMode(PIN_V_READ_TRIGGER, OUTPUT); // prepare voltage measurement, charge trigger cap
  digitalWrite(PIN_V_READ_TRIGGER,1);

  get_solar_charger_state();
  if(settings.use_baro){
    baro_start_reading(); // request data aquisition, will be read later
  }
  if(settings.use_imu){
    get_imu();
  }

  read_batt_perc();
  sleep_allowed = time() + 100; // go back to sleep after 6 secs as fallback
}

uint32_t timeToSend(int last, int interval) {
  if (interval) {
    int delay = interval * broadcast_scale_factor;
    int elapsed = time() - last;
    return (elapsed < delay) ? (delay - elapsed) : 0;
  }
  return -1;
}

// calc time to sleep til next fanet message needs to be send
uint32_t calc_time_to_sleep(){
  uint32_t tts_weather = -1;
  uint32_t tts_name = -1;
  uint32_t tts_info = -1;
  uint32_t tts = 0;
  static uint32_t last_print = 0;

  if(sensor.batt_volt){
    if(sensor.batt_volt < VBATT_LOW){
      tts = 3600*500; // sleep 30min
      undervoltage = true;
      broadcast_scale_factor = 5;
      log_i("Undervoltage\n");
      next_tx_time = time()+tts;
      return tts;
    } else if(sensor.batt_volt < settings.reduce_interval_voltage){
      reduced_interval = true;
      broadcast_scale_factor = 3;
      log_i("Low voltage\n");
    } else {
      // battery voltage is normal
      reduced_interval = false;
      broadcast_scale_factor = 1;
    }
  }

  tts_weather = timeToSend(last_msg_weather, settings.broadcast_interval_weather);
  tts_name    = timeToSend(last_msg_name, settings.broadcast_interval_name);
  tts_info    = timeToSend(last_msg_info, settings.broadcast_interval_info);

  if(millis()-last_print > 2500){
    last_print = millis();
    log_i("tts_weather: ", tts_weather);
    log_i("tts_name: ", tts_name);
    //log_i("tts_info: ", tts_info);
  }
  
  tts = min(min(tts_name, tts_info), tts_weather);
  if( fanet_cooldown && last_fnet_send  && (time() - last_fnet_send + tts < fanet_cooldown)){
    tts += fanet_cooldown - (time()-last_fnet_send);
  }
  log_i("tts: ", tts);
  if(tts == (uint32_t)-1){ tts=0;}
  

  if( !tts && loopcounter > 100){
    log_e("just looping, will sleep\n");
    tts = 12000; // set to sleep to keep updating sensor data
  }

  next_tx_time = time()+tts;

  
  return tts;
}

// RTC Handler callback, do not rename. gets called on rtc (timer) interrupt
void RTC_Handler(void){
  if (RTC->MODE1.INTFLAG.bit.OVF && RTC->MODE1.INTENSET.bit.OVF) {  // Check if an overflow caused the interrupt
    RTC->MODE1.INTFLAG.bit.OVF = 1;                                 // Reset the overflow interrupt flag
    wakeup_source = WAKEUP_RTC;
  }
}

// shut everything down, enable deepsleep
void go_sleep(){

  pinDisable(PIN_V_READ_TRIGGER);

// shut down the USB peripheral
  if(first_sleep){
    log_i("Disable USB\r\n");
    USB->DEVICE.CTRLA.bit.ENABLE = 0;                   
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE){};
    first_sleep = false;
    usb_connected = false;

    if(set_cpu_div(settings.div_cpu_slow)){ //USB needs 48Mhz clock, as we are finished with USB we can lower the cpu clock now.
      div_cpu = settings.div_cpu_slow;
      DEBUGSER.begin(115200*div_cpu); // F_CPU ist still 48M, so every clock needs to by multiplied manually
    }
  }

  uint32_t time_to_sleep = calc_time_to_sleep();
  rtc_sleep_cfg(time_to_sleep);

  if(undervoltage){
    sleep(false);
    return; // do not read sensors
  }

  if(settings.sensor_type == s_DAVIS6410){ 
    time_to_sleep = min(time_to_sleep, settings.sensor_integration_time);  // interval for gust detection 
    rtc_sleep_cfg(time_to_sleep);
  }

  // move to setup function
  if(!settings_ok){ 
    log_e("No settings file\r\n");
    log_e("Sleeping forever\r\n");
    time_to_sleep = 0xFFFFFFF;
  } // if settings not ok sleep forever
  
  //if(!time_to_sleep){ return;} // if time_to_sleep = 0, do not sleep at all

  log_i("will sleep for ", time_to_sleep > 200000UL?-1: time_to_sleep);
  log_flush();



// Using TC3 for hardware pulsecounting on Falling edge on pin PA04 (D17). No interrupts needed.
  if(settings.sensor_type == s_DAVIS6410){ // pulse counting anemometer
    uint32_t t = 0;
    setup_pulse_counter(); // need to setup GCLK6 before

    while(wakeup_source != WAKEUP_RTC){ // ignore other wakeups (external pin Interrupt, if configured)
      t += sleep(false);
    }
    pulsecount = read_pulse_counter();
    calc_pulse_sensor(pulsecount, t);
  } 
  // UART sensor, just sleep
  else if(is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD){ // UART based sensor
    int res = -1;

    while(wakeup_source != WAKEUP_RTC){
      sleep(true); // sleep til UART interrupt

      if(is_wsxx()){
        res = read_wsxx();
      }
      else if(settings.sensor_type == s_WS85_UART){
        res = read_ws85_uart();
      }
      else if(settings.sensor_type == s_WINDNERD){
        res = read_windnerd();
      }
      
      if(is_wsxx() && res == RESP_COMPLETE){ // if RESP_COMPLETE, we received a data block, so it is ok to sleep for ~ 4 seconds without listening to serial data
        int32_t t = next_tx_time - time();
        if(t > 0){
          if(settings.sensor_type == s_WS85){      rtc_sleep_cfg( min(t,8350)); }
          if(settings.sensor_type == s_WS80){      rtc_sleep_cfg( min(t,4685)); } // sleep 4750ms if tts is still longer, or less if it less
          //log_i("will sleep1: ", actual_sleep); 
          //log_flush();
          sleep(false); // sleep without UART interrupt
        }
      }

    }
  }
  
  /*
  if(set_cpu_div(settings.div_cpu_fast)){
    div_cpu = settings.div_cpu_fast;
    log_ser_begin();
  }
  */
  wakeup();
}


void print_data(){
  if((debug_enabled())){
    log_i("\r\nmillis: ", millis()); 
    log_i("time: ", time()); 
    //log_i("Wind dir_raw: ", wind_dir_raw);
    log_i("Wind Heading: ", sensor.wind_heading);
    log_i("Wind Speed: ", sensor.wind_speed);
    log_i("Wind Gust: ", sensor.wind_gust);
    log_i("Temp: ", sensor.temperature);
    //log_i("Humd: ", humidity);
    if(settings.use_baro){
    log_i("Baro: ", sensor.baro_pressure);
    log_i("PCB_Temp: ", sensor.baro_temp);
    }
    if(is_wsxx()) {
      log_i("VCC: ", sensor.wsxx_vcc);
      //log_i("LUX: ", light_lux);
      //log_i("UV: ", uv_level);
    }
    log_i("\r\n");
    log_i("V_Bat: ", sensor.batt_volt);
    log_i("Bat_perc: ", sensor.batt_perc);
    log_i("PV_charge: ", sensor.pv_charging);
    log_i("PV_done: ", sensor.pv_done);
  }
}



// this is not working yet. The file is written to flash, but the record is not added to the FAT correctly.
bool create_versionfile(char * filename){
  //flash.setIndicator(PIN_ERRORLED, 1);
  File f;
  if (fatfs.begin(&flash) ){
    if(fatfs.exists(filename)){ 
      log_i("Version file exists\n");
      //return false;
    }
    log_i("creating file\n");
    f = fatfs.open(filename, FILE_WRITE);
    if (f) {
      log_i("Creating version file\n");
      f.print("Version: "); f.println(VERSION);
      f.print("FW Build: "); f.print(__DATE__); f.println(__TIME__);
      f.print("FANET ID: "); f.print(FANET_VENDOR_ID,HEX); f.println(get_fanet_id(),HEX);
      f.print("LoRa Module: "); 
        if(lora_module == LORA_SX1276) { f.println("SX1276");}
        if(lora_module == LORA_SX1262) { f.println("SX1262");}
        if(lora_module == LORA_LLCC68) { f.println("LLCC68");}
      f.print("Barometer: "); 
        if(chip_baro == BARO_BMP280) { f.println("BMP280");}
        if(chip_baro == BARO_BMP3xx) { f.println("BMP3xx");}
        if(chip_baro == BARO_SPL06) { f.println("SPL06");}
        if(chip_baro == BARO_HP203B) { f.println("HP203B");}
      if(!f.close()){log_i("file close failed\n");}
      if(fatfs.exists(filename)){ 
      log_i("Version file sucess\n");
      }
      return true;
    } else {log_i("open file error\n");}
  } else {
    log_i("fs start fail\n");
  }
  return false;
}

// parse settingsfile
bool parse_file(char * filename){
  #define LINEBUFFERSIZE 512
  bool ret = false;
  led_status(1);
  File f;
  char linebuffer [LINEBUFFERSIZE];
  int c = 0;
  int co = 0;
  int filesize =0;
  //log_i("Reading Settings from file\r\n");

  if (fatfs.begin(&flash) ){
    f = fatfs.open(filename, FILE_READ);
    if (f) {
        filesize = f.available();
        //log_i("Filesize: ", filesize);
        while (filesize - c > 0) {
          linebuffer[co] = f.read();

          if(linebuffer[co] == '\n'){
            process_line(linebuffer, co, &apply_setting); // Line complete
            co=-1; // gets +1 below
          }
          c++;
          co++;
          if(co >= LINEBUFFERSIZE){
            log_e("File buffer error\r\n");
            return false;
          }
        }
        process_line(linebuffer, co, &apply_setting);
        f.close();
        //log_i("Settingsfile closed\n");
        if(settings.pos_lat != 0 && settings.pos_lon != 0 && settings.sensor_type != s_invalid){
          ret = true;
        }
        led_status(0); // if LED stay on, settings failed
    }else {
      log_i("File not exists\r\n");
    }
    my_internal_storage.flush_buffer(); // sync with flash
  } else {
    log_e("Failed to start FS\r\n");
  }
  if(!ret){
    led_status(0);
    led_error(1);
    log_e("Coordinates invalid\n");
    }
  return ret;
}

// Serial reads ----------------------------------------------------------------------------------------------------------------------
// read serial data from USB, for debugging
void read_serial_cmd(){
  #define CMDBUFFERSIZE 127
  static char buffer [CMDBUFFERSIZE];
  static int co = 0;
  bool ok = false;

  while (Serial.available()){
    buffer[co] = Serial.read();
    //DEBUGSER.write(buffer[co]);
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

// Send ----------------------------------------------------------------------------------------------------------------------


void send_msg_weather(){
  if(settings.sensor_type == s_invalid){ return;}

  led_status(1);
  WindSample current_wind = get_wind_from_hist(settings.wind_age);
  sensor.wind_gust = get_gust_from_hist(settings.gust_age);
  sensor.wind_speed = current_wind.wind/10.0;
  sensor.wind_dir_raw = current_wind.dir_raw;
  sensor.wind_heading = sensor.wind_dir_raw + settings.heading_offset;
  if(sensor.wind_heading > 359){ sensor.wind_heading -=360;}
  if(sensor.wind_heading < 0){ sensor.wind_heading +=360;}

  if(settings.sensor_type == s_DAVIS6410){ // no other temp sensor
    sensor.temperature= sensor.baro_temp;
  } 
  if( sensor.wind_speed > (sensor.wind_gust +3)){
    sensor.wind_gust = sensor.wind_speed;
    log_i("adapting gust speed\r\n");
  }

  weatherData wd;
  wd.vid = FANET_VENDOR_ID;
  wd.fanet_id = get_fanet_id();
  wd.lat = settings.pos_lat;
  wd.lon = settings.pos_lon;
  wd.bWind = true;
  wd.wHeading = sensor.wind_heading;
  wd.wSpeed = sensor.wind_speed;
  wd.wGust = sensor.wind_gust;      
  wd.bTemp = true;
  wd.temp = sensor.temperature;

  if(is_wsxx()){
    wd.bHumidity = true;
    wd.Humidity = sensor.humidity;
  }

  if(settings.use_baro){
    wd.bBaro = true;
    wd.Baro = sensor.baro_pressure;  
  } else {
    wd.bBaro = true; // baro is required to forward data in OGN
    wd.Baro = -1;  
  }
  wd.bStateOfCharge = true;
  wd.Charge = sensor.batt_perc;

  if(settings.testmode){
    wd.bBaro = true;
    wd.Baro = sensor.baro_pressure;  
    wd.wHeading = 123;
    wd.wSpeed = 5;
    wd.wGust = 7;      
    wd.temp = 10;
    wd.Humidity = 15;
    log_i("\r\nTESTMODE - Fake values\r\n");
  }

  log_i("\r\nSending Weather\r\n");

  constexpr size_t msgSize = sizeof(fanet_packet_t4);
  std::array<uint8_t, sizeof(fanet_packet_t4)> buffer = {0};
  pack_weatherdata(&wd, buffer.data());

// write buffer content to console
#if 0
  for (int i = 0; i< msgSize; i++){
    printf("%02X ", (buffer)[i]);
  }
  DEBUGSER.println();
#endif

  dis_rx_sleep(); // radio_phy->standby();
  radio_phy->startTransmit(buffer.data(), msgSize);

  print_data();
  led_status(0);
  save_history(sensor.wind_speed, sensor.temperature, sensor.humidity, sensor.light_lux, sensor.batt_volt, sensor.pv_charging, sensor.pv_done); // only save history on send
  
}


// check if everything is ok to send the wather data now
bool allowed_to_send_weather(){
  bool ok = settings_ok;
  

  if (ok){
    if(settings.use_baro){ok &= ((time() - sensor.last_baro_reading) < 10000 );}
    if(is_wsxx() || settings.sensor_type == s_WS85_UART) { ok &= (sensor.last_data || settings.testmode); } // only send if weather data is up to date or testmode is enabled // && (time()- last_ws80_data < 9000))
    if(settings.use_gps)  { ok &= (tinyGps.location.isValid()); } // only send if position is valid
  }

 // if(next_baro_reading){
 //   log_i("next_baro_reading: ", next_baro_reading);
 // }

 // Print reasons for not beeing ready
  if(!ok){
    //if(last_data){
      log_i("Wind Sensor data age: ", time()- sensor.last_data);
      log_i("Baro data age: ", time()- sensor.last_baro_reading);
   // }
    if(settings.use_gps){
      if(time()-sensor.last_gps_valid > 3000){
        log_i("No GPS fix");
      }
    }
  }


  return ok;
}

// check if last fanet package want sent recenctly
bool fanet_cooldown_ok(){
  if(lora_module && (time() -last_fnet_send > fanet_cooldown)){
    return true;
  }
  return false;
}

void send_msg_name(const char* name, int len){
  constexpr int FANET_MAX_PACKET_SIZE = 255;
  if (len < 0) {
    return;
  }
  if ((len + 4) > FANET_MAX_PACKET_SIZE) {
    len = FANET_MAX_PACKET_SIZE - 4;
  }
  std::array<uint8_t, FANET_MAX_PACKET_SIZE> buffer = {0};
  fanet_header header;
  header.type = 2;
  header.vendor = FANET_VENDOR_ID;
  header.forward = false;
  header.ext_header = false;
  header.address = get_fanet_id();

  memcpy(buffer.data(), (uint8_t*)&header, 4);
  memcpy(&buffer[4], name, len);

// write buffer content to console
#if 0
  for (int i = 0; i< len+4; i++){
    printf("%02X ", (buffer)[i]);
  }
  DEBUGSER.println();
#endif


  dis_rx_sleep(); // radio_phy->standby();
  radio_phy->startTransmit(buffer.data(), len+4);
}

void send_msg_info(){
  char data[50] = {0x00};
  uint8_t data_len = 1;
  // Test: send battery voltage and charging state
  data_len += sprintf(&data[1], "%04X:%s %0.2fV C%i", get_fanet_id(), VERSION, sensor.batt_volt, sensor.pv_charging);


  constexpr int FANET_MAX_PACKET_SIZE = 255;
  if ((data_len + 4) > FANET_MAX_PACKET_SIZE) {
    data_len = FANET_MAX_PACKET_SIZE - 4;
  }
  std::array<uint8_t, FANET_MAX_PACKET_SIZE> buffer = {0};
  fanet_header header;
  header.type = 3;
  header.vendor = FANET_VENDOR_ID;
  header.forward = false;
  header.ext_header = false;
  header.address = get_fanet_id();

  memcpy(buffer.data(), (uint8_t*)&header, 4);
  memcpy(&buffer[4], data, data_len);

// write buffer content to console
#if 1
  for (int i = 0; i< data_len+4; i++){
    printf("%02X ", (buffer)[i]);
  }
  DEBUGSER.println();
#endif
  log_i("Sending Info Msg\n");
  dis_rx_sleep(); // radio_phy->standby();
  radio_phy->startTransmit(buffer.data(), data_len+4);
}




// Setup ----------------------------------------------------------------------------------------------------------------------

extern uint32_t __etext;

void setup(){

  log_set_debug(true);
  DEBUGSER.begin(115200); // on boot start with 48Mhz clock // log_ser_begin(); 

  pinPeripheral(PIN_SERCOM1_RX, PIO_SERCOM_ALT); 
  pinPeripheral(PIN_SERCOM1_TX, PIO_SERCOM_ALT); 

  printf_init(DEBUGSER);
  log_i("\r\n--------------- RESET -------------------\r\n");
  log_i("Version: ");  log_i(VERSION); log_i("\r\n");
  log_i("FW Build Time: ");  log_i(__DATE__); log_i(" "); log_i(__TIME__); log_i("\r\n");

  
  //printf("code end: %p\n", (void *)(&__etext));
  //printf("flash_start: %p\n", my_internal_storage.get_flash_address());
  //printf("flash_size: %lu\n", my_internal_storage.get_flash_size());

  Wire.begin();
  i2c_scan();

  setup_display();
  if(display_present()){
    log_i("I2C Display enabled\n");
  }
  
  //printf("FANET ID: %02X%04X\r\n",fmac.myAddr.manufacturer,fmac.myAddr.id);
  if(setup_flash()){
    settings_ok = parse_file(SETTINGSFILE);
    if(!debug_enabled()){
      DEBUGSER.println("Debug messages disabled");
      DEBUGSER.flush();
      DEBUGSER.end();
      pinDisable(PIN_SERCOM1_RX);
      pinDisable(PIN_SERCOM1_TX);
    }
  }

  // Init radio after reading settings
  if(radio_init()){
    radio_phy->setPacketSentAction(set_fanet_send_flag);
    radio_sleep(); // radio_phy->sleep();
  } else { 
    led_error(1);
    display_delay(2000);
  }


  if(settings_ok){
    //led_error(0);
    // Add altitude to station name, gets splittet by breezedude ogn parser
    if(settings.altitude > -1){
      settings.station_name += " (" + String(int(settings.altitude)) + "m)"; // Testation (1234m)
    }

    settings.use_pulse_counter = true; // settings.sensor_type == s_DAVIS6410; // add other pulsecounting sensors here
    settings.use_rtc_counter = true; // use for all sensors

    setup_PM(settings.use_pulse_counter, settings.use_rtc_counter);
    wdt_enable(WDT_PERIOD,false); // setup clocks
    if(!settings.use_wdt) {
      wdt_disable();
    }

    if(settings.use_baro){
      if(!init_baro()){
        led_error(1);
      }
    }

    // check for IMU
    if(settings.use_imu){
      if(!init_imu()){
        led_error(1);
      }
    }

    print_settings();
  }

  if(!settings_ok) { // invalid settings, just lite the red LED and sleep
    // Needed for deepsleep
    setup_PM(false, true);
    wdt_enable(WDT_PERIOD,false); // setup clocks
    wdt_disable();
    //setup_rtc_time_counter();
  }

  if(is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD){
    switch_WS_power(1); // Turn on WS80 Power supply with P-MOSFET
    setup_rtc_time_counter();

    if(is_wsxx() || settings.sensor_type == s_WINDNERD){
      SENSOR_UART.begin(115200);
    }
    if(settings.sensor_type == s_WS85_UART){
      log_i("Setup WS85\n");
      
      //ws85uart.setAutoSendInterval(8500);
      ws85uart.begin();
      //ws85uart.requestAutoSendInterval();
      ws85uart.set_baud_115200();
    }
  }

  if(settings.sensor_type == s_DAVIS6410){
    setup_rtc_time_counter();
  }

  if(settings.use_gps){
    GPS_SERIAL.begin(settings.gps_baud);
    log_i("Starting GPS with baud: ", settings.gps_baud);
  }

// init history array
  for( int i = 0; i< HISTORY_LEN; i++){
    history[i].set = false;
  }
  for( int i=0; i< WIND_HIST_LEN; i++){
    wind_history[i].time = 0;
    wind_history[i].gust = 0;
    wind_history[i].dir_raw = 0;
    wind_history[i].wind = 0;
  }
  // create_versionfile(VERSIONFILE); // create version file if not exists (not working)
  log_flush();
  wakeup();
}

// loop ----------------------------------------------------------------------------------------------------------------------

void loop(){

static uint32_t send_active=0; // if > 0, time() last message was send to tx queue, reset to 0 if send is complete
static uint32_t last_settings_check = 0; // timee() ckecked if a settings file is present if settings not read yet sucessfully

// print millis as alive counter
static uint32_t last_call = 0;
static bool s = false;

loopcounter++;

if(last_call && (time()-last_call > 15)){
  if(!s){led_status(0);}
}
//if(usb_connected && time()-last_call > 500){
  if(time()-last_call > 500){
  log_i("Time: ", time());
  //Serial.println(time());
  // Store led states and restore after blink
  s = led_status(1);
  last_call=time();
}

  if(settings.use_baro){read_baro();}
  if(settings.use_gps){read_gps();}

  if(fanet_cooldown_ok() && settings.broadcast_interval_name && ( (time()- last_msg_name) > (settings.broadcast_interval_name* broadcast_scale_factor)) ){ // once a hour
    if(settings.station_name.length() > 1){
      led_status(1);
      send_msg_name(settings.station_name.c_str(),settings.station_name.length());
      log_i("Send name: "); log_i(settings.station_name.c_str()); log_i("\r\n");
      last_fnet_send = time();
      last_msg_name = time();
      send_active = time();
      led_status(0);
    }
  }

  if(fanet_cooldown_ok() && settings.broadcast_interval_weather && ( (time()- last_msg_weather) > (settings.broadcast_interval_weather * broadcast_scale_factor)) ){
    if( allowed_to_send_weather() ){
      send_msg_weather();
      last_fnet_send = time();
      last_msg_weather = time();
      send_active = time();
    } else {
      if((is_wsxx() || settings.sensor_type == s_WS85_UART) && sensor.last_data && (time()- sensor.last_data > 10000)){
        log_i("Wdata not ready. Wdata age: ", (time()- sensor.last_data) );
        log_i("Last Baro reading age: ", (time()- sensor.last_baro_reading) );
        sleep_allowed = time() + 1;
        //sensor.last_data = 0;
      }
      
    }
  }
  if(settings.broadcast_interval_info && fanet_cooldown_ok() && ( (time()- last_msg_info) > (settings.broadcast_interval_info * broadcast_scale_factor)) ){
      led_status(1);
      send_msg_info();
      last_fnet_send = time();
      last_msg_info = time();
      send_active = time();
      led_status(0);
  }

  if(send_active){
    if( (time()- send_active > (3500))){
      led_error(1);
      log_i("Send timed out\r\n");
      led_status(0);
      send_active =0;
      sleep_allowed = time() + (1);
      radio_sleep(); // radio_phy->sleep();
      delay(10);
      led_error(0);
    }

    if(transmittedFlag){
      transmittedFlag = false;
      //log_i("Send complete\r\n");
      send_active = 0;
      sleep_allowed = time() + (1);
      radio_phy->finishTransmit();
      radio_sleep(); // radio_phy->sleep();
    }
  }

  if(settings.lora_smart_rcv){
    fanet_rx();
  }

// Check if everything is done --> sleep
  if(!send_active && sleep_allowed && (time() > sleep_allowed) && (!usb_connected) && (time() > 2500)){ // allow sleep after 2500 ms to get a change to detect usb connected
    go_sleep();
  }

  if(!settings_ok){ // Settings not ok. Try few times, then sleep
    if(!sleep_allowed){ 
      sleep_allowed = time() + 180000UL; // Sleep after 3 minutes
    }
    if(time()- last_settings_check > 15000){
      last_settings_check = time();
      log_e("\r\nFailed to obtain settings from file. Trying again\r\n");
      settings_ok = parse_file(SETTINGSFILE);
      if(settings_ok){ 
        NVIC_SystemReset();      // processor software reset
        }
    }
  }

  // during Dev
  if(usb_connected){
    if(settings.test_with_usb){read_wsxx();} // to simulate normal behavior without sleep read and parse data from serial port
    else {forward_sensor_serial();} // otherwise just forward the data
    read_serial_cmd(); // read setting values from serial for testing

    // keep usb alive for 15 min. Its not easyly possible to detect if still connected, so just restart after 15min
    if(!settings.test_with_usb && (time() > 15UL*60UL*1000UL)){
      log_i("Restart\r\n");
      log_flush();
      usb_connected = false;
      NVIC_SystemReset();
    }
  }

  if((time() > 5UL*60UL*1000UL)){ // trun off error LED after 5minutes to save energy if an error occures with no one around
    led_error(0);
  }

  if(settings.use_wdt){
    wdt_reset();
  }
}


/* Heater voltage/current:
3.5V 280mA
3.9 320mA
3.7 300mA
4.2v 340mA
5V 400mA
8V 650mA
10V 800mA
12V 980mA
*/

/*
========== WS85 Ver:1.0.7 ===========
>> g_RrFreqSel = 868M
>> Device_ID  = 0x002794
-------------------------------------
WindDir      = 76
WindSpeed    = 0.5
WindGust     = 0.6
GXTS04Temp   = 24.4

UltSignalRssi  = 2
UltStatus      = 0
SwitchCnt      = 0
RainIntSum     = 0
Rain           = 0.0
WaveCnt[CH1]   = 0
WaveCnt[CH2]   = 0
WaveRain       = 0
ToaltWave[CH1] = 0
ToaltWave[CH2] = 0
ResAdcCH1      = 4095
ResAdcSloCH1   = 0.0
ResAdcCH2      = 4095
ResAdcSloCH2   = 0.0
CapVoltage     = 0.80V
BatVoltage     = 3.26V
=====================================

========== WH80 Ver:1.2.8 ===========
>> RF_FreqSel = 868M
>> Device_ID  = 0x70014
-------------------------------------
WindDir      = 63
WindSpeed    = 0.6
WindGust     = 0.6

-------SHT30--------
Temperature  = 20.7
Humi         = 56%

-------Si1132-------
Light        = 150 lux

UV_Value     = 0.0

Not Detected Pressure Sensor!
Pressure     = --

BatVoltage      = 3.26V
=====================================
*/

// WS80 extended serial output

/*========== WH80 Ver:1.2.5 ===========
>> RF_FreqSel = 868M
>> Device_ID  = 0x00048
-------------------------------------
WindDir      = 338
WindSpeed    = 0.0
WindGust     = 0.8

-------SHT40--------
Temperature  = 24.3
Humi         = 57%

-------Si1132-------
Light        = 2630 lux
UV_Value     = 0.2

Not Detected Pressure Sensor!
Pressure     = --

BatVoltage      = 2.60V
=====================================

=====================================
max = 787, min = 783
max -min = 4
max = 786, min = 783
max -min = 3
max = 788, min = 783
max -min = 5
max = 787, min = 785
max -min = 2
------------------
CH_1 mag. normal
CH_2 mag. normal
CH_3 mag. normal
CH_4 mag. normal
------------------
Vol_CH1_3 = 252
Vol_CH3_1 = 262
Vol_CH4_2 = 264
Vol_CH2_4 = 265
SqWave_CH1_3 = 2
SqWave_CH3_1 = 2
SqWave_CH4_2 = 2
SqWave_CH2_4 = 2
min_index = 0
Min_Voltage = 252
absTv0 = 5
Source_CH1_3 = 100.00,3200
Source_CH3_1 = 99.81,3194
Source_CH4_2 = 99.88,3196
Source_CH2_4 = 99.56,3186
g_UltTimeV01_3 = 99.91,3197
datCnt1_3 = 2
g_UltTimeV04_2 = 99.72,3191
datCnt4_2 = -5
x_y = 53
Get_Cali_Ult_X = 51778
direction = 290
wind = 3

=====================================
max = 787, min = 784
max -min = 3
max = 784, min = 782
max -min = 2
max = 787, min = 783
max -min = 4
max = 789, min = 781
max -min = 8
------------------
CH_1 mag. normal
CH_2 mag. normal
CH_3 mag. normal
CH_4 mag. normal
------------------
Vol_CH1_3 = 254
Vol_CH3_1 = 261
Vol_CH4_2 = 263
Vol_CH2_4 = 263
SqWave_CH1_3 = 2
SqWave_CH3_1 = 2
SqWave_CH4_2 = 2
SqWave_CH2_4 = 2
min_index = 0
Min_Voltage = 254
absTv0 = 5
Source_CH1_3 = 100.06,3202
Source_CH3_1 = 100.00,3200
Source_CH4_2 = 99.91,3197
Source_CH2_4 = 99.78,3193
g_UltTimeV01_3 = 100.03,3201
datCnt1_3 = 6
g_UltTimeV04_2 = 99.84,3195
datCnt4_2 = 1
x_y = 60
Get_Cali_Ult_X = 55200
direction = 9
wind = 4
*/