#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <array>

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

uint32_t sleep_allowed = 0; // time() when is is ok so enter deepsleep
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
    // Re-connect UART pins to SERCOM0 peripheral before the sensor can respond
    pinPeripheral(PIN_RX, PIO_SERCOM);
    pinPeripheral(PIN_TX, PIO_SERCOM);
    SENSOR_UART.begin(115200 * div_cpu);
    log_i("WS sensor power ON\r\n");
  }
  if(!state && current_power_state){ // turn off
    current_power_state = false;
    digitalWrite(PIN_PS_WS, 1);
    // Disable UART pins so the idle-HIGH TX line cannot leak current into the
    // sensor through its input protection diodes when its VCC is removed.
    pinDisable(PIN_TX); // PA10 – UART idles HIGH, would source 8mA into sensor
    pinDisable(PIN_RX); // PA11 – symmetric isolation
    log_i("WS sensor power OFF\r\n");
  }
}

uint8_t voltageToSOCNonLinear(float v) {
    if(v < 0.8) {led_error(1); log_i("V_Batt read error: ", v); return 0;} // bad reading
    if(v < 3.15) {switch_WS_power(0); settings.uv_triggered = true; undervoltage = true; return 0;} // undervoltage, turn off sensor to save power
    if(v > 3.3) {switch_WS_power(1); undervoltage = false;}
    if(v > 4.15){settings.uv_triggered = false; return 100;}

    // Piecewise linear OCV->SOC for 3.7V Li-Ion at low discharge rate
    static const float vt[] = { 3.15f, 3.40f, 3.60f, 3.75f, 3.90f, 4.00f, 4.10f, 4.15f };
    static const float st[] = {  0.0f,  5.0f, 20.0f, 50.0f, 75.0f, 88.0f, 97.0f,100.0f };
    constexpr int N = 8;
    if (v <= vt[0]) return 0;
    if (v >= vt[N-1]) return 100;
    for (int i = 1; i < N; i++) {
        if (v <= vt[i]) {
            float soc = st[i-1] + (v - vt[i-1]) * (st[i] - st[i-1]) / (vt[i] - vt[i-1]);
            return (uint8_t)(soc + 0.5f);
        }
    }
    return 100;
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

    // Extend ADC sampling phase via SAMPCTRL instead of software delays between reads.
    // ADC clock = 48 MHz / 512 ≈ 94 kHz  →  half-cycle ≈ 5.3 µs.
    // SAMPLEN=4 adds 5 half-cycles ≈ 26 µs extra sample time per conversion.
    //ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(4);
    //while (ADC->STATUS.bit.SYNCBUSY);

    constexpr int N_SAMPLES = 15;
    uint16_t samples[N_SAMPLES];
    digitalWrite(PIN_V_READ_TRIGGER,0);
    delayMicroseconds(50); // allow divider to settle
    for (int i = 0; i < N_SAMPLES; i++) {
      samples[i] = analogRead(PIN_V_READ);
    }
    digitalWrite(PIN_V_READ_TRIGGER,1);

    //ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0); // restore default for other ADC users
    //while (ADC->STATUS.bit.SYNCBUSY);

    // Temporary: dump all raw samples to check trigger window width
    /*
    log_v("ADC samples: ");
    for (int i = 0; i < N_SAMPLES; i++) {
      log_v(", ", samples[i]);
    }
    log_v("\r\n");
    */

    // Trimmed mean: discard min + max, average remaining N_SAMPLES-2
    uint16_t vmin = samples[0], vmax = samples[0];
    uint32_t vsum = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
      vsum += samples[i];
      if (samples[i] < vmin) vmin = samples[i];
      if (samples[i] > vmax) vmax = samples[i];
    }
    float val = (float)(vsum - vmin - vmax) / (float)(N_SAMPLES - 2);

    pinDisable(PIN_V_READ);
    val *= 0.0040925; // 100k/330k 1.0V Vref

    // Moving average over last 5 readings to suppress short voltage dips
    constexpr int V_AVG_LEN = 5;
    static float v_history[V_AVG_LEN] = {0};
    static int v_idx = 0;
    static int v_count = 0;
    v_history[v_idx] = val;
    v_idx = (v_idx + 1) % V_AVG_LEN;
    if (v_count < V_AVG_LEN) v_count++;
    float v_sum = 0;
    for (int i = 0; i < v_count; i++) v_sum += v_history[i];
    sensor.batt_volt = v_sum / v_count;

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

void mark_uart_wakeup(){
  wakeup_source = WAKEUP_UART;
}

static bool keep_usb_active_during_sleep(){
  return settings.test_with_usb && usb_connected;
}

uint32_t sleep(bool enable_uart_interrupt){
  // Reset the wake reason before each sleep cycle so spurious USB IRQs can be ignored.
  wakeup_source = WAKEUP_NONE;

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
  if(settings.test_with_usb){
    log_flush();
  }
  reset_time_counter(); // start counting sleeptime from zero
  bool keep_usb = keep_usb_active_during_sleep();

  // With USB test mode active we use light sleep and ignore wakeups that do not
  // set a wakeup source, otherwise USB IRQ traffic would break the normal flow.
  do {
    deepsleep(keep_usb);
  } while (keep_usb && wakeup_source == WAKEUP_NONE);
  uint32_t p = micros();



  if(is_wsxx() || settings.sensor_type == s_WINDNERD){
    disable_sercom0_int();
    p = micros()-p;
    //DEBUGSER.println(p);
    SENSOR_UART.begin(115200*div_cpu);
  }

  if(settings.sensor_type == s_WS85_UART) {
    disable_sercom0_int();
    ws85uart.begin(div_cpu); // required to get data
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
  pinMode(PIN_V_READ_TRIGGER, OUTPUT); // prepare voltage measurement, charge trigger cap
  digitalWrite(PIN_V_READ_TRIGGER,1);
  log_i("\r\n########\r\n");
  log_i("Wakeup: ", time()); 
  log_v("Wakeup source: "); log_v(wakeup_source_string[wakeup_source]);log_v("\r\n");
  wakeup_source = WAKEUP_NONE;



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

enum TxAttemptResult : uint8_t {
  TX_ATTEMPT_NONE = 0,
  TX_ATTEMPT_STARTED,
  TX_ATTEMPT_DEFERRED_LBT,
};

// LBT (Listen Before Talk) checks whether the LoRa channel is free before TX.
// The radio is first taken out of RX sleep and prepared for transmission.
// If LBT is enabled, a CAD/channel scan is executed to detect ongoing LoRa activity.
// When the channel is free, the packet is transmitted normally and TX done is handled by IRQ.
// When the channel is busy, no packet is sent and the scheduler retries after a random 0-2 s delay.
// If LBT is disabled, the scan is skipped and transmission starts immediately.
static TxAttemptResult start_lbt_transmit(const uint8_t *data, size_t len) {
  dis_rx_sleep();
  // Arm TX-done callback here (not in dis_rx_sleep) to avoid re-arming DIO1
  // while the radio is in the sleep path, which causes spurious IRQ on LLCC68.
  radio_phy->setPacketSentAction(set_fanet_send_flag);
  if(!settings.lora_lbt) {
    radio_phy->startTransmit(data, len);
    return TX_ATTEMPT_STARTED;
  }
  if(!lbt_channel_free()) {
    radio_sleep();
    return TX_ATTEMPT_DEFERRED_LBT;
  }
  radio_phy->startTransmit(data, len);
  return TX_ATTEMPT_STARTED;
}

static uint32_t next_lbt_retry_delay() {
  return (millis() ^ (time() << 1) ^ get_fanet_id()) % 2001;
}

static void defer_next_send(uint32_t &last_msg, uint32_t interval, const char *label) {
  uint32_t retry_delay = next_lbt_retry_delay();
  uint32_t scaled_interval = interval * broadcast_scale_factor;
  last_msg = time() - scaled_interval + retry_delay;
  next_tx_time = time() + retry_delay;
  sleep_allowed = time() + 1;

  log_i("LBT: defer ");
  log_i(label);
  log_i(" TX by ms: ", retry_delay);
  log_i("\r\n");
}

// calc time to sleep til next fanet message needs to be send
uint32_t calc_time_to_sleep(){
  uint32_t tts_weather = -1;
  uint32_t tts_name = -1;
  uint32_t tts_info = -1;
  uint32_t tts = 0;
  static uint32_t last_print = 0;

  if(sensor.batt_volt){
    if(undervoltage){
      tts = 1000*30*60; // sleep 30min
      broadcast_scale_factor = 5;
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
    log_v("tts_weather: ", tts_weather);
    log_v("tts_name: ", tts_name);
    //log_i("tts_info: ", tts_info);
  }
  
  tts = min(min(tts_name, tts_info), tts_weather);
  if( fanet_cooldown && last_fnet_send  && (time() - last_fnet_send + tts < fanet_cooldown)){
    tts += fanet_cooldown - (time()-last_fnet_send);
  }
  log_v("tts: ", tts);
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
  bool keep_usb = keep_usb_active_during_sleep();

  pinDisable(PIN_V_READ_TRIGGER);

// shut down the USB peripheral
  if(first_sleep && !keep_usb){
    log_i("Disable USB\r\n");
    usb_ignore_detach_event = true;
    USB->DEVICE.CTRLA.bit.ENABLE = 0;                   
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE){};
    first_sleep = false;
    usb_connected = false;

    if(set_cpu_div(settings.div_cpu_slow)){ // USB test mode keeps 48MHz; only lower the clock for real low-power sleep.
      div_cpu = settings.div_cpu_slow;
      DEBUGSER.begin(115200*div_cpu); // F_CPU ist still 48M, so every clock needs to by multiplied manually
    }
  }

  uint32_t time_to_sleep = calc_time_to_sleep();
  if(time_to_sleep == 0){ return;} // if time_to_sleep = 0, do not sleep at all
  rtc_sleep_cfg(time_to_sleep);

  if(undervoltage){
    radio_sleep(); // ensure radio is in PHY sleep and DIO1 interrupt is detached
    log_i("Undervoltage. will sleep for ", time_to_sleep);
    log_flush();
    sleep(false); // will sleep in 30min blocks with 70µA current draw until battery recovers.
    wakeup(); // re-read battery so undervoltage clears if voltage has recovered
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

  // Ensure radio is in the correct state for the sleep window (RX if conditions
  // are met, PHY sleep otherwise). This also covers the period before the first
  // TX, when radio_sleep() has not yet been called from the TX-done path.
  radio_sleep();

// Using TC3 for hardware pulsecounting on Falling edge on pin PA04 (D17). No interrupts needed.
  if(settings.sensor_type == s_DAVIS6410){ // pulse counting anemometer
    uint32_t t = 0;
    setup_pulse_counter(); // need to setup GCLK6 before

    while(wakeup_source != WAKEUP_RTC){ // ignore other wakeups (external pin Interrupt, if configured)
      t += sleep(false);
      if(loraReceivedFlag && settings.lora_smart_rcv) {
        // Process immediately: readData() clears the LLCC68 IRQ flag so DIO1 goes LOW.
        // Without this, DIO1 stays HIGH and subsequent packets cannot trigger a rising edge.
        if(fanet_rx()) {
          radio_sleep();
          // If a forward was queued, reschedule RTC to wake at the forward time
          // rather than waiting for the full RTC period (which could be up to 40s).
          uint32_t fwd_delay = fanet_forward_delay_ms();
          if (fwd_delay) rtc_sleep_cfg(fwd_delay);
        }
      }
    }
    pulsecount = read_pulse_counter();
    calc_pulse_sensor(pulsecount, t);
  } 
  // UART sensor, just sleep
  else if(is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD){ // UART based sensor
    int res = -1;

    while(wakeup_source != WAKEUP_RTC){
      sleep(true); // sleep til UART interrupt

      // Process any received LoRa packet immediately so readData() clears the LLCC68
      // IRQ / DIO1 line. If DIO1 stays HIGH, no rising edge fires for subsequent packets.
      if(loraReceivedFlag && settings.lora_smart_rcv) {
        if(fanet_rx()) {
          radio_sleep();
          // If a forward was queued, reschedule RTC to wake at the forward time
          // rather than waiting for the full RTC period (up to 40s).
          uint32_t fwd_delay = fanet_forward_delay_ms();
          if (fwd_delay) rtc_sleep_cfg(fwd_delay);
        }
      }

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
        if(settings.pos_lat != 0 && settings.pos_lon != 0){
          ret = true;
        } else {
          log_e("Coordinates invalid\n");
        }
        if(settings.sensor_type == s_invalid){
          ret = false;
          log_e("No sensor configured\n");
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
    usb_connected = true;
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

static void log_v_hex_dump(const uint8_t *data, size_t len){
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


TxAttemptResult send_msg_weather(){
  if(settings.sensor_type == s_invalid){ return TX_ATTEMPT_NONE;}

  
  WindSample current_wind = get_wind_from_hist(settings.wind_age);
  if(!current_wind.valid){
    log_i("No valid wind data\r\n");
    return TX_ATTEMPT_NONE;
  }
  led_status(1);
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

  if(is_wsxx() && sensor.humidity > 0){
    wd.bHumidity = true;
    wd.Humidity = sensor.humidity;
  } else {
    wd.bHumidity = false;
    wd.Humidity = -1;
  }

  if(settings.use_baro){
    wd.bBaro = true;
    wd.Baro = sensor.baro_pressure;  
  } else {
    wd.bBaro = false;
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

  log_v_hex_dump(buffer.data(), msgSize);

  TxAttemptResult tx_result = start_lbt_transmit(buffer.data(), msgSize);
  if(tx_result != TX_ATTEMPT_STARTED) {
    led_status(0);
    return tx_result;
  }

  print_data();
  led_status(0);
  save_history(sensor.wind_speed, sensor.temperature, sensor.humidity, sensor.light_lux, sensor.batt_volt, sensor.pv_charging, sensor.pv_done); // only save history on send
  return tx_result;
}


// check if everything is ok to send the wather data now
bool allowed_to_send_weather(){
  bool ok = settings_ok;
  

  if (ok){
    if(settings.use_baro){ok &= ((time() - sensor.last_baro_reading) < 40000 );} // accept baro reading from last cacle. this reduces cp
    if(is_wsxx() || settings.sensor_type == s_WS85_UART) { ok &= (sensor.last_data || settings.testmode); } // only send if weather data is up to date or testmode is enabled // && (time()- last_ws80_data < 9000))
    #if BREEZEDUDE_ENABLE_GPS
    if(settings.use_gps)  { ok &= (tinyGps.location.isValid()); } // only send if position is valid
    #endif
    if(undervoltage) { ok = false;} // if undervoltage, do not send at all
  }

 // if(next_baro_reading){
 //   log_i("next_baro_reading: ", next_baro_reading);
 // }

 // Print reasons for not beeing ready
  if(!ok){
    if(sensor.last_data && !sensor.next_baro_reading && !undervoltage){ // ever got some sensor data and baro currently not waiting for data
      log_i("Wind Sensor data age: ", time()- sensor.last_data);
      log_i("Baro data age: ", time()- sensor.last_baro_reading);
    }
    #if BREEZEDUDE_ENABLE_GPS
    if(settings.use_gps){
      if(time()-sensor.last_gps_valid > 3000){
        log_i("No GPS fix");
      }
    }
    #endif
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

TxAttemptResult send_msg_name(const char* name, int len){
  constexpr int FANET_MAX_PACKET_SIZE = 255;
  if (len < 0) {
    return TX_ATTEMPT_NONE;
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

  log_v_hex_dump(buffer.data(), len + 4);

  return start_lbt_transmit(buffer.data(), len+4);
}

TxAttemptResult send_msg_info(){
  constexpr size_t HWINFO_BUF_SIZE = 32;
  std::array<uint8_t, HWINFO_BUF_SIZE> buffer = {0};

  hwInfoData info = {};
  info.vid      = FANET_VENDOR_ID;
  info.fanet_id = get_fanet_id();

  // Hardware Subtype + Build Date (byte0 bit 6)
  info.bSubtypeBuild = true;
  info.device_type   = FANET_BD_DEVICE_TRANSMITTER;
  info.develop_mode  = true; // todo: change to false for production, true for development devices
  // Parse __DATE__ string "Mon DD YYYY" into day/month/year
  const char *date_str = __DATE__;
  const char *months[] = {"Jan","Feb","Mar","Apr","May","Jun",
                           "Jul","Aug","Sep","Oct","Nov","Dec"};
  info.build_month = 1;
  for (int i = 0; i < 12; i++) {
    if (strncmp(date_str, months[i], 3) == 0) { info.build_month = (uint8_t)(i + 1); break; }
  }
  info.build_day  = (uint8_t)atoi(date_str + 4);
  info.build_year = (uint16_t)atoi(date_str + 7);

  // Uptime in minutes (byte0 bit 4); time() returns ms since last reset
  info.bUptime    = true;
  info.uptime_min = (uint16_t)(time() / 60000UL);

  // Rotate debug type on each call: 0x01, 0x02, 0x03, 0x01, ...
  // Add new types by incrementing DEBUG_TYPE_COUNT.
  static uint8_t s_debug_type_idx = 0;
  constexpr uint8_t DEBUG_TYPE_COUNT = 3;
  const uint8_t debug_type = (s_debug_type_idx % DEBUG_TYPE_COUNT) + 1u;
  s_debug_type_idx++;

  info.debug_type = debug_type;

  if (debug_type == 0x01u) {
    // Debug type 1: power + sensor config + firmware version
    info.debug.vbatt_mv       = (uint16_t)(sensor.batt_volt * 1000.0f);
    info.debug.batt_perc      = sensor.batt_perc;
    info.debug.pv_state       = (uint8_t)((sensor.pv_charging ? 0x01u : 0x00u) |
                                           (sensor.pv_done     ? 0x02u : 0x00u));
    info.debug.sensor_type    = (uint8_t)settings.sensor_type;
    info.debug.use_baro       = settings.use_baro ? 1u : 0u;
    info.debug.uv_triggered   = settings.uv_triggered ? 1u : 0u;

    info.debug.lbt            = settings.lora_lbt ? 1u : 0u;
    info.debug.lbt_counter    = settings.lbt_counter;
    info.debug.lora_rssi_threshold = (uint8_t)(-settings.lora_rssi_threshold);

    // Compact version encoding: major.minor.patch, each decimal digit 0..9.
    uint8_t ver_digits[3] = {0, 0, 0};
    uint8_t vd = 0;
    for (const char *p = VERSION; *p && vd < 3; ++p) {
      if (*p >= '0' && *p <= '9') {
        ver_digits[vd++] = (uint8_t)(*p - '0');
      }
    }
    info.debug.version_bcd = (uint16_t)(((ver_digits[0] % 10u) << 8) |
                                        ((ver_digits[1] % 10u) << 4) |
                                         (ver_digits[2] % 10u));

  } else if (debug_type == 0x02u) {
    // Debug type 2: configuration
    info.debug2.wind_age      = (uint16_t)(settings.wind_age/1000);
    info.debug2.gust_age      = (uint16_t)(settings.gust_age/1000);
    info.debug2.sensor_integ_s = (uint8_t)(settings.sensor_integration_time / 1000u);
    info.debug2.reduce_interval_voltage = (uint8_t)((settings.reduce_interval_voltage-2.0f) * 100.0f);

  } else if (debug_type == 0x03u) {
    // Debug type 3: RX / forwarding statistics
    info.debug3.rx_count      = fanet_rx_counter;
    info.debug3.forward_count = fanet_forward_counter;
  }

  size_t msg_size = pack_hwinfo(&info, buffer.data());
  log_v_hex_dump(buffer.data(), msg_size);
  log_i("Sending HW Info\r\n");
  return start_lbt_transmit(buffer.data(), msg_size);
}




// Setup ----------------------------------------------------------------------------------------------------------------------

extern uint32_t __etext;

void setup(){

  log_set_debug(true);
  DEBUGSER.begin(115200); // on boot start with 48Mhz clock // log_ser_begin(); 

  pinPeripheral(PIN_SERCOM1_RX, PIO_SERCOM_ALT); 
  pinPeripheral(PIN_SERCOM1_TX, PIO_SERCOM_ALT); 

  log_i("\r\n--------------- RESET -------------------\r\n");
  log_i("Version: ");  log_i(VERSION); log_i("\r\n");
  log_i("FW Build Time: ");  log_i(__DATE__); log_i(" "); log_i(__TIME__); log_i("\r\n");
  log_i("FANET ID: ");
  log_write_hex(FANET_VENDOR_ID, 2);
  log_write_hex(get_fanet_id(), 4);
  log_i("\r\n");

  
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
      baro_start_reading();
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
      SENSOR_UART.begin(115200); // div_cpu not required, as its 1 at reset
    }
    if(settings.sensor_type == s_WS85_UART){
      log_i("Setup WS85\r\n");
      
      //ws85uart.setAutoSendInterval(8500);
      ws85uart.begin(div_cpu);
      //ws85uart.requestAutoSendInterval();
      ws85uart.set_baud_115200();
    }
  } else if(settings.sensor_type == s_DAVIS6410){
    setup_rtc_time_counter();
  } else {
    // No sensor configured
    led_error(1);
  }

  #if BREEZEDUDE_ENABLE_GPS
  if(settings.use_gps){
    GPS_SERIAL.begin(settings.gps_baud);
    log_i("Starting GPS with baud: ", settings.gps_baud);
  }
  #else
  settings.use_gps = false;
  #endif

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
  create_versionfile(VERSIONFILE);
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

  // Always poll CDC commands so TEST_USB can switch from USB wait mode into the
  // normal sleep/wakeup flow without requiring a reboot.
  read_serial_cmd();
  handle_usb_link_watchdog();

  // In MSC mode (USB mounted and test mode disabled), keep the MCU in USB service
  // only and skip the normal sensor/radio workflow.
  if(!settings.test_with_usb && TinyUSBDevice.mounted()){
    usb_connected = true;
    forward_sensor_serial();
    if(settings.use_wdt){
      wdt_reset();
    }
    if(time()-last_call > 500){
      log_i("Time: ", time());
      //Serial.println(time());
      // Store led states and restore after blink
      s = led_status(1);
      last_call=time();
    }
    forward_analog_test_serial();

    return;
  }

  if(settings.use_baro){read_baro();}
  #if BREEZEDUDE_ENABLE_GPS
  if(settings.use_gps){read_gps();}
  #endif

  if(fanet_cooldown_ok() && settings.broadcast_interval_name && ( (time()- last_msg_name) > (settings.broadcast_interval_name* broadcast_scale_factor)) ){ // once a hour
    if(settings.station_name.length() > 1){
      led_status(1);
      TxAttemptResult tx_result = send_msg_name(settings.station_name.c_str(),settings.station_name.length());
      if(tx_result == TX_ATTEMPT_STARTED) {
        log_i("Send name: "); log_i(settings.station_name.c_str()); log_i("\r\n");
        last_fnet_send = time();
        last_msg_name = time();
        send_active = time();
      } else if(tx_result == TX_ATTEMPT_DEFERRED_LBT) {
        defer_next_send(last_msg_name, settings.broadcast_interval_name, "name");
      }
      led_status(0);
    }
  }

  if(fanet_cooldown_ok() && settings.broadcast_interval_weather && ( (time()- last_msg_weather) > (settings.broadcast_interval_weather * broadcast_scale_factor)) ){
    if( allowed_to_send_weather() ){
      TxAttemptResult tx_result = send_msg_weather();
      if(tx_result == TX_ATTEMPT_STARTED) {
        last_fnet_send = time();
        last_msg_weather = time();
        send_active = time();
      } else if(tx_result == TX_ATTEMPT_DEFERRED_LBT) {
        defer_next_send(last_msg_weather, settings.broadcast_interval_weather, "weather");
      } else if(tx_result == TX_ATTEMPT_NONE) {
        // no valid wind data yet, retry in 10 seconds
        last_msg_weather = time() - (settings.broadcast_interval_weather * broadcast_scale_factor) + 10000;
        sleep_allowed = time() + 1;
        log_i("Weather TX skipped, retry in 10s\r\n");
      }
    } else {
      if((is_wsxx() || settings.sensor_type == s_WS85_UART) && sensor.last_data && !sensor.next_baro_reading && (time()- sensor.last_data > 10000)){
        log_i("Wdata not ready. Wdata age: ", (time()- sensor.last_data) );
        log_i("Last Baro reading age: ", (time()- sensor.last_baro_reading) );
        sleep_allowed = time() + 1;
        //sensor.last_data = 0;
      }
      
    }
  }
  if(settings.broadcast_interval_info && fanet_cooldown_ok() && ( (time()- last_msg_info) > (settings.broadcast_interval_info * broadcast_scale_factor)) ){
      led_status(1);
      TxAttemptResult tx_result = send_msg_info();
      if(tx_result == TX_ATTEMPT_STARTED) {
        last_fnet_send = time();
        last_msg_info = time();
        send_active = time();
      } else if(tx_result == TX_ATTEMPT_DEFERRED_LBT) {
        defer_next_send(last_msg_info, settings.broadcast_interval_info, "info");
      }
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
    if(fanet_rx() && !send_active) {
      // readData() in fanet_rx() left radio in standby; re-arm RX now that we
      // know no TX is in progress (avoids clearing the TX-done DIO1 handler).
      radio_sleep();
    }
  }

  if(!send_active && settings.lora_smart_rcv) {
    if(fanet_forward_check()) {
      send_active = time();
    }
  }

// Check if everything is done --> sleep
  if(!send_active && sleep_allowed && (time() > sleep_allowed) && (!usb_connected || keep_usb_active_during_sleep()) && (time() > 2500)){ // allow sleep after 2500 ms to get a chance to detect USB / CDC commands
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

  if(usb_connected){
    // keep usb alive for 15 min. Its not easyly possible to detect if still connected, so just restart after 15min
    if(!settings.test_with_usb && (time() > 15UL*60UL*1000UL)){
      log_i("Restart\r\n");
      log_flush();
      usb_ignore_detach_event = true;
      usb_connected = false; // clear before reset so disconnect detector above is not triggered
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