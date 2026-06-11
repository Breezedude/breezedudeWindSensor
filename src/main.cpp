#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <array>
#include "wiring_private.h" // pinPeripheral() function


#include "defines.h"
#include "hist.h"
#include "sensors.h"
#include "logging.h"
#include "sleep.h"
#include "fanet.h"
#include "display.h"
#include "tools.h"
#include "radio.h"
#include "ota_ab.h"
#include "ota_lora.h"

#ifndef BREEZEDUDE_DEVELOP_MODE
#define BREEZEDUDE_DEVELOP_MODE 0
#endif


// Todo list:
// * Read bootloader version
// * detect sensor frozen?

// https://github.com/adafruit/Adafruit_TinyUSB_Arduin
// https://github.com/adafruit/ArduinoCore-samd
// https://github.com/Microsoft/uf2
// https://github.com/adafruit/uf2-samdx1

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
uint32_t boot_usb_hold_until_ms = 0;

uint32_t sleep_allowed = 0; // time() when is is ok so enter deepsleep
bool reduced_interval = false; // reduced interval active
bool undervoltage = false;
bool deep_undervoltage = false; // set after 3 consecutive voltage readings < 2.9V; triggers extended sleep for solar charging

// ### Variables for storing settings from file, may be overewritten #####

bool use_mcp4652 = true; // used on first version of PCB (<=1.3) to set MPPT and DCDC voltage

bool settings_ok = false;
static uint32_t no_settings_first_seen_ms = 0; // set on first !settings_ok wakeup


// Message timing

uint32_t last_msg_weather = 0;
uint32_t last_msg_name = 0;
uint32_t last_msg_info = 0;

uint32_t broadcast_scale_factor = 1; // multiplier for broadcast values. is set to 2..5 if battery is low


uint32_t last_fnet_send = 0; // last package send
uint32_t fanet_cooldown = 4000;
uint32_t loopcounter = 0;
bool force_wait_for_wdata = false;

volatile int wakeup_source = WAKEUP_NONE;

// Sleep ----------------------------------------------------------------------------------------------------------------------

// dummy function
void wakeup_EIC(){
  wakeup_source = WAKEUP_EIC;
}

void mark_uart_wakeup(){
  wakeup_source = WAKEUP_UART;
}

static bool usb_host_connected(){
  // Use USB enumeration as primary signal. DTR remains a fallback for hosts
  // that do not expose mount state reliably.
  return TinyUSBDevice.mounted() || usb_connected || Serial.dtr();
}

static bool keep_usb_active_during_sleep(){
  return usb_host_connected();
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
  if(usb_host_connected()){
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

  // In !settings_ok mode: after the first 5 minutes, flash the red LED only while awake.
  if (!settings_ok && no_settings_first_seen_ms != 0 &&
      (millis() - no_settings_first_seen_ms) >= 300000UL) {
    led_error(1);
  }



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
      log_i("Low voltage\r\n");
    } else {
      // battery voltage is normal
      reduced_interval = false;
      broadcast_scale_factor = 1;
    }
  }

// Undervoltage Voltage behavior (WS85 connected):
// * 2439mV: sleeps @ 5.90µA
// * 2560mV: deepsleeps @9.20µA
// * 2650mV: deepsleep 11.70µA
// * 2754mV. when in depsleep stays there @ 15µA, on reset goes to brownout drawwing 1,47mA
// ** when us-uart power mosfeet weakens, current draw rises until ws85 starts working, drawing 7.55mA oveer UART

// * <2800mV: Brownout, draws 4mA, boots to bootloader on wake, never comes back
// * > 2800mV. resets normally, goes to deepsleep 17µA for 30min
// * 2800-3100mV: Undervoltage, draws 20µA in sleep, stays in sleep for 1800000ms (=30min) before waking up to check voltage again




  tts_weather = timeToSend(last_msg_weather, settings.broadcast_interval_weather);
  tts_name    = timeToSend(last_msg_name, settings.broadcast_interval_name);
  tts_info    = ota_lora_next_hwinfo_due_ms();


  
  tts = min(min(tts_name, tts_info), tts_weather);
  if( fanet_cooldown && last_fnet_send  && (time() - last_fnet_send + tts < fanet_cooldown)){
    tts += fanet_cooldown - (time()-last_fnet_send);
  }

    if(millis()-last_print > 2500){
    last_print = millis();
    log_v("tts_weather: ", tts_weather);
    log_v("tts_name: ", tts_name);
    log_v("tts_info: ", tts_info);
    log_v("tts: ", tts);
  }

  if(tts == (uint32_t)-1){ tts=0;}
  

  if( !tts && loopcounter > 25){
    log_e("just looping, will sleep\r\n");
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
  // One-shot wait window when weather TX is due but wind data is stale.
  if (force_wait_for_wdata) {
    time_to_sleep = 12000UL;
    force_wait_for_wdata = false;
  }
  if(time_to_sleep == 0){ return;} // if time_to_sleep = 0, do not sleep at all
  rtc_sleep_cfg(time_to_sleep);

  if(deep_undervoltage){
    radio_sleep();
    log_i("Deep UV (<2.9V). Sleeping until solar charges or voltage recovers\r\n");
    log_flush();
    // PV_CHARGE (PB23) is active-LOW: falls LOW when the solar charger draws ~3mA.
    // INPUT_PULLUP ensures pin is HIGH (defined state); EIC only fires on genuine FALLING edge.
    // Without this, the floating pin (left hi-Z by get_solar_charger_state) triggers EIC immediately.
    pinMode(PIN_PV_CHARGE, INPUT_PULLUP);
    attachInterruptWakeup(PIN_PV_CHARGE, wakeup_EIC, FALLING, false);
    do {
      rtc_sleep_cfg(524000UL); // max RTC period ~8.7 min per cycle
      sleep(false);
      read_batt_perc(); // updates deep_undervoltage and undervoltage via voltageToSOCNonLinear
      log_i("Deep UV wake: V=", sensor.batt_volt);
      log_flush();
    } while (deep_undervoltage && wakeup_source != WAKEUP_EIC);
    detachInterrupt(digitalPinToInterrupt(PIN_PV_CHARGE));
    pinDisable(PIN_PV_CHARGE);
    wakeup();
    return;
  }

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
    // No valid settings: sleep 12s per cycle so OTA config updates can still arrive.
    // Keep the red LED on for the first 5 minutes, then only while awake.
    if (no_settings_first_seen_ms == 0) {
      no_settings_first_seen_ms = millis();
    }
    bool in_first_5min = (millis() - no_settings_first_seen_ms) < 300000UL;
    if (in_first_5min) {
      led_error(1);  // stay on continuously during first 5 minutes
    } else {
      led_error(0);  // turn off before sleep; wakeup() will re-enable it briefly
    }
    log_e("No settings file\r\n");
    time_to_sleep = 12000UL;
  }

  //if(!time_to_sleep){ return;} // if time_to_sleep = 0, do not sleep at all

  if(time_to_sleep > 200000UL) {
    log_i("will sleep for s: ", time_to_sleep / 1000);
  } else {
    log_i("will sleep for ms: ", time_to_sleep);
  }
  log_flush();

  // Ensure radio is in the correct state for the sleep window (RX if conditions
  // are met, PHY sleep otherwise). This also covers the period before the first
  // TX, when radio_sleep() has not yet been called from the TX-done path.
  radio_sleep();

// Using TC3 for hardware pulsecounting on Falling edge on pin PA04 (D17). No interrupts needed.
  if(settings.sensor_type == s_DAVIS6410){ // pulse counting anemometer
    uint32_t t = 0;
    setup_pulse_counter(); // need to setup GCLK6 before

    while(wakeup_source != WAKEUP_RTC && wakeup_source != WAKEUP_USB && !Serial.available()){ // ignore other wakeups (external pin Interrupt, if configured)
      t += sleep(false);
      if(loraReceivedFlag && (settings.lora_smart_rcv || settings.repeater)) {
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

    while(wakeup_source != WAKEUP_RTC && wakeup_source != WAKEUP_USB && !Serial.available()){
      sleep(true); // sleep til UART interrupt

      // Process any received LoRa packet immediately so readData() clears the LLCC68
      // IRQ / DIO1 line. If DIO1 stays HIGH, no rising edge fires for subsequent packets.
      if(loraReceivedFlag && (settings.lora_smart_rcv || settings.repeater)) {
        if(fanet_rx()) {
          radio_sleep();
          // If a forward was queued, reschedule RTC to wake at the forward time
          // rather than waiting for the full RTC period (up to 40s).
          uint32_t fwd_delay = fanet_forward_delay_ms();
          if (fwd_delay) rtc_sleep_cfg(fwd_delay);
        }
      }

      if(is_wsxx()){
        //led_error(1);
        res = read_wsxx();
        //led_error(0);
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
          wakeup_source = WAKEUP_NONE;
        }
      }

    }
  }
  // Repeater without sensor: sleep in RX, forward incoming packets, wake on RTC for name/info TX
  else if(settings.repeater) {
    while(wakeup_source != WAKEUP_RTC && wakeup_source != WAKEUP_USB && !Serial.available()) {
      sleep(false);
      if(loraReceivedFlag) {
        if(fanet_rx()) {
          radio_sleep(); // re-arm RX
          uint32_t fwd_delay = fanet_forward_delay_ms();
          if(fwd_delay) rtc_sleep_cfg(fwd_delay);
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

// Send ----------------------------------------------------------------------------------------------------------------------


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

  std::array<uint8_t, sizeof(fanet_packet_t4)> buffer = {0};
  size_t msgSize = pack_weatherdata(&wd, buffer.data());
  if(settings.forward_data || settings.repeater) buffer[0] |= (1u << 6); // set FANET forward bit

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
  header.forward = (settings.forward_data || settings.repeater);
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
  info.develop_mode  = (BREEZEDUDE_DEVELOP_MODE != 0);
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

  ota_lora_prepare_hwinfo(info);

  size_t msg_size = pack_hwinfo(&info, buffer.data());
  if(settings.forward_data || settings.repeater) buffer[0] |= (1u << 6); // set FANET forward bit
  log_v_hex_dump(buffer.data(), msg_size);
  TxAttemptResult tx = start_lbt_transmit(buffer.data(), msg_size);
  if(tx == TX_ATTEMPT_STARTED) {
    ota_lora_note_hwinfo_tx_started();
  }
  return tx;
}




// Setup ----------------------------------------------------------------------------------------------------------------------

extern uint32_t __etext;

void setup(){

  // Change BOD33 action from RESET to INTERRUPT as early as possible.
  // This prevents a brownout (< ~2.8 V) from resetting the MCU into the
  // DFU bootloader where it would hang drawing 1.5-4 mA.  The ISR instead
  // disables the UART TX leakage path and cuts sensor power before the
  // normal UV-sleep loop takes over.
  setup_BOD33_interrupt();

  log_set_debug(true);
  log_set_error(true);
  DEBUGSER.begin(115200); // on boot start with 48Mhz clock // log_ser_begin(); 

  pinPeripheral(PIN_SERCOM1_RX, PIO_SERCOM_ALT); 
  pinPeripheral(PIN_SERCOM1_TX, PIO_SERCOM_ALT); 

  log_i("\r\n--------------- RESET -------------------\r\n");
  log_reset_cause();
  log_i("Version: ");  log_i(VERSION); log_i("\r\n");
  log_i("Bootloader: "); log_i(get_bootloader_version().c_str()); log_i("\r\n");
  log_i("FW Build Time: ");  log_i(__DATE__); log_i(" "); log_i(__TIME__); log_i("\r\n");
  log_i("FANET ID: ");
  log_write_hex(FANET_VENDOR_ID, 2);
  log_write_hex(get_fanet_id(), 4);
  log_i("\r\n");
  log_ota_boot_diagnostics();
  log_ota_reboot_trace();

  
  //printf("code end: %p\n", (void *)(&__etext));
  //printf("flash_start: %p\n", my_internal_storage.get_flash_address());
  //printf("flash_size: %lu\n", my_internal_storage.get_flash_size());

  Wire.begin();
  i2c_scan();

  setup_display();
  if(display_present()){
    log_i("I2C Display enabled\n");
  }
  
  if(setup_flash()){
    settings_ok = parse_file(SETTINGSFILE);
    // Repeater without sensor: disable weather TX (no sensor data to send)
    if(settings.repeater && settings.sensor_type == s_invalid) {
      settings.broadcast_interval_weather = 0;
    }
    if(!debug_enabled()){
      DEBUGSER.flush();
      DEBUGSER.end();
      pinDisable(PIN_SERCOM1_RX);
      pinDisable(PIN_SERCOM1_TX);
    }
  }

  // Init radio only after validating mandatory settings.
  if(settings_ok){
    if(radio_init()){
      radio_phy->setPacketSentAction(set_fanet_send_flag);
      ota_lora_begin();
      radio_sleep(); // radio_phy->sleep();
    } else {
      led_error(1);
      display_delay(2000);
    }
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
    wdt_disable();
    //log_i(settings.use_wdt ? "WDT runtime: deferred until setup complete\r\n" : "WDT runtime: OFF\r\n");

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
    log_i("Settings invalid\r\n");
    setup_PM(true, true);
    wdt_enable(WDT_PERIOD,false); // setup clocks
    wdt_disable();
    //setup_rtc_time_counter();
  }

  if(settings_ok){
    if(is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD){
      switch_sensor_power(1); // Turn on WS80 Power supply with P-MOSFET
      setup_rtc_time_counter();

      if(is_wsxx() || settings.sensor_type == s_WINDNERD){
        SENSOR_UART.begin(115200); // div_cpu not required, as its 1 at reset
      }
      if(settings.sensor_type == s_WS85_UART){
        //log_i("Setup WS85\r\n");

        ws85uart.begin(div_cpu);
        // Keep the current sensor baud on boot; a forced switch here can stall
        // startup on boards that already stream correctly or are still powering up.
        ws85uart.set_baud_115200();
        //ws85uart.setAutoSendInterval(8500);
      }
    } else if(settings.sensor_type == s_DAVIS6410){
      setup_rtc_time_counter();
    } else {
      // No sensor configured
      if(settings.repeater) {
        setup_rtc_time_counter(); // repeater without sensor still needs RTC for sleep
      } else {
        led_error(1);
      }
    }

    #if BREEZEDUDE_ENABLE_GPS
    if(settings.use_gps){
      GPS_SERIAL.begin(settings.gps_baud);
      log_i("Starting GPS with baud: ", settings.gps_baud);
    }
    #else
    settings.use_gps = false;
    #endif
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
  if(settings_ok){
    create_versionfile(VERSIONFILE);
  }

  // When USB is connected on boot, keep the MCU awake for a short
  // window so host-side serial/USB commands are handled before first sleep.
  if(usb_host_connected()) {
    boot_usb_hold_until_ms = millis() + 10000UL;
    log_v("USB connected on boot, keep awake for 10s\r\n");
  }

  //log_i("Startup init complete\r\n");
  log_flush();
  wakeup();
  if(settings_ok && settings.use_wdt){
    wdt_enable(WDT_PERIOD,false);
    log_i("WDT runtime: ON\r\n");
    log_flush();
  }
}

// loop ----------------------------------------------------------------------------------------------------------------------

void loop(){

static uint32_t send_active=0; // if > 0, time() last message was send to tx queue, reset to 0 if send is complete
static uint32_t last_settings_check = 0; // timee() ckecked if a settings file is present if settings not read yet sucessfully

// print millis as alive counter
static uint32_t last_call = 0;
static bool s = false;

loopcounter++;

  if(settings.use_wdt){
    wdt_reset();
  }

  // If enumeration completes shortly after startup, still arm the boot hold.
  if(!boot_usb_hold_until_ms && usb_host_connected()) {
    boot_usb_hold_until_ms = millis() + 10000UL;
    log_i("USB connected after boot, keep awake for 10s\r\n");
  }

  bool boot_usb_hold_active = (boot_usb_hold_until_ms != 0) && ((int32_t)(millis() - boot_usb_hold_until_ms) < 0);

  if(last_call && (time()-last_call > 15)){
    if(!s){led_status(0);}
  }

  // Always poll CDC commands so TEST_USB can switch from USB wait mode into the
  // normal sleep/wakeup flow without requiring a reboot.
  read_serial_cmd();
  handle_usb_link_watchdog();

  // Hard-stop normal operation until a valid user configuration is present.
  if(!settings_ok){
    if(!sleep_allowed){
      sleep_allowed = time() + 180000UL; // sleep after 3 minutes
    }
    if(time() - last_settings_check > 15000){
      last_settings_check = time();
      log_e("\r\nFailed to obtain valid settings. Waiting for user config\r\n");
      settings_ok = parse_file(SETTINGSFILE);
      if(settings_ok){
        NVIC_SystemReset();
      }
    }
    if((time() > 5UL*60UL*1000UL)){
      led_error(0);
    }
    if(settings.use_wdt){
      wdt_reset();
    }
    return;
  }

  if(settings.use_baro){read_baro();}
  #if BREEZEDUDE_ENABLE_GPS
  if(settings.use_gps){read_gps();}
  #endif

  if(!ota_lora_busy() && fanet_cooldown_ok() && settings.broadcast_interval_name && ( (time()- last_msg_name) > (settings.broadcast_interval_name* broadcast_scale_factor)) ){ // once a hour
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

  if(!ota_lora_busy() && fanet_cooldown_ok() && settings.broadcast_interval_weather && ( (time()- last_msg_weather) > (settings.broadcast_interval_weather * broadcast_scale_factor)) ){
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
        const uint32_t wdata_age = time() - sensor.last_data;
        log_i("Wdata not ready. Wdata age: ", (time()- sensor.last_data) );
        log_i("Last Baro reading age: ", (time()- sensor.last_baro_reading) );
        if(wdata_age > 3600000UL){
          log_e("Wind data stale > 1h, restarting to recover\r\n");
          log_flush();
          NVIC_SystemReset();
        }
        force_wait_for_wdata = true;
        sleep_allowed = time();
        //sensor.last_data = 0;
      }
      
    }
  }
  const uint32_t info_due_ms = ota_lora_next_hwinfo_due_ms();
  if(!ota_lora_busy() && fanet_cooldown_ok() && info_due_ms == 0u ){
      led_status(1);
      TxAttemptResult tx_result = send_msg_info();
      if(tx_result == TX_ATTEMPT_STARTED) {
        last_fnet_send = time();
        last_msg_info = time();
        send_active = time();
      } else if(tx_result == TX_ATTEMPT_DEFERRED_LBT) {
        uint32_t retry_delay = next_lbt_retry_delay();
        ota_lora_defer_hwinfo_retry(retry_delay);
        next_tx_time = time() + retry_delay;
        sleep_allowed = time() + 1;
        log_i("LBT: defer info TX by ms: ", retry_delay);
        log_i("\r\n");
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
      send_active = 0;
      sleep_allowed = time() + (1);
      radio_phy->finishTransmit();
      if(!ota_lora_on_tx_complete()) {
        radio_sleep(); // radio_phy->sleep();
      }
    }
  }

  if(!send_active && ota_lora_poll()) {
    sleep_allowed = time() + 1;
  } else if(settings.lora_smart_rcv || settings.repeater){
    if(fanet_rx() && !send_active) {
      // readData() in fanet_rx() left radio in standby; re-arm RX now that we
      // know no TX is in progress (avoids clearing the TX-done DIO1 handler).
      radio_sleep();
    }
  }

  if(!send_active && !ota_lora_busy() && (settings.lora_smart_rcv || settings.repeater)) {
    if(fanet_forward_check()) {
      send_active = time();
    }
  }

  if(settings.analog_test_mode){
    forward_analog_test_serial();
  }

// Check if everything is done --> sleep
  if(!settings.analog_test_mode && !send_active && !ota_lora_busy() && !boot_usb_hold_active && sleep_allowed && (time() > sleep_allowed) && (time() > 2500)){ // allow sleep after 2500 ms to get a chance to detect USB / CDC commands
    go_sleep();
  }

  /*
  
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
*/

  if((time() > 5UL*60UL*1000UL)){ // trun off error LED after 5minutes to save energy if an error occures with no one around
    led_error(0);
  }

  if(settings.use_wdt){
    wdt_reset();
  }
}