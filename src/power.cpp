#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#include "power.h"
#include "defines.h"
#include "hist.h"
#include "sensors.h"
#include "logging.h"
#include "sleep.h"
#include "tools.h"
#include "radio.h"
#include "ota_lora.h"
#include "scheduler.h"

// Pulsecounter
volatile uint32_t pulsecount = 0; // pulses from wind speed sensor using reed switch

int div_cpu = 1; // current div
bool first_sleep = true; // first sleep after reset, USB perephial could still be on

uint32_t sleep_allowed = 0; // time() when is is ok so enter deepsleep
uint32_t loopcounter = 0;
bool reduced_interval = false; // reduced interval active
bool undervoltage = false;
bool deep_undervoltage = false; // set after 3 consecutive voltage readings < 2.9V; triggers extended sleep for solar charging

volatile int wakeup_source = WAKEUP_NONE;

static uint32_t no_settings_first_seen_ms = 0; // set on first !settings_ok wakeup

// Sleep ----------------------------------------------------------------------------------------------------------------------

// dummy function
void wakeup_EIC(){
  wakeup_source = WAKEUP_EIC;
}

void mark_uart_wakeup(){
  wakeup_source = WAKEUP_UART;
}

bool usb_host_connected(){
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
    if(sensor_uses_uart()){
      enable_sercom0_int();
      //log_i("Enable UART Interrupt\r\n");
    }
  }
  // Drain both outputs before sleeping: the SERCOM clocks stop in standby, so
  // anything still sitting in the UART TX buffer would be lost or corrupted.
  log_flush();
  reset_time_counter(); // start counting sleeptime from zero
  bool keep_usb = keep_usb_active_during_sleep();

  // With USB test mode active we use light sleep and ignore wakeups that do not
  // set a wakeup source, otherwise USB IRQ traffic would break the normal flow.
  do {
    deepsleep(keep_usb);
  } while (keep_usb && wakeup_source == WAKEUP_NONE);
  uint32_t p = micros();



  if(sensor_uses_raw_uart()){
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

// Wake reasons that end a sleep window. Any other wakeup (a LoRa DIO edge, a
// configured external pin interrupt) is serviced and then slept through again.
static bool sleep_window_over(){
  return wakeup_source == WAKEUP_RTC || wakeup_source == WAKEUP_USB || Serial.available();
}

// Service a LoRa packet that arrived while sleeping.
// readData() inside fanet_rx() clears the LLCC68 IRQ so DIO1 goes LOW again;
// without that no rising edge fires for subsequent packets. If the packet
// queued a forward, wake at the forward time instead of sitting out the full
// RTC period, which can be up to 40 s.
static void service_lora_during_sleep(){
  if(!loraReceivedFlag) return;
  if(!(settings.lora_smart_rcv || settings.repeater)) return;
  if(!fanet_rx()) return;
  radio_sleep();
  const uint32_t fwd_delay = fanet_forward_delay_ms();
  if(fwd_delay) rtc_sleep_cfg(fwd_delay);
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

    while(!sleep_window_over()){ // ignore other wakeups (external pin Interrupt, if configured)
      t += sleep(false);
      service_lora_during_sleep();
    }
    pulsecount = read_pulse_counter();
    calc_pulse_sensor(pulsecount, t);
  } 
  // UART sensor, just sleep
  else if(sensor_uses_uart()){ // UART based sensor
    int res = -1;

    while(!sleep_window_over()){
      sleep(true); // sleep til UART interrupt
      service_lora_during_sleep();

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
    while(!sleep_window_over()) {
      sleep(false);
      service_lora_during_sleep(); // settings.repeater is set, so the gate passes
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
