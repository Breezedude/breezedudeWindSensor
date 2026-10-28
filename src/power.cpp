#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "wiring_private.h" // pinPeripheral(), to put muxed pins back after a sleep
#include "wake_profile.h"

#include "power.h"
#include "defines.h"
#include "hist.h"
#include "sensors.h"
#include "logging.h"
#include "sleep.h"
#include "tools.h"
#include "radio.h"
#include "fanet_sched.h"
#include "ota_lora.h"
#include "scheduler.h"

// Pulsecounter
volatile uint32_t pulsecount = 0; // pulses from wind speed sensor using reed switch

int div_cpu = 1; // current div
bool first_sleep = true; // first sleep after reset, USB perephial could still be on

uint32_t sleep_allowed = 0; // time() when is is ok so enter deepsleep
uint32_t loopcounter = 0;
static uint32_t wake_started_ms = 0;   // millis() at wakeup(), for the awake time in the compact log

// Awake and elapsed time since the last power_duty_reset(), the awake duty
// the HWInfo dynamic report carries. Booked in sleep(), where both are
// measured anyway; time spent before the first sleep does not count.
static uint32_t duty_awake_ms = 0;
static uint32_t duty_total_ms = 0;
void power_duty_get(uint32_t *awake_ms, uint32_t *total_ms){
  *awake_ms = duty_awake_ms;
  *total_ms = duty_total_ms;
}
void power_duty_reset(){
  duty_awake_ms = 0;
  duty_total_ms = 0;
}
static int last_wake_source = 0;
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

void mark_console_wakeup(){
  wakeup_source = WAKEUP_CONSOLE;
}

void mark_uart_wakeup(){
  wakeup_source = WAKEUP_UART;
}

bool usb_host_connected(){
  // A host that is there sends a start-of-frame every millisecond; when the
  // frames stop, TinyUSB reports the device suspended. That is the signal,
  // because this board has no VBUS sensing: pulled without a reset, the USB
  // stack never sees a disconnect, mounted() stays true and DTR keeps the last
  // state the host set. Going by those, a station that had USB at boot kept
  // the USB peripheral and the 48 MHz clock running for good once the cable
  // was gone - measured 8 mA instead of the normal sleep. A host that suspends
  // the bus itself (a sleeping PC) now lets the station go to low power as
  // well, which is what it should do then.
  return TinyUSBDevice.mounted() && !TinyUSBDevice.suspended();
}

static bool keep_usb_active_during_sleep(){
  return usb_host_connected();
}

// Pins released for the duration of the standby, and put back on the way out.
//
// The point is to measure what an idle pin costs without changing what the
// station does. Releasing a pin permanently does not do that: it breaks the
// peripheral behind it, the station stops reaching standby, and the reading is
// then the CPU being awake rather than any pin leakage. Measured that way, the
// SPI group appeared to cost 2.1 mA - it cost nothing, the radio had simply
// stopped answering.
//
// Between quiet_pins_for_standby() and restore_pins_after_standby() nothing can
// use these pins: the radio is in PHY sleep and the barometer is not addressed.
// The mask is runtime only, set with the SLEEPQUIET console command.
//
//   1  the SPI bus - MISO, MOSI, SCK
//   2  MISO alone, which floats whenever the radio is deselected
//   4  the LoRa status inputs DIO0/DIO1/DIO2
//   8  I2C - SDA and SCL
//
// CS and RESET are never released: floating, they would let the module drift
// into a selected or reset state.
static uint32_t sleep_quiet_mask = 0;

void set_sleep_quiet_mask(uint32_t mask){ sleep_quiet_mask = mask; }
uint32_t get_sleep_quiet_mask(){ return sleep_quiet_mask; }

/** Back to whatever the variant says the pin is for - the same call SPI.begin()
 *  and Wire.begin() use, so no mux is hard-coded here. */
static inline void pin_restore(uint32_t pin){
  pinPeripheral(pin, (EPioType)g_APinDescription[pin].ulPinType);
}

static void quiet_pins_for_standby(){
  if(!sleep_quiet_mask) return;
  if(sleep_quiet_mask & 0x01){
    pinDisable(PIN_LORA_MISO); pinDisable(PIN_LORA_MOSI); pinDisable(PIN_LORA_SCK);
  }
  if(sleep_quiet_mask & 0x02){ pinDisable(PIN_LORA_MISO); }
  if(sleep_quiet_mask & 0x04){
    #if BREEZEDUDE_RADIO_SX1276
    pinDisable(PIN_LORA_DIO0);   // only that radio has a DIO0; on hardware 2.5 the pin is the red LED
    #endif
    pinDisable(PIN_LORA_DIO1); pinDisable(PIN_LORA_DIO2);
  }
  if(sleep_quiet_mask & 0x08){ pinDisable(PIN_SDA); pinDisable(PIN_SCL); }
}

static void restore_pins_after_standby(){
  if(!sleep_quiet_mask) return;
  if(sleep_quiet_mask & 0x01){
    pin_restore(PIN_LORA_MISO); pin_restore(PIN_LORA_MOSI); pin_restore(PIN_LORA_SCK);
  }
  if(sleep_quiet_mask & 0x02){ pin_restore(PIN_LORA_MISO); }
  if(sleep_quiet_mask & 0x04){
    #if BREEZEDUDE_RADIO_SX1276
    pin_restore(PIN_LORA_DIO0);
    #endif
    pin_restore(PIN_LORA_DIO1); pin_restore(PIN_LORA_DIO2);
  }
  if(sleep_quiet_mask & 0x08){ pin_restore(PIN_SDA); pin_restore(PIN_SCL); }
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

  // Only worth arming for a sensor that actually has supply. With the sensor
  // switched off its UART pins are released, the receiver reads a permanent low
  // and would wake the station continuously.
  const bool sensor_uart_armed = enable_uart_interrupt &&
                                 sensor_uses_uart() &&
                                 sensor_power_is_on();
  if(sensor_uart_armed){
    enable_sercom0_int();
  }
  wake_profile_report(div_cpu);
  wake_mark(WP_FLUSH0);
  // Drain both outputs before sleeping: the SERCOM clocks stop in standby, so
  // anything still sitting in the UART TX buffer would be lost or corrupted.
  log_flush();

  // Nothing to do here for the debug console any more. It is set up once, by
  // configure_debug_uart_standby() after each DEBUGSER.begin(), and stays that
  // way through the sleep - which is precisely what stops it losing bytes.
  wake_mark(WP_FLUSH1);
  bool keep_usb = keep_usb_active_during_sleep();

  // With USB test mode active we use light sleep and ignore wakeups that do not
  // set a wakeup source, otherwise USB IRQ traffic would break the normal flow.
  quiet_pins_for_standby();
  wake_mark(WP_DEEP);
  do {
    deepsleep(keep_usb);
  } while (keep_usb && wakeup_source == WAKEUP_NONE);
  restore_pins_after_standby();
  wake_mark(WP_WAKE);

  uint32_t p = micros();



  // Symmetric to the arming above: without supply the sensor UART stays down,
  // and re-beginning it would put the receiver back on a floating pin.
  if(sensor_uart_armed && sensor_uses_raw_uart()){
    disable_sercom0_int();
    p = micros()-p;
    //DEBUGSER.println(p);
    SENSOR_UART.begin(115200*div_cpu);
  }

  if(sensor_uart_armed && settings.sensor_type == s_WS85_UART) {
#ifdef WS85_TRACE
    // WS85_TRACE: the state the wakeup finds the sensor UART in, before
    // disable_sercom0_int() changes it. tr_wake_us is time zero of the report,
    // see ws85_trace_report() in sensors.cpp.
    ws85uart.tr_wake_src = (uint8_t)wakeup_source;
    ws85uart.tr_flag = SERCOM0->USART.INTFLAG.reg;
    ws85uart.tr_status = SERCOM0->USART.STATUS.reg;
    ws85uart.tr_wake_us = micros();
#endif
    disable_sercom0_int();
#ifdef WS85_TRACE
    ws85uart.tr_disable_us = micros();
#endif
    ws85uart.resync((uint8_t)div_cpu); // cheaper than begin(), see WS85WindSensor::resync()
#ifdef WS85_TRACE
    // WS85_TRACE: the receiver as the parser will find it.
    ws85uart.tr_restart_us = micros();
    ws85uart.tr_ctrla = SERCOM0->USART.CTRLA.reg;
    ws85uart.tr_ctrlb = SERCOM0->USART.CTRLB.reg;
    ws85uart.tr_baud = SERCOM0->USART.BAUD.reg;
    ws85uart.tr_ie = SERCOM0->USART.INTENSET.reg;
    // CLKCTRL reads back the channel its ID field names, so an 8 bit write of
    // just the ID selects the SERCOM0 core channel first. Routing is untouched.
    *((uint8_t *)&GCLK->CLKCTRL.reg) = SERCOM0_GCLK_ID_CORE;
    ws85uart.tr_gen = GCLK->CLKCTRL.bit.GEN;
    ws85uart.tr_clken = GCLK->CLKCTRL.bit.CLKEN;
#endif
  }

  // Ticks since the read at the previous wakeup: this sleep plus the wake phase
  // in front of it. The counter is not reset any more, which saves a
  // synchronised write before every sleep.
  static uint32_t last_read_ms = 0;
  uint32_t t = elapsed_time_counter();
#ifdef WS85_TRACE
  ws85uart.tr_tc4_us = micros();
#endif
  // millis() stands still in standby, so whatever it advanced since that read is
  // awake time the counter holds as well. It is already in millis().
  const uint32_t awake_ms = millis() - last_read_ms;
  last_read_ms = millis();
  sleeptime_cum += (t > awake_ms) ? t - awake_ms : 0;
  duty_awake_ms += awake_ms;
  duty_total_ms += t;

  // Disable Debug uart again
  //if(debug_enabled()){
  //  DEBUGSER.begin(115200*div_cpu);
  //}
  // re-enable wdt after sleep
  if(settings.use_wdt) {
    wdt_enable(WDT_PERIOD, false);
#ifdef WS85_TRACE
    ws85uart.tr_wdt_us = micros();
#endif
  }

  wake_mark(WP_SLEEP_RET);
  return t;
}

// called after sleep
void wakeup(){
  // USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE; // Re-enable USB, no need, not working?
  loopcounter = 0;
  wake_started_ms = millis();
  last_wake_source = wakeup_source;
  log_v("\r\n########\r\n");
  log_v("Wakeup: ", time());
  log_v("Wakeup source: "); log_v(wakeup_source_string[wakeup_source]);log_v("\r\n");
  if(settings.uart_wake){
    // What was this wakeup, really? See the note above SERCOM1_Handler: rxs
    // alone only reports an edge on RxD, rxc says a valid byte followed, and
    // err says the frame behind the edge was not a frame. Together they tell a
    // genuine command apart from noise on a high-impedance line.
    log_v("SERCOM1 irq: ", (uint32_t)sercom1_irq_count);
    log_v("SERCOM1 rxs: ", (uint32_t)sercom1_rxs_count);
    log_v("SERCOM1 rxc: ", (uint32_t)sercom1_rxc_count);
    log_v("SERCOM1 err: ", (uint32_t)sercom1_err_count);
    log_v("SERCOM1 status: ", (uint32_t)sercom1_last_status);
    // The one number that separates "woke up" from "received something": how
    // much of the command actually made it into the ring buffer.
    log_v("Console bytes: ", (uint32_t)DEBUGSER.available());
  }
  wakeup_source = WAKEUP_NONE;

  // In !settings_ok mode: after the first 5 minutes, flash the red LED only while awake.
  if (!settings_ok && no_settings_first_seen_ms != 0 &&
      (millis() - no_settings_first_seen_ms) >= 300000UL) {
    led_error(1);
  }



  get_solar_charger_state();
  if(settings.use_imu){
    get_imu();
  }

  // Barometer and battery are no longer read here. Neither value is used unless
  // something is transmitted, so tx_run_jobs() refreshes them right before a job
  // that carries them - once per send interval instead of once per wakeup.
  //
  // No minimum awake time either. The 100 ms that stood here dated from when the
  // barometer conversion had to be waited for after every wakeup; now the loop
  // sleeps as soon as nothing is left to do. A transmission still keeps it awake
  // until the radio reports TX done, see tx_send_in_progress().
  //
  // A console wakeup is the exception: the start bit that woke the station
  // belongs to a byte that is still on the wire. console_line_pending() holds
  // the loop until it and the rest of the line are in.
  sleep_allowed = time();
  if(last_wake_source == WAKEUP_CONSOLE){ console_note_wakeup(); }
}

// Wake reasons that end a sleep window. Any other wakeup (a LoRa DIO edge, a
// configured external pin interrupt) is serviced and then slept through again.
static bool sleep_window_over(){
  // With UART_WAKE=0, SERCOM1 is unclocked in standby and DEBUGSER cannot wake
  // the MCU at all - but once the station is up for any other reason, a pending
  // console command should end the sleep window instead of being sat on for
  // another cycle.
  //
  // With UART_WAKE=1 it can wake the MCU, and then WAKEUP_CONSOLE is what says
  // so. Without it the wakeup happened but the command was only serviced at the
  // next regular wake: measured, 5 answers out of 60 requests instead of 2 -
  // the extra three all fell into the ~40 s broadcast cycle, not into the
  // moment the byte arrived.
  return wakeup_source == WAKEUP_RTC || wakeup_source == WAKEUP_USB
         || wakeup_source == WAKEUP_CONSOLE
         || Serial.available()
         || (debug_enabled() && DEBUGSER.available());
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
  wake_mark(WP_GO);
  bool keep_usb = keep_usb_active_during_sleep();

  pinDisable(PIN_V_READ_TRIGGER);

// shut down the USB peripheral
  if(first_sleep && !keep_usb){
    log_d("Disable USB\r\n");
    usb_ignore_detach_event = true;
    USB->DEVICE.CTRLA.bit.ENABLE = 0;                   
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE){};
    first_sleep = false;
    usb_connected = false;

    if(set_cpu_div(settings.div_cpu_slow)){ // USB test mode keeps 48MHz; only lower the clock for real low-power sleep.
      div_cpu = settings.div_cpu_slow;
      DEBUGSER.begin(115200*div_cpu); // F_CPU ist still 48M, so every clock needs to by multiplied manually
      // begin() does a SWRST, so the standby configuration has to go back on.
      configure_debug_uart_standby(settings.uart_wake && debug_enabled());
    }
  }

  uint32_t time_to_sleep = calc_time_to_sleep();
  // One-shot wait window when weather TX is due but wind data is stale.
  if (force_wait_for_wdata) {
    time_to_sleep = 12000UL;
    force_wait_for_wdata = false;
  }
  // A forward that is already scheduled has to go out on time, and
  // calc_time_to_sleep() only knows the station's own transmissions. Forwards
  // are queued 500-1500 ms out (500-2000 ms after an LBT retry), so one queued
  // while the station was awake used to be slept through until its next regular
  // wakeup - with or without the old 100 ms awake time. Inside the sleep loop
  // service_lora_during_sleep() already handled this; now this path does too.
  const uint32_t fwd_delay = fanet_forward_delay_ms();
  if(fwd_delay && fwd_delay < time_to_sleep){
    time_to_sleep = fwd_delay;
  }
  // The scheduled repeater has to be awake when the next packet of a station it
  // knows is due, and may sleep through everything in between - see
  // fanet_sched.h. Its deadlines are the edges of those windows and of the scan.
  if(repeater_sched_active()){
    uint32_t sched_delay = sched_next_deadline(time());
    if(sched_delay != SCHED_NO_DEADLINE){
      if(sched_delay < 20){ sched_delay = 20; } // a sleep of zero would skip the sleep entirely
      if(sched_delay < time_to_sleep){ time_to_sleep = sched_delay; }
    }
  }
  wake_mark(WP_TTS);
  if(time_to_sleep == 0){ return;} // if time_to_sleep = 0, do not sleep at all
  // The RTC is set up further down, once the final sleep time is known (DAVIS
  // integration time, the no-settings interval) - each setup costs synchronised
  // writes.

  if(deep_undervoltage){
    radio_sleep();
    log_e("Deep UV (<2.9V). Sleeping until solar charges or voltage recovers\r\n");
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
    rtc_sleep_cfg(time_to_sleep);
    log_w("Undervoltage. will sleep for ", time_to_sleep);
    log_flush();
    sleep(false); // will sleep in 30min blocks with 70µA current draw until battery recovers.
    // wakeup() does not read the battery any more, so it is read here: this is
    // how undervoltage clears once the voltage has recovered.
    read_batt_perc();
    wakeup();
    return; // do not read sensors
  }

  if(settings.sensor_type == s_DAVIS6410){ 
    time_to_sleep = min(time_to_sleep, settings.sensor_integration_time);  // interval for gust detection 
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

  log_v("awake ms: ", (uint32_t)(millis() - wake_started_ms));
  if(time_to_sleep > 200000UL) {
    log_v("will sleep for s: ", time_to_sleep / 1000);
  } else {
    log_v("will sleep for ms: ", time_to_sleep);
  }
  // The compact log says the same in one line: what woke the station, what the
  // sensor window brought, how long it was awake, how long it sleeps.
  log_s("wake "); log_write(wakeup_source_string[last_wake_source]);
  log_write(" at "); log_write((uint32_t)time());
  log_sensor_window_fragment();
  log_write(" | awake "); log_write((uint32_t)(millis() - wake_started_ms));
  log_write(" ms, sleep "); log_write((uint32_t)time_to_sleep); log_write("\r\n");
  // No log_flush() here. The RTC, radio and pulse counter setup below is mostly
  // synchronisation waits: done while the UART is still sending, that time is
  // shared with the drain instead of added after it. sleep() flushes right
  // before standby.

  wake_mark(WP_LOG);
  rtc_sleep_cfg(time_to_sleep);
  // rtc_sleep_cfg() started a new period. An RTC wakeup recorded before it - the
  // end of an OTA offer wait or a TX-done wait, or a short period that ran out
  // while the station was awake - belongs to the old one. Left in wakeup_source,
  // it ended the sleep window below before it began: a DAVIS integration of 0 ms.
  if(wakeup_source == WAKEUP_RTC){
    wakeup_source = WAKEUP_NONE;
  }
  wake_mark(WP_RTC);

  // Ensure radio is in the correct state for the sleep window (RX if conditions
  // are met, PHY sleep otherwise). This also covers the period before the first
  // TX, when radio_sleep() has not yet been called from the TX-done path.
  radio_sleep();
  wake_mark(WP_RADIO);

// Using TC3 for hardware pulsecounting on Falling edge on pin PA04 (D17). No interrupts needed.
  if(settings.sensor_type == s_DAVIS6410){ // pulse counting anemometer
    uint32_t t = 0;
    arm_pulse_counter(); // full setup only on the first cycle, or when something undid it
    wake_mark(WP_ARM);

    while(!sleep_window_over()){ // ignore other wakeups (external pin Interrupt, if configured)
      t += sleep(false);
      service_lora_during_sleep();
    }
    pulsecount = pulse_count_since_arm();
    wake_mark(WP_PULSE);
    calc_pulse_sensor(pulsecount, t);
    wake_mark(WP_DIR);
  } 
  // UART sensor, just sleep
  else if(sensor_uses_uart()){ // UART based sensor
    int res = -1;

    while(!sleep_window_over()){
      sleep(true); // sleep til UART interrupt
      service_lora_during_sleep();
#ifdef WS85_TRACE
      ws85uart.tr_lora_us = micros();
#endif

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
      
      // A complete block: nothing is coming from the sensor until shortly
      // before the next one, so sleep without listening to it - noise on the
      // sensor line cannot wake the station in that time. When the next block
      // is due is measured from the blocks themselves, see
      // wsxx_sleep_before_next_block_ms().
      if(is_wsxx() && res == RESP_COMPLETE){
        int32_t t = next_tx_time - time();
        if(t > 0){
          const uint32_t s = min((uint32_t)t, wsxx_sleep_before_next_block_ms());
          if(s > 5u){
            rtc_sleep_cfg(s);
            sleep(false); // sleep without UART interrupt
            wakeup_source = WAKEUP_NONE;
          }
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
  wake_mark(WP_WAKEUP);
}
