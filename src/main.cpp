#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "wiring_private.h" // pinPeripheral() function


#include "defines.h"
#include "hist.h"
#include "sensors.h"
#include "logging.h"
#include "sleep.h"
#include "display.h"
#include "tools.h"
#include "status.h"
#include "radio.h"
#include "ota_ab.h"
#include "ota_lora.h"
#include "power.h"
#include "scheduler.h"
#include "fanet_sched.h"

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

bool usb_connected = false;
uint32_t boot_usb_hold_until_ms = 0;


// ### Variables for storing settings from file, may be overewritten #####


bool settings_ok = false;











// Setup ----------------------------------------------------------------------------------------------------------------------

extern uint32_t __etext;

void setup(){

  // Change BOD33 action from RESET to INTERRUPT as early as possible.
  // This prevents a brownout (< ~2.8 V) from resetting the MCU into the
  // DFU bootloader where it would hang drawing 1.5-4 mA.  The ISR instead
  // disables the UART TX leakage path and cuts sensor power before the
  // normal UV-sleep loop takes over.
  setup_BOD33_interrupt();
  // Before anything touches the red LED or the battery divider: both sit on
  // different pins from hardware 2.5 on, see hw.h.
  hw_detect();

  log_set_debug(true);
  log_set_error(true);
  DEBUGSER.begin(115200); // on boot start with 48Mhz clock // log_ser_begin(); 

  pinPeripheral(PIN_SERCOM1_RX, PIO_SERCOM_ALT); 
  pinPeripheral(PIN_SERCOM1_TX, PIO_SERCOM_ALT); 

  log_i("\r\n--------------- RESET -------------------\r\n");
  log_reset_cause();
  reset_history_note();
  log_i("Version: ");  log_i(VERSION); log_i("\r\n");
  log_i("Bootloader: "); log_i(get_bootloader_version()); log_i("\r\n");
  log_i("Hardware: "); log_i(hw_name()); log_i(" (id "); log_write((uint32_t)hw_id_code()); log_i(")\r\n");
  if(hw_id_overruled()){
    // The readings are the evidence here, so they go out at W.
    log_e("Hardware: the ID pin and the battery divider disagree - going by the divider\r\n");
    log_w("Hardware: divider "); log_w(hw_divider_text()); log_w("\r\n");
  } else {
    log_d("Hardware: battery divider [V]: ", hw_divider_v());
  }
  if(hw_id_unknown()){
    log_w("Hardware: ID code not known to this firmware, using the 2.5 pin map\r\n");
  }
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

  // An I2C slave caught in the middle of a transfer by a reset keeps SDA low
  // until it sees nine clocks, and the Wire library then waits for the bus for
  // ever: that is a station that hangs right after "Radio:" on every boot
  // until it is powered down. Clocking the bus free as a plain GPIO before
  // Wire.begin() costs nothing when it is idle.
  i2c_bus_recover();
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
    // A scheduled repeater hears nothing but its windows and its scans, so
    // without scans it would never find a station.
    if(settings.repeater_sched && !settings.repeater_scan_duration){
      settings.repeater_scan_duration = SCHED_SCAN_DEFAULT_MS;
      log_w("REPEATER_SCAN_DURATION=0 would never find a station, using the default\r\n");
    }
    // The classic repeater keeps the table too, for its repeater report, but
    // has no use for scan windows: it listens all the time anyway.
    sched_configure(settings.repeater_scan_interval,
                    settings.repeater_sched ? settings.repeater_scan_duration : 0);
    if(hw_forced()){
      log_w("Hardware: pin map pinned by HW_REVISION to "); log_w(hw_name()); log_w("\r\n");
    }
    if(!debug_enabled()){
      DEBUGSER.flush();
      DEBUGSER.end();
      pinDisable(PIN_SERCOM1_RX);
      pinDisable(PIN_SERCOM1_TX);
    } else {
      // The settings are known only now, so this is the first point where
      // UART_WAKE can be honoured - the begin() at the top of setup() ran
      // before they had been read.
      configure_debug_uart_standby(settings.uart_wake);
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
      // Testation (1234m)
      const size_t len = strlen(settings.station_name);
      snprintf(settings.station_name + len, sizeof(settings.station_name) - len,
               " (%dm)", (int)settings.altitude);
    }

    state.use_pulse_counter = true; // settings.sensor_type == s_DAVIS6410; // add other pulsecounting sensors here
    state.use_rtc_counter = true; // use for all sensors

    setup_PM(state.use_pulse_counter, state.use_rtc_counter);
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
    log_e("Settings invalid\r\n");
    setup_PM(true, true);
    wdt_enable(WDT_PERIOD,false); // setup clocks
    wdt_disable();
    //setup_rtc_time_counter();
  }

  if(settings_ok){
    if(sensor_uses_uart()){
      switch_sensor_power(1); // Turn on WS80 Power supply with P-MOSFET
      setup_rtc_time_counter();

      if(sensor_uses_raw_uart()){
        SENSOR_UART.begin(115200); // div_cpu not required, as its 1 at reset
      }
      if(settings.sensor_type == s_WS85_UART){
        //log_i("Setup WS85\r\n");

        ws85uart.begin(div_cpu);
        // Keep the current sensor baud on boot; a forced switch here can stall
        // startup on boards that already stream correctly or are still powering up.
        ws85uart.set_baud_115200();
#ifdef WS85_TRACE
        // WS85_TRACE debug build only: 25 s of what the sensor really sends, with
        // the station awake, before the first sleep. See ws85_reference_capture().
        ws85_reference_capture(25000UL);
#endif
        //ws85uart.setAutoSendInterval(8500);
      }
    } else if(settings.sensor_type == s_DAVIS6410){
      setup_rtc_time_counter();
    } else {
      // No sensor configured
      if(settings.repeater) {
        setup_rtc_time_counter(); // repeater without sensor still needs RTC for sleep
      } else {
        // The red LED alone said nothing about which of the mandatory settings
        // was missing.
        log_e("No sensor configured (no SENSOR_* key), nothing to measure\r\n");
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

// init wind history ringbuffer
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
  // One battery reading at boot, so undervoltage is known before the first
  // sleep decision. After that it is read when a transmission needs it.
  read_batt_perc();
  wakeup();
  if(settings_ok && settings.use_wdt){
    wdt_enable(WDT_PERIOD,false);
    log_i("WDT runtime: ON\r\n");
    // Which clock the debug console receives on: with UART_WAKE it is the
    // internal 8 MHz oscillator, awake and asleep, otherwise the main clock.
    // The two behave differently enough that a console test has to know.
    log_i(settings.uart_wake ? "Console UART: wake from standby ON (8 MHz clock)\r\n"
                             : "Console UART: wake from standby OFF\r\n");
    log_flush();
  } else if(settings_ok){
    log_w("WDT: OFF - a test setting, a station in the field always runs with the watchdog\r\n");
  }
}

// loop ----------------------------------------------------------------------------------------------------------------------

// Below this, sleep() costs more in synchronisation waits than it saves, so a
// nearly closed OTA window is simply waited out.
static constexpr uint32_t OTA_OFFER_SLEEP_MIN_MS = 50u;

void loop(){

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
    log_d("USB connected after boot, keep awake for 10s\r\n");
  }

  bool boot_usb_hold_active = (boot_usb_hold_until_ms != 0) && ((int32_t)(millis() - boot_usb_hold_until_ms) < 0);

  if(last_call && (time()-last_call > 15)){
    if(!s){led_status(0);}
  }

  // Always poll CDC commands so TEST_USB can switch from USB wait mode into the
  // normal sleep/wakeup flow without requiring a reboot.
  read_serial_cmd();
  handle_usb_link_watchdog();
  status_periodic();

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
        log_flush();
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

  // The barometer is read when a transmission needs it, see tx_run_jobs().
  #if BREEZEDUDE_ENABLE_GPS
  if(settings.use_gps){read_gps();}
  #endif

  tx_run_jobs();
  tx_service_active_send();

  if(!tx_send_in_progress() && ota_lora_poll()) {
    sleep_allowed = time() + 1;
  } else if(settings.lora_smart_rcv || settings.repeater){
    if(fanet_rx() && !tx_send_in_progress()) {
      // readData() in fanet_rx() left radio in standby; re-arm RX now that we
      // know no TX is in progress (avoids clearing the TX-done DIO1 handler).
      radio_sleep();
    }
  }

  if(!tx_send_in_progress() && !ota_lora_busy() && (settings.lora_smart_rcv || settings.repeater)) {
    if(fanet_forward_check()) {
      tx_note_send_started();
    }
  }

  if(state.analog_test_mode){
    forward_analog_test_serial();
  }

  // Waiting for the answer to the OTA HWInfo: only the radio has to listen.
  // Sleep until it hears something (DIO1 wakes the MCU) or the window closes
  // (RTC), instead of spinning this loop at full speed for the whole window -
  // which cost about 12 mA, radio and MCU together, once an hour.
  //
  // loraReceivedFlag is checked once more right before sleep(): a packet that
  // arrives while the RTC is being set up would otherwise wait for the deadline.
  // The same holds for the START of a session that was just acknowledged:
  // nothing to do but listen, so sleep with the radio on.
  if(!tx_send_in_progress() && ota_lora_rx_waiting() && !loraReceivedFlag){
    const uint32_t left = ota_lora_rx_wait_ms_left();
    if(left >= OTA_OFFER_SLEEP_MIN_MS){
      rtc_sleep_cfg(left);
      if(!loraReceivedFlag){
        sleep(false);
      }
    }
  }

// Check if everything is done --> sleep
  if(!state.analog_test_mode && !tx_send_in_progress() && !ota_lora_busy() && !boot_usb_hold_active && !console_line_pending() && sleep_allowed && time_reached(sleep_allowed) && (time() > 2500)){ // allow sleep after 2500 ms to get a chance to detect USB / CDC commands
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