#include <Arduino.h>
#include <array>

#include "scheduler.h"
#include "tx_grid.h"
#include "defines.h"
#include "hist.h"
#include "sensors.h"
#include "logging.h"
#include "sleep.h"
#include "fanet.h"
#include "tools.h"
#include "radio.h"
#include "ota_ab.h"
#include "ota_lora.h"
#include "power.h"

#ifndef BREEZEDUDE_DEVELOP_MODE
#define BREEZEDUDE_DEVELOP_MODE 0
#endif

// Message timing

static uint32_t last_msg_weather = 0;
static uint32_t last_msg_name = 0;
static uint32_t last_msg_info = 0;

uint32_t next_tx_time = 0;
uint32_t broadcast_scale_factor = 1; // multiplier for broadcast values. is set to 2..5 if battery is low

uint32_t last_fnet_send = 0; // last package send
uint32_t fanet_cooldown = 4000;
bool force_wait_for_wdata = false;

// Sentinel returned by timeToSend()/calc_time_to_sleep() for "no message due".
static constexpr uint32_t TTS_NEVER = 0xFFFFFFFFu;

// > 0: time() the message was handed to the radio, cleared when TX completes.
static uint32_t send_active = 0;

// Milliseconds until the next message of this kind is due, 0 if it is due now,
// TTS_NEVER if the interval is disabled.
// All arithmetic is unsigned: time() is a uint32_t ms counter and the signed
// version returned nonsense once time() passed INT32_MAX (~24.8 days uptime).
static uint32_t timeToSend(uint32_t last, uint32_t interval) {
  if (interval) {
    const uint32_t delay = interval * broadcast_scale_factor;
    const uint32_t elapsed = time() - last; // wrap-safe difference
    return (elapsed < delay) ? (delay - elapsed) : 0;
  }
  return TTS_NEVER;
}

// The weather packet is the one message that has to keep a rhythm, so it is
// scheduled on a fixed grid instead of "last send plus interval". The grid
// itself lives in tx_grid.cpp; see tx_grid.h for what that buys and why the
// other messages do not need it.
static uint32_t weather_interval_ms(){
  return settings.broadcast_interval_weather * broadcast_scale_factor;
}

// Milliseconds until the next weather packet is due, in the same terms as
// timeToSend().
static uint32_t weather_due_in_ms(){
  if(!settings.broadcast_interval_weather){ return TTS_NEVER; }
  tx_grid_set_interval(time(), weather_interval_ms());
  const uint32_t due = tx_grid_due_in(time());
  return due == TX_GRID_NEVER ? TTS_NEVER : due;
}

enum TxAttemptResult : uint8_t {
  TX_ATTEMPT_NONE = 0,
  TX_ATTEMPT_STARTED,
  TX_ATTEMPT_DEFERRED_LBT,
  TX_ATTEMPT_FAILED,        // the radio refused the packet; retried like a deferral
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
  if(settings.lora_lbt && !lbt_channel_free()) {
    radio_sleep();
    return TX_ATTEMPT_DEFERRED_LBT;
  }
  const int state = radio_phy->startTransmit(data, len);
  if(state != RADIOLIB_ERR_NONE){
    // Taken for a sent packet, this waited 3.5 s for a TX-done that never
    // comes and left nothing in the log but "Send timed out".
    log_w("TX start failed, code=", state);
    radio_sleep();
    return TX_ATTEMPT_FAILED;
  }
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

  log_d("LBT: defer ");
  log_d(label);
  log_d(" TX by ms: ", retry_delay);
  log_d("\r\n");
}

// calc time to sleep til next fanet message needs to be send
uint32_t calc_time_to_sleep(){
  uint32_t tts_weather = TTS_NEVER;
  uint32_t tts_name = TTS_NEVER;
  uint32_t tts_info = TTS_NEVER;
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
      log_w("Low voltage\r\n");
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




  tts_weather = weather_due_in_ms();   // the fixed grid, see tx_grid.h
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

  if(tts == TTS_NEVER){
    // Nothing scheduled at all (every interval disabled). Sleep a fixed window
    // instead of returning 0, which meant "do not sleep" and left the MCU
    // spinning until the loopcounter fallback below caught it.
    tts = 12000;
  }

  // While an OTA offer or session runs, tx_run_jobs() starts nothing, so a job
  // that is due stays due. Left at 0 that spun the loop into the "just
  // looping" fallback below on every OTA window. Wait the window out instead.
  if(!tts && ota_lora_busy()){
    const uint32_t left = ota_lora_offer_ms_left();
    tts = left > 1000u ? left : 1000u;
  }

  if( !tts && loopcounter > 25){
    log_w("just looping, will sleep\r\n");
    tts = 12000; // set to sleep to keep updating sensor data
  }

  next_tx_time = time()+tts;

  
  return tts;
}

static void print_data(){
  if((debug_enabled())){
    log_d("\r\nmillis: ", millis()); 
    log_d("time: ", time()); 
    //log_i("Wind dir_raw: ", wind_dir_raw);
    log_d("Wind Heading: ", sensor.wind_heading);
    log_d("Wind Speed: ", sensor.wind_speed);
    log_d("Wind Gust: ", sensor.wind_gust);
    log_d("Temp: ", sensor.temperature);
    //log_i("Humd: ", humidity);
    if(baro_fresh()){
      log_d("Baro: ", sensor.baro_pressure);
      log_d("PCB_Temp: ", sensor.baro_temp);
    } else if(settings.use_baro){
      log_d("Baro: no reading\r\n");
    }
    if(is_wsxx()) {
      log_d("VCC: ", sensor.wsxx_vcc);
      //log_i("LUX: ", light_lux);
      //log_i("UV: ", uv_level);
    }
    log_d("\r\n");
    log_d("V_Bat: ", sensor.batt_volt);
    log_d("Bat_perc: ", sensor.batt_perc);
    log_d("PV_charge: ", sensor.pv_charging);
    log_d("PV_done: ", sensor.pv_done);
  }
}

// Send ----------------------------------------------------------------------------------------------------------------------


static TxAttemptResult send_msg_weather(){
  if(settings.sensor_type == s_invalid){ return TX_ATTEMPT_NONE;}

  
  WindSample current_wind = get_wind_from_hist(settings.wind_age);
  if(!current_wind.valid){
    log_w("No wind sample in the last [s]: ", settings.wind_age / 1000u);
    return TX_ATTEMPT_NONE;
  }
  led_status(1);
  sensor.wind_gust = get_gust_from_hist(settings.gust_age);
  sensor.wind_speed = current_wind.wind/10.0f;
  sensor.wind_dir_raw = current_wind.dir_raw;
  sensor.wind_heading = sensor.wind_dir_raw + settings.heading_offset;
  if(sensor.wind_heading > 359){ sensor.wind_heading -=360;}
  if(sensor.wind_heading < 0){ sensor.wind_heading +=360;}

  if(settings.sensor_type == s_DAVIS6410){ // no other temp sensor
    sensor.temperature= sensor.baro_temp;
  } 
  if( sensor.wind_speed > (sensor.wind_gust +3)){
    sensor.wind_gust = sensor.wind_speed;
    log_d("adapting gust speed\r\n");
  }

  weatherData wd;
  wd.vid = FANET_VENDOR_ID;
  wd.fanet_id = get_fanet_id();
  wd.lat = position_lat();
  wd.lon = position_lon();
  wd.bWind = true;
  wd.wHeading = sensor.wind_heading;
  wd.wSpeed = sensor.wind_speed;
  wd.wGust = sensor.wind_gust;      
  // A DAVIS station has no thermometer but the barometer's; without a reading
  // the packet carries no temperature rather than 0 degrees.
  wd.bTemp = !(settings.sensor_type == s_DAVIS6410 && !baro_fresh());
  wd.temp = sensor.temperature;

  if(is_wsxx() && sensor.humidity > 0){
    wd.bHumidity = true;
    wd.Humidity = sensor.humidity;
  } else {
    wd.bHumidity = false;
    wd.Humidity = -1;
  }

  if(baro_fresh()){
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
    log_w("\r\nTESTMODE - Fake values\r\n");
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
  return tx_result;
}


// check if everything is ok to send the wather data now
static bool allowed_to_send_weather(){
  // The first thing in the way, named. A station that holds its weather back
  // used to say nothing at all when the sensor keeps no data timestamp - with a
  // DAVIS anemometer the only trace of a barometer that never answers was
  // "just looping, will sleep" on every wakeup, and that line does not say why.
  //
  // The barometer is not on this list: a packet is never held back for
  // pressure. The wind is what the packet is for, and a reading that is not
  // there is left out of it - see baro_fresh() in send_msg_weather(). Holding
  // the packet made a barometer that never answers silence the station.
  const char *why = nullptr;
  if(!settings_ok){
    why = "no settings";
  } else if(sensor_tracks_last_data() && !sensor.last_data && !settings.testmode){
    why = "no wind data yet";
  #if BREEZEDUDE_ENABLE_GPS
  } else if(settings.use_gps && !tinyGps.location.isValid()){
    why = "no GPS fix";
  #endif
  } else if(undervoltage){
    why = "undervoltage";
  }
  if(!why){ return true; }

  // Once per retry, not once per loop iteration: the scheduler asks again on
  // every pass until the packet goes out.
  static uint32_t last_said = 0;
  if(!last_said || (time() - last_said) > 5000u){
    last_said = time();
    log_w("Weather TX held back: "); log_w(why); log_w("\r\n");
    if(sensor.last_data && !baro_conversion_running() && !undervoltage){ // ever got some sensor data and baro currently not waiting for data
      log_d("Wind Sensor data age: ", time()- sensor.last_data);
      log_d("Baro data age: ", time()- sensor.last_baro_reading);
    }
  }
  return false;
}

// check if last fanet package want sent recenctly
static bool fanet_cooldown_ok(){
  if(lora_module && (time() -last_fnet_send > fanet_cooldown)){
    return true;
  }
  return false;
}

static TxAttemptResult send_msg_name(const char* name, int len){
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

static TxAttemptResult send_msg_info(){
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




// TX scheduling ---------------------------------------------------------------------------------------------------------------

// The weather, name and info transmissions used to be three copy-pasted blocks
// in loop() that differed in only a handful of places. They are rows in a table
// now: one place to see what is scheduled, and in which order.
typedef struct {
  const char      *label;
  uint32_t        *last_msg;      // stamped when a TX actually starts
  const uint32_t  *interval;      // configured interval; nullptr if due_in_ms is used
  uint32_t       (*due_in_ms)();  // for jobs that own their schedule (OTA hwinfo)
  bool           (*ready)();      // optional gate, checked once the job is due
  TxAttemptResult (*send)();
  void           (*on_deferred)(uint32_t retry_delay); // nullptr -> defer_next_send()
  void           (*on_none)();    // nullptr -> nothing
  bool            fresh_readings; // refresh battery and barometer before sending
  void           (*on_sent)();    // nullptr -> only last_msg is stamped
  bool            keeps_slot;     // owns a fixed slot, the others give way to it
} TxJob;

static bool tx_name_ready(){
  return strlen(settings.station_name) > 1;
}

static TxAttemptResult tx_send_name(){
  led_status(1);
  TxAttemptResult result = send_msg_name(settings.station_name, strlen(settings.station_name));
  if(result == TX_ATTEMPT_STARTED) {
    log_i("Send name: "); log_i(settings.station_name); log_i("\r\n");
  }
  led_status(0);
  return result;
}

// Wind data too old to send: log the ages, force a short wait window, and if the
// sensor has been silent for over an hour, reset to recover.
static void handle_stale_wind_data(){
  if(!(sensor_tracks_last_data() && sensor.last_data && !baro_conversion_running() &&
       (time() - sensor.last_data > 10000))) {
    return;
  }
  const uint32_t wdata_age = time() - sensor.last_data;
  log_w("Wdata not ready. Wdata age: ", wdata_age);
  log_d("Last Baro reading age: ", (time() - sensor.last_baro_reading));
  if(wdata_age > 3600000UL){
    log_e("Wind data stale > 1h, restarting to recover\r\n");
    log_flush();
    NVIC_SystemReset();
  }
  force_wait_for_wdata = true;
  sleep_allowed = time();
}

static bool tx_weather_ready(){
  if(allowed_to_send_weather()) {
    return true;
  }
  handle_stale_wind_data();
  if(!sensor_tracks_last_data()){
    // A pulse-counting sensor has nothing to wait for but its next window, and
    // handle_stale_wind_data() above does not apply to it. Without this the
    // packet stayed due, the loop ran 25 times to the "just looping" fallback
    // in calc_time_to_sleep(), and only that put the station to sleep. Ask
    // again in ten seconds instead; the slot itself stays where it is.
    tx_grid_retry(time(), 10000);
    sleep_allowed = time() + 1;
  }
  return false;
}

static void tx_weather_none(){
  // No valid wind data yet. The packet waits ten seconds; the grid does not
  // move, so the rhythm picks up again with the next slot.
  tx_grid_retry(time(), 10000);
  sleep_allowed = time() + 1;
  log_w("Weather TX skipped, retry in 10s\r\n");
}

static void tx_weather_deferred(uint32_t retry_delay){
  // The channel was busy. Try again shortly, but leave the grid where it is:
  // this packet is late, the next one need not be.
  tx_grid_retry(time(), retry_delay);
  next_tx_time = time() + retry_delay;
  sleep_allowed = time() + 1;
  log_d("LBT: defer weather TX by ms: ", retry_delay);
  log_d("\r\n");
}

static void tx_weather_sent(){
  tx_grid_sent(time());
}

static TxAttemptResult tx_send_info(){
  led_status(1);
  TxAttemptResult result = send_msg_info();
  led_status(0);
  return result;
}

static void tx_info_deferred(uint32_t retry_delay){
  ota_lora_defer_hwinfo_retry(retry_delay);
  next_tx_time = time() + retry_delay;
  sleep_allowed = time() + 1;
  log_d("LBT: defer info TX by ms: ", retry_delay);
  log_d("\r\n");
}

// Weather first, and it is the one job that keeps its slot: when several are
// due in the same wakeup it goes out first and the others wait out the cooldown
// behind it. The other way round - which is what the old order did - pushed the
// weather packet four seconds off its rhythm.
static const TxJob tx_jobs[] = {
  {"weather", &last_msg_weather, &settings.broadcast_interval_weather, weather_due_in_ms,
   tx_weather_ready, send_msg_weather, tx_weather_deferred, tx_weather_none, true,
   tx_weather_sent, true},
  {"name",    &last_msg_name,    &settings.broadcast_interval_name,    nullptr,
   tx_name_ready,    tx_send_name,     nullptr,          nullptr,         false,
   nullptr,         false},
  {"info",    &last_msg_info,    nullptr, ota_lora_next_hwinfo_due_ms,
   nullptr,          tx_send_info,     tx_info_deferred, nullptr,         true,
   nullptr,         false},
};

// Nothing may go out in the cooldown that runs up to the weather slot: a
// transmission started there holds the weather packet back past its grid, which
// is the very thing the grid exists to prevent. Name and HWInfo have no rhythm
// to keep and simply follow it.
static bool would_push_weather(const TxJob &job){
  if(job.keeps_slot){ return false; }
  const uint32_t tts = weather_due_in_ms();
  // Only the run-up to the slot is kept clear. A weather packet that is due
  // right now has already had its turn in this pass - it is first in the table,
  // so it either went out or could not go out at all, for want of wind data -
  // and holding the others back on that would silence the station completely.
  return tts != TTS_NEVER && tts > 0 && tts <= fanet_cooldown;
}

static bool tx_job_due(const TxJob &job){
  if(job.due_in_ms){
    return job.due_in_ms() == 0u;
  }
  const uint32_t interval = *job.interval;
  if(!interval){
    return false; // 0 disables this message
  }
  return (time() - *job.last_msg) > (interval * broadcast_scale_factor);
}

// Battery voltage and barometer, refreshed right before a transmission that
// carries them. Both used to be read on every wakeup although neither value is
// used otherwise, and a 1000 mAh battery does not change on that time scale.
//
// At most once a second: a job that is due but not ready yet comes back on the
// next loop iteration, and reading again there achieves nothing.
static void refresh_tx_readings(){
  static uint32_t last_refresh = 0;
  if(last_refresh && (time() - last_refresh) < 1000u) {
    return;
  }
  last_refresh = time();
  read_batt_perc();
  if(settings.use_baro) {
    read_baro();
  }
}

// Start any transmission that is due. The busy/cooldown check is re-evaluated
// per job, exactly as the three original blocks did: a TX started by an earlier
// job pushes last_fnet_send forward, and the cooldown then holds the rest back
// until a later loop iteration.
void tx_run_jobs(){
  for(const TxJob &job : tx_jobs){
    if(ota_lora_busy() || !fanet_cooldown_ok()) continue;
    if(would_push_weather(job)) continue;
    if(!tx_job_due(job)) continue;
    // Before ready(): the weather gate checks how old the barometer reading is.
    if(job.fresh_readings) refresh_tx_readings();
    if(job.ready && !job.ready()) continue;

    const TxAttemptResult result = job.send();
    if(result == TX_ATTEMPT_STARTED){
      last_fnet_send = time();
      *job.last_msg  = time();
      send_active    = time();
      if(job.on_sent) job.on_sent();
    } else if(result == TX_ATTEMPT_DEFERRED_LBT || result == TX_ATTEMPT_FAILED){
      if(job.on_deferred){
        job.on_deferred(next_lbt_retry_delay());
      } else {
        defer_next_send(*job.last_msg, *job.interval, job.label);
      }
    } else if(result == TX_ATTEMPT_NONE && job.on_none){
      job.on_none();
    }
  }
}

// Setup ------------------------------------------------------------------------------------------------------------------
bool tx_send_in_progress(){
  return send_active != 0u;
}

void tx_note_send_started(){
  send_active = time();
}

// TX completion and timeout handling, previously inline in loop().
void tx_service_active_send(){
  if(!send_active){
    return;
  }

  if( (time()- send_active > (3500))){
    led_error(1);
    log_w("Send timed out\r\n");
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
