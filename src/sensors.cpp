#include "sensors.h"
#include "sleep.h"
#include "wiring_private.h"

extern int div_cpu;
extern bool undervoltage;
extern bool deep_undervoltage;


#if BREEZEDUDE_ENABLE_GPS
TinyGPSPlus tinyGps;
#endif

// I2C Barometer
#if BREEZEDUDE_BARO_BMP280
BMP280 bmp280; // Bosch BMP280
#endif
#if BREEZEDUDE_BARO_SPL06
SPL06 spl; // Goertek SPL06-001
#endif
#if BREEZEDUDE_BARO_HP203B
HP203B hp; // HP203B 0x76 or 0x77
#endif
#if BREEZEDUDE_BARO_BMP3XX
Adafruit_BMP3XX bmp3xx;
#endif
QMC5883P qmc5883p; // GY-271
// DPS310 as alternative?


CHIP_BARO chip_baro = BARO_NONE;
CHIP_IMU chip_imu = IMU_NONE;

Sensors sensor;

// Test override for the sensor supply, set by the WS_POWER console command.
// -1 leaves the undervoltage state machine in charge, 0 and 1 force it.
//
// Without this a forced state would not survive: read_batt_perc() runs on every
// wakeup and battery_update_power_state() would switch the sensor straight back
// on, which is precisely what happens during a measurement.
static int8_t sensor_power_override = -1;

void set_sensor_power_override(int8_t mode){
  sensor_power_override = mode;
  if(mode >= 0){ switch_sensor_power(mode != 0); }
}

int8_t get_sensor_power_override(){ return sensor_power_override; }

// File scope rather than a function static, so the sleep path can ask whether
// it still makes sense to arm SERCOM0 for a sensor that has no supply.
static bool current_power_state = false;

bool sensor_power_is_on(){ return current_power_state; }

void switch_sensor_power(bool state){
  const bool uses_uart = sensor_uses_uart();
  if(state && !current_power_state){ // turn on
    current_power_state = true;
    pinMode(PIN_PS_WS,OUTPUT);
    digitalWrite(PIN_PS_WS, 0);
    if(uses_uart){
      // Re-connect UART pins to SERCOM0 peripheral before the sensor can respond
      pinPeripheral(PIN_RX, PIO_SERCOM);
      pinPeripheral(PIN_TX, PIO_SERCOM);
      SENSOR_UART.begin(115200 * div_cpu);
      log_d("WS sensor power ON\r\n");
    }
  }
  if(!state && current_power_state){ // turn off
    current_power_state = false;
    digitalWrite(PIN_PS_WS, 1);
    if(uses_uart){
      // Stop the SERCOM before letting go of the pins. Once PMUXEN is cleared
      // the receiver no longer sees a driven line and reads a permanent low -
      // which to a UART is an endless run of start bits and framing errors.
      // Leaving it enabled cost 13 mA instead of 0.78 mA: the station never got
      // to stay asleep, because SERCOM0 kept waking it.
      SENSOR_UART.end();
      // Disable UART pins so the idle-HIGH TX line cannot leak current into the
      // sensor through its input protection diodes when its VCC is removed.
      pinDisable(PIN_TX); // PA10 - UART idles HIGH, would source current into sensor
      pinDisable(PIN_RX); // PA11 - symmetric isolation
      log_d("WS sensor power OFF\r\n");
    }
  }
}

// Undervoltage / sensor-power state machine.  This used to sit inside the SOC
// conversion, so a function named "voltage to percent" cut the sensor supply
// and set global flags as a side effect.
void battery_update_power_state(float v) {
  if(v < 0.8f) { led_error(1); log_w("V_Batt read error: ", v); return; } // bad reading, keep state

  // Deep UV: 3 consecutive readings < 2.9V trigger extended sleep for solar charging recovery.
  // Hysteresis: clears when voltage rises above 3.0V.
  static uint8_t deep_uv_count = 0;
  if (v < 2.9f) {
    if (deep_uv_count < 3) ++deep_uv_count;
    if (deep_uv_count >= 3) deep_undervoltage = true;
  } else if (v > 3.0f) {
    deep_uv_count = 0;
    deep_undervoltage = false;
  }

  if(v < 3.15f) { // undervoltage, turn off sensor to save power
    if(sensor_power_override < 0) { switch_sensor_power(0); }
    state.uv_triggered = true;
    undervoltage = true;
    return;
  }
  if(v > 3.3f) {
    if(sensor_power_override < 0 && sensor_uses_uart()) { switch_sensor_power(1); }
    undervoltage = false;
  }
  if(v > 4.15f) { state.uv_triggered = false; }
}

// Pure open-circuit-voltage -> state of charge conversion, no side effects.
uint8_t battery_soc_percent(float v) {
  if(v < 3.15f) { return 0; }   // also covers the bad-reading case
  if(v > 4.15f) { return 100; }

  // Piecewise linear OCV->SOC for 3.7V Li-Ion at low discharge rate.
  // Use compact integer tables to save flash; 1% SOC resolution is sufficient here.
  static const uint16_t vt_mv[] = { 3150, 3400, 3600, 3750, 3900, 4000, 4100, 4150 };
  static const uint8_t  st[]    = {    0,    5,   20,   50,   75,   88,   97,  100 };
  constexpr int N = 8;

  uint16_t v_mv = (uint16_t)(v * 1000.0f + 0.5f);
  if (v_mv <= vt_mv[0]) return 0;
  if (v_mv >= vt_mv[N-1]) return 100;

  for (int i = 1; i < N; i++) {
    if (v_mv <= vt_mv[i]) {
      uint16_t v0 = vt_mv[i - 1];
      uint16_t v1 = vt_mv[i];
      uint8_t s0 = st[i - 1];
      uint8_t s1 = st[i];
      uint32_t num = (uint32_t)(v_mv - v0) * (uint32_t)(s1 - s0) + (uint32_t)(v1 - v0) / 2u;
      return (uint8_t)(s0 + (num / (uint32_t)(v1 - v0)));
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
    pinMode(hw_pin_v_read(), INPUT);

    // Extend ADC sampling phase via SAMPCTRL instead of software delays between reads.
    // ADC clock = 48 MHz / 512 ~= 94 kHz  ->  half-cycle ~= 5.3 us.
    // SAMPLEN=4 adds 5 half-cycles ~= 26 us extra sample time per conversion.
    //ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(4);
    //while (ADC->STATUS.bit.SYNCBUSY);

    // 7 samples with min/max discarded removes ADC glitches; the moving
    // average further down smooths load dips across wakeups. 15 samples bought
    // no extra accuracy and doubled the time the ADC stays powered.
    constexpr int N_SAMPLES = 7;
    uint16_t samples[N_SAMPLES];
    // V_IMP is driven here instead of relying on wakeup() having done it. Q2 is
    // switched by an edge coupled through C9, so the pin must be a real output:
    // on a pin that is not an output, digitalWrite() only changes the pull
    // resistor, and no edge reaches the gate. The deep undervoltage loop in
    // go_sleep() called this with the pin already released by pinDisable().
    pinMode(PIN_V_READ_TRIGGER, OUTPUT);
    digitalWrite(PIN_V_READ_TRIGGER,1);
    digitalWrite(PIN_V_READ_TRIGGER,0);
    delayMicroseconds(50); // allow divider to settle
    for (int i = 0; i < N_SAMPLES; i++) {
      samples[i] = analogRead(hw_pin_v_read());
    }
    digitalWrite(PIN_V_READ_TRIGGER,1);

    //ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0); // restore default for other ADC users
    //while (ADC->STATUS.bit.SYNCBUSY);

    // Trimmed mean: discard min + max, average remaining N_SAMPLES-2
    uint16_t vmin = samples[0], vmax = samples[0];
    uint32_t vsum = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
      vsum += samples[i];
      if (samples[i] < vmin) vmin = samples[i];
      if (samples[i] > vmax) vmax = samples[i];
    }
    float val = (float)(vsum - vmin - vmax) / (float)(N_SAMPLES - 2);

    pinDisable(hw_pin_v_read());
    val *= 0.0040925f; // 100k/330k 1.0V Vref

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

    battery_update_power_state(sensor.batt_volt);
    sensor.batt_perc = battery_soc_percent(sensor.batt_volt);
    log_d("V_Bat: ", sensor.batt_volt);
    log_d("Bat_perc: ", sensor.batt_perc);
  }
}

static float apply_altitude_correction(float pressure_hpa, float temp_c, float altitude_m) {
  // Lightweight approximation of the barometric altitude correction. (−2664 Byte flash)
  // For the typical Breezedude altitude range, a 2nd-order expansion avoids
  // pulling in powf() while staying close to the original result.
  static float last_altitude = -100000.0f;
  static int16_t last_temp_tenths = INT16_MIN;
  static float last_factor = 1.0f;

  if (altitude_m < 0.0f) {
    return pressure_hpa;
  }

  int16_t temp_tenths = (int16_t)(temp_c * 10.0f);
  if ((altitude_m != last_altitude) || (temp_tenths != last_temp_tenths)) {
    const float lapse_times_alt = 0.0065f * altitude_m;
    const float denom = temp_c + lapse_times_alt + 273.15f;
    if (denom > 0.0f) {
      const float x = lapse_times_alt / denom;
      if (x > 0.0f && x < 0.25f) {
        const float x2 = x * x;
        last_factor = 1.0f + (5.257f * x) + (16.45f * x2);
      } else {
        last_factor = 1.0f;
      }
    } else {
      last_factor = 1.0f;
    }
    last_altitude = altitude_m;
    last_temp_tenths = temp_tenths;
  }

  return pressure_hpa * last_factor;
}


// Sensors ----------------------------------------------------------------------------------------------------------------------


bool init_imu(){
   // if( begin imu()){ // add imu
   //     //chip_imu = IMU_;
   //     log_i("IMU: \r\n");
   //     return true;
   // }
      // add other IMU models here

    settings.use_imu = false;
    log_w("IMU not found");
    return false;
}

void get_imu(){
  if(chip_imu == IMU_){

    }
}


// Barometer state beyond the chip itself.
//
// A barometer that answers on the bus but never returns a reading used to hold
// every weather packet back, and the HP203B driver makes that easy to get: its
// begin() reads one byte with command 0x8F and takes anything but zero for the
// real thing, so a BMP280 at the same address - which hands it a calibration
// byte - counts as found, and from then on every command comes back as zeros.
// That is the "Baro: 0.00" of a board carrying a chip this build was not made
// for. Now the packet goes out without pressure (see baro_fresh()), the first
// failed reads are logged in detail, the bus is asked who is really there, and
// after BARO_FAIL_LIMIT attempts in a row the chip is given a rest of
// BARO_REST_MS before the next single attempt - so a sensor that comes back,
// or was only loose at boot, is picked up again without a reboot.
#define BARO_FAIL_LIMIT 10
#define BARO_REST_MS (10u * 60u * 1000u)
static uint8_t  baro_fails = 0;        // reads in a row that returned nothing, capped
static bool     baro_resting = false;  // nothing is asked of the chip until baro_rest_until
static uint32_t baro_rest_until = 0;
static uint32_t baro_started_at = 0;   // when the pending conversion was started

static void baro_rest(uint32_t now){
  baro_resting = true;
  baro_rest_until = now + BARO_REST_MS;
  sensor.next_baro_reading = 0;
}

bool baro_fresh(){
  return settings.use_baro && sensor.last_baro_reading && (time() - sensor.last_baro_reading) < 40000u;
}

const char *baro_state_text(){
  static char text[40];
  if(!settings.use_baro){ return "off"; }
  if(chip_baro == BARO_NONE){ return baro_resting ? "not found, retrying" : "not found"; }
  const char *name = "unknown";
  if(chip_baro == BARO_BMP280) { name = "BMP280"; }
  if(chip_baro == BARO_BMP3xx) { name = "BMP3xx"; }
  if(chip_baro == BARO_SPL06)  { name = "SPL06"; }
  if(chip_baro == BARO_HP203B) { name = "HP203B"; }
  const char *state = baro_resting ? ", no reading, resting" : (baro_fails ? ", no reading" : "");
  snprintf(text, sizeof(text), "%s%s", name, state);
  return text;
}

static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission(false) != 0){ return false; }
  if(Wire.requestFrom(addr, (uint8_t)1) != 1){ return false; }
  *val = Wire.read();
  return true;
}

// Who is really on the bus? The other chips carry an ID register, the HP203B
// has none - so this runs only once a read has come back empty, and the
// command bytes it sends are harmless to a chip that is already not answering.
static void baro_identify_stranger(){
  const uint8_t addrs[2] = {0x76, 0x77};
  for(uint8_t i = 0; i < 2; i++){
    const uint8_t addr = addrs[i];
    uint8_t id = 0;
    const char *who = nullptr;
    const char *flag = nullptr;
    if(i2c_read_reg(addr, 0xD0, &id) && (id == 0x58 || id == 0x60)){
      who = (id == 0x58) ? "BMP280" : "BME280"; flag = "BREEZEDUDE_BARO_BMP280";
    } else if(i2c_read_reg(addr, 0x00, &id) && (id == 0x50 || id == 0x60)){
      who = "BMP388/BMP390"; flag = "BREEZEDUDE_BARO_BMP3XX";
    } else if(i2c_read_reg(addr, 0x0D, &id) && id == 0x10){
      who = "SPL06"; flag = "BREEZEDUDE_BARO_SPL06";
    }
    if(who){
      log_w("Baro: the chip at 0x"); log_write_hex(addr, 2); log_w(" answers as "); log_w(who);
      log_w(" (id 0x"); log_write_hex(id, 2); log_w(") - this build does not drive it, see "); log_w(flag); log_w("=0\r\n");
      return;
    }
  }
  log_w("Baro: nothing at 0x76/0x77 identifies as BMP280, BMP3xx or SPL06\r\n");
}

bool init_baro(){
    // ToDo: Crashes with HP203B installed when checking for bmp3xx

    #if BREEZEDUDE_BARO_BMP280
    if(bmp280.begin()){
      chip_baro = BARO_BMP280;
      log_i("Baro: BMP280\r\n");
      bmp280.setOversampling(4);
      return true;
    }
    #endif

    #if BREEZEDUDE_BARO_SPL06
    if(spl.begin(0x77)){
      chip_baro = BARO_SPL06;
      log_i("Baro: SPL06\r\n");
      return true;
    }
    #endif

    #if BREEZEDUDE_BARO_BMP3XX
    if(bmp3xx.begin_I2C(0x76)){
      chip_baro = BARO_BMP3xx;
      log_i("Baro: BMP3XX\r\n");
      return true;
    }
    #endif

    #if BREEZEDUDE_BARO_HP203B
    if(hp.begin(0x76, OSR_2048)){
      chip_baro = BARO_HP203B;
      hp.setOSR(OSR_512); // 16.4ms conversion time
      log_i("Baro: HP203B\r\n");
      return true;
    }
    #endif

    // Not switched off for good: a connector that was loose at boot, or a
    // sensor that powers up late, gets another look every ten minutes. The
    // red LED at boot stays as the hint that something is wrong.
    log_w("Baro: not found, next try in 10 min\r\n");
    chip_baro = BARO_NONE;
    baro_rest(time());
    return false; 
}


// request baro sampling & conversion
void baro_start_reading(){
  if(chip_baro == BARO_NONE || baro_resting){ return; }
  uint32_t now = time();
  if(sensor.next_baro_reading){
    // One is running - or one was due and never read. The second case used to
    // be zeroed here and then nobody started another, which left the
    // barometer silent for good. Start over instead.
    if(!time_reached(sensor.next_baro_reading + 200)){ return; }
    sensor.next_baro_reading = 0;
  }
  baro_started_at = now;
  {
    #if BREEZEDUDE_BARO_BMP280
    if(chip_baro == BARO_BMP280){ 
      sensor.next_baro_reading = now + bmp280.startMeasurment();
    }
    #endif
    #if BREEZEDUDE_BARO_SPL06
    if(chip_baro == BARO_SPL06){ 
      spl.start_measure();
      sensor.next_baro_reading = now + 27;
    }
    #endif
    #if BREEZEDUDE_BARO_BMP3XX
    if(chip_baro == BARO_BMP3xx){ 
      bmp3xx.setOutputDataRate(BMP3_ODR_50_HZ);
      bmp3xx.performReading();
      sensor.next_baro_reading = now + 5;
    }
    #endif
    #if BREEZEDUDE_BARO_HP203B
    if(chip_baro == BARO_HP203B){ 
      hp.startMeasure();
      sensor.next_baro_reading = now + 25; // OSR512
    }
    #endif
  }
}

// read value of baro, needs baro_start_reading in advance
void read_baro(){
  bool data_ok = false;
  // float, not double: the SAMD21 has no FPU and double math pulls in the
  // soft-float library. Only the BMP280 driver API needs doubles (see below).
  float T = 0, P = 0;
  uint32_t now = time();

  if(baro_resting){
    if(!time_reached(baro_rest_until)){ return; }
    // The rest is over: one more try. A chip that was never found is looked
    // for again, one that stopped answering gets a fresh conversion. The
    // reading itself happens on the next call, once that has had its time.
    baro_resting = false;
    if(chip_baro == BARO_NONE && !init_baro()){ return; }   // init_baro() rests it again
    log_d("Baro: trying again after the rest\r\n");
    baro_start_reading();
    return;
  }
  if(chip_baro == BARO_NONE){ return; }
  if(!sensor.next_baro_reading){
    // Nothing to read because nothing was started. This used to return in
    // silence, and a barometer that had lost its conversion was never asked
    // again.
    log_d("Baro: no conversion pending, starting one\r\n");
    baro_start_reading();
    return;
  }

  #if BREEZEDUDE_BARO_BMP280
  if(chip_baro == BARO_BMP280){
    if(sensor.next_baro_reading && time_reached(sensor.next_baro_reading)){
      double dT = 0, dP = 0; // BMP280 driver signature takes double&
      uint8_t result = bmp280.getTemperatureAndPressure(dT, dP);
      if(result != 0){
        T = (float)dT;
        P = (float)dP;
        data_ok = true;
      }
    } else { return; }
  }
  #endif

  #if BREEZEDUDE_BARO_SPL06
  if(chip_baro == BARO_SPL06){
    if(sensor.next_baro_reading && time_reached(sensor.next_baro_reading)){
      P = (float)spl.get_pressure();
      T = (float)spl.get_temp_c();
      spl.sleep();
      if(P > 0){
        data_ok = true;
      }
    } else { return; }
  }
  #endif

  #if BREEZEDUDE_BARO_BMP3XX
  if(chip_baro == BARO_BMP3xx){
    if(sensor.next_baro_reading && time_reached(sensor.next_baro_reading)){
      P = bmp3xx.pressure / 100;
      T = bmp3xx.temperature;
      bmp3xx.setOutputDataRate(BMP3_ODR_0_001_HZ);
      if(P > 0){ data_ok = true; }
    } else { return; }
  }
  #endif

  #if BREEZEDUDE_BARO_HP203B
  if(chip_baro == BARO_HP203B){
    if(sensor.next_baro_reading && time_reached(sensor.next_baro_reading)){
      hp.Measure_Pressure();
      hp.Measure_Temperature();
      P = hp.hp_sensorData.P;
      T = hp.hp_sensorData.T;
      if(P > 0){ data_ok = true; }
    } else { return; }
  }
  #endif

  if(data_ok){
    baro_fails = 0;
    sensor.baro_temp = T;
    sensor.last_baro_reading = now;
    sensor.baro_pressure = apply_altitude_correction(P, T, position_alt());
    log_v("Baro: ", sensor.baro_pressure);
    log_v("Baro Temp: ", sensor.baro_temp);
  } else {
    if(baro_fails < 255){ baro_fails++; }
    if(baro_fails <= 3){
      // The first few in detail: how long after the start it was read and what
      // came back. Zeros from a chip that was "found" at boot are the
      // signature of a chip the driver does not actually drive.
      log_d("Baro: no data, attempt ", (uint32_t)baro_fails);
      log_d("Baro: ms since the conversion was started: ", (uint32_t)(now - baro_started_at));
      log_d("Baro: raw pressure ", P);
      log_d("Baro: raw temperature ", T);
      if(baro_fails == 1){ baro_identify_stranger(); }
    }
    if(baro_fails >= BARO_FAIL_LIMIT){
      // Ten in a row - a few minutes at one attempt per transmission - and the
      // chip gets a rest. Weather goes out without pressure meanwhile; after
      // the rest it gets one more attempt, see the top of this function.
      if(baro_fails == BARO_FAIL_LIMIT){
        log_e("Baro: no reading in 10 attempts, resting it for 10 min, weather goes out without pressure\r\n");
      } else {
        log_w("Baro: still no reading, next try in 10 min\r\n");
      }
      baro_rest(now);
      return;
    }
  }
  // Start the next conversion right away. It completes within milliseconds and
  // the sensor holds the result, so the next transmission reads it without
  // waiting. The price is that pressure and temperature are up to one send
  // interval old when they go out, which matters for neither. After a failed
  // read this is also the retry.
  sensor.next_baro_reading = 0;
  baro_start_reading();
}

// A conversion is running and its result is not due yet. That is not the same
// as next_baro_reading being set: it now stays set between transmissions,
// because the next conversion starts as soon as a reading has been taken.
bool baro_conversion_running(){
  return sensor.next_baro_reading && !time_reached(sensor.next_baro_reading);
}

static volatile bool davis_speed_pulse = false;
static void davis_speed_isr() { davis_speed_pulse = true; }

void forward_analog_test_serial(){
  static bool isr_attached = false;

  if(state.analog_test_mode){
    static uint32_t last_print = 0;

    analogReference(AR_DEFAULT); //3.3V refernece
    pinMode(PIN_DAVIS_DIR, INPUT);  
    pinMode(PIN_DAVIS_POWER,OUTPUT);
    digitalWrite(PIN_DAVIS_POWER,1);
    switch_sensor_power(1);

    if (!isr_attached) {
      davis_speed_pulse = false;
      pinMode(PIN_DAVIS_SPEED, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(PIN_DAVIS_SPEED), davis_speed_isr, FALLING);
      isr_attached = true;
    }

    int d = analogRead(PIN_DAVIS_DIR);

    if(time() - last_print > 25){
      last_print = time();
      bool pulse = davis_speed_pulse;
      davis_speed_pulse = false; // clear after reading
      log_write((uint32_t)time());
      log_d(" Speed/Dir state: ");
      log_d(" ");
      log_d(pulse ? "-" : "o");
      log_d("\t");
      log_write(d);
      log_d("\r\n");
    }
    
  } else {
    if (isr_attached) {
      detachInterrupt(digitalPinToInterrupt(PIN_DAVIS_SPEED));
      isr_attached = false;
    }
    switch_sensor_power(0);
    digitalWrite(PIN_DAVIS_POWER,0);
    pinDisable(PIN_DAVIS_DIR);
    pinDisable(PIN_DAVIS_POWER);
    pinDisable(PIN_DAVIS_SPEED);
  }
}

// Set measurement value read from WS80 UART
bool set_value(char* key,  char* value){
  //printf("%s = %s\r\n",key, value);

  if(strcmp(key,"WindDir")==0) {sensor.wind_dir_raw = atoi(value); add_wind_history_dir(sensor.wind_dir_raw); return false;}
  if(strcmp(key,"WindSpeed")==0) {sensor.wind_speed = parse_decimal_fast(value)*3.6f; add_wind_history_wind(sensor.wind_speed); log_v("WindSpeed: ", sensor.wind_speed); return false;}
  if(strcmp(key,"WindGust")==0) {sensor.wind_gust = parse_decimal_fast(value)*3.6f; add_wind_history_gust(sensor.wind_gust); log_v("WindGust: ", sensor.wind_gust); return false;}
  if(strcmp(key,"Temperature")==0) {sensor.temperature = parse_decimal_fast(value); if(settings.sensor_type != s_WS80){settings.sensor_type = s_WS80; log_i("Detected WS80\n");} return false;} // WS80 only - autodetection
  if(strcmp(key,"GXTS04Temp")==0) {sensor.temperature = parse_decimal_fast(value);  if(settings.sensor_type != s_WS85){settings.sensor_type = s_WS85; log_i("Detected WS85\n");} return false;} // WS85 only
  if(strcmp(key,"Humi")==0) {sensor.humidity = atoi(value); return false;}
  if(strcmp(key,"Light")==0) {sensor.light_lux = atoi(value); return false;}
  if(strcmp(key,"UV_Value")==0) {sensor.uv_level = parse_decimal_fast(value); return false;}
  if(strcmp(key,"CapVoltage")==0) {sensor.cap_voltage = parse_decimal_fast(value); return false;} // WS85
  if(strcmp(key,"BatVoltage")==0) {
    sensor.wsxx_vcc = parse_decimal_fast(value); 
    sensor.last_data =time();
    
    log_d("WSXX data complete\r\n");
    return true;
  }

  //log_i(" ->not_found\n");
  return false;
}

// read UART and process input buffer, needs to be called periodically until new block is complete (last_ws80_data = time())
// RESP_COMPLETE 2: data complete
// RESP_ERROR 1: error
// RESP_OK 0: data ok, continue
int read_wsxx(){
  constexpr int WSXX_LINE_BUFFER_SIZE = 128; // size of one parsed line
  static char line_buffer[WSXX_LINE_BUFFER_SIZE];
  int line_len = 0;
  bool found_data = false;
  int eq_count = 0;
  uint32_t last_data = micros();
  static uint32_t serial_wait = 1200;
  uint32_t wait_budget = serial_wait;
  bool parsed_complete_block = false;

  // Keep WS80 polling windows short to avoid long active CPU time.
  if(settings.sensor_type == s_WS80 && wait_budget > 500){
    wait_budget = 500;
  }

// Compare String 1
  const char comp1_arr[] = "FreqSel";
  const int comp1_len = sizeof(comp1_arr) - 1;
  int comp1_pos = 0;

// Compare String 2
  const char* comp2_arr = (settings.sensor_type == s_WS85) ? "WS85" : "WH80";
  const int comp2_len = 4;
  int comp2_pos = 0;

  //uint32_t micros_start = micros();


  while(micros()- last_data < wait_budget){
    while (SENSOR_UART.available()){
      //led_error(1); // blink LED, for debugging
      const char c = (char)SENSOR_UART.read();
      //DEBUGSER.write(c);
      if(c >= 127){
        continue; // skip garbage
      }

      if(c == '=') {eq_count++;} else {eq_count =0;}

      if(c == comp1_arr[comp1_pos]) {comp1_pos++;}
      else {comp1_pos = (c == comp1_arr[0]) ? 1 : 0;}

      if(c == comp2_arr[comp2_pos]) {comp2_pos++;}
      else {comp2_pos = (c == comp2_arr[0]) ? 1 : 0;}

      if((comp1_pos == comp1_len) || (comp2_pos == comp2_len)){ // if we found pattern, increase wait time to get full block
        found_data = true;
        if(settings.sensor_type == s_WS80){ wait_budget = 500;}
        else if(settings.sensor_type == s_WS85){ wait_budget = 3800;}
      }

      last_data = micros();

      if(found_data && eq_count > 35) { // block ends with 37x =, if detected, lower wait time
        wait_budget = 1;
      }

      if(!found_data){
        continue;
      }

      if(c == '\r'){
        continue;
      }

      if(c == '\n'){
        if(line_len > 0){
          if(usb_connected && settings.verbose_usb){
            Serial.write(line_buffer, line_len);
          }
          if(process_line(line_buffer, line_len, &set_value)){
            serial_wait = 120; // decrease value if one valid measuremnt was fount to dertimne if it is ws80 or ws85
            parsed_complete_block = true;
            return RESP_COMPLETE;
          }
          line_len = 0;
        }
        continue;
      }

      if(line_len < (WSXX_LINE_BUFFER_SIZE - 1)){
        line_buffer[line_len++] = c;
      } else {
        // Drop overlong line to avoid carrying stale partial data.
        line_len = 0;
      }
    }
    //led_error(0);
  }

  // Do not carry a very short timeout into the next wake cycle.
  // If we did not complete a block, fall back to a safe default window.
  if(!parsed_complete_block){
    serial_wait = 1200;
  }
  return RESP_OK;
}


#ifdef WS85_TRACE
// =============================================================================
// WS85_TRACE - debug firmware for the WS85 UART wake path
// =============================================================================
//
// The WS85 sends one 33 byte Modbus frame (85 03 1C, 28 payload bytes, CRC)
// every 8.3 s at 115200 baud. With SENSOR_WS85_UART the station sleeps between
// frames, and the start bit of the next frame wakes it: enable_sercom0_int()
// before the sleep, then disable_sercom0_int(), WS85WindSensor::resync() and
// the parser after it. Bytes can go missing anywhere on that way. This build
// shows where, and whether the parser still gets a measurement out of what is
// left. It was written to find the faults now fixed in disable_sercom0_int()
// (receive interrupt switched off, baud register in the wrong format), in
// wdt_enable() (bus stall in the middle of a frame) and in
// WS85WindSensor::take_frame() (lost and garbled leading bytes).
//
// Build with -DWS85_TRACE added to the build flags, for a single build e.g.
//   PLATFORMIO_BUILD_FLAGS=-DWS85_TRACE pio run
// Everything is behind the flag; a build without it does not change. The
// station needs SENSOR_WS85_UART=1, and DEBUG=1 for the log output.
//
// Not for the field, and not for current measurements: the reference capture
// keeps the station awake for 25 s after every boot, the SERCOM0 interrupt
// takes a timestamp per byte, and two extra log lines per frame lengthen every
// wake.
//
// Three kinds of line, the examples recorded on the rig:
//
// 1) Once after boot, ws85_reference_capture(): the station stays awake with the
//    UART on its fast clock, so nothing is lost. This is what the sensor really
//    sends, to hold the wake path against.
//
//      W85 ref #3 t=18535 gap=8337 len=33 crc=ok 85 03 1C 00 85 00 04 00 85 00 ...
//
//    #n     burst number; a pause of more than 5 ms ends a burst
//    t      ms since the capture started
//    gap    ms since the previous burst, about 8335 from frame to frame
//    len    bytes in the burst
//    crc    ok, bad, or n/a for a fragment that does not start like a frame -
//           the first burst after power-up can be one
//
// 2) Per wake cycle, ws85_trace_report(): what reached the parser. Printed only
//    for cycles in which the sensor UART saw anything.
//
//      W85 q=0 src=UART fl=01 st=0000 isr=2/31/0 buf=0 seen=31[1C 00 85 00 04 ...]
//          t1=821 t2=3034 frame=put2 res=ok
//
//    q      wake cycles without any UART activity since the previous line
//    src    wakeup source: UART for the frame's start bit, RTC for the timer
//    fl st  SERCOM0 INTFLAG and STATUS as the wakeup found them
//    isr    SERCOM0 interrupts since the UART was armed: start of frame /
//           byte received / error
//    buf    bytes already buffered when resync() ran
//    seen   bytes the parser read: the count, then the first 40 in hex
//    t1 t2  us from the wakeup to the first and to the last of them
//    frame  how take_frame() got the frame out of the burst: put<n> head bytes
//           put in front, rep<n> received bytes overwritten with a different
//           head byte, drop<n> bytes dropped in front; none if no frame came out
//    res    ok     a measurement came out
//           short  too few bytes, even with the lost leading bytes put back
//           crc    CRC mismatch
//           none   nothing parsed, or parseResponse() refused the frame for
//                  another reason
//
//    A frame that wakes the station loses its first 2 to 4 bytes - here 85 03
//    in front of 1C, frame=put2 - and take_frame() puts them back. So seen=29
//    to 31 with res=ok is the healthy case, and so is seen=32[E0 1C 00 85 ...]
//    with frame=put1,rep1: a byte decoded out of step in front of the rest,
//    overwritten with the head.
//
// 3) The timing of the same wake cycle, and the receiver's registers:
//
//      W85 time dis=109 rst=113 tc4=803 wdt=807 lora=810 poll=812 rx0=301 n=31
//          gaps=[] A=40312006 B=00030000 baud=001A gen=0 en=1 ie=84
//
//    dis rst tc4 wdt lora
//           us from the wakeup until disable_sercom0_int(), resync(),
//           elapsed_time_counter(), wdt_enable() and
//           service_lora_during_sleep() were done; wdt is stale with WDT=0
//    poll   us until read_ws85_uart() started
//    rx0    us until the first byte interrupt
//    n      bytes with an arrival time, at most WS85_TRACE_RX_TIMES
//    gaps   i@us for every pause of more than 200 us in front of byte i. A byte
//           takes 87 us at 115200 baud and the sensor sends a frame in one go,
//           so a pause inside a frame means the CPU was held. The watchdog
//           stall looked like this: gaps=[6@795] in every frame, lined up
//           with the wdt_enable() call between the tc4 and wdt marks.
//    A B    SERCOM0 CTRLA and CTRLB after resync(). The values above are
//           healthy: RUNSTDBY and SFDE clear, RXEN and TXEN set.
//    baud   BAUD register. 001A is 115200 at 48 MHz in the fractional format
//           the Arduino core uses.
//    gen en generator and enable of the SERCOM0 core clock; healthy gen=0
//           (48 MHz) and en=1
//    ie     INTENSET; healthy 84, RXC and ERROR. 00 means received bytes never
//           reach the buffer.
// =============================================================================
extern const char* wakeup_source_string[];
void read_serial_cmd();

// "XX XX ..." into out, as much as fits, always terminated.
static int w85_hex(char *out, size_t len, const uint8_t *b, uint16_t n){
  int k = 0;
  if(len){ out[0] = 0; }
  for(uint16_t i = 0; i < n && (size_t)k + 4 < len; i++){
    k += snprintf(out + k, len - k, i ? " %02X" : "%02X", (unsigned)b[i]);
  }
  return k;
}

// Modbus CRC-16, as the WS85 frame uses it.
static uint16_t w85_crc16(const uint8_t *d, uint16_t n){
  uint16_t crc = 0xFFFF;
  for(uint16_t i = 0; i < n; i++){
    crc ^= d[i];
    for(uint8_t b = 0; b < 8; b++){ crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1); }
  }
  return crc;
}

// Lines 2) and 3) above. read_ws85_uart() calls this once per wake cycle: with
// "ok" when a measurement came out, with the parser's verdict otherwise. The
// per-cycle fields are cleared afterwards.
static void ws85_trace_report(const char *result){
  static uint32_t quiet_cycles = 0;
  WS85WindSensor &w = ws85uart;
  const bool quiet = !sercom0_rxs_count && !sercom0_rxc_count && !sercom0_err_count &&
                     !w.tr_buffered && !w.tr_seen_total;
  if(quiet){
    quiet_cycles++;
  } else {
    static char line[400];
    char seen[130];
    w85_hex(seen, sizeof(seen), w.tr_seen, w.tr_seen_n);
    char frame[24] = "none";
    if(w.tr_framed){
      int k = snprintf(frame, sizeof(frame), "put%u", (unsigned)w.tr_put);
      if(w.tr_replaced){ k += snprintf(frame + k, sizeof(frame) - k, ",rep%u", (unsigned)w.tr_replaced); }
      if(w.tr_dropped){ snprintf(frame + k, sizeof(frame) - k, ",drop%u", (unsigned)w.tr_dropped); }
    }
    const uint8_t src = w.tr_wake_src < 8 ? w.tr_wake_src : 0; // wakeup_source_string[] has 8 entries
    snprintf(line, sizeof(line),
             "W85 q=%lu src=%s fl=%02X st=%04X isr=%lu/%lu/%lu buf=%u seen=%u[%s] t1=%lu t2=%lu frame=%s res=%s",
             (unsigned long)quiet_cycles, wakeup_source_string[src], w.tr_flag, w.tr_status,
             (unsigned long)sercom0_rxs_count, (unsigned long)sercom0_rxc_count,
             (unsigned long)sercom0_err_count, w.tr_buffered, w.tr_seen_total, seen,
             (unsigned long)(w.tr_first_us ? w.tr_first_us - w.tr_wake_us : 0),
             (unsigned long)(w.tr_last_us ? w.tr_last_us - w.tr_wake_us : 0), frame, result);
    log_i(line);
    log_i("\r\n");

    static char tline[420];
    char gaps[140];
    int g = 0;
    gaps[0] = 0;
    const uint8_t rn = sercom0_rx_n;
    for(uint8_t i = 1; i < rn && g + 16 < (int)sizeof(gaps); i++){
      const uint32_t d = sercom0_rx_us[i] - sercom0_rx_us[i - 1];
      if(d > 200UL){
        g += snprintf(gaps + g, sizeof(gaps) - g, "%s%u@%lu", g ? "," : "", (unsigned)i, (unsigned long)d);
      }
    }
    const uint32_t w0 = w.tr_wake_us;
    snprintf(tline, sizeof(tline),
             "W85 time dis=%lu rst=%lu tc4=%lu wdt=%lu lora=%lu poll=%lu rx0=%lu n=%u gaps=[%s] A=%08lX B=%08lX baud=%04X gen=%u en=%u ie=%02X",
             (unsigned long)(w.tr_disable_us - w0), (unsigned long)(w.tr_restart_us - w0), (unsigned long)(w.tr_tc4_us - w0), (unsigned long)(w.tr_wdt_us - w0), (unsigned long)(w.tr_lora_us - w0),
             (unsigned long)(w.tr_poll_us - w0), (unsigned long)(rn ? sercom0_rx_us[0] - w0 : 0),
             (unsigned)rn, gaps, (unsigned long)w.tr_ctrla, (unsigned long)w.tr_ctrlb,
             (unsigned)w.tr_baud, (unsigned)w.tr_gen, (unsigned)w.tr_clken, (unsigned)w.tr_ie);
    log_i(tline);
    log_i("\r\n");
    quiet_cycles = 0;
  }
  w.tr_reset();
}

// Line 1) above, called from setup() right after the sensor baud is set. Reads
// Serial1 - SERCOM0, the sensor UART - with the station awake for duration_ms,
// and prints every burst once it is over. USB and the consoles stay serviced,
// so the station still enumerates and still takes BOOTLOADER=1 meanwhile.
void ws85_reference_capture(uint32_t duration_ms){
  uint8_t burst[80];
  uint16_t n = 0;
  uint32_t last_us = 0, prev_ms = 0, count = 0;
  const uint32_t start = millis();
  static char line[420];
  log_i("W85 reference capture: station awake, UART on the fast clock\r\n");
  while(millis() - start < duration_ms){
    wdt_reset();
    yield(); // keeps TinyUSB serviced: without it the station does not enumerate during the capture
    read_serial_cmd(); // so a BOOTLOADER=1 sent during the capture is acted on at once
    while(Serial1.available()){
      const int b = Serial1.read();
      if(n < sizeof(burst)){ burst[n] = (uint8_t)b; }
      n++;
      last_us = micros();
    }
    if(n && (micros() - last_us) > 5000UL){
      const uint32_t now = millis();
      const uint16_t stored = n < sizeof(burst) ? n : sizeof(burst);
      // The CRC is checked on the whole frame. A burst that lost only the
      // address and function code gets them back first, the way the parser
      // used to handle it.
      uint8_t frame[84];
      uint16_t flen = 0;
      if(stored >= 2 && burst[0] == 0x85 && burst[1] == 0x03){
        memcpy(frame, burst, stored);
        flen = stored;
      } else if(stored >= 1 && burst[0] == 0x1C){
        frame[0] = 0x85;
        frame[1] = 0x03;
        memcpy(frame + 2, burst, stored);
        flen = stored + 2;
      }
      const char *crc = "n/a";
      if(flen >= 5){
        const uint16_t rcrc = frame[flen - 2] | (frame[flen - 1] << 8);
        crc = (w85_crc16(frame, flen - 2) == rcrc) ? "ok" : "bad";
      }
      const int k = snprintf(line, sizeof(line), "W85 ref #%lu t=%lu gap=%lu len=%u crc=%s ",
                             (unsigned long)++count, (unsigned long)(now - start),
                             (unsigned long)(prev_ms ? now - prev_ms : 0), n, crc);
      w85_hex(line + k, sizeof(line) - k, burst, stored);
      log_i(line);
      log_i("\r\n");
      prev_ms = now;
      n = 0;
    }
  }
  log_i("W85 reference capture done\r\n");
}
#endif

// read UART and process input buffer, needs to be called periodically until new block is complete (last_ws80_data = time())
// RESP_COMPLETE 2: data complete
// RESP_ERROR 1: error
// RESP_OK 0: data ok, continue
int read_ws85_uart(){
#ifdef WS85_TRACE
  ws85uart.tr_poll_us = micros(); // WS85_TRACE: "poll" in the timing line
#endif
  static uint32_t serial_wait = 500;
  static int errorcount = 0;
  //uint32_t micros_start = micros();

  ws85uart.lastByte = micros();
  while(micros()- ws85uart.lastByte < (serial_wait+100)){
    ws85uart.poll(serial_wait);
    if(ws85uart.available()){
      WS85Measurement w85m = ws85uart.get();

      if(settings.verbose_usb && usb_connected){
        ws85uart.printWS85Measurement(w85m, Serial);
      } else {
        log_d("Wind: ", w85m.windSpeed);
        log_d("Dir: " , w85m.windDirection);
      }

      add_wind_history_dir(w85m.windDirection);
      add_wind_history_wind(w85m.windSpeed);
      add_wind_history_gust(w85m.gustSpeed);
      sensor.temperature = w85m.temperatureC;
      sensor.last_data = time();

      //log_i("WS85_UART data ok\r\n");
      errorcount = 0;
#ifdef WS85_TRACE
      ws85_trace_report("ok");
#endif
      return RESP_COMPLETE;
    }
  }
  errorcount++;
  if(errorcount == 20){ // try a single baud resync after repeated failed reads
    log_w("Trying WS85 baud sync\r\n");
    ws85uart.set_baud_115200();
  }
  if(errorcount > 1000){
    led_error(1);
  }
    //led_error(0);
   // ws85uart.requestAutoSendInterval();
   // delay(10);
   // ws85uart.set_baud_115200();
#ifdef WS85_TRACE
  ws85_trace_report(ws85uart.tr_result);
#endif
  return RESP_OK;
}

int read_windnerd(){
  // A frame is exactly 4 bytes: speed, dir_hi, dir_lo, checksum.
  // Keep a 4-byte sliding window holding the most recent bytes. The previous
  // ring buffer reset its index to 0 and then incremented it, so after a wrap
  // it restarted at index 1 and the "last 4 bytes" slice picked up a stale byte.
  constexpr int WINDNERD_FRAME_LEN = 4;
  static uint8_t frame[WINDNERD_FRAME_LEN];
  static int filled = 0;

  const uint32_t serial_wait = 2000;
  uint32_t lastByte = micros();

  while( (micros()- lastByte) < serial_wait){

    while(SENSOR_UART.available()){
      const uint8_t c = (uint8_t)SENSOR_UART.read();
      lastByte = micros();

      if(filled < WINDNERD_FRAME_LEN){
        frame[filled++] = c;
      } else { // window full: shift the oldest byte out
        frame[0] = frame[1];
        frame[1] = frame[2];
        frame[2] = frame[3];
        frame[3] = c;
      }
    }
  }

  // Must have a full frame to parse
  if (filled < WINDNERD_FRAME_LEN) return RESP_OK;

  const uint8_t b0 = frame[0];     // speed
  const uint8_t b1 = frame[1];     // dir high
  const uint8_t b2 = frame[2];     // dir low
  const uint8_t cs = frame[3];     // checksum received

  const uint8_t calc_cs = b0 ^ b1 ^ b2;   // compute checksum

  filled = 0; // drop the frame either way, same as the previous implementation

  if (cs == calc_cs) {
    int wind = b0;
    add_wind_history_wind(wind);
    int direction = ((uint16_t)b1 << 8) | b2;
    add_wind_history_dir(direction);

    log_d("Wind: ", wind);
    log_d("Direction: ", direction);
    return RESP_COMPLETE;
  }

  log_w("Data invalid\r\n");
  return RESP_OK;
}
  

// Davis 6410 Sensor ----------------------------------------------------------------------------------------------------------------------
int read_wind_dir(){
  int val = 0;
  analogReference(AR_DEFAULT); //3.3V refernece
  pinMode(PIN_DAVIS_DIR, INPUT);
  pinMode(PIN_DAVIS_POWER,OUTPUT);
  digitalWrite(PIN_DAVIS_POWER,1);
  switch_sensor_power(1);

  if(settings.use_protection_board){
    delayMicroseconds(420); // allow protection board to switch on powerswitch SY6280AAC. takes 160us + timeconstant of 4.7k + 10nF RC filter (220us)
  }

  //delayMicroseconds(1); // settlement time

  constexpr int N_DIR = 4;
  int samples[N_DIR];
  int32_t sum = 0;
  for (int i = 0; i < N_DIR; i++) {
    samples[i] = analogRead(PIN_DAVIS_DIR);
    sum += samples[i];
  }
  int d = (int)(sum / N_DIR);

 // ... other analog sensors
digitalWrite(PIN_DAVIS_POWER,0);
pinDisable(PIN_DAVIS_POWER);
switch_sensor_power(0);
pinDisable(PIN_DAVIS_DIR);

if(settings.sensor_type == s_DAVIS6410){
  // Variable resistance 0 - 20KΩ; 10KΩ = south, 180°)
  val = (int)(360.0f/1023.0f * (float)d);
}

#if 0
  log_d("DIR samples:");
  for (int i = 0; i < N_DIR; i++) { log_d(" ", samples[i]); }
  log_d("\r\n");
#endif


return val;
}


void calc_pulse_sensor(uint32_t pulses, uint32_t dmillis){
  log_d("delta_t: ", dmillis);
  log_d("pulses: ", pulses); 
  sensor.wind_dir_raw = read_wind_dir();
  add_wind_history_dir(sensor.wind_dir_raw);
  
  
  if(settings.sensor_type == s_DAVIS6410){
    sensor.wind_speed = (float) pulses * 1.609f * (2250.0f/((float)dmillis+1) ); // avoid div/0
  }
  // ... other analog sensors
  add_wind_history_wind(sensor.wind_speed);
  add_wind_history_gust(sensor.wind_speed);
  log_d("wind speed: ", sensor.wind_speed); 
  log_d("wind dir (raw): ", sensor.wind_dir_raw); 
}


// read status pins of solar charger
void get_solar_charger_state(){
  pinMode(PIN_PV_CHARGE, INPUT_PULLUP);
  pinMode(PIN_PV_DONE, INPUT_PULLUP);
  sensor.pv_done = !digitalRead(PIN_PV_DONE);
  sensor.pv_charging = !digitalRead(PIN_PV_CHARGE);
  pinDisable(PIN_PV_CHARGE);
  pinDisable(PIN_PV_DONE);
}

// GPS ----------------------------------------------------------------------------------------------------------------------
// Position used for transmission and for the baro altitude correction: the GPS
// fix when GPS is enabled and has produced one, the configured position
// otherwise.  read_gps() used to overwrite settings.pos_* directly, so a
// measurement silently rewrote the user's configuration.
float position_lat(){
  return (settings.use_gps && sensor.last_gps_valid) ? sensor.gps_lat : settings.pos_lat;
}
float position_lon(){
  return (settings.use_gps && sensor.last_gps_valid) ? sensor.gps_lon : settings.pos_lon;
}
float position_alt(){
  return (settings.use_gps && sensor.last_gps_valid) ? sensor.gps_alt : settings.altitude;
}

void read_gps(){
#if BREEZEDUDE_ENABLE_GPS
  while (GPS_SERIAL.available() > 0){
    uint8_t c = GPS_SERIAL.read();
    //DEBUGSER.print(c);
    tinyGps.encode(c);
    if (tinyGps.location.isUpdated() && tinyGps.location.isValid() ){
      sensor.gps_lat = tinyGps.location.lat();
      sensor.gps_lon = tinyGps.location.lng();
      sensor.gps_alt = tinyGps.altitude.meters();
      sensor.last_gps_valid = time();
      //DEBUGSER.println("Position Update");
    }
  }
#else
  settings.use_gps = false;
#endif
}