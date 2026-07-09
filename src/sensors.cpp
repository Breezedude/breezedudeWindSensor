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

void switch_sensor_power(bool state){
  static bool current_power_state = false;
  const bool uses_uart = (is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD);
  if(state && !current_power_state){ // turn on
    current_power_state = true;
    pinMode(PIN_PS_WS,OUTPUT);
    digitalWrite(PIN_PS_WS, 0);
    if(uses_uart){
      // Re-connect UART pins to SERCOM0 peripheral before the sensor can respond
      pinPeripheral(PIN_RX, PIO_SERCOM);
      pinPeripheral(PIN_TX, PIO_SERCOM);
      SENSOR_UART.begin(115200 * div_cpu);
      log_i("WS sensor power ON\r\n");
    }
  }
  if(!state && current_power_state){ // turn off
    current_power_state = false;
    digitalWrite(PIN_PS_WS, 1);
    if(uses_uart){
      // Disable UART pins so the idle-HIGH TX line cannot leak current into the
      // sensor through its input protection diodes when its VCC is removed.
      pinDisable(PIN_TX); // PA10 - UART idles HIGH, would source current into sensor
      pinDisable(PIN_RX); // PA11 - symmetric isolation
      log_i("WS sensor power OFF\r\n");
    }
  }
}

static uint8_t voltageToSOCNonLinear(float v) {
  if(v < 0.8f) {led_error(1); log_i("V_Batt read error: ", v); return 0;} // bad reading
  // Deep UV: 3 consecutive readings < 2.9V trigger extended sleep for solar charging recovery.
  // Hysteresis: clears when voltage rises above 3.0V.
  {
    static uint8_t deep_uv_count = 0;
    if (v < 2.9f) {
      if (deep_uv_count < 3) ++deep_uv_count;
      if (deep_uv_count >= 3) deep_undervoltage = true;
    } else if (v > 3.0f) {
      deep_uv_count = 0;
      deep_undervoltage = false;
    }
  }
  if(v < 3.15f) {switch_sensor_power(0); settings.uv_triggered = true; undervoltage = true; return 0;} // undervoltage, turn off sensor to save power
  if(v > 3.3f) {if(is_wsxx() || settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD){
                  switch_sensor_power(1);
                } undervoltage = false;}
  if(v > 4.15f){settings.uv_triggered = false; return 100;}

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
    pinMode(PIN_V_READ, INPUT);

    // Extend ADC sampling phase via SAMPCTRL instead of software delays between reads.
    // ADC clock = 48 MHz / 512 ~= 94 kHz  ->  half-cycle ~= 5.3 us.
    // SAMPLEN=4 adds 5 half-cycles ~= 26 us extra sample time per conversion.
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
    log_e("IMU not found");
    return false;
}

void get_imu(){
  if(chip_imu == IMU_){

    }
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

    log_e("Baro: not found\r\n");
    settings.use_baro = false;
    return false; 
}


// request baro sampling & conversion
void baro_start_reading(){
  //sercom3.resetWIRE();
  //Wire.begin();
  uint32_t now = time();
  if(!sensor.next_baro_reading){
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
  } else {
    if(now > (sensor.next_baro_reading + 200)){
      sensor.next_baro_reading = 0;
    }
  }
}

// read value of baro, needs baro_start_reading in advance
void read_baro(){
  bool data_ok = false;
  double T = 0, P = 0;
  uint32_t now = time();

  #if BREEZEDUDE_BARO_BMP280
  if(chip_baro == BARO_BMP280){
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
      uint8_t result = bmp280.getTemperatureAndPressure(T, P);
      if(result != 0){
        data_ok = true;
      }
    } else { return; }
  }
  #endif

  #if BREEZEDUDE_BARO_SPL06
  if(chip_baro == BARO_SPL06){
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
      P = spl.get_pressure();
      T = spl.get_temp_c();
      spl.sleep();
      if(P > 0){
        data_ok = true;
      }
    } else { return; }
  }
  #endif

  #if BREEZEDUDE_BARO_BMP3XX
  if(chip_baro == BARO_BMP3xx){
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
      P = bmp3xx.pressure / 100;
      T = bmp3xx.temperature;
      bmp3xx.setOutputDataRate(BMP3_ODR_0_001_HZ);
      if(P > 0){ data_ok = true; }
    } else { return; }
  }
  #endif

  #if BREEZEDUDE_BARO_HP203B
  if(chip_baro == BARO_HP203B){
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
      hp.Measure_Pressure();
      hp.Measure_Temperature();
      hp.startMeasure();
      P = hp.hp_sensorData.P;
      T = hp.hp_sensorData.T;
      if(P > 0){ data_ok = true; }
    } else { return; }
  }
  #endif

  if(data_ok){
    sensor.baro_temp = T;
    sensor.next_baro_reading = 0;
    sensor.last_baro_reading = now;
    sensor.baro_pressure = apply_altitude_correction((float)P, (float)T, settings.altitude);
    log_v("Baro: ", sensor.baro_pressure);
    log_v("Baro Temp: ", sensor.baro_temp);
  } else {
    //log_v("Baro no data, retry\r\n");
    //baro_start_reading();
  }
}

static volatile bool davis_speed_pulse = false;
static void davis_speed_isr() { davis_speed_pulse = true; }

void forward_analog_test_serial(){
  static bool isr_attached = false;

  if(settings.analog_test_mode){
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
      log_i(" Speed/Dir state: ");
      log_i(" ");
      log_i(pulse ? "-" : "o");
      log_i("\t");
      log_write(d);
      log_i("\r\n");
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
    
    log_i("WSXX data complete\r\n");
    return true;
  }

  //log_i(" ->not_found\n");
  return false;
}


bool parse_wsdat(char* input, int len){
    char buffer[20];
    int num=0;
    strncpy(buffer, input, len+1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token = strtok(buffer, ",");
    while (token != NULL) {
        if(num == 0){
          if(strcmp(token, "$WSDAT")==0){
            log_v("$WSDAT\n");
            num =1;
          }
        }
        else if(num == 1){
          sensor.wind_speed = parse_decimal_fast(token);
          num =2;
        }
        else if(num == 2){
          sensor.wind_gust = parse_decimal_fast(token);
          num =3;
        }
        else if(num == 3){
          sensor.wind_dir_raw = atoi(token);
          //printf("WSDAT: %0.2f, %0.2f, %i\n", wind_speed, wind_gust, wind_dir_raw);
          //DEBUGSER.flush();
          return true;
        }
        token = strtok(NULL, ",");
    }
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


// read UART and process input buffer, needs to be called periodically until new block is complete (last_ws80_data = time())
// RESP_COMPLETE 2: data complete
// RESP_ERROR 1: error
// RESP_OK 0: data ok, continue
int read_ws85_uart(){
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
        log_i("Wind: ", w85m.windSpeed);
        log_i("Dir: " , w85m.windDirection);
      }

      add_wind_history_dir(w85m.windDirection);
      add_wind_history_wind(w85m.windSpeed);
      add_wind_history_gust(w85m.gustSpeed);
      sensor.temperature = w85m.temperatureC;
      sensor.last_data = time();

      //log_i("WS85_UART data ok\r\n");
      errorcount = 0;
      return RESP_COMPLETE;
    }
  }
  errorcount++;
  if(errorcount == 20){ // try a single baud resync after repeated failed reads
    log_i("Trying WS85 baud sync\r\n");
    ws85uart.set_baud_115200();
  }
  if(errorcount > 1000){
    led_error(1);
  }
    //led_error(0);
   // ws85uart.requestAutoSendInterval();
   // delay(10);
   // ws85uart.set_baud_115200();
  return RESP_OK;
}

int read_windnerd(){
  constexpr int WINDNERD_BUFFER_SIZE = 10; // size of linebuffer
  static char buffer[WINDNERD_BUFFER_SIZE];
  static int pos = 0;

  const uint32_t serial_wait = 2000;
  uint32_t lastByte = micros();

  while( (micros()- lastByte) < serial_wait){

    while(SENSOR_UART.available()){
      buffer[pos] = SENSOR_UART.read();
      lastByte = micros();
      
      // prevent overflow
      if(pos > WINDNERD_BUFFER_SIZE - 2){
        pos = 0;
      }
      //if(settings.test_with_usb && usb_connected){
      //  Serial.write(&buffer[pos]);
      //}
      pos++;
    }
  }

  // parse buffer
  // Must have at least 4 bytes to parse
  if (pos < 4) return RESP_OK;
  //log_i("Buffer: ", pos);

  // Extract last 4 bytes
  uint8_t b0 = buffer[pos - 4];     // speed
  uint8_t b1 = buffer[pos - 3];     // dir high
  uint8_t b2 = buffer[pos - 2];     // dir low
  uint8_t cs = buffer[pos - 1];     // checksum received

  uint8_t calc_cs = b0 ^ b1 ^ b2;   // compute checksum

  if (cs == calc_cs) {
    int wind = b0;
    add_wind_history_wind(wind);
    int direction = ((uint16_t)b1 << 8) | b2;
    add_wind_history_dir(direction);

    log_i("Wind: ", wind);
    log_i("Direction: ", direction);
    pos = 0;
    return RESP_COMPLETE;

  } else {
    log_i("Data invalid\r\n");
  }
  pos = 0;  
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
  val = (int)(360.0/1023.0 * (float)d);
}

#if 0
  log_i("DIR samples:");
  for (int i = 0; i < N_DIR; i++) { log_i(" ", samples[i]); }
  log_i("\r\n");
#endif


return val;
}


void calc_pulse_sensor(uint32_t pulses, uint32_t dmillis){
  log_i("delta_t: ", dmillis);
  log_i("pulses: ", pulses); 
  sensor.wind_dir_raw = read_wind_dir();
  add_wind_history_dir(sensor.wind_dir_raw);
  
  
  if(settings.sensor_type == s_DAVIS6410){
    sensor.wind_speed = (float) pulses * 1.609 * (2250.0/((float)dmillis+1) ); // avoid div/0
  }
  // ... other analog sensors
  add_wind_history_wind(sensor.wind_speed);
  add_wind_history_gust(sensor.wind_speed);
  save_history(sensor.wind_speed, sensor.temperature, sensor.humidity, sensor.light_lux, sensor.batt_volt, sensor.pv_charging, sensor.pv_done);
  log_i("wind speed: ", sensor.wind_speed); 
  log_i("wind dir (raw): ", sensor.wind_dir_raw); 
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
void read_gps(){
#if BREEZEDUDE_ENABLE_GPS
  while (GPS_SERIAL.available() > 0){
    uint8_t c = GPS_SERIAL.read();
    //DEBUGSER.print(c);
    tinyGps.encode(c);
    if (tinyGps.location.isUpdated() && tinyGps.location.isValid() ){
      settings.pos_lat = tinyGps.location.lat();
      settings.pos_lon = tinyGps.location.lng();
      settings.altitude = tinyGps.altitude.meters();
      sensor.last_gps_valid = time();
      //DEBUGSER.println("Position Update");
    }
  }
#else
  settings.use_gps = false;
#endif
}