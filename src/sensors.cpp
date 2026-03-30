#include "sensors.h"


TinyGPSPlus tinyGps;


// I2C Barometer
BMP280 bmp280; // Bosch BMP280
SPL06 spl; // Goertek SPL06-001
HP203B hp; // HP203B 0x76 or 0x77
Adafruit_BMP3XX bmp3xx;
QMC5883P qmc5883p; // GY-271
// DPS310 as alternative?


CHIP_BARO chip_baro = BARO_NONE;
CHIP_IMU chip_imu = IMU_NONE;

Sensors sensor;

static float apply_altitude_correction(float pressure_hpa, float temp_c, float altitude_m) {
  // Cache expensive powf() result for the last altitude + quantized temperature.
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
      const float base = 1.0f - (lapse_times_alt / denom);
      if (base > 0.0f) {
        last_factor = powf(base, -5.257f);
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

    //if(bmp280.begin()){ //0x76
    //   chip_baro = BARO_BMP280;
    //   log_i("Baro: BMP280\r\n");
    //   bmp280.setOversampling(4);
    // return true;
    // }

    if(spl.begin(0x77)){
      chip_baro = BARO_SPL06;
      log_i("Baro: SPL06\r\n");
      return true;
    }

    //if(bmp3xx.begin_I2C(0x76)){
    //  chip_baro = BARO_BMP3xx;
    //  log_i("Baro: BMP3XX\r\n");
    // return true;
    //}

    if(hp.begin(0x76, OSR_2048)){
        chip_baro = BARO_HP203B;
        hp.setOSR(OSR_512); // 16.4ms conversion time
        log_i("Baro: HP203B\r\n");
        return true;
    }
 
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
    if(chip_baro == BARO_BMP280){ 
      sensor.next_baro_reading = now + bmp280.startMeasurment();
    }
    if(chip_baro == BARO_SPL06){ 
      spl.start_measure();
      sensor.next_baro_reading = now + 27;
    }
    if(chip_baro == BARO_BMP3xx){ 
      bmp3xx.setOutputDataRate(BMP3_ODR_50_HZ);
    // bmp3xx.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    // bmp3xx.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    // bmp3xx.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp3xx.performReading();
      sensor.next_baro_reading = now + 5;
    }
    if(chip_baro == BARO_HP203B){ 
      hp.startMeasure();
      sensor.next_baro_reading = now + 25; // OSR512
    }
    } else {
      if(now > (sensor.next_baro_reading +200)){
        sensor.next_baro_reading = 0;
      }
    }
}

// read value of baro, needs baro_start_reading in advance
void read_baro(){
  bool data_ok = false;
  double T,P;
  uint32_t now = time();

  if(chip_baro == BARO_BMP280){
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
      uint8_t result = bmp280.getTemperatureAndPressure(T,P);
      if(result!=0){
        data_ok = true;
      }
    } else {return;}
  }

  else if(chip_baro == BARO_SPL06){
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
      P = spl.get_pressure();
      T = spl.get_temp_c();
      spl.sleep(); 
      
      if(P > 0){
        data_ok = true;
      }
    }  else {return;}
  }
  else if(chip_baro == BARO_BMP3xx){ 
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
    P = bmp3xx.pressure/100;
    T = bmp3xx.temperature;
    bmp3xx.setOutputDataRate(BMP3_ODR_0_001_HZ);
    if(P > 0){data_ok = true;}
    } else {return;}
  }
  else if(chip_baro == BARO_HP203B){ 
    if(sensor.next_baro_reading && (now > sensor.next_baro_reading)){
    hp.Measure_Pressure();
    hp.Measure_Temperature();
    hp.startMeasure();
    P = hp.hp_sensorData.P;
    T = hp.hp_sensorData.T;
    if(P > 0){data_ok = true;}
    } else {return;}
  }


  if(data_ok){
    sensor.baro_temp = T;
    sensor.next_baro_reading = 0;
    sensor.last_baro_reading = now;
    sensor.baro_pressure = apply_altitude_correction((float)P, (float)T, settings.altitude);
    log_v("Baro: ", sensor.baro_pressure);
    log_v("Baro Temp: ", sensor.baro_temp);
  } else {
    log_v("Baro no data, retry\r\n");
    //baro_start_reading();
  }
}

// while USB is connected, forward ws80 data to usb serial port
void forward_sensor_serial(){
  if(settings.forward_serial_while_usb){
    static uint8_t buffer[512];
    static int bufferpos = 0;

    if(is_wsxx()){
      while (SENSOR_UART.available()){
        if (bufferpos >= (int)sizeof(buffer)) {
          bufferpos = 0;
        }
        buffer[bufferpos] = SENSOR_UART.read();
        if (buffer[bufferpos] == '\n') {
          bufferpos++;
          if(usb_connected){Serial.write(buffer, bufferpos);}
          Serial1.write(buffer, bufferpos);
          bufferpos = 0;
        } else {
          bufferpos++;
        }
      }
    } else if(settings.sensor_type == s_WS85_UART || settings.sensor_type == s_WINDNERD){
      // Read data block first
      while((bufferpos < (int)sizeof(buffer)) && ws85uart.get_char(&(buffer[bufferpos]))){
        bufferpos++;
        delay(1); // waste some time to wait for the serial data to complete receiving
      }
      // Output the data
      if(bufferpos){
        if(usb_connected){
          char line[3 * 64 + 1];
          int i = 0;
          while (i < bufferpos) {
            int chunk = min(64, bufferpos - i);
            int off = 0;
            for (int j = 0; j < chunk; j++) {
              off += snprintf(&line[off], sizeof(line) - off, "%02X ", buffer[i + j]);
            }
            Serial.println(line);
            i += chunk;
          }
        }
        bufferpos = 0;
      }
    }
  }
}


// Set measurement value read from WS80 UART
bool set_value(char* key,  char* value){
  //printf("%s = %s\r\n",key, value);

  if(strcmp(key,"WindDir")==0) {sensor.wind_dir_raw = atoi(value); add_wind_history_dir(sensor.wind_dir_raw); return false;}
  if(strcmp(key,"WindSpeed")==0) {sensor.wind_speed = atof(value)*3.6; add_wind_history_wind(sensor.wind_speed); printf("%s = %0.2f\r\n",key, sensor.wind_speed); return false;}
  if(strcmp(key,"WindGust")==0) {sensor.wind_gust = atof(value)*3.6; add_wind_history_gust(sensor.wind_gust); printf("%s = %0.2f\r\n",key, sensor.wind_gust); return false;}
  if(strcmp(key,"Temperature")==0) {sensor.temperature = atof(value); if(settings.sensor_type != s_WS80){settings.sensor_type = s_WS80; log_i("Detected WS80\n");} return false;} // WS80 only - autodetection
  if(strcmp(key,"GXTS04Temp")==0) {sensor.temperature = atof(value);  if(settings.sensor_type != s_WS85){settings.sensor_type = s_WS85; log_i("Detected WS85\n");} return false;} // WS85 only
  if(strcmp(key,"Humi")==0) {sensor.humidity = atoi(value); return false;}
  if(strcmp(key,"Light")==0) {sensor.light_lux = atoi(value); return false;}
  if(strcmp(key,"UV_Value")==0) {sensor.uv_level = atof(value); return false;}
  if(strcmp(key,"CapVoltage")==0) {sensor.cap_voltage = atof(value); return false;} // WS85
  if(strcmp(key,"BatVoltage")==0) {
    sensor.wsxx_vcc = atof(value); 
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
            printf("%s\n", token);
            num =1;
          }
        }
        else if(num == 1){
          sensor.wind_speed = atof(token);
          num =2;
        }
        else if(num == 2){
          sensor.wind_gust = atof(token);
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
  constexpr int WSXX_BUFFER_SIZE = 1024; // size of linebuffer
  static char buffer[WSXX_BUFFER_SIZE];
  int co = 0;
  bool found_data = false;
  int eq_count = 0;
  uint32_t last_data = micros();
  static uint32_t serial_wait = 3800;

// Compare String 1
  const char comp1_arr[] = "FreqSel";
  const int comp1_len = sizeof(comp1_arr) - 1;
  int comp1_pos = 0;

// Compare String 2
  const char* comp2_arr = (settings.sensor_type == s_WS85) ? "WS85" : "WH80";
  const int comp2_len = 4;
  int comp2_pos = 0;

  //uint32_t micros_start = micros();


  while(micros()- last_data < serial_wait){
    while (SENSOR_UART.available()){
      //led_error(1); // blink LED, for debugging
      buffer[co] = SENSOR_UART.read();
      //DEBUGSER.write(buffer[co]);
      if(buffer[co] < 127){ // skip garbage
        if(buffer[co] == '=') {eq_count++;} else {eq_count =0;}

        if(buffer[co] == comp1_arr[comp1_pos]) {comp1_pos++;}
        else {comp1_pos=0;}

        if(buffer[co] == comp2_arr[comp2_pos]) {comp2_pos++;}
        else {comp2_pos=0;}

        if((comp1_pos == comp1_len) ||(comp2_pos == comp2_len)){ // if we found or pattern, increase wait time to get full block
          found_data = true;
          if(settings.sensor_type == s_WS80){ serial_wait = 500;}
          else if(settings.sensor_type == s_WS85){ serial_wait = 3800;}
        }

        last_data = micros();
        co++;


        if(found_data && eq_count > 35) { // block ends with 37x =, if detected, lower wait time
          serial_wait = 1;
        }
        
        if(co >= WSXX_BUFFER_SIZE){
          log_e("Buffer size exeeded\r\n");
          co = 0;
          //led_error(0);
          return RESP_ERROR;
        }
      }
    }
    //led_error(0);
  }

// now parse buffer content
if(found_data){
  int i =0;
  int pos = 0;
  while (i < co){
    if(buffer[i] == '\n'){
          if(settings.test_with_usb && usb_connected){
            Serial.write(&buffer[pos], i-pos);
          }
          if(process_line(&buffer[pos], i-pos-1, &set_value)){
            serial_wait = 120; // decrease value if one valid measuremnt was fount to dertimne if it is ws80 or ws85
            return RESP_COMPLETE;
          }
      pos = i+1;
    }
    i++;

  }
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
  if(errorcount % 4 == 0){ // if we get too many bad reading, maybe the baud rate does not match
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

int d = analogRead(PIN_DAVIS_DIR);


if(settings.sensor_type == s_DAVIS6410){
  // Variable resistance 0 - 20KΩ; 10KΩ = south, 180°)
  val = (int)(360.0/1023.0 * (float)d);
}
 // ... other analog sensors
digitalWrite(PIN_DAVIS_POWER,0);
pinDisable(PIN_DAVIS_POWER);
pinDisable(PIN_DAVIS_DIR);
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
  log_i("wind dir raw: ", sensor.wind_dir_raw); 
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
}