#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <BMP280.h>
#include <HP203B.h> // https://github.com/ncdcommunity/Arduino_Library_HP203B_Barometer_Altimeter_Sensor/tree/master
#include <SPL06.h>
#include <Adafruit_BMP3XX.h>
#include <LibPrintf.h>
#include <RadioLib.h>
#include <SdFat.h>
#include <SAMD_InternalFlash.h>
#include <TinyGPS++.h>
#include "wiring_private.h" // pinPeripheral() function

#include "defines.h"

#include "logging.h"
#include "sleep.h"
#include "types.h"
#include "display.h"
#include "msc.h"
#include "hist.h"
#include "tools.h"
#include "ws85uart.h"

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


TinyGPSPlus tinyGps;

// Serial2 for Degugging on HW > 1.3
Uart Serial2(&sercom1, PIN_SERCOM1_RX, PIN_SERCOM1_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0 ) ;

LORA_MODULE lora_module = LORA_NONE;

// I2C Barometer
BMP280 bmp280; // Bosch BMP280
SPL06 spl; // Goertek SPL06-001
HP203B hp; // HP203B 0x76 or 0x77
Adafruit_BMP3XX bmp3xx;
// DPS310 as alternative?

#define MCP4652_I2C_ADDR	0x2F
#define WRITE_WIPER_DCDC	(0b0000<<4) // Wiper 0 = DCDC
#define WRITE_WIPER_MPPT	(0b0001<<4) // Wiper 1 = MPPT
#define WRITE_TCON	(0b0100<<4)
#define CMD_WRITE	(0b00<<2)

#define LORA_SYNCWORD 0xF1 //SX1262: 0xF4 0x14 https://blog.classycode.com/lora-sync-word-compatibility-between-sx127x-and-sx126x-460324d1787a is handled by RadioLib

SX1276 radio_sx1276 = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RESET, PIN_LORA_DIO1);
SX1262 radio_sx1262 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);
LLCC68 radio_llcc68 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);

PhysicalLayer* radio_phy = nullptr;
bool transmittedFlag = false;
bool loraReceivedFlag = false;

// WS85 with UART
WS85WindSensor ws85uart(&Serial1);

// Pulsecounter
volatile uint32_t pulsecount =0; // pulses from wind speed sensor using reed switch

// WDT and CPU Clock
#define WDT_PERIOD 2500 // ms for wdt to wait to reset mcu if not reset in time

int div_cpu = 1; // current div
bool first_sleep = true; // first sleep after reset, USB perephial could still be on
bool usb_connected = false;

uint32_t sleep_allowed = 0; // time() when is is ok so eenter deepsleep
bool reduced_interval = false; // reduced interval active
bool undervoltage = false;

// ### Variables for storing settings from file, may be overewritten #####

#ifdef HAS_HEATER
  bool is_heater = false;
  float mppt_voltage = 5.5;
  float heater_voltage = 4.5;
  uint32_t heater_on_time_cum = 0;
  #define MAX_HEAT_TIME 30*60*1000UL //30 min
#endif
bool use_mcp4652 = true; // used on first version of PCB (<=1.3) to set MPPT and DCDC voltage

BARO_CHIP baro_chip = BARO_NONE;

bool settings_ok = false;
uint32_t next_baro_reading = 0;
uint32_t last_baro_reading = 0;

// Message timing

uint32_t last_msg_weather = 0;
uint32_t last_msg_name = 0;
uint32_t last_msg_info = 0;
uint32_t broadcast_scale_factor = 1; // multiplier for broadcast values. is set to 2..5 if battery is low

uint32_t last_wsxx_data = 0;
uint32_t last_fnet_send = 0; // last package send
uint32_t fanet_cooldown = 4000;
uint32_t loopcounter = 0;

// Measurement values
float baro_pressure = 0;
float baro_temp = 0; // temp from bmp280, inside case/on pcb
int wind_dir_raw = 0;
int wind_heading = 0;
float wind_speed = 0;
float wind_gust = 0;
float temperature = 0;
int humidity = 0;
int light_lux = 0;
float uv_level = 0;
float wsxx_vcc = 0;
float cap_voltage = 0; // WS85 supercap voltage
float batt_volt = 0;
int batt_perc = 0;

int wakeup_source = WAKEUP_NONE;
uint32_t last_gps_valid = 0;

bool pv_charging; // currently charging, state from pv charger
bool pv_done; // battery fully charged, state from pv charger


// read solderjumers once and disable pins again
uint8_t read_id(){
  uint8_t id= 0;
  pinMode(PIN_ID0, INPUT_PULLUP);
  pinMode(PIN_ID1, INPUT_PULLUP);

  id &= digitalRead(PIN_ID0);
  id &= digitalRead(PIN_ID1) << 1;

  pinDisable(PIN_ID0);
  pinDisable(PIN_ID1);
  return id;
}

// process line in 'key=value' format and hand to callback function
bool process_line(char * in, int len, bool (*cb)(char*, char*)){
  #define BUFFLEN 127
  char name [BUFFLEN];
  char value [BUFFLEN];
  memset(name,'\0',BUFFLEN);
  memset(value,'\0',BUFFLEN);
  bool literal = false; // 
  char* ptr = name; //start with name filed
  int c = 0; // counter
  int oc= 0; // output counter
  bool cont = true; //continue flag

  while (cont && (c < max(len,BUFFLEN)) && (oc < (BUFFLEN-1))){ // limit to 255 chars per line
    if(in[c] < 127){
      switch (in[c]) {
        case '\r': if(c != 0) {cont = false;} break;
        case '\n': cont = false; break;
        //case '-': cont = false; break; // catches negative temps
        case '>': cont = false; break;
        case '!':  cont = false; break;
        case '#':  cont = false; break;
        case '?':  literal = true; break; // enable literal mode
        case '=':  if(oc && (ptr == name) ){ *(ptr+oc) = '\0'; ptr = value; oc = 0;} break; // switch to value
        case ' ':  if(!literal) {break;} // avoid removing whitespaces from name
        default:   *(ptr+oc) = in[c]; oc++; break; // copy char
      }
    }
    c++;
  }
  if(oc && (ptr == value)){ // value is set
    return cb(name, value);
  }
  
  return false;
}


// Set measurement value read from WS80 UART
bool set_value(char* key,  char* value){
  //printf("%s = %s\r\n",key, value);

  if(strcmp(key,"WindDir")==0) {wind_dir_raw = atoi(value); add_wind_history_dir(wind_dir_raw); return false;}
  if(strcmp(key,"WindSpeed")==0) {wind_speed = atof(value)*3.6; add_wind_history_wind(wind_speed); printf("%s = %0.2f\r\n",key, wind_speed); return false;}
  if(strcmp(key,"WindGust")==0) {wind_gust = atof(value)*3.6; add_wind_history_gust(wind_gust); printf("%s = %0.2f\r\n",key, wind_gust); return false;}
  if(strcmp(key,"Temperature")==0) {temperature = atof(value); if(!settings.sensor_type == s_WS80){settings.sensor_type = s_WS80; log_i("Detected WS80\n");} return false;} // WS80 only - autodetection
  if(strcmp(key,"GXTS04Temp")==0) {temperature = atof(value);  if(!settings.sensor_type == s_WS85){settings.sensor_type = s_WS85; log_i("Detected WS85\n");} return false;} // WS85 only
  if(strcmp(key,"Humi")==0) {humidity = atoi(value); return false;}
  if(strcmp(key,"Light")==0) {light_lux = atoi(value); return false;}
  if(strcmp(key,"UV_Value")==0) {uv_level = atof(value); return false;}
  if(strcmp(key,"CapVoltage")==0) {cap_voltage = atof(value); return false;} // WS85
  if(strcmp(key,"BatVoltage")==0) {
    wsxx_vcc = atof(value); 
    last_wsxx_data =time();
    
    log_i("WSXX data complete\r\n");
    return true;
  }

  //log_i(" ->not_found\n");
  return false;
}

#ifdef HAS_HEATER
// Digipot, Solar & DCDC ----------------------------------------------------------------------------------------------------------------------

// Calc register value for digipot with resistor constellation given
int calc_regval(float val, float setpoint, int r1, int r2, int steps, int rmax){
  int regval = ((setpoint * r1 / (val-setpoint)) - r2)/rmax*steps;
  if(regval > 255){regval = 255;}
  if(regval < 0 ){regval = 0;}
  return regval;
}

// Solar MPPT voltage to register value
int calc_cn3791(float val){
  const int r1 = 300;
  const int r2 = 12;
  const int steps = 256;
  const float setpoint = 1.205;
  const int rmax = 100; // 100k
  return calc_regval(val, setpoint, r1, r2, steps, rmax);
}

// DCDC voltage to register value
int calc_mt3608(float val){
  const int r1 = 510;
  const int r2 = 27;
  const int steps = 256;
  const float setpoint = 0.6;
  const int rmax = 100; // 100k
  return calc_regval(val, setpoint, r1, r2, steps, rmax);

}
// Set output of digipot channel
void mcp4652_write(unsigned char addr, unsigned char value){
  if(use_mcp4652){
    //log_i("Setting Whiper to: ", (uint32_t) value);
    unsigned char cmd_byte = 0;
    cmd_byte |= (addr | CMD_WRITE);
    //Wire.begin();
    Wire.beginTransmission(MCP4652_I2C_ADDR);
    Wire.write(cmd_byte);
    Wire.write(value);
    if(Wire.endTransmission() != 0){
      //log_e("Faild to set MCP4652. Disabling\r\n");
      use_mcp4652 = false;
      bd_hw_version = HW_2_0;
      return;
    } else {
      bd_hw_version = HW_1_3;
    }
  }
}

// Write both outputs of digipot
void apply_mcp4652(){
  if(use_mcp4652){
    if(heater_voltage > HEATER_MAX_V){
        heater_voltage = HEATER_MAX_V;
    }
    //log_i("Setting Digipot\r\n"); log_flush();
    mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(heater_voltage));
    delay(3);
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
  }
}
#endif

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

// read status pins of solar charger
void get_solar_charger_state(){
  pinMode(PIN_PV_CHARGE, INPUT_PULLUP);
  pinMode(PIN_PV_DONE, INPUT_PULLUP);
  pv_done = !digitalRead(PIN_PV_DONE);
  pv_charging = !digitalRead(PIN_PV_CHARGE);
  pinDisable(PIN_PV_CHARGE);
  pinDisable(PIN_PV_DONE);
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
    //log_i("V_Batt_raw: ", val);
    if(bd_hw_version == HW_1_3){
      val *= 0.00432; // 100k/360k 1.0V Vref
    }
     else if(bd_hw_version == HW_2_0){
      val *= 0.0040925; // 100k/330k 1.0V Vref
    }
    batt_volt = val;
  
  batt_perc = voltageToSOCNonLinear(batt_volt);
  log_i("V_Bat: ", batt_volt);
  log_i("Bat_perc: ", batt_perc);
  }
}



// Sensors ----------------------------------------------------------------------------------------------------------------------
// request baro sampling & conversion
void baro_start_reading(){
  //sercom3.resetWIRE();
  //Wire.begin();
  if(!next_baro_reading){
    if(baro_chip == BARO_BMP280){ 
      next_baro_reading = time() + bmp280.startMeasurment();
    }
    if(baro_chip == BARO_SPL06){ 
      spl.start_measure();
      next_baro_reading = time() + 27;
    }
    if(baro_chip == BARO_BMP3xx){ 
      bmp3xx.setOutputDataRate(BMP3_ODR_50_HZ);
    // bmp3xx.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    // bmp3xx.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    // bmp3xx.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp3xx.performReading();
      next_baro_reading = time() + 5;
    }
    if(baro_chip == BARO_HP203B){ 
      hp.startMeasure();
      next_baro_reading = time() + 25; // OSR512
    }
    } else {
      if(time() > (next_baro_reading +200)){
        next_baro_reading = 0;
      }
    }
}

// read value of baro, needs baro_start_reading in advance
void read_baro(){
  bool data_ok = false;
  double T,P;

  if(baro_chip == BARO_BMP280){
    if(next_baro_reading && (time() > next_baro_reading)){
      uint8_t result = bmp280.getTemperatureAndPressure(T,P);
      if(result!=0){
        data_ok = true;
      }
    }
  }

  else if(baro_chip == BARO_SPL06){
    if(next_baro_reading && (time() > next_baro_reading)){
      P = spl.get_pressure();
      T = spl.get_temp_c();
      spl.sleep(); 
      
      if(P > 0){
        data_ok = true;
      }
    }
  }
  else if(baro_chip == BARO_BMP3xx){ 
    P = bmp3xx.pressure/100;
    T = bmp3xx.temperature;
    bmp3xx.setOutputDataRate(BMP3_ODR_0_001_HZ);
    if(P > 0){data_ok = true;}
  }
  else if(baro_chip == BARO_HP203B){ 

    hp.Measure_Pressure();
    hp.Measure_Temperature();
    hp.startMeasure();
    P = hp.hp_sensorData.P;
    T = hp.hp_sensorData.T;
    if(P > 0){data_ok = true;}
  }


  if(data_ok){
    baro_temp = T;
    next_baro_reading = 0;
    last_baro_reading = time();
    if (settings.altitude > -1){
        baro_pressure = (P * pow(1-(0.0065*settings.altitude/(T + (0.0065*settings.altitude) + 273.15)),-5.257));
    } else {
      baro_pressure = P;
    }
    //log_i("current Baro: ", baro_pressure);
  } else {
    //log_i("Baro no data, retry\n");
    //baro_start_reading();
  }
}

// while USB is connected, forward ws80 data to usb serial port
void forward_wsxx_serial(){
  if(settings.forward_serial_while_usb){
    uint8_t buffer [512];
    static int bufferpos = 0;

    if(is_wsxx()){
      while (SENSOR_UART.available()){
        buffer[bufferpos] = SENSOR_UART.read();
        bufferpos++;
        if(bufferpos > 512){bufferpos =0;}
      

        if(bufferpos && (buffer[bufferpos] == '\n')){
          if(usb_connected){Serial.write(buffer, bufferpos);}
          Serial1.write(buffer, bufferpos);
          bufferpos = 0;
        }
      }
    } else if(settings.sensor_type == s_WS85_UART){
      // Read data block first
      while(ws85uart.get_char(&(buffer[bufferpos]))){
        bufferpos++;
        delay(1); // wast some time to wait for the serial data to complete receiving
      }
      // Outpiut the data
      if(bufferpos){
        if(usb_connected){
          for(int i = 0; i < bufferpos; i++){
            Serial.print(buffer[i], HEX);
            Serial.print(" "); // todo: replace by proper string building
          }
        }
        bufferpos = 0;
        Serial.println();
      }
    }
  }
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
          wind_speed = atof(token);
          num =2;
        }
        else if(num == 2){
          wind_gust = atof(token);
          num =3;
        }
        else if(num == 3){
          wind_dir_raw = atoi(token);
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
  #define BUFFERSIZE 1024 // size of linebuffer
  static char buffer [BUFFERSIZE];
  int co = 0;
  bool found_data = false;
  int eq_count = 0;
  uint32_t last_data = micros();
  //uint32_t cpy_last_wsxx_data = last_wsxx_data;
  static uint32_t serial_wait = 3800;

// Compare String 1
  char comp1_arr[8] = {"FreqSel"};
  const int comp1_len = 7;
  int comp1_pos = 0;

// Compare String 2
  char comp2_arr[5];
  if(settings.sensor_type == s_WS85){ sprintf(comp2_arr,"WS85");}
  else if(settings.sensor_type == s_WS80){ sprintf(comp2_arr,"WH80");}
  const int comp2_len = 4;
  int comp2_pos = 0;

  //uint32_t micros_start = micros();


  while(micros()- last_data < serial_wait){ //  cpy_last_wsxx_data == last_wsxx_data &&
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
        
        if(co >= BUFFERSIZE){
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
      //ws85uart.printWS85Measurement(w85m, Serial2);

      add_wind_history_dir(w85m.windDirection);
      add_wind_history_wind(w85m.windSpeed);
      add_wind_history_gust(w85m.gustSpeed);
      temperature = w85m.temperatureC;
      last_wsxx_data = time();

      log_i("Wind: ", w85m.windSpeed);

      //log_i("WS85_UART data ok\r\n");
      errorcount = 0;
      return RESP_COMPLETE;
    }
  }
  errorcount++;
  if(errorcount % 2 == 0){ // if we get too many bad reading, maybe the baud rate does not match
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
  wind_dir_raw = read_wind_dir();
  add_wind_history_dir(wind_dir_raw);
  
  if(settings.sensor_type == s_DAVIS6410){
    wind_speed = (float) pulses * 1.609 * (2250.0/((float)dmillis+1) ); // avoid div/0
  }
  // ... other analog sensors
  add_wind_history_wind(wind_speed);
  add_wind_history_gust(wind_speed);
  save_history(wind_speed, temperature, humidity, light_lux, batt_volt, pv_charging, pv_done);
}

// Heater ----------------------------------------------------------------------------------------------------------------------
  #ifdef HAS_HEATER
void run_heater(){
  static uint32_t last_heater_calc = 0;
  static uint32_t h_switch_on_time = 0;
  static float current_output_v = 0;
  static bool en_heater = false;
  static uint32_t rampstep = 0;

  // calc if heater needs to be on or off
  if(time()- last_heater_calc > 10000){
    last_heater_calc = time();
    read_batt_perc(); // get new voltage if outdated
    uint32_t light_hist= history_sum_light(24);
    uint32_t wind_hist= history_sum_wind(24);
    //log_i("# Heater Info\r\n");
    log_i("light_hist: ", light_hist);
    log_i("wind_hist: ",wind_hist);
    if(h_switch_on_time){
      log_i("Heater on since [s]: ", (time()-h_switch_on_time)/1000);
    }
    // Turn on heater
    if(( temperature < 3 && \
        batt_volt > 3.6 && \
        light_hist < 500 && \
        wind_hist < 10 && \
        time() > 1800000) // min 30 min on
        || settings.test_heater
        ){
          en_heater = true;
        }
  }
  // Turn off (checked on every call)
  if( (h_switch_on_time && (time()-h_switch_on_time > MAX_HEAT_TIME) ) || (batt_volt <= 3.1) || !settings.test_heater){  // limit heatertime
    heater_on_time_cum += time()-h_switch_on_time; 
    en_heater = false;
  }

  if( (bd_hw_version == HW_1_3) && en_heater){
    if(!current_output_v){ // is currently off
      current_output_v = 3; // start at 3V (non switching)
      mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(current_output_v)); // to avoid 1,1A short circuit detection set to half voltage at load switch enable
      pinMode(PIN_EN_HEATER,OUTPUT);
      pinMode(PIN_EN_DCDC,OUTPUT);
      digitalWrite(PIN_EN_HEATER,1);
      delay(5);
      digitalWrite(PIN_EN_DCDC,1);
      h_switch_on_time = time();
      log_i("Heater turned on: ", time());
    }

    if(time() - h_switch_on_time > 20000){ // after 20 secs ramp up
      if(time()- rampstep > 1000){
        rampstep = time();
        if(current_output_v < heater_voltage){
          current_output_v += 0.025;
          mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(current_output_v));
        }
        if(current_output_v > heater_voltage){
          current_output_v = heater_voltage;
          mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(current_output_v));
        }
      }
    }
    // VBAT = 3V, max 5V output ~3W
    // 3.2V = 5,5V ~ 3,36W
    // 3,5V = 6V
    
  // turn heater off if running
  } else {
    if(h_switch_on_time){
      h_switch_on_time=0;
      current_output_v=0;
      digitalWrite(PIN_EN_HEATER,0);
      digitalWrite(PIN_EN_DCDC,0);
      pinDisable(PIN_EN_HEATER);
      pinDisable(PIN_EN_DCDC);
      log_i("Heater turned off: ", time());
    }
  }
}
  #endif

// Sleep ----------------------------------------------------------------------------------------------------------------------

// dummy function
void wakeup_EIC(){
  wakeup_source = WAKEUP_EIC;
}

void sleep(bool eic){

  if(is_wsxx()){
    enable_sercom0_int();
  }
  
  if(settings.sensor_type == s_WS85_UART){
    enable_sercom0_int();
  }
  deepsleep(false); // no light sleep

  if(is_wsxx()){
    disable_sercom0_int(115200);
    SENSOR_UART.begin(115200*div_cpu);
  }

  if(settings.sensor_type == s_WS85_UART) {
      disable_sercom0_int(115200);
      ws85uart.begin(); // required to get data
    }
}

// enable interrupt on uart rx pin and wait for data
uint32_t sleep_til_serial_data(){
  uint32_t sleepcounter =0;
  //log_i("sleep\r\n"); log_flush();
  sleep(true);
  sleepcounter = read_time_counter();
  //log_i("Wakeup: Actual_sleep: ", sleepcounter);
  //log_i("Wakeup_source: "); log_i(wakeup_source_string[wakeup_source]);log_i("\r\n");

    return sleepcounter;
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
  if(settings.is_baro){
    baro_start_reading(); // request data aquisition, will be read later
  }

  read_batt_perc();
  sleep_allowed = time() + 100; // go back to sleep after 6 secs as fallback
}

// calc time to sleep til next fanet message needs to be send
uint32_t calc_time_to_sleep(){
  uint32_t tts_weather = -1;
  uint32_t tts_name = -1;
  uint32_t tts_info = -1;
  uint32_t tts = 0;

  if(batt_volt){
    if(batt_volt < VBATT_LOW){
      tts = 3600*500; // sleep 30min
      undervoltage = true;
      broadcast_scale_factor = 5;
      log_i("Undervoltage\n");
    } else if(batt_volt < settings.reduce_interval_voltage){
      reduced_interval = true;
      broadcast_scale_factor = 3;
      log_i("Low voltage\n");
    } else {
      // battery voltage is normal
      reduced_interval = false;
      broadcast_scale_factor = 1;
    }


  }
  if(!undervoltage){
    if( last_msg_weather && settings.broadcast_interval_weather){
      if( (last_msg_weather + (settings.broadcast_interval_weather * broadcast_scale_factor)) > time() ){
        tts_weather = (settings.broadcast_interval_weather * broadcast_scale_factor) - (time()-last_msg_weather);
      } else {
        tts_weather = 0;
      }
    }
    if( last_msg_name && settings.broadcast_interval_name){
      if( (last_msg_name + (settings.broadcast_interval_name * broadcast_scale_factor)) > time()){
        tts_name = (settings.broadcast_interval_name * broadcast_scale_factor) - (time()-last_msg_name);
      } else {
        tts_name = 0;
      }
    }
    if( last_msg_info && settings.broadcast_interval_info){
      if( (last_msg_info + (settings.broadcast_interval_info * broadcast_scale_factor)) > time()){
        tts_info = (settings.broadcast_interval_info * broadcast_scale_factor) - (time()-last_msg_info);
      }else {
        tts_info = 0;
      }
    }
    //log_i("tts_weather: ", tts_weather);
    //log_i("tts_name: ", tts_name);
    //log_i("tts_info: ", tts_info);
    
    tts = min(min(tts_name, tts_info), tts_weather);
    if( fanet_cooldown && last_fnet_send  && (time() - last_fnet_send + tts < fanet_cooldown)){
      tts += fanet_cooldown - (time()-last_fnet_send);
    }
    if(tts == (uint32_t)-1){ tts=0;}
  }

  if( !tts && loopcounter > 100){
   // log_e("just looping, will sleep\n");
    // tts = 12000; // debug to keep in loop
  }

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
  uint32_t actual_sleep = 0;

  pinDisable(PIN_V_READ_TRIGGER);

// shut down the USB peripheral
  if(first_sleep && !(settings.test_with_usb || usb_connected) ){
    log_i("Disable USB\n");
    USB->DEVICE.CTRLA.bit.ENABLE = 0;                   
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE){};
    first_sleep = false;
    usb_connected = false;
    if(set_cpu_div(settings.div_cpu_slow)){ //USB needs 48Mhz clock, as we are finished with USB we can lower the cpu clock now.
      div_cpu = settings.div_cpu_slow;
      DEBUGSER.begin(115200*div_cpu); // F_CPU ist still 48M, so every clock needs to by multiplied manually
    }
  }

// if never send a weather msg, assume it was send now
  if(!last_msg_weather){
    last_msg_weather = time();
  }

  int32_t time_to_sleep = calc_time_to_sleep();
  if(!undervoltage){
    if(( is_wsxx() || settings.sensor_type == s_WS85_UART) && last_wsxx_data == 0){ time_to_sleep = 12000;} // if no data from WS80 received 
    if(settings.sensor_type == s_DAVIS6410){ time_to_sleep = min(time_to_sleep, settings.sensor_integration_time);} // interval for gust detection 
    if(!settings_ok){ 
      log_e("No settings file\n");
      log_e("Sleeping forever\r\n");
      time_to_sleep = 0xFFFFFFF;
      } // if settings not ok sleep forever
  }
  if(!time_to_sleep){ return;} // if time_to_sleep = 0, do not sleep at all

  log_i("will sleep for ", time_to_sleep > 200000UL?-1: time_to_sleep);
  log_flush();

// disable wdt during sleep
  if(settings.use_wdt) {
    wdt_disable();
  }

// Using TC4 for hardware pulsecounting on Falling edge on pin PA04 (D17). No interrupts needed.
  if(!undervoltage && settings.sensor_type == s_DAVIS6410){ // pulse counting anemometer
    actual_sleep = rtc_sleep_cfg(time_to_sleep);
    setup_pulse_counter(); // need to setup GCLK6 before

    if(debug_enabled()){
      DEBUGSER.end();
      pinDisable(PIN_TX);
      pinMode(PIN_RX, INPUT_PULLUP);
    }

    while(wakeup_source != WAKEUP_RTC){ // ignore other wakeups (external pin Interrupt, if configured)
      sleep(false);
    }
    if(debug_enabled()){
      DEBUGSER.begin(115200*div_cpu);
    }
    sleeptime_cum += actual_sleep;
    pulsecount = read_pulse_counter();
    calc_pulse_sensor(pulsecount, actual_sleep);
    // elseif (is_other_pulsecounting_sensor){
      //calc_...(pulsecount, actual_sleep);
    //}

// UART sensor, just sleep
  } else if(!undervoltage && ( is_wsxx() || settings.sensor_type == s_WS85_UART)){ // no pulse counting anemometer, no interrupts
    int32_t t =0;
    int res = -1;
    if(settings_ok && (time()> 2500)  && !usb_connected && !settings.no_sleep && !settings.testmode){
      reset_time_counter();
      actual_sleep = rtc_sleep_cfg(time_to_sleep);
      while(wakeup_source != WAKEUP_RTC){
        t = sleep_til_serial_data();

        if(is_wsxx()){
          res = read_wsxx();
        }

        if(settings.sensor_type == s_WS85_UART){
          res = read_ws85_uart();
        }
        
        if(res == RESP_COMPLETE){ // if true, we received a data block, so it is ok to sleep for ~ 4 seconds without listening to serial data
          if(time_to_sleep > t ){
            if(settings.sensor_type == s_WS85){actual_sleep = rtc_sleep_cfg( min(time_to_sleep - t,8350));}
            if(settings.sensor_type == s_WS80){actual_sleep = rtc_sleep_cfg( min(time_to_sleep - t,4685));} // sleep 4750ms if tts is still longer, or less if it less
            if(settings.sensor_type == s_WS85_UART){actual_sleep = rtc_sleep_cfg( min(time_to_sleep - t,8350));}
            //log_i("will sleep1: ", actual_sleep); 
            //log_flush();
            sleep(false);
            t = read_time_counter();
            if(time_to_sleep > t ){
              time_to_sleep -= t;
              rtc_sleep_cfg( time_to_sleep);
              wakeup_source = WAKEUP_NONE; // reset wakeup reason
            }
            sleeptime_cum += t;
            reset_time_counter();
          }
        }
      }
    t = read_time_counter();
    sleeptime_cum += t;
    }
    sleep_offset = 0; // reset temporary offset
    
  } else { // no sensor configured
    actual_sleep = rtc_sleep_cfg(time_to_sleep);
    sleep(false);
    sleeptime_cum += actual_sleep;
  }

// If sleep is disabled for debugging, use delay
  if(!usb_connected && (settings.no_sleep || settings.testmode)){
    log_i("INSOMNIA or Testmode enabled, unsing delay() instead of deepsleep\n");
    delay(time_to_sleep);
    sleeptime_cum += time_to_sleep;
  }

// re-enable wdt after sleep
  if(settings.use_wdt) {
    wdt_enable(WDT_PERIOD,false);
  }
  if(set_cpu_div(settings.div_cpu_fast)){
    div_cpu = settings.div_cpu_fast;
    log_ser_begin();
  }
  wakeup();
  if(settings.test_with_usb){
    sleep_allowed = time() + time_to_sleep;
  }
}


void print_data(){
  if((debug_enabled())){
    log_i("\r\nmillis: ", millis()); 
    log_i("time: ", time()); 
    //log_i("Wind dir_raw: ", wind_dir_raw);
    log_i("Wind Heading: ", wind_heading);
    log_i("Wind Speed: ", wind_speed);
    log_i("Wind Gust: ", wind_gust);
    log_i("Temp: ", temperature);
    //log_i("Humd: ", humidity);
    if(settings.is_baro){
    log_i("Baro: ", baro_pressure);
    log_i("PCB_Temp: ", baro_temp);
    }
    if(is_wsxx()) {
      log_i("VCC: ", wsxx_vcc);
      //log_i("LUX: ", light_lux);
      //log_i("UV: ", uv_level);
    }
    log_i("\r\n");
    log_i("V_Bat: ", batt_volt);
    log_i("Bat_perc: ", batt_perc);
    log_i("PV_charge: ", pv_charging);
    log_i("PV_done: ", pv_done);
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
      f.print("HW Version: "); 
        if(bd_hw_version == HW_1_3) { f.println("V1.3");}
        if(bd_hw_version == HW_2_0) { f.println("V2.0");}
      f.print("LoRa Module: "); 
        if(lora_module == LORA_SX1276) { f.println("SX1276");}
        if(lora_module == LORA_SX1262) { f.println("SX1262");}
        if(lora_module == LORA_LLCC68) { f.println("LLCC68");}
      f.print("Barometer: "); 
        if(baro_chip == BARO_BMP280) { f.println("BMP280");}
        if(baro_chip == BARO_BMP3xx) { f.println("BMP3xx");}
        if(baro_chip == BARO_SPL06) { f.println("SPL06");}
        if(baro_chip == BARO_HP203B) { f.println("HP203B");}
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
  #ifdef HAS_HEATER
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
    //apply_mcp4652(); // set voltages
    //Serial.println("V set");
  #endif

  }
}

// Send ----------------------------------------------------------------------------------------------------------------------

void send_msg_weather(){
  if(settings.sensor_type == s_invalid){ return;}

  led_status(1);
  WindSample current_wind = get_wind_from_hist(settings.wind_age);
  wind_gust = get_gust_from_hist(settings.gust_age);
  wind_speed = current_wind.wind/10.0;
  wind_dir_raw = current_wind.dir_raw;
  wind_heading = wind_dir_raw + settings.heading_offset;
  if(wind_heading > 359){ wind_heading -=360;}
  if(wind_heading < 0){ wind_heading +=360;}

  if(settings.sensor_type == s_DAVIS6410){ // no other temp sensor
    temperature= baro_temp;
  } 
  if( wind_speed > (wind_gust +3)){
    wind_gust = wind_speed;
    log_i("adapting gust speed\r\n");
  }

  weatherData wd;
  wd.vid = FANET_VENDOR_ID;
  wd.fanet_id = get_fanet_id();
  wd.lat = settings.pos_lat;
  wd.lon = settings.pos_lon;
  wd.bWind = true;
  wd.wHeading = wind_heading;
  wd.wSpeed = wind_speed;
  wd.wGust = wind_gust;      
  wd.bTemp = true;
  wd.temp = temperature;

  if(is_wsxx()){
    wd.bHumidity = true;
    wd.Humidity = humidity;
  }

  if(settings.is_baro){
    wd.bBaro = true;
    wd.Baro = baro_pressure;  
  } else {
    wd.bBaro = true; // baro is required to forward data in OGN
    wd.Baro = -1;  
  }
  wd.bStateOfCharge = true;
  wd.Charge = batt_perc;

  if(settings.testmode){
    wd.bBaro = true;
    wd.Baro = baro_pressure;  
    wd.wHeading = 123;
    wd.wSpeed = 5;
    wd.wGust = 7;      
    wd.temp = 10;
    wd.Humidity = 15;
    log_i("\r\nTESTMODE - Fake values\r\n");
  }

  log_i("\r\nSending Weather\r\n");

  int msgSize = sizeof(fanet_packet_t4);
  uint8_t* buffer = new uint8_t[msgSize];
  pack_weatherdata(&wd, buffer);

// write buffer content to console
#if 0
  for (int i = 0; i< msgSize; i++){
    printf("%02X ", (buffer)[i]);
  }
  DEBUGSER.println();
#endif

  radio_phy->standby();
  radio_phy->startTransmit(buffer, msgSize);

  print_data();
  led_status(0);
  save_history(wind_speed, temperature, humidity, light_lux, batt_volt, pv_charging, pv_done); // only save history on send
  
}

void set_fanet_send_flag(void) {
  transmittedFlag = true;
}

void irq_lora_rec(){
  loraReceivedFlag = true;
}

void rx_sleep(){
  //radio_phy->startReceive();
  RadioLibIrqFlags_t irqFlags = RADIOLIB_IRQ_RX_DONE | RADIOLIB_IRQ_TX_DONE;
  RadioLibIrqFlags_t irqMask  = RADIOLIB_IRQ_RX_DONE | RADIOLIB_IRQ_TX_DONE | RADIOLIB_IRQ_TIMEOUT;
  if((lora_module == LORA_SX1262) || (lora_module == LORA_LLCC68)){
  static_cast<SX126x*>(radio_phy)->startReceiveDutyCycleAuto(8,4, irqFlags,irqMask ); // workaround as virtual function is missing in PysicalLayer class
  } else if(lora_module == LORA_SX1276){
    // not supported yet
  }
  //radio_phy->setPacketReceivedAction(irq_lora_rec); see line below
  attachInterruptWakeup(PIN_LORA_DIO1, irq_lora_rec, RISING, false);
  // sleep
}

void fanet_rx(){
  if(loraReceivedFlag) {
    loraReceivedFlag = false;

    int numBytes = radio_phy->getPacketLength();
    byte byteArr[numBytes];
    int state = radio_phy->readData(byteArr, numBytes);

    if (state == RADIOLIB_ERR_NONE) {

     // Serial.println(F("[SX1262] Received packet!"));
     //  Serial.print(F("[SX1262] Data:\t\t"));
     //  for ( int i =0; i< numBytes; i++){
     //    Serial.printf("%02X ", byteArr[i]);
     //  }
     //  Serial.println();

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("CRC error!"));
      return;
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      return;
    }

    fanet_header *header = (fanet_header *)byteArr;
    if(header->type == FANET_PCK_TYPE_WEATHER){
      if(header->forward){
        // queque package for forwarding
      }
    }
}
}

// check if everything is ok to send the wather data now
bool allowed_to_send_weather(){
  bool ok = settings_ok;
  

  if (ok){
    if(settings.is_baro){ok &= ((time() - last_baro_reading) < 10000 );}
    if(is_wsxx() || settings.sensor_type == s_WS85_UART) { ok &= (last_wsxx_data || settings.testmode); } // only send if weather data is up to date or testmode is enabled // && (time()- last_ws80_data < 9000))
    if(settings.is_gps)  { ok &= (tinyGps.location.isValid()); } // only send if position is valid
  }

 // if(next_baro_reading){
 //   log_i("next_baro_reading: ", next_baro_reading);
 // }

  if(!ok){
    //if(last_wsxx_data){
      //log_i("WS80 data age: ", time()- last_ws80_data);
   // }
    if(settings.is_gps){
      if(time()-last_gps_valid > 3000){
        log_i("No GPS fix");
      }
    }
  }


  return ok;
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
        last_gps_valid = time();
        //DEBUGSER.println("Position Update");
    }
  }
}

// check if last fanet package want sent recenctly
bool fanet_cooldown_ok(){
  if(lora_module && (time() -last_fnet_send > fanet_cooldown)){
    return true;
  }
  return false;
}

void send_msg_name(const char* name, int len){
  uint8_t* buffer = new uint8_t[len+4];
  fanet_header header;
  header.type = 2;
  header.vendor = FANET_VENDOR_ID;
  header.forward = false;
  header.ext_header = false;
  header.address = get_fanet_id();

  memcpy(buffer, (uint8_t*)&header, 4);
  memcpy(&buffer[4], name, len);

// write buffer content to console
#if 0
  for (int i = 0; i< len+4; i++){
    printf("%02X ", (buffer)[i]);
  }
  DEBUGSER.println();
#endif

  radio_phy->standby();
  radio_phy->startTransmit((uint8_t*) buffer, len+4);
}

void send_msg_info(){
  char data[50] = {0x00};
  uint8_t data_len = 1;
  // Test: send battery voltage and charging state
  data_len += sprintf(&data[1], "%04X:%s %0.2fV C%i", get_fanet_id(), VERSION, batt_volt, pv_charging);


  uint8_t* buffer = new uint8_t[data_len+4];
  fanet_header header;
  header.type = 3;
  header.vendor = FANET_VENDOR_ID;
  header.forward = false;
  header.ext_header = false;
  header.address = get_fanet_id();

  memcpy(buffer, (uint8_t*)&header, 4);
  memcpy(&buffer[4], data, data_len);

// write buffer content to console
#if 1
  for (int i = 0; i< data_len+4; i++){
    printf("%02X ", (buffer)[i]);
  }
  DEBUGSER.println();
#endif
  log_i("Sending Info Msg\n");
  radio_phy->standby();
  radio_phy->startTransmit((uint8_t*) buffer, data_len+4);
}

bool radio_init(){
  if(settings.skip_lora){return false;}

  if(zone_not_eu()){ // check GPS coordinates for freuency selection
    settings.lora_freq = LORA_FREQ_NA;
    settings.lora_bw = LORA_BW_NA;
  }



    if(radio_sx1276.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12, 0) == RADIOLIB_ERR_NONE){
      radio_phy = (PhysicalLayer*)&radio_sx1276;
      log_i("Found LoRa SX1276\r\n");
      lora_module = LORA_SX1276;
      return true;
    } 
    if(radio_llcc68.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12) == RADIOLIB_ERR_NONE){
      radio_phy = (PhysicalLayer*)&radio_llcc68;
      log_i("Found LoRa LLCC68\r\n");
      lora_module = LORA_LLCC68;
      return true;
    }
    if(radio_sx1262.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12) == RADIOLIB_ERR_NONE){
      // NiceRF SX1262 issue https://github.com/jgromes/RadioLib/issues/689
      radio_phy = (PhysicalLayer*)&radio_sx1262;
      log_i("Found LoRa SX1262\r\n");
      lora_module = LORA_SX1262;
      return true;
    }  else {
        log_i("No LoRa found\r\n");
        return false;
    }
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

  bd_hw_version = HW_2_0; 
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
    radio_phy->sleep();
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
    setup_PM(is_wsxx() || settings.sensor_type == s_WS85_UART); // powermanagement add || other sensors using 32bit counter
    wdt_enable(WDT_PERIOD,false); // setup clocks
    if(!settings.use_wdt) {
      wdt_disable();
    }
  if(settings.is_baro){
    bool baro_ok = false;

    // ToDo: Crashes with HP203B installed when checking for bmp3xx
   // if(!baro_ok && bmp280.begin()){ //0x76
   //   baro_ok = true;
   //   baro_chip = BARO_BMP280;
   //   log_i("Baro: BMP280\r\n");
   //   bmp280.setOversampling(4);
   // }
   // if(!baro_ok && spl.begin(0x77)){
   //   baro_ok = true;
   //   baro_chip = BARO_SPL06;
   //   log_i("Baro: SPL06\r\n");
   // }
    //if(!baro_ok && bmp3xx.begin_I2C(0x76)){
    //  baro_ok = true;
    //  baro_chip = BARO_BMP3xx;
    //  log_i("Baro: BMP3XX\r\n");
    //}
    if(!baro_ok && hp.begin(0x76, OSR_2048)){
        baro_ok = true;
        baro_chip = BARO_HP203B;
        hp.setOSR(OSR_512); // 16.4ms conversion time
        log_i("Baro: HP203B\r\n");
    }
    if(!baro_ok){
      log_e("Baro: not found\r\n");
      settings.is_baro = false;
      led_error(1);
    }
  }
  
#ifdef HAS_HEATER
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
    apply_mcp4652();
#else
bd_hw_version = HW_2_0;
#endif
  print_settings();
  } 
  if(!settings_ok) {
    // Needed for deepsleep
    setup_PM(false);
    wdt_enable(WDT_PERIOD,false); // setup clocks
    wdt_disable();
    //setup_rtc_time_counter();
  }

  if(is_wsxx() || settings.sensor_type == s_WS85_UART){
    switch_WS_power(1); // Turn on WS80 Power supply with P-MOSFET (HW >= 2.2)
    setup_rtc_time_counter();
    if(is_wsxx()) {SENSOR_UART.begin(115200);}
    
  }
  if(settings.is_gps){
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
  if(bd_hw_version == HW_unknown){log_i("Hardware detection failed\n");}
  if(bd_hw_version == HW_1_3){log_i("Detected HW1.x\n");}
  if(bd_hw_version == HW_2_0){log_i("Detected HW2.x\n");}
  log_flush();
  wakeup();

if(settings.sensor_type == s_WS85_UART){
  log_i("Setup WS85\n");
  
  //ws85uart.setAutoSendInterval(8500);
  ws85uart.begin();
  //ws85uart.requestAutoSendInterval();
  ws85uart.set_baud_115200();
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

#ifdef HAS_HEATER
  if(is_heater && (bd_hw_version == HW_1_3)){run_heater();}
#endif

  if(settings.is_baro){read_baro();}
  if(settings.is_gps){read_gps();}

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
      if((is_wsxx() || settings.sensor_type == s_WS85_UART) && last_wsxx_data && (time()- last_wsxx_data > 10000)){
        log_i("Wdata not ready. Wdata age: ", (time()- last_wsxx_data) );
        log_i("Last Baro reading age: ", (time()- last_baro_reading) );
        sleep_allowed = time() + 1;
        last_wsxx_data = 0;
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
      radio_phy->sleep();
      delay(10);
      led_error(0);
    }

    if(transmittedFlag){
      transmittedFlag = false;
      //log_i("Send complete\r\n");
      send_active = 0;
      sleep_allowed = time() + (1);
      radio_phy->finishTransmit();
      radio_phy->sleep();
    }
  }

// Check if everything is done --> sleep
  if(!send_active && sleep_allowed && (time() > sleep_allowed) && (!usb_connected || settings.test_with_usb) && (time() > 2500)){ // allow sleep after 2500 ms to get a change to detect usb connected
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
    else {forward_wsxx_serial();} // otherwise just forward the data
    read_serial_cmd(); // read setting values from serial for testing
    if(!settings.no_sleep && !settings.test_with_usb && (time() > 15UL*60UL*1000UL)){
      log_i("Restart\r\n");
      log_flush();
      usb_connected = false;
      NVIC_SystemReset();
      } // keep usb alive for 15 min
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