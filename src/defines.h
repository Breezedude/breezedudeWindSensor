#pragma once

#define HAS_HEATER // support for Heater (HW V1.x)

#define BROADCAST_INTERVAL 40*1000UL // dafault value,  overwritten by settingfile

#define LORA_FREQ_EU 868.2
#define LORA_BW_EU 250
#define LORA_FREQ_NA 920.8
#define LORA_BW_NA 500

#define DEBUGSER Serial2
#define SENSOR_UART Serial1
#define GPS_SERIAL Serial1 // if no WS80 is connected a GPS receiver can be used. Mainly for OGN range/coverage check

// https://github.com/adafruit/ArduinoCore-samd/blob/master/variants/itsybitsy_m0/variant.cpp
// 0 { PORTA, 11, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
// 1 { PORTA, 10, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

// SERCOM 4 - SPI RFM95W LoRa
#define PIN_LORA_MISO MISO  // 28 PA12  - SERCOM2/PAD[0] | alt SERCOM4/PAD[0]
#define PIN_LORA_MOSI MOSI  // 29 PB10  -                | alt SERCOM4/PAD[2]
#define PIN_LORA_SCK SCK    // 30 PB11  -                | alt SERCOM4/PAD[3]

#define PIN_LORA_CS 5    // PA15 - SERCOM2/PAD[3] | alt SERCOM4/PAD[3]
#define PIN_LORA_RESET 2 // PA14 - SERCOM2/PAD[2] | alt SERCOM4/PAD[2]
#define PIN_LORA_DIO0 11 // PA16 - SERCOM1/PAD[0] | alt SERCOM3/PAD[0]
#define PIN_LORA_DIO1 10 // PA18 - SERCOM1/PAD[2] | alt SERCOM3/PAD[2]
#define PIN_LORA_DIO2 12 // PA19 - SERCOM1/PAD[3] | alt SERCOM3/PAD[3]
#define PIN_LORA_DIO3 3  // PA09 - SERCOM0/PAD[1], ADC
#define PIN_PV_CHARGE 38// PB23
#define PIN_PV_DONE 37 // PB22
#define PIN_PS_WS 36 // PB03 PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },  // SPI Flash MISO on ItsyBitsy M0, Power switch control for WSXX Sensor. HW >= 2.2

// Serial2
#define PIN_SERCOM1_TX 40 // PA00 SERCOM1 PAD0
#define PIN_SERCOM1_RX 41 // PA01 SERCOM1 PAD1

// Pins with solder jumpers 
#define PIN_ID0 6 // PA20 - SERCOM5/PAD[2]
#define PIN_ID1 7 // PA21 - SERCOM5/PAD[3]

#ifdef HAS_HEATER
  #define HEATER_MAX_V 9.20 // Maximum setpoint voltage, thermal limit by inductor (~1,8A conntiniuos, actual voltage ~8V)
  #define PIN_EN_DCDC 19 // A5 19 PB02
  #define PIN_EN_HEATER 4 //D4/A8 PA08
#endif


#define FANET_VENDOR_ID 0xBD
#define VBATT_LOW 3.35 // Volt


#define PIN_DAVIS_SPEED 17 // A3 17 PA04
#define PIN_DAVIS_DIR 18 // A4 18 PA05
#define PIN_DAVIS_POWER 9 // 9 PA07 - power for direction potentiometer. Turn off if not used to save power

// SERCOM 3 - I²C Barometer / Digipot
#define PIN_SDA 26 // PA22
#define PIN_SCL 27 // PA23

// DAVIS6410 Pinout
// Black - Wind speed contact (closure to ground)
// Red - Ground
// Green - Wind direction pot wiper (20KΩ potentiometer)
// Yellow - Pot supply voltage

// SERCOM 0 - uart serial Sensor/ GPS / Debug
#define PIN_RX 0 // A6 PA11
#define PIN_TX 1 // A7 PA10

#define PIN_STATUSLED 13 // PA17 // blue, 1.2mA @ 2.77V = 470Ohm
#define PIN_ERRORLED 4 //D4/A8 PA08 // red 2.9mA @ 2.16V = 390Ohm
#define PIN_V_READ_TRIGGER A2 // D16 PB09
#define PIN_V_READ A1 // D15 PB08 - { PORTB,  8, PIO_ANALOG, (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel2, PWM4_CH0, TC4_CH0, EXTERNAL_INT_8 }, // ADC/AIN[2]

#define VERSIONFILE (char*) "version.txt"
#define SETTINGSFILE (char*) "settings.txt"



#define HISTORY_INTERVAL 30*1000 // 30s 3600*1000 // 1h
#define HISTORY_LEN 15*24 // in hours, 15 days

#define WIND_AGE 1000*30 // 30s history
#define GUST_AGE 1000*60*10 // 10 min history
#define WIND_HIST_STEP 1000*4 //ms history slots, 22 sek
#define WIND_HIST_LEN 150 // number so slots. should match GUST_AGE / GUST_HIST_STEP


enum ResParseChar {
  RESP_OK,
  RESP_ERROR,
  RESP_COMPLETE
};

enum LORA_MODULE{
  LORA_NONE,
  LORA_SX1276,
  LORA_SX1262,
  LORA_LLCC68
};

enum BARO_CHIP{
  BARO_NONE,
  BARO_BMP280,
  BARO_SPL06,
  BARO_BMP3xx,
  BARO_HP203B
};


enum Wakeup_Source{
  WAKEUP_NONE,
  WAKEUP_RTC,
  WAKEUP_EIC,
  WAKEUP_WDT
};