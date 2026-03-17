 #pragma once
 #include <Arduino.h>
 #include "defines.h"

  enum {
    FANET_PCK_TYPE_TRACKING = 0x01,
    FANET_PCK_TYPE_NAME = 0x02,
    FANET_PCK_TYPE_WEATHER = 0x04
  };
  
  typedef struct {
    uint32_t tLastMsg; //timestamp of neighbour (if 0 --> empty slot)
    uint32_t devId; //devId
    String name; //name of neighbour
    uint16_t vid;
    uint16_t fanet_id;
    int rssi; //rssi
    int snr; //signal to noise ratio
    float lat; //latitude
    float lon; //longitude
    bool bTemp;
    float temp; //temp [°C]
    float wHeading; //wind heading [°]
    bool bWind;
    float wSpeed; //km/h
    float wGust; //km/h
    bool bHumidity;
    float Humidity;
    bool bBaro;
    float Baro;
    bool bStateOfCharge;
    float Charge; //+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%
  } weatherData;

  typedef struct {
    
    unsigned int type           :6;
    unsigned int forward        :1;
    unsigned int ext_header     :1;
    unsigned int vendor         :8;
    unsigned int address        :16;
    }  __attribute__((packed)) fanet_header;

    typedef struct {
    fanet_header header;
    unsigned int bExt_header2     :1;
    unsigned int bStateOfCharge   :1;
    unsigned int bRemoteConfig    :1;
    unsigned int bBaro            :1;
    unsigned int bHumidity        :1;
    unsigned int bWind            :1;
    unsigned int bTemp            :1;
    unsigned int bInternetGateway :1;

    unsigned int latitude       :24;
    unsigned int longitude      :24;

    int8_t temp                 :8;
    unsigned int heading        :8;
    unsigned int speed          :7;
    unsigned int speed_scale    :1;
    
    unsigned int gust          :7;
    unsigned int gust_scale    :1;

    unsigned int humidity      :8;

    int baro          :16;

    unsigned int charge        :8;
  } __attribute__((packed)) fanet_packet_t4;


  void pack_weatherdata(weatherData *wData, uint8_t * buffer);