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


  void pack_weatherdata(weatherData *wData, uint8_t * buffer){

  fanet_packet_t4 *pkt = (fanet_packet_t4 *)buffer;
  pkt->header.type = 4;
  pkt->header.vendor = wData->vid;
  pkt->header.forward = false;
  pkt->header.ext_header = false;
  pkt->header.address = wData->fanet_id;
  pkt->bExt_header2 = false;
  pkt->bStateOfCharge = wData->bStateOfCharge;
  pkt->bRemoteConfig = false;
  pkt->bBaro = wData->bBaro;
  pkt->bHumidity = wData->bHumidity;
  pkt->bWind = wData->bWind;
  pkt->bTemp = wData->bTemp;
  pkt->bInternetGateway = false;

  int32_t lat_i = roundf(wData->lat * 93206.0f);
	int32_t lon_i = roundf(wData->lon * 46603.0f);

  pkt->latitude = lat_i;
  pkt->longitude = lon_i;

  if (wData->bTemp){
    int iTemp = (int)(round(wData->temp * 2)); //Temperature (+1byte in 0.5 degree, 2-Complement)
    pkt->temp = iTemp & 0xFF;
  }
  if (wData->bWind){
    pkt->heading = uint8_t(round(wData->wHeading * 256.0 / 360.0)); //Wind (+3byte: 1byte Heading in 360/256 degree, 1byte speed and 1byte gusts in 0.2km/h (each: bit 7 scale 5x or 1x, bit 0-6))
    int speed = (int)roundf(wData->wSpeed * 5.0f);
    if(speed > 127) {
        pkt->speed_scale  = 1;
        pkt->speed        = (speed / 5);
    } else {
        pkt->speed_scale  = 0;
        pkt->speed        = speed & 0x7F;
    }
    speed = (int)roundf(wData->wGust * 5.0f);
    if(speed > 127) {
        pkt->gust_scale  = 1;
        pkt->gust        = (speed / 5);
    } else {
        pkt->gust_scale  = 0;
        pkt->gust        = speed & 0x7F;
    }
  }
  if (wData->bHumidity){
      pkt->humidity = uint8_t(round(wData->Humidity * 10 / 4)); //Humidity (+1byte: in 0.4% (%rh*10/4))
  }
  if (wData->bHumidity){
    pkt->baro = int16_t(round((wData->Baro - 430.0) * 10));  //Barometric pressure normailized (+2byte: in 10Pa, offset by 430hPa, unsigned little endian (hPa-430)*10)
  }
  pkt->charge = constrain(roundf(float(wData->Charge) / 100.0 * 15.0),0,15); //State of Charge  (+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%)
}