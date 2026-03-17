#include "fanet.h"

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