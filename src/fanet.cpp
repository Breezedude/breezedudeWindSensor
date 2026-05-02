#include "fanet.h"
#include "logging.h"

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

  if (wData-> bBaro){
    pkt->baro = int16_t(round((wData->Baro - 430.0) * 10));  //Barometric pressure normailized (+2byte: in 10Pa, offset by 430hPa, unsigned little endian (hPa-430)*10)
  }

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

size_t pack_hwinfo(const hwInfoData *data, uint8_t *buffer) {
  size_t pos = 0;

  // FANET header (4 bytes)
  fanet_header *hdr = (fanet_header *)buffer;
  hdr->type       = FANET_PCK_TYPE_HWINFO;
  hdr->vendor     = data->vid;
  hdr->forward    = false;
  hdr->ext_header = false;
  hdr->address    = data->fanet_id;
  pos += sizeof(fanet_header);

  // Byte 0: subheader flags
  hwinfo_byte0_t b0 = {};
  b0.bSubtypeBuild = data->bSubtypeBuild ? 1u : 0u;
  b0.bUptime       = data->bUptime       ? 1u : 0u;
  buffer[pos++] = *(const uint8_t *)&b0;

  // Bit 6 payload: Hardware Subtype (byte 1) + Build Date (bytes 2-3)
  if (data->bSubtypeBuild) {
    buffer[pos++] = data->device_type;

    hwinfo_builddate_t bdate = {};
    bdate.day         = data->build_day;
    bdate.month       = data->build_month;
    bdate.year_offset = (data->build_year >= 2019u)
                        ? (uint16_t)(data->build_year - 2019u) : 0u;
    bdate.develop     = data->develop_mode ? 1u : 0u;
    memcpy(&buffer[pos], &bdate, sizeof(hwinfo_builddate_t));
    pos += sizeof(hwinfo_builddate_t);
  }

  // Bit 4 payload: Uptime in minutes (2 bytes, little-endian)
  if (data->bUptime) {
    buffer[pos++] = (uint8_t)(data->uptime_min & 0xFFu);
    buffer[pos++] = (uint8_t)(data->uptime_min >> 8u);
  }

  // Debug data: decode type byte followed by the matching grouped payload struct
  buffer[pos++] = data->debug_type;
  if (data->debug_type == HWINFO_DEBUG_DYNAMIC) {
    memcpy(&buffer[pos], &data->debug_dynamic, sizeof(hwinfo_debug_t1));
    pos += sizeof(hwinfo_debug_t1);
  } else if (data->debug_type == HWINFO_DEBUG_OTA) {
    memcpy(&buffer[pos], &data->debug_ota, sizeof(hwinfo_debug_t2));
    pos += sizeof(hwinfo_debug_t2);
  } else if (data->debug_type == HWINFO_DEBUG_STATIC) {
    memcpy(&buffer[pos], &data->debug_static, sizeof(hwinfo_debug_t3));
    pos += sizeof(hwinfo_debug_t3);
  }

  log_i("Sending HW Info type:", (uint8_t)data->debug_type);

  return pos;

}