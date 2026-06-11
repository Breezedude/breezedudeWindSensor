#include "fanet.h"
#include "logging.h"

size_t pack_weatherdata(const weatherData *wData, uint8_t *buffer) {
  size_t pos = 0;

  fanet_header hdr = {};
  hdr.type = FANET_PCK_TYPE_WEATHER;
  hdr.vendor = wData->vid;
  hdr.forward = false;
  hdr.ext_header = false;
  hdr.address = wData->fanet_id;
  memcpy(&buffer[pos], &hdr, sizeof(fanet_header));
  pos += sizeof(fanet_header);

  // Byte 0 service flags, payload order is bit 6 down to 1.
  uint8_t flags = 0;
  if (wData->bStateOfCharge) { flags |= (1u << 1); }
  if (wData->bBaro)          { flags |= (1u << 3); }
  if (wData->bHumidity)      { flags |= (1u << 4); }
  if (wData->bWind)          { flags |= (1u << 5); }
  if (wData->bTemp)          { flags |= (1u << 6); }
  buffer[pos++] = flags;

  int32_t lat_i = (int32_t)roundf(wData->lat * 93206.0f);
  int32_t lon_i = (int32_t)roundf(wData->lon * 46603.0f);
  uint32_t lat_u = (uint32_t)lat_i;
  uint32_t lon_u = (uint32_t)lon_i;

  // Position is signed 24-bit little-endian (2's complement).
  buffer[pos++] = (uint8_t)(lat_u & 0xFFu);
  buffer[pos++] = (uint8_t)((lat_u >> 8u) & 0xFFu);
  buffer[pos++] = (uint8_t)((lat_u >> 16u) & 0xFFu);
  buffer[pos++] = (uint8_t)(lon_u & 0xFFu);
  buffer[pos++] = (uint8_t)((lon_u >> 8u) & 0xFFu);
  buffer[pos++] = (uint8_t)((lon_u >> 16u) & 0xFFu);

  if (wData->bTemp) {
    int iTemp = (int)roundf(wData->temp * 2.0f);  // 0.5 deg C, 2-complement
    buffer[pos++] = (uint8_t)(iTemp & 0xFF);
  }

  if (wData->bWind) {
    buffer[pos++] = (uint8_t)roundf(wData->wHeading * 256.0f / 360.0f);

    auto encode_wind = [](float kmh) -> uint8_t {
      int speed = (int)roundf(kmh * 5.0f);  // 0.2 km/h
      if (speed < 0) {
        speed = 0;
      }
      if (speed > 127) {
        int scaled = speed / 5;
        if (scaled > 127) {
          scaled = 127;
        }
        return (uint8_t)(0x80u | (uint8_t)(scaled & 0x7F));
      }
      return (uint8_t)(speed & 0x7F);
    };

    buffer[pos++] = encode_wind(wData->wSpeed);
    buffer[pos++] = encode_wind(wData->wGust);
  }

  if (wData->bHumidity) {
    int humidity = (int)roundf(wData->Humidity * 10.0f / 4.0f);  // 0.4%rh steps
    humidity = constrain(humidity, 0, 250);
    buffer[pos++] = (uint8_t)humidity;
  }

  if (wData->bBaro) {
    int32_t baro = (int32_t)roundf((wData->Baro - 430.0f) * 10.0f);  // unsigned 10 Pa
    if (baro < 0) {
      baro = 0;
    }
    if (baro > 65535) {
      baro = 65535;
    }
    uint16_t baro_u16 = (uint16_t)baro;
    buffer[pos++] = (uint8_t)(baro_u16 & 0xFFu);
    buffer[pos++] = (uint8_t)(baro_u16 >> 8u);
  }

  if (wData->bStateOfCharge) {
    uint8_t charge = (uint8_t)constrain((int)roundf((float)wData->Charge / 100.0f * 15.0f), 0, 15);
    buffer[pos++] = charge;
  }

  return pos;
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

  log_i("Sending HW Info type: ", (uint8_t)data->debug_type);

  return pos;

}