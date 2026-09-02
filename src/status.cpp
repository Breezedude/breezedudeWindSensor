#include "status.h"

#include <stdio.h>
#include <string.h>

#include "defines.h"
#include "tools.h"
#include "logging.h"
#include "sensors.h"
#include "power.h"
#include "ota_ab.h"

extern LORA_MODULE lora_module;
extern CHIP_BARO chip_baro;

// Free RAM estimate: distance between the current stack pointer and the top of
// the heap. Only an estimate, but a shrinking value across a long run is the
// signal worth having.
extern "C" char *sbrk(int incr);
static int free_ram(){
  char stack_top;
  return (int)(&stack_top - (char *)sbrk(0));
}

static const char* sensor_type_name(){
  switch(settings.sensor_type){
    case s_WS80:      return "WS80";
    case s_WS85:      return "WS85";
    case s_WS85_UART: return "WS85_UART";
    case s_DAVIS6410: return "DAVIS6410";
    case s_WINDNERD:  return "WINDNERD";
    default:          return "none";
  }
}

static const char* lora_module_name(){
  switch(lora_module){
    case LORA_SX1276: return "SX1276";
    case LORA_SX1262: return "SX1262";
    case LORA_LLCC68: return "LLCC68";
    default:          return "none";
  }
}

static const char* baro_name(){
  switch(chip_baro){
    case BARO_BMP280: return "BMP280";
    case BARO_SPL06:  return "SPL06";
    case BARO_BMP3xx: return "BMP3xx";
    case BARO_HP203B: return "HP203B";
    default:          return "none";
  }
}

// First matching flag wins; PM->RCAUSE normally has exactly one bit set.
static const char* reset_cause_name(uint8_t cause){
  if(cause & PM_RCAUSE_POR)   { return "POR"; }
  if(cause & PM_RCAUSE_BOD12) { return "BOD12"; }
  if(cause & PM_RCAUSE_BOD33) { return "BOD33"; }
  if(cause & PM_RCAUSE_WDT)   { return "WDT"; }
  if(cause & PM_RCAUSE_SYST)  { return "SYST"; }
  if(cause & PM_RCAUSE_EXT)   { return "EXT"; }
  return "unknown";
}

// Copies src into dst with the two characters JSON cannot carry raw escaped.
// The station name is the only free-form field, and it comes from a file the
// user wrote, so it cannot be trusted to be JSON-safe.
static void json_escape(char *dst, size_t dst_len, const char *src){
  size_t out = 0;
  for(size_t i = 0; src[i] != '\0' && out + 2u < dst_len; ++i){
    const char c = src[i];
    if(c == '"' || c == '\\'){
      dst[out++] = '\\';
    } else if((unsigned char)c < 0x20u){
      continue; // control characters would break the line framing
    }
    dst[out++] = c;
  }
  dst[out] = '\0';
}

size_t status_build_json(char *out, size_t out_len){
  if(out == nullptr || out_len < 32u){
    return 0;
  }

  char name[STATION_NAME_LEN * 2];
  json_escape(name, sizeof(name), settings.station_name);

  const uint8_t rcause = PM->RCAUSE.reg;
  const uint32_t now = time();

  // Voltages in mV and pressure in Pa: integers only, see status.h.
  const uint32_t vbat_mv   = (uint32_t)(sensor.batt_volt * 1000.0f + 0.5f);
  const int32_t  baro_pa   = (int32_t)(sensor.baro_pressure * 100.0f);
  const int32_t  temp_dc   = (int32_t)(sensor.temperature * 10.0f);   // deci-degrees
  const int32_t  wind_dkmh = (int32_t)(sensor.wind_speed * 10.0f);    // deci-km/h
  const int32_t  gust_dkmh = (int32_t)(sensor.wind_gust * 10.0f);

  const int n = snprintf(out, out_len,
    "{\"fw\":\"%s\""
    ",\"build\":\"%s %s\""
    ",\"bl\":\"%s\""
    ",\"id\":\"%02X%04X\""
    ",\"name\":\"%s\""
    ",\"sensor\":\"%s\""
    ",\"lora\":\"%s\""
    ",\"baro\":\"%s\""
    ",\"settings_ok\":%d"
    ",\"uptime_s\":%lu"
    ",\"rcause\":%u"
    ",\"reset\":\"%s\""
    ",\"vbat_mv\":%lu"
    ",\"soc\":%d"
    ",\"uv\":%d"
    ",\"deep_uv\":%d"
    ",\"uv_seen\":%d"
    ",\"reduced\":%d"
    ",\"wind_age_ms\":%lu"
    ",\"baro_age_ms\":%lu"
    ",\"wind_dkmh\":%ld"
    ",\"gust_dkmh\":%ld"
    ",\"dir\":%d"
    ",\"temp_dc\":%ld"
    ",\"hum\":%d"
    ",\"baro_pa\":%ld"
    ",\"lbt_cnt\":%u"
    ",\"slot\":%u"
    ",\"free_ram\":%d"
    "}",
    VERSION,
    __DATE__, __TIME__,
    get_bootloader_version(),
    FANET_VENDOR_ID, get_fanet_id(),
    name,
    sensor_type_name(),
    lora_module_name(),
    baro_name(),
    settings_ok ? 1 : 0,
    (unsigned long)(now / 1000UL),
    (unsigned)rcause,
    reset_cause_name(rcause),
    (unsigned long)vbat_mv,
    sensor.batt_perc,
    undervoltage ? 1 : 0,
    deep_undervoltage ? 1 : 0,
    state.uv_triggered ? 1 : 0,
    reduced_interval ? 1 : 0,
    (unsigned long)(sensor.last_data ? (now - sensor.last_data) : 0UL),
    (unsigned long)(sensor.last_baro_reading ? (now - sensor.last_baro_reading) : 0UL),
    (long)wind_dkmh,
    (long)gust_dkmh,
    sensor.wind_heading,
    (long)temp_dc,
    sensor.humidity,
    (long)baro_pa,
    (unsigned)state.lbt_counter,
    (unsigned)ota_ab_current_slot(),
    free_ram());

  if(n <= 0 || (size_t)n >= out_len){
    // Never hand out half a JSON document: a consumer would either fail to
    // parse it or, worse, parse a truncated number as a real value.
    snprintf(out, out_len, "{\"error\":\"status truncated\"}");
    return 0;
  }
  return (size_t)n;
}

void status_print_json(){
  // Sized against the worst case the host test drives: a 63 character station
  // name in which every character needs escaping, plus the longest bootloader
  // string. 520 was not enough - it fit the typical 450 byte document and
  // truncated on a long name.
  char buf[768];
  status_build_json(buf, sizeof(buf));
  log_i(buf);
  log_i("\r\n");
  log_flush();
}
