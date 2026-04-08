 #pragma once
 #include <Arduino.h>
 #include "defines.h"

  enum {
    FANET_PCK_TYPE_TRACKING = 0x01,
    FANET_PCK_TYPE_NAME     = 0x02,
    FANET_PCK_TYPE_WEATHER  = 0x04,
    FANET_PCK_TYPE_HWINFO   = 0x0A   // HW Info (replaces type 8), recommended interval: every 10 min
  };

  // Device subtypes for manufacturer 0xBD (Breezedude)
  enum {
    FANET_BD_DEVICE_TRANSMITTER = 0x01  // Breezedude Transmitter
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

  // ---- HW Info (Type 0x0A) ------------------------------------------------

  // Byte 0: subheader flags
  // Payload is appended in bit order 6 → 1, Extended Header 7 → 0 once defined.
  typedef struct {
    uint8_t ext_header    :1;  // bit 0: Extended Header
    uint8_t tbd           :2;  // bit 1-2: TBD
    uint8_t bRxRssi       :1;  // bit 3: Rx RSSI (+1byte RSSI+50, +3byte FANET addr)
    uint8_t bUptime       :1;  // bit 4: Uptime (+2byte, minutes)
    uint8_t bIcao         :1;  // bit 5: ICAO address (+3byte)
    uint8_t bSubtypeBuild :1;  // bit 6: Hardware Subtype + Build Date (bytes 1-3)
    uint8_t bPingPong     :1;  // bit 7: Ping-Pong Request (unicast only, no data)
  } __attribute__((packed)) hwinfo_byte0_t;

  // Bytes 2-3 of HW Info: Firmware Build Date (16-bit, little-endian)
  typedef struct {
    uint16_t day         :5;  // bit  0-4:  Day (1-31)
    uint16_t month       :4;  // bit  5-8:  Month (1-12)
    uint16_t year_offset :6;  // bit  9-14: Year - 2019 (0 → 2019, …)
    uint16_t develop     :1;  // bit  15:   0 = Release, 1 = Develop/Experimental
  } __attribute__((packed)) hwinfo_builddate_t;

  // Debug data appended after HW Info payload.
  // First byte is the decode type; struct below is for decode type 0x01.
  typedef struct {
    uint16_t vbatt_mv;          // battery voltage in mV
    uint8_t  batt_perc;         // battery state-of-charge 0-100 %
    uint8_t  pv_state;          // bit0: charging active, bit1: charge done
    // Config byte
    unsigned int sensor_type  :5;   // wind sensor type (maps to Sensor enum: 0=invalid,1=WS80,2=WS85,3=WS85_UART,4=DAVIS6410,5=WINDNERD)
    unsigned int use_baro     :1;   // barometer enabled
    unsigned int uv_triggered :1;   // undervoltage triggered
    unsigned int lbt          :1;   // LBT enabled
    uint8_t  lbt_counter;           // LBT retry counter, resets on boot
    uint8_t  lora_rssi_threshold;   // LBT RSSI threshold in dBm, inversed to fit in uint8_t (stored as -rssi_threshold, e.g. 104 for -104dBm)
    // Compact BCD version: major.minor.patch in 3 nibbles, e.g. 0.7.1 -> 0x071
    uint16_t version_bcd;
  } __attribute__((packed)) hwinfo_debug_t1;

  // Debug data decode type 0x02: LoRa + station configuration
  typedef struct {
    uint16_t gust_age;       // gust age in seconds
    uint16_t wind_age;        // wind age in seconds
    uint8_t sensor_integ_s; // sensor integration time in seconds (e.g. for pulse counting)
    uint8_t reduce_interval_voltage; // voltage threshold for reduced send interval in 2V + 0.01V (e.g. 141 = 3.41V)

  } __attribute__((packed)) hwinfo_debug_t2;

  // Input data passed to pack_hwinfo()
  typedef struct {
    uint16_t vid;
    uint16_t fanet_id;

    // bit 6 payload
    bool     bSubtypeBuild;
    uint8_t  device_type;   // e.g. FANET_BD_DEVICE_TRANSMITTER
    bool     develop_mode;
    uint8_t  build_day;
    uint8_t  build_month;
    uint16_t build_year;

    // bit 4 payload
    bool     bUptime;
    uint16_t uptime_min;    // uptime in minutes

    // debug data: debug_type selects which struct is serialised (0x01 / 0x02 / ...)
    uint8_t         debug_type;
    hwinfo_debug_t1 debug;
    hwinfo_debug_t2 debug2;
  } hwInfoData;


  // Returns total packet length written into buffer.
  // buffer must be at least 32 bytes.
  size_t pack_hwinfo(const hwInfoData *data, uint8_t *buffer);