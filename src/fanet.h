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

    unsigned int baro :16;

    unsigned int charge        :8;
  } __attribute__((packed)) fanet_packet_t4;


  size_t pack_weatherdata(const weatherData *wData, uint8_t *buffer);

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

  enum {
    HWINFO_DEBUG_DYNAMIC  = 0x01,
    HWINFO_DEBUG_OTA      = 0x02,
    HWINFO_DEBUG_STATIC   = 0x03,
    HWINFO_DEBUG_REPEATER = 0x04,
  };

  // Debug data appended after HW Info payload.
  // First byte is the decode type selecting one of the grouped payloads below.
  // The byte layout is the contract with the decoders, see doc/hwinfo-protocol.md.

  // The cause of the last reset, 3 bits in hwinfo_debug_t1::resets. POR is a
  // code of its own so the decoder can tell a power cycle (count back to 0,
  // cause POR) from the count wrapping (count back to 0, any other cause).
  enum {
    HWINFO_RESET_POR     = 0,
    HWINFO_RESET_WDT     = 1,
    HWINFO_RESET_SYST    = 2,  // software reset: REBOOT, OTA, config update
    HWINFO_RESET_BOD     = 3,
    HWINFO_RESET_EXT     = 4,
    HWINFO_RESET_SEVERAL = 5,  // more than one of the above set at once
    HWINFO_RESET_UNKNOWN = 6,
  };

  // Awake duty since the previous dynamic report, 2 bits in
  // hwinfo_debug_t1::flags. A station that only wakes to measure and send sits
  // around a tenth of a per cent; anything above a few per cent is a sensor or
  // UART path that keeps the CPU busy.
  enum {
    HWINFO_AWAKE_BELOW_1_PCT  = 0,
    HWINFO_AWAKE_BELOW_5_PCT  = 1,
    HWINFO_AWAKE_BELOW_20_PCT = 2,
    HWINFO_AWAKE_20_PCT_UP    = 3,
  };

  // Debug data decode type 0x01: fast-changing runtime data
  typedef struct {
    uint16_t vbatt_mv;       // battery voltage in mV
    uint8_t  batt_perc;      // battery state-of-charge 0-100 %
    uint8_t  flags;          // bit0: charging active, bit1: charge done, bit2-3: awake duty class
    uint16_t rx_count;       // total FANET packets received since boot
    uint16_t forward_count;  // total FANET packets forwarded since boot
    uint8_t  lbt_counter;    // LBT retry counter, resets on boot
    uint8_t  resets;         // bit0-4: resets since power-on mod 32, bit5-7: last cause (HWINFO_RESET_*)
  } __attribute__((packed)) hwinfo_debug_t1;

  // Debug data decode type 0x02: OTA / update-relevant data
  typedef struct {
    uint16_t version_bcd;      // major.minor.patch packed into nibbles, e.g. 0x072
    uint32_t nonce;            // challenge nonce for the OTA offer reply
    uint16_t slot_capacity_kb; // currently available inactive slot size in KiB
    uint8_t  ota_proto;        // OTA protocol version
    uint8_t  ota_state;        // bit0: currently active slot (0=A, 1=B)
  } __attribute__((packed)) hwinfo_debug_t2;

  // Debug data decode type 0x03: static / slowly changing configuration
  typedef struct {
    uint8_t  sensor_type  :5;       // wind sensor type (maps to Sensor enum: 0=invalid,1=WS80,2=WS85,3=WS85_UART,4=DAVIS6410,5=WINDNERD)
    uint8_t  use_baro     :1;       // barometer enabled
    uint8_t  uv_triggered :1;       // undervoltage triggered
    uint8_t  lbt          :1;       // LBT enabled
    uint8_t  lora_rssi_threshold;   // inverted RSSI threshold, e.g. 104 for -104 dBm
    uint8_t  sensor_integ_s;        // sensor integration time in seconds
    uint8_t  reduce_interval_voltage; // threshold encoded as voltage*100 - 200 (e.g. 141 = 3.41 V)
    // Which board the firmware decided it runs on, not the board itself: two
    // ID pins cannot name every revision, but a wrong pin map is what breaks a
    // station, and that is what this byte shows. hw.h.
    uint8_t  hw_rev;                // bit0-3: pin map (0 = up to 2.4, 1 = 2.5), bit4: forced by HW_REVISION,
                                    // bit5: ID pin overruled by the divider, bit6-7: raw ID pins
    uint16_t bl_version;            // bootloader: major << 12 | minor << 6 | patch (4/6/6 bit), 0 = unknown
  } __attribute__((packed)) hwinfo_debug_t3;

  // Debug data decode type 0x04: repeater state. Only stations with the
  // repeater on send it. The header is followed by one entry per station
  // the repeater follows, at most HWINFO_REPEATER_MAX_STATIONS per packet;
  // the decoder gets the count from the packet length.
  enum {
    HWINFO_REPEATER_OFF        = 0,  // repeater off (never sent)
    HWINFO_REPEATER_CLASSIC    = 1,  // listens continuously
    HWINFO_REPEATER_SCHED_FULL = 2,  // scheduled, but listening continuously: battery full and charging
    HWINFO_REPEATER_SCHED      = 3,  // scheduled, the receiver follows the learned rhythms
  };
  #define HWINFO_REPEATER_MAX_STATIONS 4
  typedef struct {
    uint8_t  flags;      // bit0-1: mode (HWINFO_REPEATER_*), bit2: scan window open, bit3-5: stations in
                         // the table, bit6: a station was dropped since the previous report to make
                         // room for another, bit7: receiver off because of undervoltage
    uint8_t  rx_duty;    // per cent of the time the receiver was on since the previous report
    uint8_t  forwarded;  // packets forwarded since the previous report, capped at 255
  } __attribute__((packed)) hwinfo_debug_t4;
  typedef struct {
    uint8_t  vendor;     // FANET address of the followed station
    uint16_t address;
    uint8_t  period_s;   // learned weather period in seconds, 0 while unknown
    uint8_t  status;     // bit0-2: weather state (SchedState), bit3-4: empty windows in a row, capped at 3,
                         // bit5-6: HWInfo state (SchedState, never RESYNC), bit7: reserved
  } __attribute__((packed)) hwinfo_repeater_station_t;

  // The sizes the decoders count on, see doc/hwinfo-protocol.md.
  static_assert(sizeof(hwinfo_debug_t1) == 10, "HWInfo type 1 layout changed");
  static_assert(sizeof(hwinfo_debug_t2) == 10, "HWInfo type 2 layout changed");
  static_assert(sizeof(hwinfo_debug_t3) == 7,  "HWInfo type 3 layout changed");
  static_assert(sizeof(hwinfo_debug_t4) == 3,  "HWInfo type 4 layout changed");
  static_assert(sizeof(hwinfo_repeater_station_t) == 5, "HWInfo type 4 station entry changed");

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

    // debug data: debug_type selects one grouped payload (dynamic / ota / static / repeater)
    uint8_t         debug_type;
    hwinfo_debug_t1 debug_dynamic;
    hwinfo_debug_t2 debug_ota;
    hwinfo_debug_t3 debug_static;
    hwinfo_debug_t4 debug_repeater;
    uint8_t         repeater_station_count;
    hwinfo_repeater_station_t repeater_stations[HWINFO_REPEATER_MAX_STATIONS];
  } hwInfoData;


  // Returns total packet length written into buffer.
  // buffer must be at least 32 bytes.
  size_t pack_hwinfo(const hwInfoData *data, uint8_t *buffer);