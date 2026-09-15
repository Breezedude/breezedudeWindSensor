#pragma once
#include <Arduino.h>

struct WS85Measurement {
  uint32_t lightLux;
  float uvi;
  float temperatureC;
  uint8_t humidity;
  float windSpeed;
  float gustSpeed;
  uint16_t windDirection;
  float rainfall;
  float absPressure;
  bool valid;
};

class WS85WindSensor {
public:

  bool intervalValid = false;
  uint16_t intervalValue = 0;
  uint32_t lastByte = 0;
  uint32_t baud = 9600;
  uint8_t begun_div = 0;   // CPU divider the baud register was last set for
#ifdef WS85_TRACE
  // WS85_TRACE debug build only: what a standby wakeup does to a frame. Filled
  // along the wake path - sleep() in power.cpp, resync(), poll(),
  // parseResponse() - then printed and cleared by ws85_trace_report() in
  // sensors.cpp, which documents the output. Times are micros().
  uint16_t tr_buffered = 0;       // bytes already in the ring buffer when resync() ran
  uint8_t  tr_seen[40];           // the first bytes poll() read in this wake cycle
  uint8_t  tr_seen_n = 0;         // entries used in tr_seen
  uint16_t tr_seen_total = 0;     // all bytes poll() read in this wake cycle
  uint32_t tr_first_us = 0;       // poll() read the first of them
  uint32_t tr_last_us = 0;        // poll() read the last of them
  uint32_t tr_wake_us = 0;        // back from standby: time zero of the report
  uint8_t  tr_wake_src = 0;       // wakeup_source at that moment
  uint8_t  tr_flag = 0;           // SERCOM0 INTFLAG at that moment
  uint16_t tr_status = 0;         // SERCOM0 STATUS at that moment
  const char *tr_result = "none"; // why parseResponse() gave up: "short" or "crc"
  uint32_t tr_disable_us = 0;     // disable_sercom0_int() done
  uint32_t tr_restart_us = 0;     // resync() done
  uint32_t tr_tc4_us = 0;         // elapsed_time_counter() done
  uint32_t tr_wdt_us = 0;         // wdt_enable() done, only set with WDT=1
  uint32_t tr_lora_us = 0;        // service_lora_during_sleep() done
  uint32_t tr_poll_us = 0;        // read_ws85_uart() entered
  uint32_t tr_ctrla = 0;          // SERCOM0 CTRLA, CTRLB, BAUD and INTENSET
  uint32_t tr_ctrlb = 0;          // right after resync()
  uint16_t tr_baud = 0;
  uint8_t  tr_ie = 0;
  uint8_t  tr_gen = 0xFF;         // GCLK generator feeding the SERCOM0 core clock
  uint8_t  tr_clken = 0;          // and whether that clock channel is enabled
  void tr_reset() {
    tr_buffered = 0; tr_seen_n = 0; tr_seen_total = 0;
    tr_first_us = 0; tr_last_us = 0; tr_result = "none";
  }
#endif


  WS85WindSensor(HardwareSerial* s, uint8_t addr = 0x85)
    : serial(s), slaveAddr(addr) {}

  void begin(uint8_t div_cpu = 1) {
    serial->begin(baud*div_cpu);
    begun_div = div_cpu;
    reset_rx();
  }

  // Wake-up function for low-power mode
  void wake() {
    serial->write(0xFF);       // Send the wake-up byte
    delayMicroseconds(70);     // Wait at least 70µs
  }

  void set_baud_115200(){
    log_v("Setting baud to 115200\r\n");
    // Avoid blocking startup on targets where UART flush can stall indefinitely.
    serial->end();
    serial->begin(9600);
    delay(20);
    set_baud(115200);
    delay(30);
    serial->end();
    serial->begin(115200);
    baud = 115200;
    begun_div = 1;
  }

  void set_baud(uint32_t baud){
    uint8_t d [4] = {0x01, 0x61, 0x00, 0x00};
    if(baud == 4800)  {d[3] = 0x01;}
    if(baud == 9600)  {d[3] = 0x02;} //default after reset (1.1.5) FF 85 06 01 61 00 02 46 6D
    if(baud == 19200) {d[3] = 0x03;} 
    if(baud == 115200){d[3] = 0x04;} // FF 85 06 01 61 00 04 C6 6F
    sendWrite(slaveAddr, 4, d);
  }

  void requestMeasurement() {
    sendRead(slaveAddr, 0x0165, 9);
    expectedRegs = 9;
  }

  void requestAutoSendInterval() {
    sendRead(slaveAddr, 0x9C73, 1);
    expectedRegs = 1;
    readingInterval = true;
    log_i("Requested interval\n");
  }

  void setAutoSendInterval(uint32_t ms) {
    if(ms < 8500) ms = 34*250;
    if(ms > 16383750) ms = 65535ul*250; // ms
    uint16_t value = ms / 250;
    uint8_t data [4] = {0x9C, 0x73, 0x00, 0x00};
    memcpy(&data[2], &value, sizeof(uint16_t));
    sendWrite(slaveAddr,4,data);
    log_i("Set interval to:", ms);
  }

  #define RX_OFFSET 2
  #define WS85_FRAME_LEN 33   // 85 03 1C, 28 payload bytes, CRC
  void reset_rx(){
    // A frame that begins while the station sleeps loses its first bytes:
    // the receiver decodes only once the clock is back. assemble() puts them
    // back. RX_OFFSET is where the bytes go when nothing better fits.
    rx[0] = 0x85;
    rx[1] = 0x03;
    rxPos = RX_OFFSET;
    burstLen = 0;
  }

  // Called after a standby wakeup instead of begin(). disable_sercom0_int() has
  // the USART back on its fast clock, with the baud rate begin() set and the
  // start-of-frame detection off, so the receiver already runs: the bytes it
  // took since then belong to the frame and stay in the buffer. Only the parser
  // starts over. The first bytes of the frame arrive before the clock is back
  // and are lost; assemble() puts them back.
  void resync(uint8_t div_cpu) {
    // The baud register only fits the CPU clock it was computed for. After a
    // change of the divider that needs a real begin(), once.
    if (div_cpu != begun_div) {
      begin(div_cpu);
      return;
    }
#ifdef WS85_TRACE
    tr_buffered = (uint16_t)serial->available(); // WS85_TRACE: kept for the parser
#endif
    reset_rx();
  }

  void poll(uint32_t timeout) {
    while (serial->available()) {
      const uint8_t b = (uint8_t)serial->read();
#ifdef WS85_TRACE
      // WS85_TRACE: what the parser gets, for ws85_trace_report().
      if (tr_seen_n < sizeof(tr_seen)) { tr_seen[tr_seen_n++] = b; }
      tr_seen_total++;
      if (!tr_first_us) { tr_first_us = micros(); }
      tr_last_us = micros();
#endif
      if (burstLen < sizeof(burst)) { burst[burstLen++] = b; }
      lastByte = micros();
    }
    // A measurement frame is 33 bytes: parse as soon as that many are in, or once
    // the line has been idle for the timeout, which means bytes were lost.
    if (burstLen && (burstLen >= WS85_FRAME_LEN || (micros() - lastByte >= timeout))) {
      assemble();
      reset_rx();
    }
  }

  // Measured after a standby wakeup: the first 2 to 4 bytes of a frame arrive
  // before the receiver runs again. Every measurement frame starts the same -
  // address 0x85, function 0x03, byte count 0x1C, model high byte 0x00 - so
  // each possible number of lost bytes is tried with those put back, and the
  // CRC decides. A burst that fits none of them is parsed as received, which
  // reports it as short or broken.
  void assemble() {
    static const uint8_t head[4] = {0x85, 0x03, 0x1C, 0x00};
    for (uint8_t lost = 0; lost <= 4; lost++) {
      if ((uint16_t)burstLen + lost != WS85_FRAME_LEN) { continue; }
      memcpy(rx, head, lost);
      memcpy(rx + lost, burst, burstLen);
      rxPos = WS85_FRAME_LEN;
      const uint16_t rcrc = rx[rxPos - 2] | (rx[rxPos - 1] << 8);
      if (crc16(rx, rxPos - 2) == rcrc) {
        parseResponse();
        return;
      }
    }
    const uint8_t room = (uint8_t)(sizeof(rx) - RX_OFFSET);
    const uint8_t n = burstLen <= room ? burstLen : room;
    rx[0] = 0x85;
    rx[1] = 0x03;
    memcpy(rx + RX_OFFSET, burst, n);
    rxPos = RX_OFFSET + n;
    parseResponse();
  }


  // for debug reading
  int get_char(uint8_t * buf){
    int n = serial->available();
    if (n) {
      *buf = serial->read();
    }
    return n;
  }


  bool available() const { return measurement.valid; }
  
  WS85Measurement get() {  
    WS85Measurement ret = measurement;
    measurement.valid=false; 
    return ret;
  }

  bool intervalAvailable() const { return intervalValid; }
  uint16_t getAutoSendIntervalMs() const { return intervalValue * 250; }

  void printWS85Measurement(const WS85Measurement& m, Stream& out) {
  if (!m.valid) {
    out.println(F("WS85 Measurement: INVALID"));
    return;
  }

  out.println(F("WS85 Measurement:"));
  //out.print(F("  Light: "));        out.print(m.lightLux);        out.println(F(" lux"));
  //out.print(F("  UVI: "));          out.println(m.uvi, 1);
  out.print(F("  Temp: "));         out.print(m.temperatureC, 1); out.println(F(" °C"));
  //out.print(F("  Humidity: "));     out.print(m.humidity);        out.println(F(" %"));
  out.print(F("  Wind: "));         out.print(m.windSpeed, 1);    out.println(F(" km/h"));
  out.print(F("  Gust: "));         out.print(m.gustSpeed, 1);    out.println(F(" km/h"));
  out.print(F("  Wind Dir: "));     out.print(m.windDirection);   out.println(F("°\r\n"));
  //out.print(F("  Rainfall: "));     out.print(m.rainfall, 1);     out.println(F(" mm"));
  //out.print(F("  Pressure: "));     out.print(m.absPressure, 1);  out.println(F(" hPa"));
  out.flush();
}

private:
  HardwareSerial* serial;
  uint8_t slaveAddr;
  uint8_t rx[64];
  uint8_t burst[40];  // bytes of the current burst, as received
  uint8_t burstLen = 0;
  size_t rxPos = 0;
  int expectedRegs = 0;
  bool readingInterval = false;

  WS85Measurement measurement{};

uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;  // poly reversed
            else
                crc >>= 1;
        }
    }
    return crc; // note: Modbus transmits low byte first!
}


  void sendRead(uint8_t id, uint16_t reg, uint16_t qty) {
    wake();
    uint8_t frame[8];
    frame[0] = id;
    frame[1] = 0x03;
    frame[2] = reg >> 8;
    frame[3] = reg & 0xFF;
    frame[4] = qty >> 8;
    frame[5] = qty & 0xFF;
    uint16_t c = crc16(frame, 6);
    frame[6] = c & 0xFF;
    frame[7] = c >> 8;
    serial->write(frame, 8);
  }

  void sendWrite(uint8_t id, uint8_t len, uint8_t* data) {
    
    uint8_t frame[len + 4];
    frame[0] = id;
    frame[1] = 0x06;
    for (int i = 0; i< len; i++){
      frame[2+i] = data[i];
    }
    uint16_t c = crc16(frame, len+2);
    frame[len+2] = c & 0xFF;
    frame[len+3] = c >> 8;
    wake();
    serial->write(frame, len + 4);
  }

  
  bool parseResponse() {
    measurement.valid = false;
    if (rxPos < 27) {
      log_i("WS85 UART only got: ", (uint32_t) rxPos);
#ifdef WS85_TRACE
      tr_result = "short";
#endif
      return false;
    }

    uint16_t rcrc = rx[rxPos - 2] | (rx[rxPos - 1] << 8);
    uint16_t ccrc = crc16(rx, rxPos - 2);

    if (ccrc != rcrc) {
#ifdef WS85_TRACE
      tr_result = "crc";
#endif
      if (debug_enabled()) {
        log_i("WS85 CRC error\r\n");
      }
      return false;
    } else {
     // log_i("CRC ok\r\n");
    }

      //  printf("Payload (%d bytes):", rxPos);
      //  for (int i = 0; i < rxPos; i++) {
      //      printf(" %02X", rx[i]);
      //  }
      //  printf("\n");

    if (rx[1] == 0x03 && rx[0] == slaveAddr) {
      uint8_t count = rx[2];
      if (readingInterval && count == 2) {
        intervalValue = (rx[3] << 8) | rx[4];
        intervalValid = true;
        log_i("interval is: ", intervalValue);
        readingInterval = false;
      } else { // if (count == expectedRegs * 2) 

        auto readReg = [&](int i)->uint16_t {
          return (rx[3 + (2 * i)] << 8) | rx[4 + (2 * i)];
        };

        auto inv = [](uint16_t v) { return v == 0xFFFF; };

        /* https://osswww.ecowitt.net/uploads/20231122/WS90ModbusRTU_V1.0.5_En.pdf --> different?
        85 03 1C  00 85  00 04  00 85  00 00  2A C8  FF FF  FF FF  02 89  FF FF  00 00  00 00  00 30  00 00  FF FF
        85 03 1C  00 85  00 04  00 85  00 00  2A C8  FF FF  FF FF  02 9B  FF FF  00 0B  00 0D  00 07  00 00  FF FF 8B 87
        85 03 1C: Modbus adress 0x85, cmd read, length 0x1C = 28 bytes
        00 85 : sensor model/name
        00 04 : baud rate code
        00 85 : sensor adress
        00 00 : Device ID MSB
        2A C8 : Device ID LSB
        FF FF : Light ( Invalid value, Lux )
        FF FF : UVI ( Invalid value )
        02 88 : GXTS04 temperature (x/10)-40°C
        FF FF : Humidity ( Invalid value )
        00 00 : wind speed ( m/s )
        00 00 : wind gust ( m/s )
        00 2C : wind dir
        00 00 : Rainfall ( mm )
        FF FF : ABS Pressure ( Invalid value, hPa )
        E4 6D : CRC16_modbus
        */

        uint16_t r5 = readReg(5), 
        r6 = readReg(6), 
        r7 = readReg(7),  
        r9 = readReg(9),
        r10 = readReg(10), 
        r11 = readReg(11), 
        r12 = readReg(12);

        measurement.lightLux = inv(r5) ? 0 : uint32_t(r5) * 10;
        measurement.uvi = inv(r6) ? NAN : r6 / 10.0f;
        measurement.temperatureC = inv(r7) ? NAN : (r7 / 10.0f) - 40.0f;
        measurement.humidity = 0;
        measurement.windSpeed = inv(r9) ? NAN : r9 / 10.0f*3.6f;
        measurement.gustSpeed = inv(r10) ? NAN : r10 / 10.0f*3.6f;
        measurement.windDirection = inv(r11) ? 0 : r11;
        measurement.rainfall = inv(r12) ? NAN : r12 / 10.0f;
        measurement.absPressure = 0;
        measurement.valid = true;
        return true;
      }
    }
    return false;
  }
};
