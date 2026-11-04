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
  #define WS85_HEAD_LEN 9     // bytes every measurement frame starts with, see take_frame()
  void reset_rx(){
    // A frame that begins while the station sleeps loses its first bytes:
    // the receiver decodes only once the clock is back. take_frame() puts them
    // back. RX_OFFSET is where the bytes go when nothing better fits.
    rx[0] = 0x85;
    rx[1] = 0x03;
    rxPos = RX_OFFSET;
    burstLen = 0;
    triedLen = 0;
  }

  // Called after a standby wakeup instead of begin(). disable_sercom0_int() has
  // the USART back on its fast clock, with the baud rate begin() set and the
  // start-of-frame detection off, so the receiver already runs: the bytes it
  // took since then belong to the frame and stay in the buffer. Only the parser
  // starts over. The first bytes of the frame arrive before the clock is back
  // and are lost; take_frame() puts them back.
  void resync(uint8_t div_cpu) {
    // The baud register only fits the CPU clock it was computed for. After a
    // change of the divider that needs a real begin(), once.
    if (div_cpu != begun_div) {
      begin(div_cpu);
      return;
    }
    reset_rx();
  }

  void poll(uint32_t timeout) {
    while (serial->available()) {
      const uint8_t b = (uint8_t)serial->read();
      if (burstLen < sizeof(burst)) { burst[burstLen++] = b; }
      lastByte = micros();
    }
    if (!burstLen) {
      return;
    }
    // With 33 bytes in, the frame can be complete: take it as soon as it checks
    // out instead of waiting for the line to go quiet - tried once per length.
    if (burstLen >= WS85_FRAME_LEN && burstLen != triedLen) {
      triedLen = burstLen;
      if (take_frame()) {
        reset_rx();
        return;
      }
    }
    // Quiet for the timeout: the burst is over, with whatever arrived of it.
    if (micros() - lastByte >= timeout) {
      if (!take_frame()) {
        parse_as_received();
      }
      reset_rx();
    }
  }

  // Makes a 33 byte frame of the burst and parses it, if the CRC checks out.
  //
  // The end of a frame arrives intact. The wakeup costs its beginning, while the
  // receiver starts up - measured on the rig: 2 to 4 bytes. So the burst is
  // aligned to the end of a frame, and what is missing in front comes from the
  // head every measurement frame starts with: the address, function 0x03, byte
  // count 0x1C, then the model 0x0085, the baud rate code 0x0004 for 115200 and
  // the address register.
  //
  // A receiver switched on in the middle of a byte can also decode a byte out of
  // step, which then stands where head bytes belong - seen as E0 in front of
  // 1C 00 85, once in 5 frames in one run and once in 8 in another. So when the
  // frame fails the CRC, the whole head is put over the received bytes and the
  // CRC decides again. Doing that only second leaves a frame that passes as
  // received untouched, even one with another model or baud rate code.
  bool take_frame() {
    const uint8_t head[WS85_HEAD_LEN] = {slaveAddr, 0x03, 0x1C, 0x00, 0x85, 0x00, 0x04, 0x00, slaveAddr};
    if (burstLen + WS85_HEAD_LEN < WS85_FRAME_LEN) {
      return false;
    }
    const uint8_t take = (uint8_t)(burstLen < WS85_FRAME_LEN ? burstLen : WS85_FRAME_LEN);
    const uint8_t missing = (uint8_t)(WS85_FRAME_LEN - take);
    memcpy(rx, head, missing);
    memcpy(rx + missing, burst + (burstLen - take), take);
    rxPos = WS85_FRAME_LEN;
    if (!frame_crc_ok()) {
      memcpy(rx, head, WS85_HEAD_LEN);
      if (!frame_crc_ok()) {
        return false;
      }
    }
    parseResponse();
    return true;
  }

  bool frame_crc_ok() {
    const uint16_t rcrc = (uint16_t)(rx[WS85_FRAME_LEN - 2] | (rx[WS85_FRAME_LEN - 1] << 8));
    return crc16(rx, WS85_FRAME_LEN - 2) == rcrc;
  }

  // No frame in the burst: it is parsed as it came, so that parseResponse()
  // reports why - too few bytes, or a CRC error.
  void parse_as_received() {
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
  uint8_t triedLen = 0;   // burst length take_frame() last tried before the line went quiet
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
    
    // Address, function, the data and the CRC. Every caller writes one register,
    // four data bytes. A fixed buffer rather than a variable-length array, which
    // is a compiler extension - the host test builds with MSVC.
    uint8_t frame[4 + 4];
    if (len > sizeof(frame) - 4) {
      return;
    }
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
      return false;
    }

    uint16_t rcrc = rx[rxPos - 2] | (rx[rxPos - 1] << 8);
    uint16_t ccrc = crc16(rx, rxPos - 2);

    if (ccrc != rcrc) {
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
