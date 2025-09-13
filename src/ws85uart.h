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


  WS85WindSensor(HardwareSerial* s, uint8_t addr = 0x85)
    : serial(s), slaveAddr(addr) {}

  void begin() {
    serial->begin(baud);
    reset_rx();
  }

  // Wake-up function for low-power mode
  void wake() {
    serial->write(0xFF);       // Send the wake-up byte
    delayMicroseconds(70);     // Wait at least 70µs
  }

  void set_baud_115200(){
    log_i("Setting baud to 115200\r\n");
    serial->flush();
    serial->end();
    serial->begin(9600);
    delay(100);
    set_baud(115200);
    delay(60);
    serial->end();
    serial->begin(115200);
    baud = 115200;
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
  void reset_rx(){
    // due to deepsleep wakeup we miss the first bytes
    rx[0] = 0x85;
    rx[1] = 0x03;
    rxPos = RX_OFFSET;
  }

  void poll(uint32_t timeout) {
    if (serial->available()) {
      if (rxPos < sizeof(rx)) {
        rx[rxPos] = serial->read();
        // printf("%02X ", rx[rxPos]);
        
        if(rxPos > RX_OFFSET || ( (rxPos == RX_OFFSET) && (rx[rxPos] == 0x1C ))){
          rxPos++;
        }
        lastByte = micros();
      } else {
        reset_rx();
      }
    }
    if ((rxPos>RX_OFFSET) && (micros() - lastByte >= timeout)) {
      parseResponse();
      reset_rx();
    }
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
  out.print(F("  Gust: "));         out.print(m.gustSpeed, 1);    out.println(F(" km/h)"));
  out.print(F("  Wind Dir: "));     out.print(m.windDirection);   out.println(F("°"));
  //out.print(F("  Rainfall: "));     out.print(m.rainfall, 1);     out.println(F(" mm"));
  //out.print(F("  Pressure: "));     out.print(m.absPressure, 1);  out.println(F(" hPa"));
  out.flush();
}

private:
  HardwareSerial* serial;
  uint8_t slaveAddr;
  uint8_t rx[64];
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
      return false;
    }

    uint16_t rcrc = rx[rxPos - 2] | (rx[rxPos - 1] << 8);
    uint16_t ccrc = crc16(rx, rxPos - 2);

    if (ccrc != rcrc) {
        printf("CRC error: received=0x%04X, calculated=0x%04X\n", rcrc, ccrc);

        // For deeper debugging, you can also dump the payload bytes:
      //  printf("Payload (%d bytes):", rxPos - 2);
      //  for (int i = 0; i < rxPos - 2; i++) {
      //      printf(" %02X", rx[i]);
      //  }
      //  printf("\n");
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

        uint16_t r5 = readReg(5), r6 = readReg(6), r7 = readReg(7), r8 = readReg(8), r9 = readReg(9), r10 = readReg(10), r11 = readReg(11), r12 = readReg(12), r13 = readReg(13);

        measurement.lightLux = inv(r5) ? 0 : uint32_t(r5) * 10;
        measurement.uvi = inv(r6) ? NAN : r6 / 10.0f;
        measurement.temperatureC = inv(r7) ? NAN : (r7 / 10.0f) - 40.0f;
        measurement.humidity = 0;
        measurement.windSpeed = inv(r9) ? NAN : r9 / 10.0f*3.6;
        measurement.gustSpeed = inv(r10) ? NAN : r10 / 10.0f*3.6;
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
