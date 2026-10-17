/**************************************************************************/
/*
        Distributed with a free-will license.
        Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
        HP203B
        This code is designed to work with the HP203B_I2CS I2C Mini Module available from ControlEverything.com.
        https://shop.controleverything.com/products/precision-barometer-and-altimeter?variant=25687549067#tabs-0-product_tabset-2
*/
/**************************************************************************/
#include "Arduino.h"

extern Uart Serial2;
#include <Wire.h>

#include "HP203B.h"

// I2C without the Wire library's blocking loops. The core's SERCOM master
// waits for ever on a bus that a slave holds, and that hung a station until it
// was powered down (the firmware clocks a stuck bus free at boot now, but a
// chip can die mid-transfer at any time). Every wait here gives up after
// I2C_TIMEOUT_US and reports failure, which the readers turn into "no data".
// Wire.begin() has configured SERCOM3 - clock, pins, baud rate, smart mode -
// this only drives it, the same way TwoWire does, minus the endless loops.
#define I2C_TIMEOUT_US 5000UL
static Sercom * const i2c = SERCOM3;   // PERIPH_WIRE of the ItsyBitsy M0 variant

static bool i2c_wait_sync(void)
{
    uint32_t t0 = micros();
    while (i2c->I2CM.SYNCBUSY.bit.SYSOP) {
        if (micros() - t0 > I2C_TIMEOUT_US) return false;
    }
    return true;
}

static void i2c_stop(void)
{
    i2c->I2CM.CTRLB.bit.CMD = 3;   // stop condition
    i2c_wait_sync();
}

// After the address of a read, or after any written byte, the bus tells with
// MB (write) or SB (read) that it is done. MB during a read means the address
// was not acknowledged.
static bool i2c_wait_done(bool read)
{
    uint32_t t0 = micros();
    for (;;) {
        if (read ? i2c->I2CM.INTFLAG.bit.SB : i2c->I2CM.INTFLAG.bit.MB) return true;
        if (read && i2c->I2CM.INTFLAG.bit.MB) return false;
        if (micros() - t0 > I2C_TIMEOUT_US) return false;
    }
}

static bool i2c_start(uint8_t addr, bool read)
{
    // Bus state: 0 unknown, 1 idle, 2 owner (a repeated start is fine), 3 busy.
    if (i2c->I2CM.STATUS.bit.BUSSTATE == 0) {
        i2c->I2CM.STATUS.bit.BUSSTATE = 1;
        if (!i2c_wait_sync()) return false;
    }
    uint32_t t0 = micros();
    while (i2c->I2CM.STATUS.bit.BUSSTATE == 3) {
        if (micros() - t0 > I2C_TIMEOUT_US) return false;
    }
    i2c->I2CM.ADDR.bit.ADDR = (uint8_t)((addr << 1) | (read ? 1 : 0));
    if (!i2c_wait_done(read)) { i2c_stop(); return false; }
    if (i2c->I2CM.STATUS.bit.RXNACK) { i2c_stop(); return false; }
    return true;
}

static bool i2c_write_byte(uint8_t b)
{
    i2c->I2CM.DATA.bit.DATA = b;
    if (!i2c_wait_done(false)) { i2c_stop(); return false; }
    if (i2c->I2CM.STATUS.bit.RXNACK) { i2c_stop(); return false; }
    return true;
}

// One received byte. The last one is answered with NACK and a stop, any other
// with ACK and the request for the next - the order TwoWire uses in smart mode.
static bool i2c_read_byte(uint8_t *out, bool last)
{
    uint32_t t0 = micros();
    while (!i2c->I2CM.INTFLAG.bit.SB) {
        if (i2c->I2CM.INTFLAG.bit.MB || micros() - t0 > I2C_TIMEOUT_US) { i2c_stop(); return false; }
    }
    i2c->I2CM.CTRLB.bit.ACKACT = last ? 1 : 0;
    i2c_wait_sync();
    i2c->I2CM.CTRLB.bit.CMD = last ? 3 : 2;
    i2c_wait_sync();
    *out = i2c->I2CM.DATA.bit.DATA;
    return true;
}

/**************************************************************************/
/*
        Writes 8-bits to the destination register
*/
/**************************************************************************/
static bool writeRegister(uint8_t i2cAddress, uint8_t reg)
{
    if (!i2c_start(i2cAddress, false)) return false;
    if (!i2c_write_byte(reg)) return false;
    i2c_stop();
    return true;
}

static bool readSingleRegister(uint8_t i2cAddress, uint8_t reg, uint8_t *value)
{
    if (!writeRegister(i2cAddress, reg)) return false;
    if (!i2c_start(i2cAddress, true)) return false;
    return i2c_read_byte(value, true);
}

/**************************************************************************/
/*
    Reads 24-bits from the specified destination register
*/
/**************************************************************************/
static bool readRegister3(uint8_t i2cAddress, uint8_t reg, uint32_t *value)
{
    if (!writeRegister(i2cAddress, reg)) return false;
    if (!i2c_start(i2cAddress, true)) return false;
    uint8_t b0 = 0, b1 = 0, b2 = 0;
    if (!i2c_read_byte(&b0, false)) return false;
    if (!i2c_read_byte(&b1, false)) return false;
    if (!i2c_read_byte(&b2, true)) return false;
    *value = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;
    return true;
}

/**************************************************************************/
/*
        Instantiates a new HP203B class with appropriate properties
*/
/**************************************************************************/
void HP203B::getAddr_HP203B(uint8_t i2cAddress)
{
    hp_i2cAddress = i2cAddress;
    hp_conversionDelay = HP203B_CONVERSIONDELAY;
}

/**************************************************************************/
/*
        Sets up the Hardware
*/
/**************************************************************************/
bool HP203B::begin(uint8_t i2cAddress, hpOSR_t osr)
{
    hp_i2cAddress = i2cAddress;
    hp_osr = osr;
    // Only getAddr_HP203B() used to set this, and nothing calls it: the delay
    // after the soft reset below was 0, the first conversion requested right
    // after it was lost in the reset, and the first reading of every boot came
    // back empty. Every later request worked, which is how it went unnoticed.
    hp_conversionDelay = HP203B_CONVERSIONDELAY;
    //Wire.begin();
    
     //Reset();
    // delay(hp_conversionDelay);
    uint8_t ret = 0;   // stays 0 when nothing answers: not found
    readSingleRegister(hp_i2cAddress, HP203B_CMD_READ_REG + HP203B_REG_PARA, &ret);
    //Serial2.println(ret,HEX);
    Reset();
    return (ret != 0x00); // should be 0x80, but not always is?
}

/**************************************************************************/
/*
        Sets the Soft Reset Command
        The Sevice will Immediately be Reset No Matter What it is Working On
*/
/**************************************************************************/
void HP203B::Reset()
{
    writeRegister(hp_i2cAddress, HP203B_CMD_SOFT_RESET);
    delay(hp_conversionDelay);
}

/**************************************************************************/
/*
        Sets the OSR Command Value to select Decimation Rate of the Internal Digital Filter
*/
/**************************************************************************/
void HP203B::setOSR(hpOSR_t osr)
{
    hp_osr = osr;
}

/**************************************************************************/
/*
        Gets the OSR Command Value to select Decimation Rate of the Internal Digital Filter
 */
/**************************************************************************/
hpOSR_t HP203B::getOSR()
{
    return hp_osr;
}

/**************************************************************************/
/*
        Reads up the Device comprising of a Pressure, Altitude & Temperature Sensor
*/
/**************************************************************************/
void HP203B::Measure_Sensor(void)
{
    // Read all the Sensors
    Measure_Pressure();
    Measure_Altitude();
    Measure_Temperature();
}


void HP203B::startMeasure()
{
    // Set Up the Configuration for the Pressure Sensor
    uint8_t command =   
                        HP203B_CMD_CONVERT         |  // Convert the Sensor Output to the Digital Values
                        HP203B_CMD_CHNL_PRESTEMP;     // Sensor Pressure and Temperature Channel
    
    command |= hp_osr;       // OSR
    
    // Write the configuration to the Pressure Sensor
    writeRegister(hp_i2cAddress, command);
}

/**************************************************************************/
/*
        Reads 20-bits from the destination register
        Reads the results for Digital Pressure Value
*/
/**************************************************************************/
void HP203B::Measure_Pressure()
{
    // Reads the pressure value
    uint32_t pressure = 0;   // stays 0 when the read fails: no data
    readRegister3(hp_i2cAddress, HP203B_CMD_READ_P, &pressure);
    hp_sensorData.P = pressure /100.0;
}

/**************************************************************************/
/*
        Reads 20-bits from the destination register
        Reads the results for Digital Altitude Value
*/
/**************************************************************************/
void HP203B::Measure_Altitude()
{
    // Reads the Altitude value
    uint32_t altitude = 0;
    readRegister3(hp_i2cAddress, HP203B_CMD_READ_A, &altitude);
    hp_sensorData.A = altitude /100.0;
}

/**************************************************************************/
/*
        Reads 20-bits from the destination register
        Reads the results for Digital Temperature Value
*/
/**************************************************************************/
void HP203B::Measure_Temperature()
{
    // Reads the Temperature value
    uint32_t temperature = 0;
    readRegister3(hp_i2cAddress, HP203B_CMD_READ_T, &temperature);
    hp_sensorData.T = temperature /100.0;
}