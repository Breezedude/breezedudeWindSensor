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

/**************************************************************************/
/*
        Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static uint8_t i2cread(void)
{
        return Wire.read();
}

/**************************************************************************/
/*
        Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static void i2cwrite(uint8_t x)
{
        Wire.write((uint8_t)x);
}

/**************************************************************************/
/*
        Writes 8-bits to the destination register
*/
/**************************************************************************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg)
{
    Wire.beginTransmission(i2cAddress);
    i2cwrite((uint8_t)reg);
    Wire.endTransmission();
}

static uint8_t readSingleRegister(uint8_t i2cAddress, uint8_t reg)
{
    Wire.beginTransmission(i2cAddress);
    i2cwrite((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress, (uint8_t)1);
    return i2cread();
}

/**************************************************************************/
/*
    Reads 24-bits from the specified destination register
*/
/**************************************************************************/
static uint32_t readRegister3(uint8_t i2cAddress, uint8_t reg)
{
    Wire.beginTransmission(i2cAddress);
    i2cwrite((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress, (uint8_t)3);
    return (uint32_t)((int32_t)i2cread() << 16) | ((int32_t)i2cread() << 8) | i2cread();
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
    //Wire.begin();
    
     //Reset();
    // delay(hp_conversionDelay);
    uint8_t ret = readSingleRegister(hp_i2cAddress, HP203B_CMD_READ_REG + HP203B_REG_PARA);
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
    uint32_t pressure = readRegister3(hp_i2cAddress, HP203B_CMD_READ_P);
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
    uint32_t altitude = readRegister3(hp_i2cAddress, HP203B_CMD_READ_A);
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
    uint32_t temperature = readRegister3(hp_i2cAddress, HP203B_CMD_READ_T);
    hp_sensorData.T = temperature /100.0;
}