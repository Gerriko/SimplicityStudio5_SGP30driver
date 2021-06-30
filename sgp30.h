/*
* This library is based on an Arduino library written for the SPG30
* Arduino Library written by Ciara Jekel @ SparkFun Electronics, June 18th, 2018
* Modified for Simplicity Studio by CGerrish (Gerrikoio) @ June 2021

* https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library

* Original development environment specifics: Arduino IDE 1.8.5
* Modified development environment specifics: Simplicity Studio 5 (Gecko SDK v 3.2)

* SparkFun labored with love to create this code. Feel like supporting open
* source hardware? Buy a board from SparkFun!
* https://www.sparkfun.com/products/14813


* CRC lookup table from Bastian Molkenthin http://www.sunshine2k.de/coding/javascript/crc/crc_js.html

* Copyright (c) 2015 Bastian Molkenthin

* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#ifndef SGP30_H
#define SGP30_H

#include <stdio.h>
#include <string.h>

#include "em_common.h"
#include "em_chip.h"
#include "sl_status.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"

typedef enum
{
  SGP30_SUCCESS = 0,
  SGP30_ERR_BAD_CRC,
  SGP30_ERR_I2C_TIMEOUT,
  SGP30_SELF_TEST_FAIL,
  SGP30_UNKNOWN_FAIL
} SGP30ERR;


//Initializes sensor for air quality readings
SGP30ERR initAirQuality(void);

//Measure air quality
//Call in regular intervals of 1 second to maintain synamic baseline calculations
//CO2 returned in ppm, Total Volatile Organic Compounds (TVOC) returned in ppb
//Will give fixed values of CO2=400 and TVOC=0 for first 15 seconds after init
//returns false if CRC8 check failed and true if successful
SGP30ERR measureAirQuality(uint16_t* CO2, uint16_t* TVOC);

//Returns the current calculated baseline from
//the sensor's dynamic baseline calculations
//Save baseline periodically to non volatile memory
//(like EEPROM) to restore after new power up or
//after soft reset using setBaseline();
//returns false if CRC8 check failed and true if successful
SGP30ERR getBaseline(uint16_t* baselineCO2, uint16_t* baselineTVOC);

//Updates the baseline to a previous baseline
//Should only use with previously retrieved baselines
//to maintain accuracy
SGP30ERR setBaseline(uint16_t baselineCO2, uint16_t baselineTVOC);

//Set humidity
//humidity value is a fixed point 8.8 bit number
//Value should be absolute humidity from humidity sensor
//default value 0x0F80 = 15.5g/m^3
//minimum value 0x0001 = 1/256g/m^3
//maximum value 0xFFFF = 255+255/256 g/m^3
//sending 0x0000 resets to default and turns off humidity compensation
SGP30ERR setHumidity(uint16_t humidity);

//gives feature set version number (see data sheet)
//returns false if CRC8 check failed and true if successful
SGP30ERR getFeatureSetVersion(uint16_t* featureSetVersion);

//Intended for part verification and testing
//these raw signals are used as inputs to the onchip calibrations and algorithms
SGP30ERR measureRawSignals(uint16_t* H2, uint16_t* ethanol);

//Soft reset - not device specific
//will reset all devices that support general call mode
void generalCallReset(void);

//readout of serial ID register can identify chip and verify sensor presence
//returns false if CRC8 check failed and true if successful
SGP30ERR getSerialID(uint64_t* serialID);

//Sensor runs on chip self test
//returns true if successful
SGP30ERR measureTest(void);

#endif /* SGP30_H */
