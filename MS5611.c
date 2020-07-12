/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.

Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>

#include "MS5611.h"
#include "iic_transfer_handler.h"
#include "nrf_delay.h"


uint16_t fc[6];
uint8_t ct;
uint8_t uosr;
int32_t TEMP2;
int64_t OFF2, SENS2;

static void reset(void);
static void readPROM(void);

static uint16_t readRegister16(uint8_t reg);
static uint32_t readRegister24(uint8_t reg);
	
static uint32_t readRawTemperature(void);
static uint32_t readRawPressure(void);


bool MS5611begin(ms5611_osr_t osr)
{
	
		reset();

    MS5611setOversampling(osr);

    nrf_delay_ms(100);

    readPROM();

    return true;
}

// Set oversampling value
void MS5611setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

static void reset(void)
{
    twi_writeRegisters(MS5611_ADDRESS, MS5611_CMD_RESET, NULL, 0);
}

static void readPROM(void)
{
    for (uint8_t offset = 0; offset < 6; offset++)
    {
	fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

static uint32_t readRawTemperature(void)
{

    twi_writeRegisters(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + uosr, NULL, 0);

    nrf_delay_ms(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

static uint32_t readRawPressure(void)
{
    
	
	twi_writeRegisters(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + uosr, NULL, 0);

    nrf_delay_ms(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

int32_t MS5611readPressure()
{
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

		int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

		OFF2 = 0;
		SENS2 = 0;

		if (TEMP < 2000)
		{
				OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
				SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
		}

		if (TEMP < -1500)
		{
				OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
				SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
		}

		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
    

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

float MS5611readTemperature()
{
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

		if (TEMP < 2000)
		{
				TEMP2 = (dT * dT) / (2 << 30);
		}
    

    TEMP = TEMP - TEMP2;

    return ((float)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
float MS5611getAltitude(float pressure, float seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((float)pressure / (float)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
float MS5611getSeaLevel(float pressure, float altitude)
{
    return ((float)pressure / pow(1.0f - ((float)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
static uint16_t readRegister16(uint8_t reg)
{
    uint8_t value[2];
	
		twi_readRegisters_stop(MS5611_ADDRESS, reg, value, 2);

    return (value[0] << 8) | value[1];
}

// Read 24-bit from register (oops XSB, MSB, LSB)
static uint32_t readRegister24(uint8_t reg)
{
    uint8_t value[3];
	
		twi_readRegisters_stop(MS5611_ADDRESS, reg, value, 3);

    return ((int32_t)value[0] << 16) | ((int32_t)value[1] << 8) | value[2];
}
