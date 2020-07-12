/*

Arduino Library for Texas Instruments HDC1080 Digital Humidity and Temperature Sensor
Written by AA for ClosedCube
---

The MIT License (MIT)

Copyright (c) 2016-2017 ClosedCube Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "HDC1080.h"
#include "iic_transfer_handler.h"
#include "nrf_delay.h"

void HDC1080_setResolution(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature) {
	HDC1080_Registers reg;
	reg.HumidityMeasurementResolution = 0;
	reg.TemperatureMeasurementResolution = 0;

	if (temperature == HDC1080_RESOLUTION_11BIT)
		reg.TemperatureMeasurementResolution = 0x01;

	switch (humidity)
	{
		case HDC1080_RESOLUTION_8BIT:
			reg.HumidityMeasurementResolution = 0x02;
			break;
		case HDC1080_RESOLUTION_11BIT:
			reg.HumidityMeasurementResolution = 0x01;
			break;
		default:
			break;
	}

	HDC1080_writeRegister(reg);
}

HDC1080_SerialNumber HDC1080_readSerialNumber() {
	HDC1080_SerialNumber sernum;
	sernum.serialFirst = HDC1080_readData(HDC1080_SERIAL_ID_FIRST);
	sernum.serialMid = HDC1080_readData(HDC1080_SERIAL_ID_MID);
	sernum.serialLast = HDC1080_readData(HDC1080_SERIAL_ID_LAST);
	return sernum;
}

HDC1080_Registers HDC1080_readRegister() {
	HDC1080_Registers reg;
	reg.rawData = (HDC1080_readData(HDC1080_CONFIGURATION) >> 8);
	return reg;
}

void HDC1080_writeRegister(HDC1080_Registers reg) {
	
	ret_code_t err_code;
	uint8_t sendbuf[2] = {reg.rawData, 0x00};
	
	twi_writeRegisters(HDC1080_ADDR, HDC1080_CONFIGURATION, sendbuf, 2);
	
  nrf_delay_ms(10);
	
}

void HDC1080_heatUp(uint8_t seconds) {
	HDC1080_Registers reg = HDC1080_readRegister();
	reg.Heater = 1;
	reg.ModeOfAcquisition = 1;
	HDC1080_writeRegister(reg);
	ret_code_t err_code;

	uint8_t buf[4];
	for (int i = 1; i < (seconds*66); i++) 
		twi_readRegisters_delay(HDC1080_ADDR, 0x00, buf, 4, 20);
	reg.Heater = 0;
	reg.ModeOfAcquisition = 0;
	HDC1080_writeRegister(reg);
	
}


double HDC1080_readTemperature() {
	int16_t rawT = HDC1080_readData(HDC1080_TEMPERATURE);
	return (rawT / 65536.0) * 165.0 - 40.0;
}

double HDC1080_readHumidity() {
	uint16_t rawH = HDC1080_readData(HDC1080_HUMIDITY);
	return (rawH / 65536.0) * 100.0;
}

uint16_t HDC1080_readManufacturerId() {
	return HDC1080_readData(HDC1080_MANUFACTURER_ID);
}

uint16_t HDC1080_readDeviceId() {
	return HDC1080_readData(HDC1080_DEVICE_ID);
}

int16_t HDC1080_readData(uint8_t pointer) {
	
	ret_code_t err_code;
	
	uint8_t recvbuf[2];
	
	twi_readRegisters_delay(HDC1080_ADDR, pointer, recvbuf, 2, 7);
	
	return ((int16_t)recvbuf[0])<<8 | recvbuf[1];
	
}



