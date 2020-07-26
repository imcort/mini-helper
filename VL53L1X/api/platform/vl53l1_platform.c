
/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/


#include "vl53l1_platform.h"
// #include "vl53l1_platform_log.h"
#include "vl53l1_api.h"

#include "iic_transfer_handler.h"
#include "nrf_delay.h"

// #include "stm32xxx_hal.h"
#include <string.h>

extern uint32_t millis;

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		twi_vl53l1x_seq_write(addr, index, pdata, count);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		twi_vl53l1x_seq_read(addr, index, pdata, count);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		twi_vl53l1x_write(addr, index, data);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		uint8_t pdata[2];
		pdata[0] = data >> 8;
		pdata[1] = data;
	
		twi_vl53l1x_seq_write(addr, index, pdata, 2);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		uint8_t pdata[4];
		pdata[0] = data >> 24;
		pdata[1] = data >> 16;
		pdata[2] = data >> 8;
		pdata[3] = data;
	
		twi_vl53l1x_seq_write(addr, index, pdata, 4);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

//VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
//    VL53L1_Error Status = VL53L1_ERROR_NONE;
//    return Status;
//}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		twi_vl53l1x_read(addr, index, data);
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		uint8_t pdata[2];
		twi_vl53l1x_seq_read(addr, index, pdata, 2);
	
		*data = ((uint16_t)pdata[0] << 8) | pdata[1];
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
	
		uint8_t addr = (Dev->I2cDevAddr) >> 1;
	
		uint8_t pdata[4];
		twi_vl53l1x_seq_read(addr, index, pdata, 4);
	
		*data = ((uint16_t)pdata[0] << 24) | (pdata[1] << 16) | (pdata[2] << 16) | pdata[3];
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	*ptick_count_ms = millis;
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

//#define trace_print(level, ...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

//VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
//{
//	VL53L1_Error status  = VL53L1_ERROR_NONE;
//	return status;
//}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	
	nrf_delay_ms(wait_ms);
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
	
	nrf_delay_us(wait_us);
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	uint8_t data;
  VL53L1_Error status;

  while (timeout_ms > 0)
  {
    status = VL53L1_RdByte(pdev, index, &data);
    if (status != VL53L1_ERROR_NONE) { return status; }
    if ((data & mask) == value) { return VL53L1_ERROR_NONE; }
    nrf_delay_ms(poll_delay_ms);
    timeout_ms -= poll_delay_ms < timeout_ms ? poll_delay_ms : timeout_ms;//min(poll_delay_ms, timeout_ms);
  }

  return VL53L1_ERROR_TIME_OUT;
}




