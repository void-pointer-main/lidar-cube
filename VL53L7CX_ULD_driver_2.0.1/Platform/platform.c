/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "platform.h"
#include <stdio.h>

uint8_t VL53L7CX_RdByte(
		VL53L7CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{

	/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t buf[2] = {RegisterAdress >> 8, RegisterAdress & 0xFF};
	if (i2c_write_blocking(p_platform->i2c_inst, p_platform->address, buf, 2, true) != 2) {
		return 8;
	}

	if(i2c_read_blocking(p_platform->i2c_inst, p_platform->address, p_value, 1, false) != 1) {
		return 255;
	}

	return 0;
}

uint8_t VL53L7CX_WrByte(
		VL53L7CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t buf[3] = {RegisterAdress >> 8, RegisterAdress & 0xFF, value};
	int ret = i2c_write_blocking(p_platform->i2c_inst, p_platform->address, buf, 3, false);
	if (ret != 3) {
		// printf("sam: %d\n", ret);
		return 8;
	}

	return 0;
}

uint8_t VL53L7CX_WrMulti(
		VL53L7CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t buf[2] = {RegisterAdress >> 8, RegisterAdress & 0xFF};
	int ret = i2c_write_burst_blocking(p_platform->i2c_inst, p_platform->address, buf, 2);
	if (ret != 2) {
		// printf("ad: %d\n", ret);
		return 8;
	}

	ret = i2c_write_blocking(p_platform->i2c_inst, p_platform->address, p_values, size, false);
	if (ret != size) {
		// printf("buf: %d\n", ret);
		return 8;
	}

	return 0;
}

uint8_t VL53L7CX_RdMulti(
		VL53L7CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{	
	/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t buf[2] = {RegisterAdress >> 8, RegisterAdress & 0xFF};
	if (i2c_write_burst_blocking(p_platform->i2c_inst, p_platform->address, buf, 2) != 2) {
		return 8;
	}

	if(i2c_read_blocking(p_platform->i2c_inst, p_platform->address, p_values, size, false) != size) {
		return 255;
	}

	return 0;
}

// NOT IMPLEMENTED (NO ACCESS TO PINS)
uint8_t VL53L7CX_Reset_Sensor(
		VL53L7CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	VL53L7CX_WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	VL53L7CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L7CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L7CX_WaitMs(
		VL53L7CX_Platform *p_platform,
		uint32_t TimeMs)
{
	/* Need to be implemented by customer. This function returns 0 if OK */
	sleep_ms(TimeMs);

	return 0;
}
