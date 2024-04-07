/*
 * I2C_Lib.c
 *
 *  Created on: Mar 29, 2024
 *      Author: batuhanculhacioglu
 */

#include "mI2C.h"


mStatus mI2C_Config(mI2C *p)
{
	if(p->hi2c->Instance != NULL)
	{
		if(p->Timeout == 0)
			p->Timeout = 100;
		if(p->DevAddress == 0x00)
			return mError;
		return mOkey;
	}
	else
		return mError;
}

mStatus mI2C_IsDeviceReady(mI2C p)
{
	if(HAL_I2C_IsDeviceReady(p.hi2c, p.DevAddress, 3, 100)==HAL_OK)
		return mOkey;
	return mError;
}

uint8_t mI2C_FindAllDevice(mI2C p, uint8_t *deviceAddress, uint8_t numberOfDevice)
{
	uint8_t x = 0;
	for(int i = 0; i <= 255; i++)
		if(HAL_I2C_IsDeviceReady(p.hi2c, i, 3, p.Timeout)==HAL_OK)
		{
			deviceAddress[x] = i;
			x++;
			if(x >= numberOfDevice)
				return x;
		}

	return x;
}

mStatus mI2C_MemRead(mI2C p, uint16_t memAddress, uint8_t *pData, uint16_t size)
{
	if(HAL_I2C_Mem_Read(p.hi2c, p.DevAddress, memAddress, 1, pData, size, p.Timeout)!= HAL_OK)
		return mError;
	else
		return mOkey;
}

mStatus mI2C_MemWrite(mI2C p, uint16_t memAddress, uint8_t *pData, uint16_t size)
{
	if(HAL_I2C_Mem_Write(p.hi2c, p.DevAddress, memAddress, 1, pData, size, p.Timeout)!= HAL_OK)
		return mError;
	else
		return mOkey;
}
