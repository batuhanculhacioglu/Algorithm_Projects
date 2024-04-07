/*
 * I2C_Lib.h
 *
 *  Created on: Mar 29, 2024
 *      Author: batuhanculhacioglu
 */

#ifndef INC_I2C_LIB_H_
#define INC_I2C_LIB_H_

#include "stdint.h"
#include "main.h"

typedef enum{
	mOkey = 0x00,
	mError = 0x01
}mStatus;

typedef struct{
	I2C_HandleTypeDef *hi2c;
	uint16_t DevAddress;
	uint32_t Timeout;
}mI2C;

mStatus mI2C_Config(mI2C *p);
mStatus mI2C_IsDeviceReady(mI2C p);
uint8_t mI2C_FindAllDevice(mI2C p, uint8_t *deviceAddress, uint8_t numberOfDevice); // return numberOfDevice
mStatus mI2C_MemRead(mI2C p, uint16_t memAddress, uint8_t *pData, uint16_t size);
mStatus mI2C_MemWrite(mI2C p, uint16_t memAddress, uint8_t *pData, uint16_t size);

#endif /* INC_I2C_LIB_H_ */
