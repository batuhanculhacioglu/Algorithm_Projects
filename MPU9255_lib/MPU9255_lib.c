/*
 * MPU9255_lib.c
 *
 *  Created on: Dec 7, 2023
 *      Author: batuhanculhacioglu
 */

#include "MPU9255_lib.h"
#include "math.h"

MPU9255_init_e mpu9255_s;
float calibrationAX = 0, calibrationaY = 0, calibrationAZ = 0, calibrationGX = 0, calibrationGY = 0, calibrationGZ = 0;


void MPU9255_Init(MPU9255_init_e *mpu9255_init)
{
	mpu9255_s.i2c 			= mpu9255_init->i2c;
	mpu9255_s.ACCEL_G_FORCE = mpu9255_init->ACCEL_G_FORCE;
	mpu9255_s.GYRO_DPS 		= mpu9255_init->GYRO_DPS;
	mpu9255_s.Calibration 	= mpu9255_init->Calibration;

	if(MPU9255_WhoAmI()!=HAL_OK)
		while(1); // to do error led blink
	else
	{
		uint8_t dummy_byte;
		dummy_byte = mpu9255_init->PWR_MGMT_1;
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_PWR_MGMT_1, 1, &dummy_byte, 1, 100);

		dummy_byte = mpu9255_init->PWR_MGMT_2;
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_PWR_MGMT_2, 1, &dummy_byte, 1, 100);

		dummy_byte = mpu9255_init->SampleRateDivider;
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_SMPLRT_DIV, 1, &dummy_byte, 1, 100);

		dummy_byte = mpu9255_init->ACCEL_G_FORCE;
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_ACCEL_CONFIG, 1, &dummy_byte, 1, 100); //2g
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_ACCEL_CONFIG2, 1, &dummy_byte, 1, 100);

		dummy_byte = mpu9255_init->GYRO_DPS;
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_GYRO_CONFIG, 1, &dummy_byte, 1, 100); //250dps
	}

	if(mpu9255_s.Calibration != Calibration_OFF)
	{
		MPU9255_OffSetValues();
	}
}

HAL_StatusTypeDef MPU9255_WhoAmI()
{
	uint8_t dummy;
	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_WHO_AM_I, 1, &dummy, 1, 100);
	if(dummy != 0x71)
		return HAL_ERROR;
	else
		return HAL_OK;
}

void MPU9255_RawReadAllData(MPU9255_rawData_e *mpu9255_raw)
{
	uint8_t rdata[14] = {0};
	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_ACCEL_XOUT_H, 1, rdata, 14, 1000);
	mpu9255_raw->aX 	= (((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
	mpu9255_raw->aY 	= (((int16_t)rdata[2] << 8) | ((int16_t)rdata[3]));
	mpu9255_raw->aZ 	= (((int16_t)rdata[4] << 8) | ((int16_t)rdata[5]));
	mpu9255_raw->temp 	= (((int16_t)rdata[6] << 8) | ((int16_t)rdata[7]));
	mpu9255_raw->gX		= (((int16_t)rdata[8] << 8) | ((int16_t)rdata[9]));
	mpu9255_raw->gY 	= (((int16_t)rdata[10] << 8) | ((int16_t)rdata[11]));
	mpu9255_raw->gZ 	= (((int16_t)rdata[12] << 8) | ((int16_t)rdata[13]));
}
void MPU9255_RawReadAccData(MPU9255_data_e *mpu9255_raw)
{
	uint8_t rdata[6] = {0};
	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_ACCEL_XOUT_H, 1, rdata, 6, 1000);
	mpu9255_raw->aX 	= (float)(((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
	mpu9255_raw->aY 	= (float)(((int16_t)rdata[2] << 8) | ((int16_t)rdata[3]));
	mpu9255_raw->aZ 	= (float)(((int16_t)rdata[4] << 8) | ((int16_t)rdata[5]));
}
void MPU9255_rawReadGyroData(MPU9255_data_e *mpu9255_raw)
{
	uint8_t rdata[6] = {0};
	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_GYRO_XOUT_H, 1, rdata, 6, 1000);
	mpu9255_raw->gX 	= (float)(((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
	mpu9255_raw->gY 	= (float)(((int16_t)rdata[2] << 8) | ((int16_t)rdata[3]));
	mpu9255_raw->gZ 	= (float)(((int16_t)rdata[4] << 8) | ((int16_t)rdata[5]));
}
void MPU9255_RawReadTempData(MPU9255_data_e *mpu9255_raw)
{
	uint8_t rdata[2] = {0};
	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_TEMP_OUT_H, 1, rdata, 2, 1000);
	mpu9255_raw->temp	= (float)(((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
}

void MPU9255_ReadAllData(MPU9255_data_e *MPU9255_s)
{
	MPU9255_rawData_e mpu9255_raw;
	MPU9255_rawData_e mpu9255_raw2;
	MPU9255_data_e MPU9255_s2;
	MPU9255_RawReadAllData(&mpu9255_raw);
	MPU9255_RawReadAllData(&mpu9255_raw2);
	float aLSB, gLSB;

	if(mpu9255_s.ACCEL_G_FORCE 		== ACCEL_FS_SEL_2g)
		aLSB = 16384.0;
	else if(mpu9255_s.ACCEL_G_FORCE == ACCEL_FS_SEL_4g)
		aLSB = 8192.0;
	else if(mpu9255_s.ACCEL_G_FORCE == ACCEL_FS_SEL_8g)
		aLSB = 4096.0;
	else if(mpu9255_s.ACCEL_G_FORCE == ACCEL_FS_SEL_16g)
		aLSB = 2048.0;

	if(mpu9255_s.GYRO_DPS 		== GYRO_FS_SEL_250)
		gLSB = 131.0;
	else if(mpu9255_s.GYRO_DPS 	== GYRO_FS_SEL_500)
		gLSB = 65.5;
	else if(mpu9255_s.GYRO_DPS 	== GYRO_FS_SEL_1000)
		gLSB = 32.8;
	else if(mpu9255_s.GYRO_DPS 	== GYRO_FS_SEL_2000)
		gLSB = 16.4;

	if(mpu9255_s.Calibration == Calibration_ON)
	{
		MPU9255_Calibration(MPU9255_s, &mpu9255_raw, aLSB, gLSB);
		MPU9255_Calibration(&MPU9255_s2, &mpu9255_raw2, aLSB, gLSB);
		MPU9255_s->aX = low_pass_filter(MPU9255_s->aX, MPU9255_s2.aX);
		MPU9255_s->aY = low_pass_filter(MPU9255_s->aY, MPU9255_s2.aY);
		MPU9255_s->aZ = low_pass_filter(MPU9255_s->aZ, MPU9255_s2.aZ);

		MPU9255_s->gX = low_pass_filter(MPU9255_s->gX, MPU9255_s2.gX);
		MPU9255_s->gY = low_pass_filter(MPU9255_s->gY, MPU9255_s2.gY);
		MPU9255_s->gZ = low_pass_filter(MPU9255_s->gZ, MPU9255_s2.gZ);
	}
	else
	{
		MPU9255_s->aX = (float)(mpu9255_raw.aX) / aLSB;
		MPU9255_s->aY = (float)(mpu9255_raw.aY) / aLSB;
		MPU9255_s->aZ = (float)(mpu9255_raw.aZ) / aLSB + 0.99;

		MPU9255_s->gX = (float)(mpu9255_raw.gX) / gLSB;
		MPU9255_s->gY = (float)(mpu9255_raw.gY) / gLSB;
		MPU9255_s->gZ = (float)(mpu9255_raw.gZ) / gLSB;

		MPU9255_s->temp = MPU9255_s->temp;
	}
}

void MPU9255_OffSetValues()
{
	MPU9255_rawData_e mpu9255_offSet;
	uint8_t rdata[14] = {0};

	for(int i=0; i<1000; i++)
	{
		HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_ACCEL_XOUT_H, 1, rdata, 14, 1000);

		mpu9255_offSet.aX = (((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
		mpu9255_offSet.aY = (((int16_t)rdata[2] << 8) | ((int16_t)rdata[3]));
		mpu9255_offSet.aZ = (((int16_t)rdata[4] << 8) | ((int16_t)rdata[5]));

		mpu9255_offSet.gX = (((int16_t)rdata[8] << 8) | ((int16_t)rdata[9]));
		mpu9255_offSet.gY = (((int16_t)rdata[8] << 10) | ((int16_t)rdata[11]));
		mpu9255_offSet.gZ = (((int16_t)rdata[10] << 12) | ((int16_t)rdata[13]));

		calibrationAX += (float)mpu9255_offSet.aX;
		calibrationaY += (float)mpu9255_offSet.aY;
		calibrationAZ += (float)mpu9255_offSet.aZ;

		calibrationGX += (float)mpu9255_offSet.gX;
		calibrationGY += (float)mpu9255_offSet.gY;
		calibrationGZ += (float)mpu9255_offSet.gZ;
	}

	calibrationAX /= 1000.0;
	calibrationaY /= 1000.0;
	calibrationAZ /= 1000.0;

	calibrationGX /= 1000.0;
	calibrationGY /= 1000.0;
	calibrationGZ /= 1000.0;
}

void MPU9255_Calibration(MPU9255_data_e *MPU9255_s, MPU9255_rawData_e *MPU922_raw_s, float aLSB, float gLSB)
{
	MPU9255_s->aX = ((float)(MPU922_raw_s->aX) - (calibrationAX)) / aLSB;
	MPU9255_s->aY = ((float)(MPU922_raw_s->aY) - (calibrationaY)) / aLSB;
	MPU9255_s->aZ = ((float)((MPU922_raw_s->aZ) - (calibrationAZ)) / aLSB) + 0.99;

	MPU9255_s->gX = ((float)(MPU922_raw_s->gX) - (calibrationGX)) / gLSB;
	MPU9255_s->gY = ((float)(MPU922_raw_s->gY) - (calibrationGY)) / gLSB;
	MPU9255_s->gZ = ((float)(MPU922_raw_s->gZ) - (calibrationGZ)) / gLSB;

	MPU9255_s->temp = (float)MPU922_raw_s->temp;

	MPU9255_s->aX = map(MPU9255_s->aX, -1.04, 0.96, -1, 1);
	MPU9255_s->aY = map(MPU9255_s->aY, -1.04, 0.96, -1, 1);
	MPU9255_s->aZ = map(MPU9255_s->aZ, -1.04, 0.96, -1, 1);
}

float MPU9255_PitchAngle(MPU9255_data_e *MPU9255_data_s)
{
	float pitchAngle = 0;
	//pitchAngle = asin(MPU9255_data_s->aX);
//	pitchAngle = atan(sqrt(MPU9255_data_s->aX/((MPU9255_data_s->aY*MPU9255_data_s->aY)+(MPU9255_data_s->aZ*MPU9255_data_s->aZ))));
	pitchAngle = atan2(MPU9255_data_s->aZ,MPU9255_data_s->aX);
	return pitchAngle*180/3.1415926;
}

float MPU9255_RollAngle(MPU9255_data_e *MPU9255_data_s)
{
	float rollAngle = 0;
//	rollAngle = atan2(MPU9255_data_s->aY,MPU9255_data_s->aZ);
//	rollAngle = atan(sqrt(MPU9255_data_s->aY/((MPU9255_data_s->aX*MPU9255_data_s->aX)+(MPU9255_data_s->aZ*MPU9255_data_s->aZ))));
	rollAngle = atan2(MPU9255_data_s->aZ,MPU9255_data_s->aY);
	return rollAngle*180/3.1415926;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float low_pass_filter(float x, float xP)
{
	float n=0.2;
	x = (1 - n) * x + n * xP;
	return x;
}
