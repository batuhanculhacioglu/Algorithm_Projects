/*
 * MPU9255_lib.c
 *
 *  Created on: Dec 7, 2023
 *      Author: batuhanculhacioglu
 */

#include "MPU9255_lib.h"
#include "math.h"

MPU9255_init_e mpu9255_s;
float offSet[7] = {0};
float filtered_data[6] = {0};


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

void MPU9255_RawReadAllData(int16_t *all_array_raw)
{
	uint8_t rdata[14] = {0};

	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_ACCEL_XOUT_H, 1, rdata, 14, 1000);

	all_array_raw[0] 	= (((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
	all_array_raw[1] 	= (((int16_t)rdata[2] << 8) | ((int16_t)rdata[3]));
	all_array_raw[2] 	= (((int16_t)rdata[4] << 8) | ((int16_t)rdata[5]));

	all_array_raw[3] 	= (((int16_t)rdata[6] << 8) | ((int16_t)rdata[7]));

	all_array_raw[4]	= (((int16_t)rdata[8] << 8) | ((int16_t)rdata[9]));
	all_array_raw[5] 	= (((int16_t)rdata[10] << 8) | ((int16_t)rdata[11]));
	all_array_raw[6] 	= (((int16_t)rdata[12] << 8) | ((int16_t)rdata[13]));
}
void MPU9255_RawReadAccData(int16_t *acc_array_raw)
{
	uint8_t rdata[6] = {0};

	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_ACCEL_XOUT_H, 1, rdata, 6, 1000);

	acc_array_raw[0] 	= (((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
	acc_array_raw[1] 	= (((int16_t)rdata[2] << 8) | ((int16_t)rdata[3]));
	acc_array_raw[2] 	= (((int16_t)rdata[4] << 8) | ((int16_t)rdata[5]));
}
void MPU9255_rawReadGyroData(int16_t *gyro_array_raw)
{
	uint8_t rdata[6] = {0};

	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_GYRO_XOUT_H, 1, rdata, 6, 1000);

	gyro_array_raw[0] 	= (((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
	gyro_array_raw[1] 	= (((int16_t)rdata[2] << 8) | ((int16_t)rdata[3]));
	gyro_array_raw[2] 	= (((int16_t)rdata[4] << 8) | ((int16_t)rdata[5]));
}
void MPU9255_RawReadTempData(int16_t *temp_raw)
{
	uint8_t rdata[2] = {0};

	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_TEMP_OUT_H, 1, rdata, 2, 1000);

	*temp_raw	= (((int16_t)rdata[0] << 8) | ((int16_t)rdata[1]));
}

void MPU9255_ReadAllData(MPU9255_data_e *imu)
{
	int16_t all_raw[7];
	MPU9255_RawReadAllData(all_raw);
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
		imu->aX = ((float)(all_raw[0]) - (offSet[0])) / aLSB;
		imu->aY = ((float)(all_raw[1]) - (offSet[1])) / aLSB;
		imu->aZ = ((float)((all_raw[2]) - (offSet[2])) / aLSB) + 0.99;

		imu->gX = ((float)(all_raw[4]) - (offSet[4])) / gLSB;
		imu->gY = ((float)(all_raw[5]) - (offSet[5])) / gLSB;
		imu->gZ = ((float)(all_raw[6]) - (offSet[6])) / gLSB;

		imu->temp = (float)all_raw[3];

		imu->aX = map(imu->aX, -1.04, 0.96, -1, 1);
		imu->aY = map(imu->aY, -1.04, 0.96, -1, 1);
		imu->aZ = map(imu->aZ, -1.04, 0.96, -1, 1);
	}

	low_pass_filter(imu);
	MPU9255_Angle(imu);
}

void MPU9255_OffSetValues()
{
	int16_t r[7];
	uint8_t rdata[14] = {0};

	for(int i=0; i<1000; i++)
	{
		HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_ACCEL_XOUT_H, 1, rdata, 14, 1000);

		for(int j=0; j<7; j++)
		{
			r[j] = (((int16_t)rdata[2*j] << 8) | ((int16_t)rdata[2*j+1]));
			offSet[j] += (float)r[j];
		}
	}

	for(int i=0; i<7; i++)
	{
		offSet[i] /= 1000;
	}
}

void MPU9255_Angle(MPU9255_data_e *imu)
{
	float rtd = 180/3.141592;

	imu->roll = atanf(imu->aY / (sqrt((imu->aX * imu->aX) + (imu->aZ * imu->aZ)))) * rtd;
	imu->pitch = atanf(imu->aX / (sqrt((imu->aY * imu->aY) + (imu->aZ * imu->aZ)))) * rtd;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void low_pass_filter(MPU9255_data_e *imu)
{
	float n=0.2;
	imu->aX = (1 - n) * imu->aX + n * filtered_data[0];
	imu->aY = (1 - n) * imu->aY + n * filtered_data[1];
	imu->aZ = (1 - n) * imu->aZ + n * filtered_data[2];

	imu->gX = (1 - n) * imu->gX + n * filtered_data[3];
	imu->gY = (1 - n) * imu->gY + n * filtered_data[4];
	imu->gZ = (1 - n) * imu->gZ + n * filtered_data[5];

	filtered_data[0] = imu->aX;
	filtered_data[1] = imu->aY;
	filtered_data[2] = imu->aZ;

	filtered_data[3] = imu->gX;
	filtered_data[4] = imu->gY;
	filtered_data[5] = imu->gZ;
}
