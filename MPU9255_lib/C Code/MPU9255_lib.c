/*
 * MPU9255_lib.c
 *
 *  Created on: Dec 7, 2023
 *      Author: batuhanculhacioglu
 */

/* gyroscope calculation euler angles

		phi(roll) = gX + tanf(theta_rad) * (sinf(phi_rad) * gY + cosf(phi_rad) * gZ)
		theta(pitch) = cosf(phi_rad) * gY - sinf(phi_rad) * gZ

		phi_rad = phi_rad + SAMPLE_TIME_MS / 1000.0 * phi;
		theta_rad = theta_rad + SAMPLE_TIME_MS / 1000.0 * theta;
*/

#include "MPU9255_lib.h"
#include "math.h"

extern TIM_HandleTypeDef htim2;

MPU9255_init_e mpu9255_s;
float offSet[7] = {0};
float filtered_data[6] = {0};

float g_phi_rad = 0, g_theta_rad = 0;
uint32_t SAMPLE_TIME_MS = 1.0;
extern uint32_t time1, time2;
float a_pitch, a_roll, g_pitch, g_roll;
//float max_A[3] = {0}, min_A[3] = {0};

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

		dummy_byte = (mpu9255_init->ACCEL_G_FORCE) << 3;
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_ACCEL_CONFIG, 1, &dummy_byte, 1, 100); //2g
		HAL_I2C_Mem_Write(mpu9255_init->i2c, MPU9255_I2C_ADDR_W, REG_ACCEL_CONFIG2, 1, &dummy_byte, 1, 100);

		dummy_byte = (mpu9255_init->GYRO_DPS) << 3;
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

	for(int i=0; i<7; i++)
		{
			all_array_raw[i] = (((int16_t)rdata[2*i] << 8) | ((int16_t)rdata[2*i+1]));
		}
}
void MPU9255_RawReadAccData(int16_t *acc_array_raw)
{
	uint8_t rdata[6] = {0};

	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_ACCEL_XOUT_H, 1, rdata, 6, 1000);

	for(int i=0; i<3; i++)
		{
			acc_array_raw[i] = (((int16_t)rdata[2*i] << 8) | ((int16_t)rdata[2*i+1]));
		}
}
void MPU9255_rawReadGyroData(int16_t *gyro_array_raw)
{
	uint8_t rdata[6] = {0};

	HAL_I2C_Mem_Read(mpu9255_s.i2c, MPU9255_I2C_ADDR_R, REG_GYRO_XOUT_H, 1, rdata, 6, 1000);

	for(int i=0; i<3; i++)
		{
			gyro_array_raw[i] = (((int16_t)rdata[2*i] << 8) | ((int16_t)rdata[2*i+1]));
		}
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

		imu->aX = map(imu->aX, -1.014282, 0.991979, -1, 1);
		imu->aY = map(imu->aY, -1.011639, 0.959745, -1, 1);
		imu->aZ = map(imu->aZ, -1.024964, 1.047616, -1, 1);
	}

	low_pass_filter(imu);

//	if(max_A[0] < imu->aX)
//		max_A[0] = imu->aX;
//	if(max_A[1] < imu->aY)
//		max_A[1] = imu->aY;
//	if(max_A[2] < imu->aZ)
//		max_A[2] = imu->aZ;
//	if(min_A[0] > imu->aX)
//		min_A[0] = imu->aX;
//	if(min_A[1] > imu->aY)
//		min_A[1] = imu->aY;
//	if(min_A[2] > imu->aZ)
//		min_A[2] = imu->aZ;

	MPU9255_ACC_ANGLE(imu, &a_pitch, &a_roll);
	MPU9255_GYRO_ANGLE(imu, &g_pitch, &g_roll);
	MPU9255_ANGLE(imu, g_pitch, g_roll, a_pitch, a_roll);
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

void MPU9255_ACC_ANGLE(MPU9255_data_e *imu, float *a_pitch, float *a_roll)
{
	*a_roll = atan2f(imu->aY , (sqrt((imu->aX * imu->aX) + (imu->aZ * imu->aZ)))) * RTD;
	*a_pitch = atan2f(imu->aX , (sqrt((imu->aY * imu->aY) + (imu->aZ * imu->aZ)))) * RTD;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void low_pass_filter(MPU9255_data_e *imu)
{
	float n=0.99;
	imu->aX = (1.0 - n) * imu->aX + n * filtered_data[0];
	imu->aY = (1.0 - n) * imu->aY + n * filtered_data[1];
	imu->aZ = (1.0 - n) * imu->aZ + n * filtered_data[2];

	imu->gX = (1.0 - n) * imu->gX + n * filtered_data[3];
	imu->gY = (1.0 - n) * imu->gY + n * filtered_data[4];
	imu->gZ = (1.0 - n) * imu->gZ + n * filtered_data[5];

	filtered_data[0] = imu->aX;
	filtered_data[1] = imu->aY;
	filtered_data[2] = imu->aZ;

	filtered_data[3] = imu->gX;
	filtered_data[4] = imu->gY;
	filtered_data[5] = imu->gZ;
}

/* gyroscope calculation euler angles

		phi(roll) = gX + tanf(theta_rad) * (sinf(phi_rad) * gY + cosf(phi_rad) * gZ)
		theta(pitch) = cosf(phi_rad) * gY - sinf(phi_rad) * gZ

		phi_rad = phi_rad + SAMPLE_TIME_MS / 1000.0 * phi;
		theta_rad = theta_rad + SAMPLE_TIME_MS / 1000.0 * theta;
*/

void MPU9255_GYRO_ANGLE(MPU9255_data_e *imu, float *g_pitch, float *g_roll)
{
	time2 = __HAL_TIM_GET_COUNTER(&htim2);
	SAMPLE_TIME_MS = time2 - time1;

	*g_pitch = *g_pitch + imu->gY * (float)SAMPLE_TIME_MS / 100000.0;
	*g_roll	= *g_roll + imu->gX * (float)SAMPLE_TIME_MS / 100000.0;
	__HAL_TIM_SET_COUNTER(&htim2,0);
}

void MPU9255_ANGLE(MPU9255_data_e *imu, float g_pitch, float g_roll, float a_pitch, float a_roll)
{
	float n = 0.05;
	imu->pitch = n * a_pitch + (1.0 - n) * g_pitch;
	imu->roll = n * a_roll + (1.0 - n) * g_roll;
}
