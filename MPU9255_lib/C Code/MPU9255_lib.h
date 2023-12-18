/*
 * MPU9255_lib.h
 *
 *  Created on: Dec 7, 2023
 *      Author: batuhanculhacioglu
 */

/********************
* 	  INCLUDES 		*
********************/
#include "main.h"
#include "stdint.h"

/********************
* 	  DEFINES 		*
********************/
#ifndef INC_MPU9255_LIB_H_
#define INC_MPU9255_LIB_H_

#define PI		3.141592
#define RTD		180/PI
#define DTR		PI/180

#define MPU9255_I2C_ADDR_W 	0xD0
#define MPU9255_I2C_ADDR_R 	0xD0

#define REG_SMPLRT_DIV 		0x19
#define REG_CONFIG 			0x1A
#define REG_GYRO_CONFIG 	0x1B
#define REG_ACCEL_CONFIG 	0x1C
#define REG_ACCEL_CONFIG2 	0x1D
#define REG_PWR_MGMT_1 		0x6B
#define REG_PWR_MGMT_2 		0x6C
#define REG_WHO_AM_I 		0x75


//You can read all gyro offset data from this register. (6 bytes)
#define REG_XG_OFFSET_H 	0x13 /*	XG_H(0x13) + XG_L(0x14),
									YG_H(0x15) + YG_L(0x16),
									ZG_H(0x17) + ZG_L(0x18)
									6 bytes data */
#define REG_XG_OFFSET_L 	0x14
#define REG_YG_OFFSET_H 	0x15
#define REG_YG_OFFSET_L 	0x16
#define REG_ZG_OFFSET_H 	0x17
#define REG_ZG_OFFSET_L 	0x18


//You can read all acc offset data from this register. (6 bytes)
#define REG_XA_OFFSET_H 	0x77 /*	XA_H(0x77) + XA_L(0x78),
									YA_H(0x7A) + YA_L(0x7B),
									ZA_H(0x7D) + ZA_L(0x7E)
									6 bytes data */
#define REG_XA_OFFSET_L 	0x78
#define REG_YA_OFFSET_H 	0x7A
#define REG_YA_OFFSET_L 	0x7B
#define REG_ZA_OFFSET_H 	0x7D
#define REG_ZA_OFFSET_L 	0x7E


//You can read all acc,gyro,temp data from this register. (14 bytes)
#define REG_ACCEL_XOUT_H 	0x3B /* AX_H(0x3B) + AX_L(0x3C),
									AY_H(0x3D) + AY_L(0x3E),
									AZ_H(0x3F) + AZ_L(0x40)
									TEMP_H(0x41) + TEMP_L(0x42),
									GX_H(0x43) + GX_L(0x44),
									GY_H(0x45) + GY_L(0x46),
									GZ_H(0x47) + GZ_L(0x48)
									14 bytes data */

#define REG_ACCEL_XOUT_L 	0x3C
#define REG_ACCEL_YOUT_H 	0x3D
#define REG_ACCEL_YOUT_L 	0x3E
#define REG_ACCEL_ZOUT_H 	0x3F
#define REG_ACCEL_ZOUT_L 	0x40

#define REG_TEMP_OUT_H 		0x41
#define REG_TEMP_OUT_L 		0x42

#define REG_GYRO_XOUT_H 	0x43
#define REG_GYRO_XOUT_L 	0x44
#define REG_GYRO_YOUT_H 	0x45
#define REG_GYRO_YOUT_L 	0x46
#define REG_GYRO_ZOUT_H 	0x47
#define REG_GYRO_ZOUT_L 	0x48

/********************
* 	  ENUMS 	*
********************/
typedef enum MPU9255_GYRO_FS_SEL_e{
	GYRO_FS_SEL_250 	= 0x00,
	GYRO_FS_SEL_500		= 0x01,
	GYRO_FS_SEL_1000	= 0x02,
	GYRO_FS_SEL_2000	= 0x03
}MPU9255_GYRO_FS_SEL_t;

typedef enum MPU9255_ACCEL_FS_SEL_e{
	ACCEL_FS_SEL_2g 	= 0x00,
	ACCEL_FS_SEL_4g 	= 0x01,
	ACCEL_FS_SEL_8g 	= 0x02,
	ACCEL_FS_SEL_16g	= 0x03
}MPU9255_ACCEL_FS_SEL_t;

typedef enum MPU9255_SMPLRT_DIV_e{
	SMPLRT_DIV_1KHz 	= 0x07
}MPU9255_SMPLRT_DIV_t;

typedef enum MPU9255_PWR_MGMT_1_e{
	PWR_MGMT_STANDART_1 = 0x00
}MPU9255_PWR_MGMT_1_t;

typedef enum MPU9255_PWR_MGMT_2_e{
	PWR_MGMT_STANDART_2 = 0x00
}MPU9255_PWR_MGMT_2_t;

typedef enum MPU9255_Calibration_e{
	Calibration_ON 	= 0x00,
	Calibration_OFF = 0x01
}MPU9255_Calibration_t;

/********************
* 	  STRUCTS 		*
********************/
typedef struct MPU9255_init_s{
	I2C_HandleTypeDef *i2c;
	MPU9255_GYRO_FS_SEL_t GYRO_DPS;
	MPU9255_ACCEL_FS_SEL_t ACCEL_G_FORCE;
	MPU9255_SMPLRT_DIV_t SampleRateDivider;
	MPU9255_PWR_MGMT_1_t PWR_MGMT_1;
	MPU9255_PWR_MGMT_2_t PWR_MGMT_2;
	MPU9255_Calibration_t Calibration;
}MPU9255_init_e;

typedef struct MPU9255_data_s{
	float aX;
	float aY;
	float aZ;

	float gX;
	float gY;
	float gZ;

	float temp;

	float pitch;
	float roll;
}MPU9255_data_e;

/********************
* 	  FUNCTIONS 	*
********************/
void MPU9255_Init(MPU9255_init_e *mpu9255_init_s);
HAL_StatusTypeDef MPU9255_WhoAmI();
void MPU9255_RawReadAllData(int16_t *all_array_raw);
void MPU9255_RawReadAccData(int16_t *acc_array_raw);
void MPU9255_RawReadGyroData(int16_t *gyro_array_raw);
void MPU9255_RawReadTempData(int16_t *temp_raw);
void MPU9255_ReadAllData(MPU9255_data_e *MPU9255_data_s);
void MPU9255_ReadAccel(MPU9255_data_e *MPU9255_data_s);
void MPU9255_ReadGyro(MPU9255_data_e *MPU9255_data_s);
void MPU9255_ReadTemp(MPU9255_data_e *MPU9255_data_s);
void MPU9255_OffSetValues();
void MPU9255_ACC_ANGLE(MPU9255_data_e *imu, float *a_pitch, float *a_roll);
void MPU9255_GYRO_ANGLE(MPU9255_data_e *imu, float *g_pitch, float *g_roll);
void MPU9255_ANGLE(MPU9255_data_e *imu, float g_pitch, float g_roll, float a_pitch, float a_roll);
void low_pass_filter(MPU9255_data_e *imu);

float map(float x, float in_min, float in_max, float out_min, float out_max);
#endif /* INC_MPU9255_LIB_H_ */
