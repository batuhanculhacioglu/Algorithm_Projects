/*
 * main.c
 *
 *  Created on: Dec 7, 2023
 *      Author: batuhanculhacioglu
 */
 
#include "MPU9255_lib.h"

MPU9255_init_e mpu;
MPU9255_data_e mpudata;
float pitch=0, roll=0;


int main(void)
{
  mpu.ACCEL_G_FORCE = ACCEL_FS_SEL_2g;
  mpu.GYRO_DPS = GYRO_FS_SEL_250;
  mpu.PWR_MGMT_1 = PWR_MGMT_STANDART_1;
  mpu.PWR_MGMT_2 = PWR_MGMT_STANDART_2;
  mpu.SampleRateDivider = SMPLRT_DIV_1KHz;
  mpu.i2c = &hi2c1;
  mpu.Calibration = Calibration_ON;

  MPU9255_Init(&mpu);
 
  while (1)
  {
    
	  HAL_Delay(100);
	  MPU9255_ReadAllData(&mpudata);
	  pitch = MPU9255_PitchAngle(&mpudata);
	  roll = MPU9255_RollAngle(&mpudata);

  }
}
