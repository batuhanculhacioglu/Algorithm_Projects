/*
 * BMP280.c
 *
 *  Created on: Mar 16, 2024
 *      Author: batuhanculhacioglu
 */

#include "BMP280.h"

BMP280_config bmp280_config;
BMP280_trim bmp280_trim;

static void BMP280_I2C_read(uint16_t addr, uint8_t *data, uint8_t size)
{
	HAL_I2C_Mem_Read(&bmp280_config.i2c, bmp280_config.id, addr, 1, data, size, 100);
}

static void BMP280_I2C_write(uint16_t addr, uint8_t data, uint8_t size)
{
	HAL_I2C_Mem_Read(&bmp280_config.i2c, bmp280_config.id | 0x01, addr, 1, &data, size, 100);
}

uint16_t BMP280_I2C_isDeviceReady()
{
	uint16_t check = 0, i = 0;
	for(i = 0; i <= 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&bmp280_config.i2c, i, 2, 10) == HAL_OK)
		{
			check = i;
			break;
		}
	}
	return check;
}

BMP280_status BMP280_checkId()
{
	uint8_t check = 0;
	BMP280_I2C_read(BMP280_ID, &check, 1);
	if(check != 0)
		return OK;
	else
		return ER;
}

static void trimRead()
{
	uint8_t rData[24] = {0};

	BMP280_I2C_read(BMP280_TRIM, rData, 24);

	bmp280_trim.dig_T1 = ((uint16_t)rData[1]<<8) | rData[0];
	bmp280_trim.dig_T2 = (int16_t)((uint16_t)rData[3]<<8) | rData[2];
	bmp280_trim.dig_T3 = (int16_t)((uint16_t)rData[5]<<8) | rData[4];
	bmp280_trim.dig_P1 = ((uint16_t)rData[7]<<8) | rData[5];
	bmp280_trim.dig_P2 = (int16_t)((uint16_t)rData[9]<<8) | rData[6];
	bmp280_trim.dig_P3 = (int16_t)((uint16_t)rData[11]<<8) | rData[10];
	bmp280_trim.dig_P4 = (int16_t)((uint16_t)rData[13]<<8) | rData[12];
	bmp280_trim.dig_P5 = (int16_t)((uint16_t)rData[15]<<8) | rData[14];
	bmp280_trim.dig_P6 = (int16_t)((uint16_t)rData[17]<<8) | rData[16];
	bmp280_trim.dig_P7 = (int16_t)((uint16_t)rData[19]<<8) | rData[18];
	bmp280_trim.dig_P8 = (int16_t)((uint16_t)rData[21]<<8) | rData[20];
	bmp280_trim.dig_P9 = (int16_t)((uint16_t)rData[23]<<8) | rData[22];
}

void BMP280_setConfig(BMP280_config bmp)
{
	if(bmp.i2c.Instance != NULL)
	{
		bmp280_config.i2c = bmp.i2c;
		bmp280_config.id = bmp.id;
		bmp280_config.pM = bmp.pM;
		bmp280_config.pO = bmp.pO;
		bmp280_config.tO = bmp.tO;
		bmp280_config.IIRF = bmp.IIRF;

		if(bmp280_config.id == 0x00)
			bmp280_config.id = BMP280_ADDR_W;
		if(bmp280_config.pO == 0x00)
			bmp280_config.pO = pressureOversampling_X1;
		if(bmp280_config.tO == 0x00)
			bmp280_config.tO = temperatureOversampling_X1;
		if(BMP280_checkId() != OK)
		{
			// to do
		}
		else
		{
			trimRead();
			uint8_t resetV = 0xB6;
			BMP280_I2C_write(BMP280_RESET, resetV, 1);
			HAL_Delay(50);
			BMP280_I2C_write(BMP280_CONFIG, (tstandby_1 << 5) |  (bmp280_config.IIRF << 2), 1);
			HAL_Delay(50);
			BMP280_I2C_write(BMP280_CTRL_MEAS, (bmp280_config.tO << 5) | (bmp280_config.pO << 2) | bmp280_config.pM, 1);
			HAL_Delay(50);
		}
	}
	else
	{
		// to do
	}

}

void BMP280_readAllDatas(BMP280_datas *p)
{
	if(bmp280_config.i2c.Instance != NULL)
	{
		uint8_t rData[6] = {0};
		BMP280_I2C_read(BMP280_ALL_DATA, rData, 6);

		uint32_t temp = ((((uint32_t)rData[3] << 12) & 0x3FC) | (((uint32_t)rData[4] << 4) & 0xFF0) | (((uint32_t)rData[5] >> 4) & 0x0F));
		p->temperature = BMP280_compensate_T_int32((uint32_t)temp);

		uint32_t press = ((((uint32_t)rData[0] << 12) & 0x3FC) | (((uint32_t)rData[1] << 4) & 0xFF0) | (((uint32_t)rData[2] >> 4) & 0x0F));
		p->pressure = BMP280_compensate_P_int64((uint32_t)press);
	}
	else
	{
		// to do
	}
}

static double BMP280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2;
	double T;
	bmp280_trim.t_fine = 0;
	var1 = ((((adc_T>>3) - ((int32_t)bmp280_trim.dig_T1 << 1))) * ((int32_t)bmp280_trim.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)bmp280_trim.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_trim.dig_T1))) >> 12) *((int32_t)bmp280_trim.dig_T3)) >> 14;
	bmp280_trim.t_fine = var1 + var2;
	T = (double)((bmp280_trim.t_fine * 5 + 128) >> 8);
	return T / 100.0;
}

static double BMP280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	double P;
	var1 = ((int64_t)bmp280_trim.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bmp280_trim.dig_P6;
	var2 = var2 + ((var1 * (int64_t)bmp280_trim.dig_P5) << 17);
	var2 = var2 + (((int64_t)bmp280_trim.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)bmp280_trim.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_trim.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_trim.dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)bmp280_trim.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)bmp280_trim.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_trim.dig_P7) << 4);
	P = (double)p / 256.0;
	return P;
}
