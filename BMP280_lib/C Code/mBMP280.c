/*
 * mBMP280.c
 *
 *  Created on: Mar 16, 2024
 *      Author: batuhanculhacioglu
 */

#include "mBMP280.h"
#include "mI2C.h"

BMP280_trim bmp280_trim;
mI2C bmpI2C;
static uint8_t ctrl = 0;

static bmpStatus BMP280_I2C_read(uint16_t addr, uint8_t *data, uint8_t size)
{
	if(mI2C_MemRead(bmpI2C, addr, data, size) != mOkey)
		return bmpErrorR;
	return bmpOkey;
}

static bmpStatus BMP280_I2C_write(uint16_t addr, uint8_t *data, uint8_t size)
{
	if(mI2C_MemWrite(bmpI2C, addr, data, size) != mOkey)
		return bmpErrorW;
	return bmpOkey;
}

bmpStatus BMP280_I2C_isDeviceReady()
{
	if(mI2C_IsDeviceReady(bmpI2C) != mOkey)
		return bmpErrorID;
	return bmpOkey;
}

bmpStatus BMP280_checkId()
{
	uint8_t check = 0;
	if(BMP280_I2C_read(BMP280_ID, &check, 1) != bmpOkey)
		return bmpErrorR;
	if(check != 0xB0)
		return bmpErrorID;
	return bmpOkey;
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

bmpStatus BMP280_setConfig(BMP280_config bmp)
{
	if(bmp.i2c.Instance != NULL)
	{
		if(bmp.id == 0x00)
			bmp.id = BMP280_ADDR_W;
		if(bmp.pO == 0x00)
			bmp.pO = pressureOversampling_X1;
		if(bmp.tO == 0x00)
			bmp.tO = temperatureOversampling_X1;

		bmpI2C.hi2c = &bmp.i2c;
		bmpI2C.DevAddress = bmp.id;
		bmpI2C.Timeout = 100;

		if(BMP280_checkId() != bmpOkey)
			return bmpErrorID;
		else
		{
			trimRead();
			uint8_t value = 0xB6;
			if(BMP280_I2C_write(BMP280_RESET, &value, 1) != bmpOkey)
				return bmpErrorW;
			HAL_Delay(50);
			value = (tstandby_1 << 5) |  (bmp.IIRF << 2);
			if(BMP280_I2C_write(BMP280_CONFIG, &value, 1) != bmpOkey)
				return bmpErrorW;
			HAL_Delay(50);
			value = (bmp.tO << 5) | (bmp.pO << 2) | bmp.pM;
			if(BMP280_I2C_write(BMP280_CTRL_MEAS, &value, 1) != bmpOkey)
				return bmpErrorW;
			HAL_Delay(50);

			ctrl = 1;

			return bmpOkey;
		}
	}
	return bmpErrorC;
}

bmpStatus BMP280_readAllDatas(BMP280_datas *p)
{
	if(ctrl != 1)
		return bmpErrorC;

	uint8_t rData[6] = {0};
	if(BMP280_I2C_read(BMP280_ALL_DATA, rData, 6) != bmpOkey)
		return bmpErrorR;

	uint32_t temp = ((((uint32_t)rData[3] << 12) & 0x3FC) | (((uint32_t)rData[4] << 4) & 0xFF0) | (((uint32_t)rData[5] >> 4) & 0x0F));
	p->temperature = BMP280_compensate_T_int32((uint32_t)temp);

	uint32_t press = ((((uint32_t)rData[0] << 12) & 0x3FC) | (((uint32_t)rData[1] << 4) & 0xFF0) | (((uint32_t)rData[2] >> 4) & 0x0F));
	p->pressure = BMP280_compensate_P_int64((uint32_t)press);

	return bmpOkey;
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
