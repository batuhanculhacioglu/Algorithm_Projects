/*
 * BMP280.h
 *
 *  Created on: Mar 16, 2024
 *      Author: batuhanculhacioglu
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stdint.h"
#include "main.h"

#define BMP280_ADDR_W 0x76<<1 //EC
#define BMP280_ADDR_R (0x76<<1) | 0x01 //ED

#define BMP280_ID 0xD0 //WHO_AM_I REGISTER
/*
The “id” register contains the chip identification number chip_id[7:0], which is 0x58. This number can
be read as soon as the device finished the power-on-reset.
*/

#define BMP280_TEMP_XLSB 0xFC // read only
#define BMP280_TEMP_LSB 0xFB // read only
#define BMP280_TEMP_MSB 0xFA // read only
#define BMP280_PRESS_XLSB 0xF9 // read only
#define BMP280_PRESS_LSB 0xF8 // read only
#define BMP280_PRESS_MSB 0xF7 // read only
#define BMP280_ALL_DATA 0xF7
#define BMP280_CONFIG 0xF5 // read/write
#define BMP280_TRIM 0x88
/*
The “config” register sets the rate, filter and interface options of the device. Writes to the “config”
register in normal mode may be ignored. In sleep mode writes are not ignored.
*/
#define BMP280_CTRL_MEAS 0xF4 // read/write
/*
The “ctrl_meas” register sets the data acquisition options of the device.
*/
#define BMP280_STATUS 0xF3 // read only
/*
The “status” register contains two bits which indicate the status of the device
Bit 3 measuring[0]
Bit 0 im_update[0]
*/
#define BMP280_RESET 0xE0 // write only
/*
The “reset” register contains the soft reset word reset[7:0]. If the value 0xB6 is written to the register,
the device is reset using the complete power-on-reset procedure. Writing other values than 0xB6 has
no effect. The readout value is always 0x00.
*/

/*****************
 ****** ENUM *****
 *****************/

typedef enum{
	bmpOkey,
	bmpErrorR,
	bmpErrorW,
	bmpErrorID,
	bmpErrorC
}bmpStatus;

typedef enum{
	pressureOversampling_X1 = 0x01,
	pressureOversampling_X2 = 0x02,
	pressureOversampling_X4 = 0x03,
	pressureOversampling_X8 = 0x04,
	pressureOversampling_X16 = 0x05
}pressureOversampling;

typedef enum{
	temperatureOversampling_X1 = 0x01,
	temperatureOversampling_X2 = 0x02,
	temperatureOversampling_X4 = 0x03,
	temperatureOversampling_X8 = 0x04,
	temperatureOversampling_X16 = 0x05
}temperatureOversampling;

typedef enum{
	sleepMode = 0x00,
	forcedMode = 0x01,
	normalMode = 0x03
}powerModes;

typedef enum{
	tstandby_1 = 0x00,
	tstandby_2 = 0x01,
	tstandby_3 = 0x02,
	tstandby_4 = 0x03,
	tstandby_5 = 0x04,
	tstandby_6 = 0x05,
	tstandby_7 = 0x06,
	tstandby_8 = 0x07
}tstandby;

typedef enum{
	filter_off = 0x00,
	filter_2 = 0x01,
	filter_4 = 0x02,
	filter_8 = 0x03,
	filter_16 = 0x04
}IIRFilter;

/*****************
 ***** STRUCT ****
 *****************/

typedef struct{
	uint16_t id;
	I2C_HandleTypeDef i2c;
	pressureOversampling pO;
	temperatureOversampling tO;
	powerModes pM;
	IIRFilter IIRF;
}BMP280_config;

typedef struct{
	double pressure;
	double temperature;
}BMP280_datas;

typedef struct{
	uint16_t dig_T1, dig_P1;
	int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	int32_t t_fine;
}BMP280_trim;
/*****************
 ****** FUNC *****
 *****************/

static bmpStatus BMP280_I2C_read(uint16_t addr, uint8_t *data, uint8_t size);
static bmpStatus BMP280_I2C_write(uint16_t addr, uint8_t *data, uint8_t size);

bmpStatus BMP280_I2C_isDeviceReady();
bmpStatus BMP280_checkId();
static void trimRead();
bmpStatus BMP280_setConfig(BMP280_config bmp);

bmpStatus BMP280_readAllDatas(BMP280_datas *p);

static double BMP280_compensate_T_int32(int32_t adc_T);
static double BMP280_compensate_P_int64(int32_t adc_P);




#endif /* INC_BMP280_H_ */
