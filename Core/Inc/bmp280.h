/*
 * bmp280.h
 *
 *  Created on: Mar 24, 2023
 *      Author: Samet
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#define Soft_Reset 0xB6
#define BMP_280_DevID 0x58
#define BMP_280_IDreg 0xD0
#define BMP_Ctrl_Meas 0xF4
#define BMP_Config 0xF5
#define BMP_Status 0xF3
#define BMP_Reset 0xE0

#define BMP_DigT1 0x88
#define BMP_DigT2 0x8A
#define BMP_DigT3 0x8C

#define BMP_Temp_MSB 0xFA

#define I2C_freq 100000
#define I2C_TIMEOUT_VALUE(clk_freq,len)  (uint32_t)(((len+3) * (10000000UL/clk_freq) +1000)/1000)

typedef enum {
	BMP_NO_FILTER, BMP_FILTER_2, BMP_FILTER_4, BMP_FILTER_8, BMP_FILTER_16,
} IIR_Filter;

typedef enum {
	BMP_Standby_05,
	BMP_Standby_62_5,
	BMP_Standby_125,
	BMP_Standby_250,
	BMP_Standby_500,
	BMP_Standby_1000,
	BMP_Standby_2000,
	BMP_Standby_4000

} t_Standby;

typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	  int16_t dig_P2;
	  int16_t dig_P3;
	  int16_t dig_P4;
	  int16_t dig_P5;
	  int16_t dig_P6;
	  int16_t dig_P7;
	  int16_t dig_P8;
	  int16_t dig_P9;
} trim_Param;

typedef enum {
	temp_sampling_X1 = 1,
	temp_sampling_X2 = 2,
	temp_sampling_X4 = 3,
	temp_sampling_X8 = 4,
	temp_sampling_X16 = 7
} osrs_t;

typedef enum {
	pres_sampling_X1 = 1,
	pres_sampling_X2 = 2,
	pres_sampling_X4 = 3,
	pres_sampling_X8 = 4,
	pres_sampling_X16 = 7
} osrs_p;

typedef enum {
	Sleep, Forced, Normal = 3
} Mode;

typedef struct {
	uint8_t osrs_t :3; //Most significant bits
	uint8_t osrs_p :3;
	uint8_t mode :2;
	uint8_t set() {
		return (osrs_t << 5) | (osrs_p << 2) | mode;
	}
} ctrl_meas;

typedef struct {
	uint8_t t_sb :3;
	uint8_t filter :3 ;
	uint8_t reserved :1;
	uint8_t spi3w_en :1;
	uint8_t set() {
		return (t_sb << 5) | (filter << 2) | spi3w_en;
	}
} BMP_Cfg;

typedef struct {
	BMP_Cfg cfg;
	ctrl_meas measure;
	trim_Param calib;
} BMP_param;

class BMP_280 {
private:
	//uint8_t debugbuffer[2];
	I2C_HandleTypeDef *hi2cx;
	uint8_t dev_Addr;
	BMP_param param;
	int32_t t_fine = 0;
	//void debugreg();
public:

	BMP_280(I2C_HandleTypeDef *hi2cx, uint8_t dev_Address = 0x77);
	//BMP_280(SPI_HandleTypeDef,cspin);
	~BMP_280();
	bool isInitialized;
	void reset();
	void init(BMP_param &param);
	float getTemperature();
	int readPressure();
	float readAltitude(float seaLevelhPa);

protected:
	uint8_t readReg8(uint8_t regAdress);
	uint16_t readReg16_LBF(uint8_t regAdress);
	uint32_t readReg24_HBF(uint8_t regAdress);
	HAL_StatusTypeDef writeReg(uint8_t regAdress, uint8_t data);
	HAL_StatusTypeDef writeReg_Multi(uint8_t regAdress, uint8_t *data, int len);
	bool isAccessible();
	void readTrimValue();
};

#endif /* INC_BMP280_H_ */
