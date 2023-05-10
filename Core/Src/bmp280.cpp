/*
 * bmp280.cpp
 *
 *  Created on: Mar 24, 2023
 *      Author: Samet
 */
#include "bmp280.h"

BMP_280::BMP_280(I2C_HandleTypeDef *hi2c, uint8_t dev_Address) :
		hi2cx(hi2c), dev_Addr(dev_Address), isInitialized(isAccessible()) {

	reset();
	param.measure.osrs_p = pres_sampling_X16;
	param.measure.osrs_t = temp_sampling_X16;
	param.measure.mode = Normal;

	param.cfg.t_sb = BMP_Standby_05;
	param.cfg.filter = BMP_NO_FILTER;
	param.cfg.reserved = 0;
	param.cfg.spi3w_en = 0;

#ifdef SPI
		param.cfg.spi3w_en=1;

	#endif

	init(param);
	HAL_Delay(100);
	readTrimValue();
	//	debugreg();

}

BMP_280::~BMP_280() {

}

void BMP_280::readTrimValue() {

	param.calib.dig_T1 = (uint16_t) readReg16_LBF(BMP_DigT1);

	param.calib.dig_T2 = (int16_t) readReg16_LBF(BMP_DigT2);

	param.calib.dig_T3 = (int16_t) readReg16_LBF(BMP_DigT3);

	param.calib.dig_P1 = (uint16_t) readReg16_LBF(0x8E);

	param.calib.dig_P2 = (int16_t) readReg16_LBF(0x90);

	param.calib.dig_P3 = (int16_t) readReg16_LBF(0x92);

	param.calib.dig_P4 = (int16_t) readReg16_LBF(0x94);

	param.calib.dig_P5 = (int16_t) readReg16_LBF(0x96);

	param.calib.dig_P6 = (int16_t) readReg16_LBF(0x98);

	param.calib.dig_P7 = (int16_t) readReg16_LBF(0x9A);

	param.calib.dig_P8 = (int16_t) readReg16_LBF(0x9C);

	param.calib.dig_P9 = (int16_t) readReg16_LBF(0x9E);
}

float BMP_280::getTemperature() {

	int32_t adc_T = readReg24_HBF(BMP_Temp_MSB);
	adc_T >>= 4;
	int32_t var1, var2, T;

	var1 = ((((adc_T >> 3) - ((int32_t) param.calib.dig_T1 << 1)))
			* ((int32_t) param.calib.dig_T2)) >> 11;

	var2 = (((((adc_T >> 4) - ((int32_t) param.calib.dig_T1))
			* ((adc_T >> 4) - ((int32_t) param.calib.dig_T1))) >> 12)
			* ((int32_t) param.calib.dig_T3)) >> 14;

	t_fine = var1 + var2;

	T = (t_fine * 5 + 128) >> 8;
	return (float) T / 100;
}

int BMP_280::readPressure() {
	uint32_t adc_P = readReg24_HBF(0xF7);
	adc_P >>= 4;
	int64_t var1, var2, p;
	var1 = ((int64_t) t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) param.calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t) param.calib.dig_P5) << 17);
	var2 = var2 + (((int64_t) param.calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) param.calib.dig_P3) >> 8)
			+ ((var1 * (int64_t) param.calib.dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) param.calib.dig_P1)
			>> 33;
	if (var1 == 0) {
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) param.calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) param.calib.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t) param.calib.dig_P7) << 4);
	return (float) p / 256;
}

float BMP_280::readAltitude(float seaLevelhPa) {
	float altitude;

	float pressure = readPressure();
	pressure /= 100;

	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

	return altitude;
}

bool BMP_280::isAccessible() {

	return (readReg8( BMP_280_IDreg) == BMP_280_DevID);
}

void BMP_280::reset() {

	writeReg(BMP_Reset, Soft_Reset);
}

void BMP_280::init(BMP_param &param) {

	uint8_t buffer[2] = { param.measure.set(), param.cfg.set() };
	writeReg(BMP_Config, buffer[1]);
	writeReg(BMP_Ctrl_Meas, buffer[0]);

}

HAL_StatusTypeDef BMP_280::writeReg(uint8_t regAdress, uint8_t data) {
	return (HAL_I2C_Mem_Write(this->hi2cx, this->dev_Addr, regAdress, 1, &data,
			1, I2C_TIMEOUT_VALUE(I2C_freq, 1)));
}

HAL_StatusTypeDef BMP_280::writeReg_Multi(uint8_t regAdress, uint8_t *data,
		int len) {
	return (HAL_I2C_Mem_Write(this->hi2cx, this->dev_Addr, regAdress, 1, data,
			len, I2C_TIMEOUT_VALUE(I2C_freq, len)));
}

uint8_t BMP_280::readReg8(uint8_t regAdress) {
	uint8_t buffer;
	HAL_I2C_Mem_Read(this->hi2cx, this->dev_Addr, regAdress, 1, &buffer, 1,
			I2C_TIMEOUT_VALUE(I2C_freq, 1));

	return buffer;
}

uint16_t BMP_280::readReg16_LBF(uint8_t regAdress) {
	uint8_t buffer[2];
	HAL_I2C_Mem_Read(this->hi2cx, this->dev_Addr, regAdress, 1, buffer, 2,
			I2C_TIMEOUT_VALUE(I2C_freq, 2));

	return (uint16_t) (buffer[1] << 8 | buffer[0]);
}

uint32_t BMP_280::readReg24_HBF(uint8_t regAdress) {
	uint8_t buffer[3];
	HAL_I2C_Mem_Read(this->hi2cx, this->dev_Addr, regAdress, 1, buffer, 3,
			I2C_TIMEOUT_VALUE(I2C_freq, 3));

	return (uint32_t) (buffer[0] << 16 | buffer[1] << 8 | buffer[2]);
}

