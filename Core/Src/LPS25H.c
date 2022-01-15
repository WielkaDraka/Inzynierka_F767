/*
 * LPS25H.h
 *
 *		This librabry is for sensor AltIMU10 v5
 *		MEMS pressure sensor: 260-1260 hPa absolute digital output barometer:
 *
 *      Author: wielkadraka
 */

#include "LPS25H.h"
#include "math.h"

LPS25H alititude;

extern I2C_HandleTypeDef hi2c1;

uint8_t lps25h_read_reg(uint8_t reg)
{
	uint8_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, LPS25H_H_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);

	return value;
}
void lps25h_write_reg(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, LPS25H_H_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);

}

void lps25h_Init(void)
{
	//PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
	lps25h_write_reg(LPS25H_CTRL_REG1, 0xB0);
}

void lps25h_readPressure(void)
{
	//HAL_I2C_Mem_Read(&hi2c1, LPS25H_H_ADDR, LPS25H_PRESS_OUT_XL , 1, alititude.Buf_p, sizeof(alititude.Buf_p), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, LPS25H_H_ADDR, LPS25H_PRESS_OUT_XL | 0x80, 1, (uint8_t*)&alititude.pressure, sizeof(alititude.Buf_p), HAL_MAX_DELAY);

	//alititude.pressure = (alititude.Buf_p[2]<<16) | (alititude.Buf_p[1]<<8) | alititude.Buf_p[0];
}

void lps25h_readPressureInchesHg(void)
{
	lps25h_readPressure();
	alititude.pressure_inches = (float)alititude.pressure/138706.5;
}
void lps25h_readPressureMillibars(void)
{
	lps25h_readPressure();
	alititude.pressure_milibars = alititude.pressure/4096;
}

void lps25h_readTemperature(void)
{
	HAL_I2C_Mem_Read(&hi2c1, LPS25H_H_ADDR, LPS25H_TEMP_OUT_L | 0x80, 1, (uint8_t*)&alititude.temperature, sizeof(alititude.Buf_t), HAL_MAX_DELAY);
	//HAL_I2C_Mem_Read(&hi2c1, LPS25H_H_ADDR, LPS25H_TEMP_OUT_L, 1, alititude.Buf_t, sizeof(alititude.Buf_t), HAL_MAX_DELAY);
	//alititude.temperature = (alititude.Buf_t[1]<<8) | alititude.Buf_t[0];
}
void lps25h_readTemperatureF(void)
{
	lps25h_readTemperature();
	alititude.temperature_F = (float)108.5f + alititude.temperature/ 480.f * 1.8f;
}
void lps25h_readTemperatureC(void)
{
	lps25h_readTemperature();
	alititude.temperature_C = 42.5f + alititude.temperature/ 480.0f;
}

void lps25h_pressureToAlitudeFeet(void)
{
	lps25h_readPressureInchesHg();
	lps25h_readTemperatureF();
	alititude.altitude_feet = -29.271769 * alititude.temperature_F * log(alititude.pressure_inches/ENVIRONMENT_PRESSURE_I);

}
void lps25h_pressureToAlitudeMeters(void)
{

	lps25h_readPressureMillibars();
	lps25h_readTemperatureC();
	alititude.altitude_meters = -29.271769 * alititude.temperature_C * log(alititude.pressure_milibars/ENVIRONMENT_PRESSURE_M);
}
