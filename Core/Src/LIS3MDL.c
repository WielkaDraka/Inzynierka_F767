/*
 * 		LIS3MDL.c
 *
 *		This librabry is for sensor AltIMU10 v5
 *		Digital output magnetic sensor:
 *		ultra-low-power,high-performance 3-axis magnetometer
 *
 *      Author: wielkadraka
 */
#include "LIS3MDL.h"

sensor LIS3MDL;


extern I2C_HandleTypeDef hi2c1;

uint8_t lis3mdl_read_reg(uint8_t reg)
{
	uint8_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_H_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);

	return value;
}
void lis3mdl_write_reg(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, LIS3MDL_H_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}

void lis3mdl_readDirection(void)
{
	HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_H_ADDR, LIS3MDL_OUT_X_L, 1, LIS3MDL.Buf, sizeof(LIS3MDL.Buf), HAL_MAX_DELAY);
	LIS3MDL.x = (LIS3MDL.Buf[1]<<8) | LIS3MDL.Buf[0];
	LIS3MDL.y = (LIS3MDL.Buf[3]<<8) | LIS3MDL.Buf[2];
	LIS3MDL.z = (LIS3MDL.Buf[5]<<8) | LIS3MDL.Buf[6];
}
void lis3mdl_readTemperature(void)
{
	HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_H_ADDR, LIS3MDL_TEMP_OUT_L, 1, LIS3MDL.Buf2, sizeof(LIS3MDL.Buf2), HAL_MAX_DELAY);
	LIS3MDL.TEMP = (LIS3MDL.Buf2[1]<<8) | LIS3MDL.Buf2[0];
}

void lis3mdl_Init(void)
{
	// OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
	lis3mdl_write_reg(LIS3MDL_CTRL_REG1, 0x70);
	// FS = 00 (+/- 4 gauss full scale)
	lis3mdl_write_reg(LIS3MDL_CTRL_REG2, 0x00);
	// MD = 00 (continuous-conversion mode)
	lis3mdl_write_reg(LIS3MDL_CTRL_REG3, 0x00);
	// OMZ = 11 (ultra-high-performance mode for Z)
	lis3mdl_write_reg(LIS3MDL_CTRL_REG4, 0x0C);
}
