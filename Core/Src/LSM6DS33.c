/*
 * LSM6DS33.c
 *
 * 		This librabry is for sensor AltIMU10 v5
 *		INEMO inertial module:
 *		always-on 3D accelerometer and 3D gyroscope
 *
 *      Author: wielkadraka
 */

//#include "stm32f7xx_hal.h"
#include "LSM6DS33.h"

LSM_axis accel;
LSM_axis gyro;

extern I2C_HandleTypeDef hi2c1;

uint8_t lsm6ds33_read_reg(uint8_t reg)
{
	uint8_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_H_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);

	return value;
}
void lsm6ds33_write_reg(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, LSM6DS33_H_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}

void lsm6ds33_readGyro(void)
{
	HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_H_ADDR, LSM6DS33_OUTX_L_G, 1, gyro.Buf, sizeof(gyro.Buf), HAL_MAX_DELAY);
	gyro.x = (gyro.Buf[1]<<8) | gyro.Buf[0];
	gyro.y = (gyro.Buf[3]<<8) | gyro.Buf[2];
	gyro.z = (gyro.Buf[5]<<8) | gyro.Buf[6];
}

void lsm6ds33_readAccel(void)
{
	HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_H_ADDR, LSM6DS33_OUTX_L_XL, 1, accel.Buf, sizeof(accel.Buf), HAL_MAX_DELAY);
	accel.x = (accel.Buf[1]<<8) | accel.Buf[0];
	accel.y = (accel.Buf[3]<<8) | accel.Buf[2];
	accel.z = (accel.Buf[5]<<8) | accel.Buf[6];

//	ax_accel = (przyspieszenie_2g * x_accel)/rozmiar_int16_t;
//	ay_accel = (przyspieszenie_2g * y_accel)/rozmiar_int16_t;
//	az_accel = (przyspieszenie_2g * z_accel)/rozmiar_int16_t;
	//az_accel = rozmiar_int16_t/(przyspieszenie2g * z_accel);
}

//uint8_t WHO_AM_I(void)
//{
//	if(lsm_read_reg(WHO_AM_I) == I_AM)return 1;
//
//	return 0;
//}
void lsm6ds33_Init(void)
{
	lsm6ds33_write_reg(LSM6DS33_CTRL1_XL, 0x80);
	lsm6ds33_write_reg(LSM6DS33_CTRL2_G, 0x80);
	lsm6ds33_write_reg(LSM6DS33_CTRL3_C, 0x04);
}
