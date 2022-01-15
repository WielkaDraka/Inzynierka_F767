/*
 * 		LIS3MDL.h
 *
 *		This librabry is for sensor AltIMU10 v5
 *		Digital output magnetic sensor:
 *		ultra-low-power,high-performance 3-axis magnetometer
 *
 *      Author: wielkadraka
 */
#include "stm32f7xx_hal.h"

#ifndef INC_LIS3MDL_H_
#define INC_LIS3MDL_H_

#define DIRECTION_DATA	6
#define TEMP_DATA	2

typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

	int16_t TEMP;

	uint8_t Buf[DIRECTION_DATA];
	uint8_t Buf2[TEMP_DATA];

}sensor;

uint8_t lis3mdl_read_reg(uint8_t reg);
void lis3mdl_write_reg(uint8_t reg, uint8_t value);
void lis3mdl_read(void);
void lis3mdl_Init(void);

void lis3mdl_readDirection(void);
void lis3mdl_readTemperature(void);

enum lis3mdl__sa0Adr{
	LIS3MDL_L_ADDR = 0b00111000, //connected to GND
	LIS3MDL_H_ADDR = 0b00111100, //connected to VCC-default
};

enum lis3mdl_regAddr{

	LIS3MDL_WHO_AM_I 	=	0x0F,

	LIS3MDL_CTRL_REG1	=	0x20,
	LIS3MDL_CTRL_REG2	=	0x21,
	LIS3MDL_CTRL_REG3	= 	0x22,
	LIS3MDL_CTRL_REG4	=	0x23,
	LIS3MDL_CTRL_REG5	=	0x24,

	LIS3MDL_STATUS_REG	=	0x27,

	LIS3MDL_OUT_X_L		=	0x28,
	LIS3MDL_OUT_X_H		=	0x29,
	LIS3MDL_OUT_Y_L		= 	0x2A,
	LIS3MDL_OUT_Y_H		=	0x2B,
	LIS3MDL_OUT_Z_L		=	0x2C,
	LIS3MDL_OUT_Z_H		=	0x2D,

	LIS3MDL_TEMP_OUT_L	=	0x2E,
	LIS3MDL_TEMP_OUT_H	=	0x2F,

	LIS3MDL_INT_CFG		=	0x30,
	LIS3MDL_INT_SRC		=	0x31,
	LIS3MDL_INT_THIS_L	=	0x32,
	LIS3MDL_INT_THIS_H	=	0x33,
};

#endif /* INC_LIS3MDL_H_ */
