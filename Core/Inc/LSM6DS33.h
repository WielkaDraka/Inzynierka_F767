/*
 * LSM6DS33.h
 *
 * 		This librabry is for sensor AltIMU10 v5
 *		INEMO inertial module:
 *		always-on 3D accelerometer and 3D gyroscope
 *
 *      Author: wielkadraka
 */

#ifndef INC_LSM6DS33_H_
#define INC_LSM6DS33_H_

#include "stm32f7xx_hal.h"

#define DIRECTION_DATA	6

typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

	uint8_t Buf[DIRECTION_DATA];

}LSM_axis;

uint8_t lsm6ds33_read_reg(uint8_t reg);
void lsm6ds33_write_reg(uint8_t reg, uint8_t value);
void lsm6ds33_readGyro(void);
void lsm6ds33_readAccel(void);
void lsm6ds33_Init(void);


enum lsm6_sa0Adr{
	LSM6DS33_L_ADDR = 0b11010100, //connected to GND
	LSM6DS33_H_ADDR = 0b11010110, //connected to VCC-default
};

enum lsm6_regAddr{
	LSM6DS33_FUNC_CFG_ACCESS   = 0x01,

	LSM6DS33_FIFO_CTRL1        = 0x06,
	LSM6DS33_FIFO_CTRL2        = 0x07,
	LSM6DS33_FIFO_CTRL3        = 0x08,
	LSM6DS33_FIFO_CTRL4        = 0x09,
	LSM6DS33_FIFO_CTRL5        = 0x0A,
	LSM6DS33_ORIENT_CFG_G      = 0x0B,

	LSM6DS33_INT1_CTRL         = 0x0D,
	LSM6DS33_INT2_CTRL         = 0x0E,
	LSM6DS33_WHO_AM_I          = 0x0F,
	LSM6DS33_CTRL1_XL          = 0x10,
	LSM6DS33_CTRL2_G           = 0x11,
	LSM6DS33_CTRL3_C           = 0x12,
	LSM6DS33_CTRL4_C           = 0x13,
	LSM6DS33_CTRL5_C           = 0x14,
	LSM6DS33_CTRL6_C           = 0x15,
	LSM6DS33_CTRL7_G           = 0x16,
	LSM6DS33_CTRL8_XL          = 0x17,
	LSM6DS33_CTRL9_XL          = 0x18,
	LSM6DS33_CTRL10_C          = 0x19,

	LSM6DS33_WAKE_UP_SRC       = 0x1B,
	LSM6DS33_TAP_SRC           = 0x1C,
	LSM6DS33_D6D_SRC           = 0x1D,
	LSM6DS33_STATUS_REG        = 0x1E,

	LSM6DS33_OUT_TEMP_L        = 0x20,
	LSM6DS33_OUT_TEMP_H        = 0x21,
	LSM6DS33_OUTX_L_G          = 0x22,
	LSM6DS33_OUTX_H_G          = 0x23,
	LSM6DS33_OUTY_L_G          = 0x24,
	LSM6DS33_OUTY_H_G          = 0x25,
	LSM6DS33_OUTZ_L_G          = 0x26,
	LSM6DS33_OUTZ_H_G          = 0x27,
	LSM6DS33_OUTX_L_XL         = 0x28,
	LSM6DS33_OUTX_H_XL         = 0x29,
	LSM6DS33_OUTY_L_XL         = 0x2A,
	LSM6DS33_OUTY_H_XL         = 0x2B,
	LSM6DS33_OUTZ_L_XL         = 0x2C,
	LSM6DS33_OUTZ_H_XL         = 0x2D,

	LSM6DS33_FIFO_STATUS1      = 0x3A,
	LSM6DS33_FIFO_STATUS2      = 0x3B,
	LSM6DS33_FIFO_STATUS3      = 0x3C,
	LSM6DS33_FIFO_STATUS4      = 0x3D,
	LSM6DS33_FIFO_DATA_OUT_L   = 0x3E,
	LSM6DS33_FIFO_DATA_OUT_H   = 0x3F,
	LSM6DS33_TIMESTAMP0_REG    = 0x40,
	LSM6DS33_TIMESTAMP1_REG    = 0x41,
	LSM6DS33_TIMESTAMP2_REG    = 0x42,

	LSM6DS33_STEP_TIMESTAMP_L  = 0x49,
	LSM6DS33_STEP_TIMESTAMP_H  = 0x4A,
	LSM6DS33_STEP_COUNTER_L    = 0x4B,
	LSM6DS33_STEP_COUNTER_H    = 0x4C,

	LSM6DS33_FUNC_SRC          = 0x53,

	LSM6DS33_TAP_CFG           = 0x58,
	LSM6DS33_TAP_THS_6D        = 0x59,
	LSM6DS33_INT_DUR2          = 0x5A,
	LSM6DS33_WAKE_UP_THS       = 0x5B,
	LSM6DS33_WAKE_UP_DUR       = 0x5C,
	LSM6DS33_FREE_FALL         = 0x5D,
	LSM6DS33_MD1_CFG           = 0x5E,
	LSM6DS33_MD2_CFG           = 0x5F,

};
#endif /* INC_LSM6DS33_H_ */
