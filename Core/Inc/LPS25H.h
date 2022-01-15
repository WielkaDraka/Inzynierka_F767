/*
 * LPS25H.h
 *
 *		This librabry is for sensor AltIMU10 v5
 *		MEMS pressure sensor: 260-1260 hPa absolute digital output barometer:
 *
 *      Author: wielkadraka
 */

#ifndef INC_LPS25H_H_
#define INC_LPS25H_H_

#include "stm32f7xx_hal.h"

#define PRESSURE_DATA			3
#define TEMPERATURE_DATA		2

#define ENVIRONMENT_PRESSURE_M 	1013.15
#define ENVIRONMENT_PRESSURE_I	29.9213

typedef struct{

	int32_t pressure;
	uint8_t Buf_p[PRESSURE_DATA];

	//pressure in inches
	float pressure_inches;
	//pressure in milibars
	float pressure_milibars;

	int16_t temperature;
	uint8_t Buf_t[TEMPERATURE_DATA];

	//temperature in degrees C
	float temperature_C;
	//temperature in degrees F
	float temperature_F;

	//altitude in meters
	float altitude_meters;

	//altitude in feet
	float altitude_feet;


}LPS25H;


uint8_t lps25h_read_reg(uint8_t reg);
void lps25h_write_reg(uint8_t reg, uint8_t value);

void lps25h_Init(void);

// reads pressure and returns raw 24-bit sensor output
void lps25h_readPressure(void);
// reads pressure in inches of mercury (inHg)
void lps25h_readPressureInchesHg(void);
// reads pressure in millibars (mbar)/hectopascals (hPa)
void lps25h_readPressureMillibars(void);

// reads temperature and returns raw 16-bit sensor output
void lps25h_readTemperature(void);
// reads temperature in degrees F
void lps25h_readTemperatureF(void);
// reads temperature in degrees C
void lps25h_readTemperatureC(void);

// converts pressure in mbar to altitude in meters, using 1976 US
// Standard Atmosphere model (note that this formula only applies to a
// height of 11 km, or about 36000 ft)
//  If altimeter setting (QNH, barometric pressure adjusted to sea
//  level) is given, this function returns an indicated altitude
//  compensated for actual regional pressure; otherwise, it returns
//  the pressure altitude above the standard pressure level of 1013.25
//  mbar or 29.9213 inHg
void lps25h_pressureToAlitudeMeters(void);
// converts pressure in inHg to altitude in feet; see notes above
void lps25h_pressureToAlitudeFeet(void);


enum lps25h_sa0Adr{
	LPS25H_L_ADDR = 0b10111000, //connected to GND
	LPS25H_H_ADDR = 0b10111010, //connected to VCC-default
};

enum lps25h_regAddr{

	LPS25H_REF_P_XL 		= 0x08,
	LPS25H_REF_P_L			= 0x09,
	LPS25H_REF_P_H			= 0x0A,

	LPS25H_WHO_AM_I			= 0x0F,

	LPS25H_RES_CONF			= 0x10,

	LPS25H_CTRL_REG1		= 0x20,
	LPS25H_CTRL_REG2		= 0x21,
	LPS25H_CTRL_REG3		= 0x22,
	LPS25H_CTRL_REG4		= 0x23,

	LPS25H_INT_CFG			= 0x24,
	LPS25H_INT_SOURCE		= 0x25,

	LPS25H_STATUS_REG		= 0x27,
	LPS25H_PRESS_OUT_XL 	= 0x28,
	LPS25H_PRESS_OUT_L		= 0x29,
	LPS25H_PRESS_OUT_H		= 0x2A,
	LPS25H_TEMP_OUT_L		= 0x2B,
	LPS25H_TEMP_OUT_H		= 0x2C,

	LPS25H_FIFO_CTRL		= 0x2E,
	LPS25H_FIFO_STATUS		= 0x2F,

	LPS25H_THS_P_L			= 0x30,
	LPS25H_THS_P_H			= 0x31,

	LPS25H_RPDS_L			= 0x39,
	LPS25H_RPDS_H			= 0x3A,

};

#endif /* INC_LPS25H_H_ */
