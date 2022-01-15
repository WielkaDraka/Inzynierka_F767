/*
 * alphabetagamma.h
 *
 *	Library for alpha-beta-gamma filter
 *      Author: wielkadraka
 */

#ifndef INC_ALPHABETAGAMMA_H_
#define INC_ALPHABETAGAMMA_H_

#define rozmiar_int16_t					32768
#define przyspieszenie_graw				9.80665

float dt;

float alpha;
float beta;
float gama;

typedef struct{
	float x, v, a;
	float xk, vk, ak, rk;
	float xk_1, vk_1, ak_1;
}abg_axis;


void abg_filter_Init();
void set_filter_abg(float a, float b, float g);
float calculate_X_accel (float accel);
float calculate_Y_accel (float accel);
float calculate_Z_accel (float accel);
#endif /* INC_ALPHABETAGAMMA_H_ */
