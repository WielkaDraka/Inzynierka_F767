/*
 * alphabetagamma.c
 *
 *	Library for alpha-beta-gamma filter
 *      Author: wielkadraka
 */
#include "alphabetagamma.h"
#include "LSM6DS33.h"
#include <math.h>

abg_axis x_accel, y_accel, z_accel;

void abg_filter_Init()
{
	x_accel.xk_1 = 0;
	x_accel.vk_1 = 0;
	x_accel.ak_1 = 0;

	y_accel.xk_1 = 0;
	y_accel.vk_1 = 0;
	y_accel.ak_1 = 0;

	z_accel.xk_1 = 0;
	z_accel.vk_1 = 0;
	z_accel.ak_1 = 0;

	x_accel.x = 0;
	x_accel.v = 0;
	x_accel.a = 0;

	y_accel.x = 0;
	y_accel.v = 0;
	y_accel.a = 0;

	z_accel.x = 0;
	z_accel.v = 0;
	z_accel.a = 0;


	dt = 0.1;
}
void set_filter_abg(float a, float b, float g)
{
	alpha = a;
	beta = b;
	gama = g;
}
float calculate_X_accel (float accel)
{
	x_accel.a = accel;
	x_accel.x = x_accel.x + x_accel.v * dt + x_accel.a * dt * dt/2;
	x_accel.v = x_accel.v + x_accel.a * dt;

	x_accel.ak = x_accel.xk_1 + x_accel.vk_1 * dt + x_accel.ak_1 * dt *dt / 2;
	x_accel.vk = x_accel.vk_1 + x_accel.ak_1 * dt;
	x_accel.ak = x_accel.ak_1;

	x_accel.xk_1 = x_accel.xk + alpha * (x_accel.x - x_accel.xk);
	x_accel.vk_1 = x_accel.vk + beta * (x_accel.x - x_accel.xk)/dt;
	x_accel.ak_1 = x_accel.ak + gama * (2 * (x_accel.x - x_accel.xk)/dt/dt);

	return x_accel.xk_1;
}
float calculate_Y_accel (float accel)
{
	y_accel.a = accel;
	y_accel.x = y_accel.x + y_accel.v * dt + y_accel.a * dt * dt/2;
	y_accel.v = y_accel.v + y_accel.a * dt;

	y_accel.ak = y_accel.xk_1 + y_accel.vk_1 * dt + y_accel.ak_1 * dt *dt / 2;
	y_accel.vk = y_accel.vk_1 + y_accel.ak_1 * dt;
	y_accel.ak = y_accel.ak_1;

	y_accel.xk_1 = y_accel.xk + alpha * (y_accel.x - y_accel.xk);
	y_accel.vk_1 = y_accel.vk + beta * (y_accel.x - y_accel.xk)/dt;
	y_accel.ak_1 = y_accel.ak + gama * (2 * (y_accel.x - y_accel.xk)/dt/dt);

	return y_accel.xk_1;
}
float calculate_Z_accel (float accel)
{
	z_accel.a = accel + przyspieszenie_graw;
	z_accel.x = z_accel.x + z_accel.v * dt + z_accel.a * dt * dt/2;
	z_accel.v = z_accel.v + z_accel.a * dt;

	z_accel.ak = z_accel.xk_1 + z_accel.vk_1 * dt + z_accel.ak_1 * dt *dt / 2;
	z_accel.vk = z_accel.vk_1 + z_accel.ak_1 * dt;
	z_accel.ak = z_accel.ak_1;

	z_accel.xk_1 = z_accel.xk + alpha * (z_accel.x - z_accel.xk);
	z_accel.vk_1 = z_accel.vk + beta * (z_accel.x - z_accel.xk)/dt;
	z_accel.ak_1 = z_accel.ak + gama * (2 * (z_accel.x - z_accel.xk)/dt/dt);

	return z_accel.xk_1;
}
