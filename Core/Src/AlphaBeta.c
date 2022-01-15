/*
 * AlphaBeta.c
 *
 *  Library for alpha-beta filter
 *      Author: wielkadraka
 */
#include "AlphaBeta.h"

AB_axis AB_x, AB_y, AB_z;

void AlphaBeta_init(void)
{
	AB_x.dt = 0.1;
	AB_x.xk_1 = 0;
	AB_x.vk_1 = 0;

	AB_y.dt = 0.1;
	AB_y.xk_1 = 0;
	AB_y.vk_1 = 0;

	AB_z.dt = 0.1;
	AB_z.xk_1 = 0;
	AB_z.vk_1 = 0;
}

float AlphaBeta_gyro_X(float value, float alpha, float beta)
{
	AB_x.alfa = alpha;
	AB_x.beta = beta;

	AB_x.xk = AB_x.xk_1 + AB_x.dt * AB_x.vk_1;
	AB_x.vk = AB_x.vk_1;

	AB_x.xk_1 = AB_x.xk + AB_x.alfa * (value - AB_x.xk);
	AB_x.vk_1 = AB_x.vk + AB_x.beta * (value - AB_x.xk) / AB_x.dt;

	return AB_x.xk_1;
}
float AlphaBeta_gyro_Y(float value, float alpha, float beta)
{
	AB_y.alfa = alpha;
	AB_y.beta = beta;

	AB_y.xk = AB_y.xk_1 + AB_y.dt * AB_y.vk_1;
	AB_y.vk = AB_y.vk_1;

	AB_y.xk_1 = AB_y.xk + AB_y.alfa * (value - AB_y.xk);
	AB_y.vk_1 = AB_y.vk + AB_y.beta * (value - AB_y.xk) / AB_y.dt;

	return AB_y.xk_1;
}
float AlphaBeta_gyro_Z(float value, float alpha, float beta)
{
	AB_z.alfa = alpha;
	AB_z.beta = beta;

	AB_z.xk = AB_z.xk_1 + AB_z.dt * AB_z.vk_1;
	AB_z.vk = AB_z.vk_1;

	AB_z.xk_1 = AB_z.xk + AB_z.alfa * (value - AB_z.xk);
	AB_z.vk_1 = AB_z.vk + AB_z.beta * (value - AB_z.xk) / AB_z.dt;

	return AB_z.xk_1;
}
