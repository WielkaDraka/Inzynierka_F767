/*
 * Kalman_Filter.c
 *
 *  Created on: Nov 21, 2021
 *      Author: wielkadraka
 */

#include "Kalman_Filter.h"
#include "LSM6DS33.h"
#include "matrix.h"
#include <math.h>


#define przyspieszenie_graw				9.80665

extern LSM_axis accel;
extern LSM_axis gyro;

kalman_axis gx;
kalman_axis gy;

void filter_init(void)
{
	/* Inicjalizacja zmiennych */
	dt = 0.1;

	A[0] = 1;
	A[1] = -dt;
	A[2] = 0;
	A[3] = 1;

	B[0] = dt;
	B[1] = 0;

	C[0] = 1;
	C[1] = 0;

	/* Wartosci poczatkowe filtru */

	gx.P_post[0] = 1;
	gx.P_post[1] = 0;
	gx.P_post[2] = 0;
	gx.P_post[3] = 1;

	gy.P_post[0] = 1;
	gy.P_post[1] = 0;
	gy.P_post[2] = 0;
	gy.P_post[3] = 1;

	acc_x = accel.x;
	acc_y = accel.y;
	acc_z = accel.z;

	gx.x_post[0] = atan2(acc_x,sqrt(acc_y*acc_y+acc_z * acc_z))/(M_PI/180);
	gx.x_post[1] = 0;

	gy.x_post[0] = atan2(acc_y,sqrt(acc_x*acc_x+acc_z * acc_z))/(M_PI/180);
	gy.x_post[1] = 0;
}
void set_filter(float dev_v, float dev_w)
{
	std_dev_v = dev_v;
	std_dev_w = dev_w;
	V[0] = std_dev_v*std_dev_v*dt;
	V[1] = 0;
	V[2] = 0;
	V[3] = std_dev_v*std_dev_v*dt;
	W[0] = std_dev_w*std_dev_w;
}

void gyro_Roll(void)
{
	/* x(t+1|t) = Ax(t|t) + Bu(t) */
	u[0] = (float) gyro.x*245/32768;
	matrix_2x2_mul_2x1(A, gx.x_post, Ax);
	matrix_2x1_mul_1x1(B, u, Bu);
	matrix_2x1_add_2x1(Ax, Bu, gx.x_pri);

	/* P(t+1|t) = AP(t|t)A^T + V */
	matrix_2x2_mul_2x2(A, gx.P_post, AP);
	matrix_2x2_trans(A, AT);
	matrix_2x2_mul_2x2(AP, AT, APAT);
	matrix_2x2_add_2x2(APAT, V, gx.P_pri);

	/* eps(t) = y(t) - Cx(t|t-1) */
	lsm6ds33_readGyro();
	acc_z = accel.z;
	acc_y = accel.y;
	y[0] = atan2(acc_x,sqrt(acc_y*acc_y+acc_z * acc_z))/(M_PI/180);
	matrix_1x2_mul_2x1(C, gx.x_pri, Cx);
	eps[0] = y[0] - Cx[0];

	/* S(t) = CP(t|t-1)C^T + W */
	matrix_1x2_mul_2x2(C, gx.P_pri, CP);
	matrix_1x2_mul_2x1(C, C, CPCT);
	S[0] = CPCT[0] + W[0];

	/* K(t) = P(t|t-1)C^TS(t)^-1 */
	matrix_2x2_mul_2x1(gx.P_pri, C, PCT);
	S1[0] = 1/S[0];
	matrix_2x1_mul_1x1(PCT, S1, K);

	/* x(t|t) = x(t|t-1) + K(t)eps(t) */
	matrix_2x1_mul_1x1(K, eps, Keps);
	matrix_2x1_add_2x1(gx.x_pri, Keps, gx.x_post);

	/* P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T */
	matrix_2x1_mul_1x1(K, S, KS);
	matrix_2x1_mul_1x2(KS, K, KSKT);
	matrix_2x2_sub_2x2(gx.P_pri, KSKT, gx.P_post);
}

void gyro_Pitch(void)
{
	/* x(t+1|t) = Ax(t|t) + Bu(t) */
	u[0] = (float)gyro.y*245/32768;
	matrix_2x2_mul_2x1(A, gy.x_post, Ax);
	matrix_2x1_mul_1x1(B, u, Bu);
	matrix_2x1_add_2x1(Ax, Bu, gy.x_pri);

	/* P(t+1|t) = AP(t|t)A^T + V */
	matrix_2x2_mul_2x2(A, gy.P_post, AP);
	matrix_2x2_trans(A, AT);
	matrix_2x2_mul_2x2(AP, AT, APAT);
	matrix_2x2_add_2x2(APAT, V, gy.P_pri);

	/* eps(t) = y(t) - Cx(t|t-1) */
	lsm6ds33_readGyro();
	acc_z = accel.z;
	acc_x = accel.x;
	y[0] = atan2(acc_y,sqrt(acc_x*acc_x+acc_z * acc_z))/(M_PI/180);
	matrix_1x2_mul_2x1(C, gy.x_pri, Cx);
	eps[0] = y[0] - Cx[0];

	/* S(t) = CP(t|t-1)C^T + W */
	matrix_1x2_mul_2x2(C, gy.P_pri, CP);
	matrix_1x2_mul_2x1(C, C, CPCT);
	S[0] = CPCT[0] + W[0];

	/* K(t) = P(t|t-1)C^TS(t)^-1 */
	matrix_2x2_mul_2x1(gy.P_pri, C, PCT);
	S1[0] = 1/S[0];
	matrix_2x1_mul_1x1(PCT, S1, K);

	/* x(t|t) = x(t|t-1) + K(t)eps(t) */
	matrix_2x1_mul_1x1(K, eps, Keps);
	matrix_2x1_add_2x1(gy.x_pri, Keps, gy.x_post);

	/* P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T */
	matrix_2x1_mul_1x1(K, S, KS);
	matrix_2x1_mul_1x2(KS, K, KSKT);
	matrix_2x2_sub_2x2(gy.P_pri, KSKT, gy.P_post);
}
