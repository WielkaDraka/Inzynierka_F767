/*
 * Kalman_Filter.h
 *
 *  Created on: Nov 21, 2021
 *      Author: wielkadraka
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

	float dt;

	float A[4], B[2], C[2];
	float std_dev_v, std_dev_w;
	float V[4], W[1];

	float eps[1], S[1], K[2];
	float u[1], y[1];
	float acc_x, acc_y, acc_z;

	float Ax[2], Bu[2];
	float AP[4], AT[4], APAT[4];
	float Cx[1];
	float CP[2], CPCT[1];
	float PCT[2], S1[1];
	float Keps[2];
	float KS[2], KSKT[2];

	typedef struct{
		float x_post[2];
		float P_pri[4], P_post[4];
		float x_pri[2];
	}kalman_axis;




void filter_init(void);
void set_filter(float dev_v, float dev_w);
void gyro_Roll(void);
void gyro_Pitch(void);

#endif /* INC_KALMAN_FILTER_H_ */
