/*
 * AlphaBeta.h
 *
 *  Library for alpha-beta-gamma
 *      Author: wielkadraka
 */

#ifndef INC_ALPHABETA_H_
#define INC_ALPHABETA_H_

typedef struct
{
	float dt;
	float alfa;
	float beta;
	float xk, vk;
	float xk_1, vk_1;

}AB_axis;

void AlphaBeta_init(void);
float AlphaBeta_gyro_X(float value, float alpha, float beta);
float AlphaBeta_gyro_Y(float value, float alpha, float beta);
float AlphaBeta_gyro_Z(float value, float alpha, float beta);
#endif /* INC_ALPHABETA_H_ */
