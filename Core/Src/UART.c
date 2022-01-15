/*
 * UART.c
 *
 *  Created on: Nov 11, 2021
 *      Author: wielkadraka
 */


#include "UART.h"
#include "stm32f7xx_hal.h"

extern UART_HandleTypeDef huart2;

void send_char(char c)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 1000);
}

int __io_putchar(int ch)
{
	if (ch == '\n')
		send_char('\r');
	send_char(ch);
	return ch;
}
