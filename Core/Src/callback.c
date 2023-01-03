/*
 * callback.c
 *
 *  Created on: Dec 22, 2022
 *      Author: devilalprajapat
 */

#include <string.h>
#include "main.h"
#include "tim.h"
#include "usart.h"

//uint8_t rx_temp_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_byte;
uint8_t rx_count;
uint8_t rx_length = 0;

volatile uint8_t rx_complete_flag = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == (&huart2)->Instance)
	{
//		if(rx_count >= 64)
//		{
//			rx_count = 0;
//		}
//		rx_temp_buffer[rx_count++] = rx_byte;
//		__HAL_TIM_SET_COUNTER(&htim6, 0);
//		HAL_TIM_Base_Start_IT(&htim6);
		rx_buffer[rx_count++] = rx_byte;
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == (&htim6)->Instance)
	{
		if(rx_complete_flag ==0)
		{
			rx_length = rx_count;
//			memset(rx_buffer,0x00, 64);
//			memcpy(rx_buffer,rx_temp_buffer, rx_length);
			rx_count = 0;
			rx_complete_flag = 1;
		}
		HAL_TIM_Base_Stop_IT(&htim6);

	}

}
