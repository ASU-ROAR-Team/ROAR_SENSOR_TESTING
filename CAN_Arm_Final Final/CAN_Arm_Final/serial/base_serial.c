/*
 * base_serial.c
 *
 *  Created on: April 27, 2025
 *      Author: Adham
 */

#include "base_serial.h"

extern SERIAL_HandleTypeDef* USART1_Serial_Handler;
extern SERIAL_HandleTypeDef* USART2_Serial_Handler;
extern SERIAL_HandleTypeDef* USART3_Serial_Handler;
extern SERIAL_HandleTypeDef* USART4_Serial_Handler;

SERIAL_HandleTypeDef* get_serial_handler(UART_HandleTypeDef *huartx){

	SERIAL_HandleTypeDef* ret_serial_handler = NULL;

#ifdef USE_USART1
	if(huartx->Instance == USART1){
		ret_serial_handler = USART1_Serial_Handler;
	}
#endif
#ifdef USE_USART2
	if(huartx->Instance == USART2){
		ret_serial_handler = USART2_Serial_Handler;
	}
#endif
#ifdef USE_USART3
	if(huartx->Instance == USART3){
		ret_serial_handler = USART3_Serial_Handler;
	}
#endif
#ifdef USE_USART4
	if(huartx->Instance == USART4){
		ret_serial_handler = USART4_Serial_Handler;
	}
#endif
	return ret_serial_handler;
}

