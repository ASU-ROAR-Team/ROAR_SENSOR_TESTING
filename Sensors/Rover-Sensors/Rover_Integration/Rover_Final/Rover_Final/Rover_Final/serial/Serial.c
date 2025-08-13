/*
 * Serial.c
 *
 *  Created on: April 27, 2025
 *      Author: Adham
 */

#include "Serial.h"

#ifdef USE_USART1
SERIAL_HandleTypeDef* USART1_Serial_Handler;
ring_buffer_t USART1_Buffer_TX, USART1_Buffer_RX;
uint8_t USART1_HAL_Reg_Tx, USART1_HAL_Reg_Rx, USART1_Application_Reg_Tx;
#endif
#ifdef USE_USART2
SERIAL_HandleTypeDef* USART2_Serial_Handler;
ring_buffer_t USART2_Buffer_TX, USART2_Buffer_RX;
uint8_t USART2_HAL_Reg_Tx, USART2_HAL_Reg_Rx, USART2_Application_Reg_Tx;
#endif
#ifdef USE_USART3
SERIAL_HandleTypeDef* USART3_Serial_Handler;
ring_buffer_t USART3_Buffer_TX, USART3_Buffer_RX;
uint8_t USART3_HAL_Reg_Tx, USART3_HAL_Reg_Rx, USART3_Application_Reg_Tx;
#endif
#ifdef USE_USART4
SERIAL_HandleTypeDef* USART4_Serial_Handler;
ring_buffer_t USART4_Buffer_TX, USART4_Buffer_RX;
uint8_t USART4_HAL_Reg_Tx, USART4_HAL_Reg_Rx, USART4_Application_Reg_Tx;
#endif
#ifdef USE_USART1
SERIAL_HandleTypeDef __usart1_serial_handler = {

		.buffer_Tx = &USART1_Buffer_TX,
		.buffer_Rx = &USART1_Buffer_RX,
		.hal_reg_Tx = &USART1_HAL_Reg_Tx,
		.hal_reg_Rx = &USART1_HAL_Reg_Rx,
		.application_reg_Rx = &USART1_Application_Reg_Tx,
		.WriteLock = SERIAL_WRITE_UNLOCKED,
		.ReadLock = SERIAL_READ_UNLOCKED
};
#endif
#ifdef USE_USART2
SERIAL_HandleTypeDef __usart2_serial_handler = {

		.buffer_Tx = &USART2_Buffer_TX,
		.buffer_Rx = &USART2_Buffer_RX,
		.hal_reg_Tx = &USART2_HAL_Reg_Tx,
		.hal_reg_Rx = &USART2_HAL_Reg_Rx,
		.application_reg_Rx = &USART2_Application_Reg_Tx,
		.WriteLock = SERIAL_WRITE_UNLOCKED,
		.ReadLock = SERIAL_READ_UNLOCKED
};
#endif
#ifdef USE_USART3
SERIAL_HandleTypeDef __usart3_serial_handler = {

		.buffer_Tx = &USART3_Buffer_TX,
		.buffer_Rx = &USART3_Buffer_RX,
		.hal_reg_Tx = &USART3_HAL_Reg_Tx,
		.hal_reg_Rx = &USART3_HAL_Reg_Rx,
		.application_reg_Rx = &USART3_Application_Reg_Tx,
		.WriteLock = SERIAL_WRITE_UNLOCKED,
		.ReadLock = SERIAL_READ_UNLOCKED
};
#endif
#ifdef USE_USART4
SERIAL_HandleTypeDef __usart4_serial_handler = {
    .buffer_Tx = &USART4_Buffer_TX,
    .buffer_Rx = &USART4_Buffer_RX,
    .hal_reg_Tx = &USART4_HAL_Reg_Tx,
    .hal_reg_Rx = &USART4_HAL_Reg_Rx,
    .application_reg_Rx = &USART4_Application_Reg_Tx,
    .WriteLock = SERIAL_WRITE_UNLOCKED,
    .ReadLock = SERIAL_READ_UNLOCKED
};
#endif
SERIAL_HandleTypeDef* serial_init(UART_HandleTypeDef *huartx) {

	SERIAL_HandleTypeDef *serial_handler = NULL;

#ifdef USE_USART1
		if(huartx->Instance == USART1){

			USART1_Serial_Handler = &(__usart1_serial_handler);

			USART1_Serial_Handler->huartx = huartx;
			ring_buffer_init(USART1_Serial_Handler->buffer_Rx);
			ring_buffer_init(USART1_Serial_Handler->buffer_Tx);
			serial_handler = USART1_Serial_Handler;
			HAL_UART_Receive_DMA(USART1_Serial_Handler->huartx, USART1_Serial_Handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		}
#endif
#ifdef USE_USART2
		if(huartx->Instance == USART2){

			USART2_Serial_Handler = &(__usart2_serial_handler);

			USART2_Serial_Handler->huartx = huartx;
			ring_buffer_init(USART2_Serial_Handler->buffer_Rx);
			ring_buffer_init(USART2_Serial_Handler->buffer_Tx);
			serial_handler = USART2_Serial_Handler;
			HAL_UART_Receive_DMA(USART2_Serial_Handler->huartx, USART2_Serial_Handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		}
#endif
#ifdef USE_USART3
		if(huartx->Instance == USART3){

			USART3_Serial_Handler = &(__usart3_serial_handler);

			USART3_Serial_Handler->huartx = huartx;
			ring_buffer_init(USART3_Serial_Handler->buffer_Rx);
			ring_buffer_init(USART3_Serial_Handler->buffer_Tx);
			serial_handler = USART3_Serial_Handler;
			HAL_UART_Receive_DMA(USART3_Serial_Handler->huartx, USART3_Serial_Handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		}
#endif
#ifdef USE_USART4
		if(huartx->Instance == USART4){

			USART4_Serial_Handler = &(__usart4_serial_handler);

			USART4_Serial_Handler->huartx = huartx;
			ring_buffer_init(USART4_Serial_Handler->buffer_Rx);
			ring_buffer_init(USART4_Serial_Handler->buffer_Tx);
			serial_handler = USART4_Serial_Handler;
			HAL_UART_Receive_DMA(USART4_Serial_Handler->huartx, USART4_Serial_Handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		}
#endif

	return serial_handler;

}

void serial_write(SERIAL_HandleTypeDef* hserial, uint8_t *pData, uint16_t len) {

	if (ring_buffer_is_empty(hserial->buffer_Tx)) {

			if (HAL_UART_Transmit_DMA(hserial->huartx, pData, len) != HAL_OK) {
				ring_buffer_queue_arr(hserial->buffer_Tx, pData, len);
			}
	} else {
			ring_buffer_queue_arr(hserial->buffer_Tx, pData, len);
	}
//	ring_buffer_queue_arr(hserial->buffer_Tx, pData, len);
}

uint8_t serial_available(SERIAL_HandleTypeDef* hserial) {

	return !ring_buffer_is_empty(hserial->buffer_Rx);
}

uint8_t serial_read(SERIAL_HandleTypeDef* hserial) {

		if (!ring_buffer_is_empty(hserial->buffer_Rx)) {
			ring_buffer_dequeue(hserial->buffer_Rx, hserial->application_reg_Rx);
			return *(hserial->application_reg_Rx);
		}

		return '\0';
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	SERIAL_HandleTypeDef* serial_handler = get_serial_handler(huart);
	
	if (serial_handler != NULL) {
		if (!ring_buffer_is_empty(serial_handler->buffer_Tx)) {

			  ring_buffer_dequeue(serial_handler->buffer_Tx, serial_handler->hal_reg_Tx);
			  HAL_UART_Transmit_DMA(huart, serial_handler->hal_reg_Tx, PRIMARY_REG_SIZE);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	SERIAL_HandleTypeDef* serial_handler = get_serial_handler(huart);
	
	if (serial_handler != NULL) {
		ring_buffer_queue_arr(serial_handler->buffer_Rx, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		HAL_UART_Receive_DMA(huart, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->ErrorCode == HAL_UART_ERROR_ORE) {
		// Overrun error - restart DMA receive
		SERIAL_HandleTypeDef* serial_handler = get_serial_handler(huart);
		if (serial_handler != NULL) {
			HAL_UART_Receive_DMA(huart, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		}
	}
}

