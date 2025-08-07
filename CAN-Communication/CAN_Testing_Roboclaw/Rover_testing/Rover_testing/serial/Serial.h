/*
 * Serial.h
 *
 *  Created on: April 27, 2025
 *      Author: Adham
 */


#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_

#include "base_serial.h"

SERIAL_HandleTypeDef* serial_init(UART_HandleTypeDef *huartx);

void serial_write(SERIAL_HandleTypeDef* hserial, uint8_t *pData, uint16_t len);

uint8_t serial_available(SERIAL_HandleTypeDef* hserial);

uint8_t serial_read(SERIAL_HandleTypeDef* hserial);

#endif /* SERIAL_SERIAL_H_ */
