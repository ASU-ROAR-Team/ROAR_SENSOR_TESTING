
#ifndef GPS_H
#define GPS_H

#include "stm32f0xx_hal.h"
//------------------------- Steps ---------------------------
/*
 * 1- Configure the Uart you want to use
 * 2- create an object from the struct GPS_DATA
 * 3- use the function GPS_Operation and pass the address of the UART handle you configured and the address of the object u created to this function
 * */
//-----------------------------------------------------------

//--------------------- Defines ----------------------------
#define MSGReceived_SIZE 	140

#define  PARSING_ERROR 0
#define  PARSING_OK    1
//-----------------------------------------------------------

//------------------------- Typedefs --------------------

typedef struct {
    float utc_time;          // hhmmss.ss

    float latitude;          // ddmm.mmmm
    char ns_indicator;          // 'N' or 'S'
    float longitude;         // dddmm.mmmm
    char ew_indicator;          // 'E' or 'W'

    int  position_fix;          // 0 = Invalid, 1 = GPS fix, 2 = DGPS fix

    int  satellites_used;       // Number of satellites

    float hdop;                 // Horizontal Dilution of Precision
    float altitude;             // Altitude value
    char altitude_units;        // 'M' for meters

    float geoid_separation;     // Geoid separation
    char geoid_separation_units;// 'M' for meters
}GPS_INFO;
//------------------------------------------------------------

//---------------------- Functions Declaration -----------------

void GPS_Operation(UART_HandleTypeDef* huart, GPS_INFO *data);
char GPS_ReceiveMsg(UART_HandleTypeDef* huart);
char GPS_ParseMsg(GPS_INFO *data);

float toFloat(char *c, int size);

//-----------------------------------------------------------

#endif /*GPS_H*/
