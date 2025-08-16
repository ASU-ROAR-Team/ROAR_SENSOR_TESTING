

//-------------------- Includes ----------------------
#include "GPS.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
//----------------------------------------------------

//-------------------------- Variables ----------------------

const char MsgHeader[]="$GPGGA";
char MsgReceived[MSGReceived_SIZE]={0};
char *MsgDebug;

//-----------------------------------------------------------

//----------------------- Main functions ----------------------

void GPS_Operation(UART_HandleTypeDef* huart, GPS_INFO *data){

	if (GPS_ReceiveMsg(huart) == HAL_OK){
		MsgDebug = "Message Received";
		if(GPS_ParseMsg(data) == PARSING_OK){
			MsgDebug="Message Parsed";
		}
		else{
			MsgDebug="Cannot Parse";
		}
	}
	else{
		MsgDebug = "Cannot Receive";
	}
	HAL_Delay(20);
}


char GPS_ReceiveMsg(UART_HandleTypeDef* huart){
	return HAL_UART_Receive(huart , MsgReceived, sizeof(MsgReceived) , HAL_MAX_DELAY );
}

char GPS_ParseMsg(GPS_INFO *data){

    int i;
    char *ptr = MsgReceived; // Corrected variable name
    int field = 0;
    char *end;
    char temp[20]; // Declare temp
    int len;
    MsgReceived[0] = '$';
	for (i=0 ; i<(sizeof(MsgHeader)-1) ; i++){
		if(MsgReceived[i]!= MsgHeader[i]){
			return PARSING_ERROR;
		}
	}



    while (*ptr && field <= 14) {
        end = strchr(ptr, ',');
        if (!end) end = strchr(ptr, '*');  // Last field before checksum

				 len = end ? (end - ptr) : strlen(ptr);



        switch (field) {
            case 1: // UTC Time
                strncpy(temp, ptr, len);
                temp[len] = '\0';
				data->utc_time = toFloat(temp , len);
                break;
            case 2: // Latitude
                strncpy(temp, ptr, len);
                temp[len] = '\0';
                data->latitude = toDouble(temp , len); // Convert to float
                break;
            case 3: // N/S Indicator
                data->ns_indicator = *ptr;
                break;
            case 4: // Longitude
                strncpy(temp, ptr, len);
                temp[len] = '\0';
                data->longitude = toDouble(temp , len); // Convert to float
                break;
            case 5: // E/W Indicator
                data->ew_indicator = *ptr;
                break;
            case 6: // Position Fix
                strncpy(temp, ptr, len);
                temp[len] = '\0';
                data->position_fix = toFloat(temp , len);
                break;
            case 7: // Satellites Used
                strncpy(temp, ptr, len);
                temp[len] = '\0';
                data->satellites_used = toFloat(temp , len);
                break;
            case 8: // HDOP
                strncpy(temp, ptr, len);
                temp[len] = '\0';
                data->hdop = toFloat(temp , len);
                break;
            case 9: // Altitude
                strncpy(temp, ptr, len);
                temp[len] = '\0';
                data->altitude = toFloat(temp , len);
                break;
            case 10: // Altitude Units
                data->altitude_units = *ptr;
                break;
            case 11: // Geoid Separation
                strncpy(temp, ptr, len);
                temp[len] = '\0';
                data->geoid_separation = toFloat(temp , len);
                break;
            case 12: // Geoid Separation Units
                data->geoid_separation_units = *ptr;
                break;
            // fields 13, 14 are optional (DGPS data), can ignore safely
            default:
                break;
        }

        if (end) ptr = end + 1;
        field++;
    }
	return PARSING_OK;
}

void GPS_CANMsgSendLatitude(GPS_INFO *data, uint8_t* c){
	uint64_t lat = data->latitude * 10000000000;
    for (int i = 0; i < 7; i++) {
        c[6 - i] = (uint8_t)(lat & 0xFF);
        lat >>= 8;
    }
    if (data->ns_indicator == 'N'){
    	c[7] = 1 << 8;
    }
    else if (data->ns_indicator == 'S'){
    	c[7] = 0 << 8;
    }
    c[7] = (c[7] & 0x80) | ((c[7] + 1) & 0x7F);

}

void GPS_CANMsgSendLongitude(GPS_INFO *data, uint8_t* c){
	uint64_t lat = data->longitude * 10000000000;
    for (int i = 0; i < 7; i++) {
        c[6 - i] = (uint8_t)(lat & 0xFF);
        lat >>= 8;
    }
    if (data->ew_indicator == 'E'){
    	c[7] = 1 << 8;
    }
    else if (data->ew_indicator == 'W'){
    	c[7] = 0 << 8;
    }
    c[7] = (c[7] & 0x80) | ((c[7] + 1) & 0x7F);

}

//---------------------- Helper Functions -------------------

float toFloat(char *c, int size) {
    int i, j;
    char *ptr = c;
    float result = 0;
    int isNegative = 0;

    // Check if the number is negative
    if (ptr[0] == '-') {
        isNegative = 1;
    }

    // Find the position of the decimal point
    for (j = isNegative ? 1 : 0; j < size; j++) {
        if (ptr[j] == '.') {
            break;
        }
    }

    // Process the integer part
    for (i = j - 1; i >= (isNegative ? 1 : 0); i--) {
        result += (ptr[i] - '0') * pow(10, (j - 1) - i);
    }

    // Process the fractional part
    for (i = j + 1; i < size; i++) {
        result += (ptr[i] - '0') * pow(10, j - i);
    }

    if (isNegative) {
        result = -result;
    }

    return result;
}


double toDouble(const char *c, int size) {
    int i, j;
    const char *ptr = c;
    double result = 0;
    int isNegative = 0;

    // Check if the number is negative
    if (ptr[0] == '-') {
        isNegative = 1;
    }

    // Find the position of the decimal point
    for (j = isNegative ? 1 : 0; j < size; j++) {
        if (ptr[j] == '.') {
            break;
        }
    }

    // Process the integer part
    for (i = j - 1; i >= (isNegative ? 1 : 0); i--) {
        result += (ptr[i] - '0') * pow(10.0, (j - 1) - i);
    }

    // Process the fractional part
    for (i = j + 1; i < size; i++) {
        result += (ptr[i] - '0') * pow(10.0, j - i);
    }

    if (isNegative) {
        result = -result;
    }

    return result;
}

//-----------------------------------------------------------


