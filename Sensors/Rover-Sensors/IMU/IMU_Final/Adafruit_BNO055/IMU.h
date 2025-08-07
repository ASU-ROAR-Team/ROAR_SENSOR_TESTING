#ifndef IMU_H
#define IMU_H

//------------------ Steps of Configuration -----------------------
/*
 * 1- Configure the data you want to read
 * 2- configure i2c you want to use from the ioc file or the main code
 * 3- add the IMU_Setup to the main code and pass it the address of the i2c handle you are using
 * 4- Create an object of the IMU_INFO struct in the main file
 * 4- add the IMU_Operation function to the main and pass the address of the imu object to this function
 * */
//------------------------------------------------------------------

// ------------------------------ Includes ------------------------
#include "stm32f0xx_hal.h"
//------------------------------------------------------------------

// ---------------------- Defines & Configurations ------------------
#define YES 		1
#define NO 			0


/*
 * Use these Configs to decide what values you want to read
 *
 * */

#define IMU_ReadEuler YES
#define IMU_ReadAcceleration YES
#define IMU_ReadLinearAcceleration YES
#define IMU_ReadMagnetometer YES
#define IMU_ReadGyroscope YES
#define IMU_ReadGravityVector YES
#define IMU_ReadQuanterion YES
#define IMU_ReadTemperature YES

//-------------------------------------------------------------------

//------------------------------ Typedef ---------------------------
typedef struct {
#if IMU_ReadEuler == YES
	//Euler
	double Yaw ; // --> x // Yaw angle (heading)
	double Pitch; // --> y // Pitch angle (tilt front-to-back)
	double Roll; // --> z // Roll angle (tilt side-to-side)
#endif
#if IMU_ReadAcceleration == YES
	//Acceleration
	double AccX ;
	double AccY ;
	double AccZ ;
#endif
#if IMU_ReadLinearAcceleration == YES
	//Linear Acceleration
	double linAccX ;
	double linAccY ;
	double linAccZ ;
#endif
#if IMU_ReadMagnetometer == YES
	//Magnetometer
	double MagX ;
	double MagY ;
	double MagZ ;
#endif
#if IMU_ReadGyroscope == YES
	//Gyroscope
	double GyroX ;
	double GyroY ;
	double GyroZ ;
#endif
#if IMU_ReadGravityVector == YES
	//Gravity Vector
	double GravityX ;
	double GravityY ;
	double GravityZ ;
#endif
#if IMU_ReadQuanterion == YES
	//Quanterion
	double QuatW ;
	double QuatX ;
	double QuatY ;
	double QuatZ ;
#endif
#if IMU_ReadTemperature == YES
	double temperature ;
#endif

}IMU_INFO;
//------------------------------------------------------------------

//----------------------- Functions Declarations ------------------
void IMU_Setup(I2C_HandleTypeDef* hi2c1);
void IMU_Operation();
//-----------------------------------------------------------------

#endif
