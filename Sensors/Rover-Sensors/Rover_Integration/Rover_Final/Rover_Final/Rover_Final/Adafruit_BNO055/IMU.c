
// ------------------------ Includes -----------------------------------------
#include "IMU.h"
#include "stm32f0xx_hal.h"  // <-- ✅ Add this line
#include "bno055.h"
#include "bno055_stm32.h"

//----------------------------------------------------------------------------

// ------------------------------- Functions ---------------------------
void IMU_Setup (I2C_HandleTypeDef* hi2c1){
	  // Assign the I2C handle to the bno055 driver
	  bno055_assignI2C(hi2c1);
	  // Setup the BNO055 sensor using your provided bno055.c functions
	  bno055_setup();
	  // Set the BNO055 operation mode to NDOF (9-degrees-of-freedom).
	  bno055_setOperationModeNDOF();

}

void IMU_Operation(IMU_INFO* imu){

#if IMU_ReadEuler == YES
	// Read Euler angles (Yaw, Pitch, Roll)
    bno055_vector_t euler_vector = bno055_getVectorEuler();

    imu->Yaw = euler_vector.x;
    imu->Pitch = euler_vector.y;
    imu->Roll = euler_vector.z;
#endif

#if IMU_ReadAcceleration == YES
    // Read Accelerometer data (m/s^2)
    bno055_vector_t accel_vector = bno055_getVectorAccelerometer();

    imu->AccX = accel_vector.x;
    imu->AccY = accel_vector.y;
    imu->AccZ = accel_vector.z;
#endif

#if IMU_ReadLinearAcceleration == YES
    // Read Linear Acceleration data (m/s^2, with gravity removed)
    bno055_vector_t lin_acc_vector = bno055_getVectorLinearAccel();

    imu->linAccX = lin_acc_vector.x;
    imu->linAccY = lin_acc_vector.y;
    imu->linAccZ = lin_acc_vector.z;
#endif

#if IMU_ReadMagnetometer == YES
    // Read Magnetometer data (microTesla)
    bno055_vector_t mag_vector = bno055_getVectorMagnetometer();

    imu->MagX = mag_vector.x;
    imu->MagY = mag_vector.y;
    imu->MagZ = mag_vector.z;
#endif

#if IMU_ReadGyroscope == YES
    // Read Gyroscope data (degrees/second)
    bno055_vector_t gyro_vector = bno055_getVectorGyroscope();

    imu->GyroX = gyro_vector.x;
    imu->GyroY = gyro_vector.y;
    imu->GyroZ = gyro_vector.z;
#endif

#if IMU_ReadGravityVector == YES
    // Read Gravity vector (m/s^2)
    bno055_vector_t gravity_vector = bno055_getVectorGravity();

    imu->GravityX = gravity_vector.x;
    imu->GravityY = gravity_vector.y;
    imu->GravityZ = gravity_vector.z;
#endif

#if IMU_ReadQuanterion == YES
    // Read Quaternion data
    bno055_vector_t quat = bno055_getVectorQuaternion();

    imu->QuatW = quat.w;
    imu->QuatX = quat.x;
    imu->QuatY = quat.y;
    imu->QuatZ = quat.z;
#endif

#if IMU_ReadTemperature == YES
    // Read Temperature (Celsius)
    imu->temperature = bno055_getTemp();
#endif
}


void IMU_CANMsgSendOrientation(IMU_INFO* imu, uint8_t* buffer, uint16_t sequence_num){
	uint16_t Yaw = imu->Yaw * 100;
	uint16_t Pitch = imu->Pitch * 100;
	uint16_t Roll = imu->Roll * 100;

	buffer[0] = (Yaw >> 0) & 0xFF;  // low byte
	buffer[1] = (Yaw >> 8) & 0xFF;  // high byte

	buffer[2] = (Pitch >> 0) & 0xFF;
	buffer[3] = (Pitch >> 8) & 0xFF;

	buffer[4] = (Roll >> 0) & 0xFF;
	buffer[5] = (Roll >> 8) & 0xFF;

	buffer[6] = (sequence_num >> 0) & 0xFF;
	buffer[7] = (sequence_num >> 8) & 0xFF;
}
void IMU_CANMsgSendLinearAccel(IMU_INFO* imu, uint8_t* buffer, uint16_t sequence_num){
	uint16_t linAccX = imu->linAccX * 1000;
	uint16_t linAccY = imu->linAccY * 1000;
	uint16_t linAccZ = imu->linAccZ * 1000;

	buffer[0] = (linAccX >> 0) & 0xFF;
	buffer[1] = (linAccX >> 8) & 0xFF;

	buffer[2] = (linAccY >> 0) & 0xFF;
	buffer[3] = (linAccY >> 8) & 0xFF;

	buffer[4] = (linAccZ >> 0) & 0xFF;
	buffer[5] = (linAccZ >> 8) & 0xFF;

	buffer[6] = (sequence_num >> 0) & 0xFF;
	buffer[7] = (sequence_num >> 8) & 0xFF;
}
//void IMU_CANMsgSendLinearVelocity(IMU_INFO* imu, uint8_t* buffer){
//	uint16_t linAccX = imu->linAccX * 1000;
//	uint16_t linAccY = imu->linAccY * 1000;
//	uint16_t linAccZ = imu->linAccZ * 1000;
//
//	buffer[0] = (linAccX & 0xFF) >> 0;
//	buffer[1] = (linAccX & 0xFF) >> 8;
//
//	buffer[2] = (linAccY & 0xFF) >> 0;
//	buffer[3] = (linAccY & 0xFF) >> 8;
//
//	buffer[4] = (linAccZ & 0xFF) >> 0;
//	buffer[5] = (linAccZ & 0xFF) >> 8;
//
//
//	buffer[6] = (counter & 0xFF) >> 0;
//	buffer[7] = (counter & 0xFF) >> 8;
//}

