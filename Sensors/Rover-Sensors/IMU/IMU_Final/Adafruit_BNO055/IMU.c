
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

