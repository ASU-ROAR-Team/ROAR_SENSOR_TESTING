#include "main.h"

#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "Adafruit_BNO055.h"

#ifndef PI
#define PI 					3.14159265359
#endif /* PI */

static float euler[3] = {0.f};

uint8_t _address;
int32_t _sensorID;
adafruit_bno055_opmode_t _mode;

/*pointer to save sensor I2c configrations prameters*/
static I2C_HandleTypeDef* GlobalConfig = NULL;

void Adafruit_BNO055_Init(I2C_HandleTypeDef* Config, int32_t sensorID, uint8_t address)
{
	_sensorID = sensorID;
	_address = address;
	GlobalConfig = Config;
}
volatile uint8_t Co = 0;
void Adafruit_BNO055_Begin(adafruit_bno055_opmode_t mode) {
  uint8_t Data = 0;
  uint8_t id;
  HAL_StatusTypeDef status;

  // Reset the sensor first
  Data = 0x20;
  status = HAL_I2C_Mem_Write(GlobalConfig, _address, BNO055_SYS_TRIGGER_ADDR, 1, &Data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    printf("Error resetting the sensor\r\n");
    return;
  }
  HAL_Delay(650); // Per datasheet, reset takes ~650ms

  // Check device ID
  status = HAL_I2C_Mem_Read(GlobalConfig, _address, BNO055_CHIP_ID_ADDR, 1, &id, 1, HAL_MAX_DELAY);
  if (status != HAL_OK || id != BNO055_ID) {
    printf("Error reading device ID: %d\r\n", id);
    return;
  }

  // Set to normal power mode
  Data = POWER_MODE_NORMAL;
  HAL_I2C_Mem_Write(GlobalConfig, _address, BNO055_PWR_MODE_ADDR, 1, &Data, 1, HAL_MAX_DELAY);
  HAL_Delay(10);

  // Set the requested operating mode
  Adafruit_BNO055_SetMode(mode);
  HAL_Delay(20);
}

void Adafruit_BNO055_SetMode(adafruit_bno055_opmode_t mode)
{
  _mode = mode;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_OPR_MODE_ADDR , 1 , &_mode , 1 , HAL_MAX_DELAY);
  HAL_Delay(30);
}

int8_t Adafruit_BNO055_GetTemp(void)
{
  int8_t temp;
  HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_TEMP_ADDR , 1 , &temp , 1 , HAL_MAX_DELAY);
  return temp;
}

void Adafruit_BNO055_SetExtCrystalUse(uint8_t usextal)
{
  adafruit_bno055_opmode_t modeback = _mode;
  uint8_t Data; 
  /* Switch to config mode (just in case since this is the default) */
  Adafruit_BNO055_SetMode(OPERATION_MODE_CONFIG);
  HAL_Delay(25);
  
  Data = 0;
  HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_PAGE_ID_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  
  if (usextal)
  {
    Data = 0x80;
	HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_SYS_TRIGGER_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  }
  else
  {
    Data = 0x00;
	HAL_I2C_Mem_Write(GlobalConfig , _address , BNO055_SYS_TRIGGER_ADDR , 1 , &Data , 1 , HAL_MAX_DELAY);
  }
  HAL_Delay(10);
  /* Set the requested operating mode (see section 3.3) */
  Adafruit_BNO055_SetMode(modeback);
  HAL_Delay(20);
}

void Adafruit_BNO055_GetCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) 
{	
  uint8_t calData;
  HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_CALIB_STAT_ADDR , 1 , &calData , 1 , HAL_MAX_DELAY);

  if (sys != NULL) 
  {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL)
  {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL)
  {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL)
  {
    *mag = calData & 0x03;
  }
}

void Adafruit_BNO055_GetQuat(Quaternion *quat)
{
  uint8_t buffer[8];
  memset(buffer, 0, 8);

  int16_t xt, yt, zt, wt;
  xt = yt = zt = wt = 0;

  /* Read quat data (8 bytes) */
  HAL_I2C_Mem_Read(GlobalConfig , _address , BNO055_QUATERNION_DATA_W_LSB_ADDR , 1 , buffer , 8 , HAL_MAX_DELAY);
  wt = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  xt = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  yt = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  zt = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
  /*!
   * Assign to Quaternion
   * See
   * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
   * 3.6.5.5 Orientation (Quaternion)
   */
  const double scale = (1.0 / (1 << 14));
  
  quat->w = scale * wt;
  quat->x = scale * xt;
  quat->y = scale * yt;
  quat->z = scale * zt;
 
}

void Adafruit_BNO055_GetVector(adafruit_vector_type_t vector_type , double* xyz)
{
 
  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  HAL_I2C_Mem_Read(GlobalConfig , _address , (adafruit_bno055_reg_t)vector_type , 1 , buffer , 6 , HAL_MAX_DELAY);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type)
  {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  }

}
void Adafruit_BNO055_UpdateEuler(float qw, float qx, float qy, float qz , float Angels[])
{
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
    a12 = 2.0f * (qx * qy + qw * qz);
    a22 = qw * qw + qx * qx - qy * qy - qz * qz;
    a31 = 2.0f * (qw * qx + qy * qz);
    a32 = 2.0f * (qx * qz - qw * qy);
    a33 = qw * qw - qx * qx - qy * qy + qz * qz;
    euler[0] = atan2f(a31, a33);
    euler[1] = -asinf(a32);
    euler[2] = atan2f(a12, a22);
    euler[0] *= 180.0f / PI;
    euler[1] *= 180.0f / PI;
    euler[2] *= 180.0f / PI;
		
  if (euler[2] >= +180.f)
	{
		euler[2] -= 360.f;
	}
  else if(euler[2] < -180.f)
	{
	  euler[2] += 360.f;
	}
	else
	{
		
	}
	
	Angels[0] = euler[0];
	Angels[1] = euler[1];
	Angels[2] = euler[2];
	
}

