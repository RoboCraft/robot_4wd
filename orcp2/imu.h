// 
// Base struct for IMU
//
//
// robocraft.ru
//

#ifndef _IMU_H_
#define _IMU_H_

#include "orcp2.h"

typedef struct IMU3_data {
	int16_t Accelerometer [3];	
	int16_t Magnetometer [3];	
	int16_t Gyro [3];
} IMU3_data;

uint16_t serialize_imu3_data(IMU3_data* src, uint8_t* dst, uint16_t dst_size);
uint16_t deserialize_imu3_data(uint8_t* src, uint16_t src_size, IMU3_data* dst);

#endif //#ifndef _IMU_H_
