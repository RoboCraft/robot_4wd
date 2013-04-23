// 
// Base struct for IMU
//
//
// robocraft.ru
//

#include "imu.h"

uint16_t serialize_imu3_data(IMU3_data* src, uint8_t* dst, uint16_t dst_size)
{
	if(!src || !dst || dst_size == 0)
		return -1;

	uint16_t l = 0;	

	int i;
	for(i=0; i<3; i++) {
		orcp2::copy_int16(dst+l, src->Accelerometer[i]);
		l += sizeof(src->Accelerometer[i]);
	}
	for(i=0; i<3; i++) {
		orcp2::copy_int16(dst+l, src->Magnetometer[i]);
		l += sizeof(src->Magnetometer[i]);
	}
	for(i=0; i<3; i++) {
		orcp2::copy_int16(dst+l, src->Gyro[i]);
		l += sizeof(src->Gyro[i]);
	}

	return l;
}

uint16_t deserialize_imu3_data(uint8_t* src, uint16_t src_size, IMU3_data* dst)
{
	if(!src || src_size == 0 || !dst)
		return -1;

	uint16_t l = 0;

	int i;
	for(i=0; i<3; i++) {
		orcp2::copy_int16( (uint8_t *)&(dst->Accelerometer[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->Accelerometer[i]);
	}
	for(i=0; i<3; i++) {
		orcp2::copy_int16( (uint8_t *)&(dst->Magnetometer[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->Magnetometer[i]);
	}
	for(i=0; i<3; i++) {
		orcp2::copy_int16( (uint8_t *)&(dst->Gyro[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->Gyro[i]);
	}

	return l;
}
