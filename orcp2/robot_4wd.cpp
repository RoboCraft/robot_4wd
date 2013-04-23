// 
// Base struct for 4WD robot
//
//
// robocraft.ru
//

#include "robot_4wd.h"

uint16_t serialize_robot_4wd(Robot_4WD* src, uint8_t* dst, uint16_t dst_size)
{
	if(!src || !dst || dst_size == 0)
		return -1;

	uint16_t l = 0;	
	dst[l++] = src->Bamper;

	int i;
	for(i=0; i<ENCODERS_COUNT; i++) {
		orcp2::copy_int32(dst+l, src->Encoder[i]);
		l += sizeof(src->Encoder[i]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		orcp2::copy_int16(dst+l, src->PWM[i]);
		l += sizeof(src->PWM[i]);
	}
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		orcp2::copy_int32(dst+l, src->US[i]);
		l += sizeof(src->US[i]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		orcp2::copy_int32(dst+l, src->IR[i]);
		l += sizeof(src->IR[i]);
	}
	orcp2::copy_int32(dst+l, src->Voltage);
	l += sizeof(src->Voltage);

	return l;
}

uint16_t deserialize_robot_4wd(uint8_t* src, uint16_t src_size, Robot_4WD* dst)
{
	if(!src || src_size == 0 || !dst)
		return -1;

	uint16_t l = 0;
	dst->Bamper = src[l++];

	int i;
	for(i=0; i<ENCODERS_COUNT; i++) {
		orcp2::copy_int32( (uint8_t *)&(dst->Encoder[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->Encoder[i]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		orcp2::copy_int16( (uint8_t *)&(dst->PWM[i]), *(uint16_t*)(src+l));
		l += sizeof(dst->PWM[i]);
	}
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		orcp2::copy_int32( (uint8_t *)&(dst->US[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->US[i]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		orcp2::copy_int32( (uint8_t *)&(dst->IR[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->IR[i]);
	}
	orcp2::copy_int32( (uint8_t *)&(dst->Voltage), *(uint32_t*)(src+l));
	l += sizeof(dst->Voltage);

	return l;
}

uint16_t serialize_robot_4wd_drive_part(Robot_4WD* src, uint8_t* dst, uint16_t dst_size)
{
	if(!src || !dst || dst_size == 0)
		return -1;

	uint16_t l = 0;	
	dst[l++] = src->Bamper;

	int i;
	for(i=0; i<ENCODERS_COUNT; i++) {
		orcp2::copy_int32(dst+l, src->Encoder[i]);
		l += sizeof(src->Encoder[i]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		orcp2::copy_int16(dst+l, src->PWM[i]);
		l += sizeof(src->PWM[i]);
	}

	return l;
}

uint16_t deserialize_robot_4wd_drive_part(uint8_t* src, uint16_t src_size, Robot_4WD* dst)
{
	if(!src || src_size == 0 || !dst)
		return -1;

	uint16_t l = 0;
	dst->Bamper = src[l++];

	int i;
	for(i=0; i<ENCODERS_COUNT; i++) {
		orcp2::copy_int32( (uint8_t *)&(dst->Encoder[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->Encoder[i]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		orcp2::copy_int16( (uint8_t *)&(dst->PWM[i]), *(uint16_t*)(src+l));
		l += sizeof(dst->PWM[i]);
	}

	return l;
}

uint16_t serialize_robot_4wd_sensors_part(Robot_4WD* src, uint8_t* dst, uint16_t dst_size)
{
	if(!src || !dst || dst_size == 0)
		return -1;

	uint16_t l = 0;	

	int i;
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		orcp2::copy_int32(dst+l, src->US[i]);
		l += sizeof(src->US[i]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		orcp2::copy_int32(dst+l, src->IR[i]);
		l += sizeof(src->IR[i]);
	}
	orcp2::copy_int32(dst+l, src->Voltage);
	l += sizeof(src->Voltage);

	return l;
}

uint16_t deserialize_robot_4wd_sensors_part(uint8_t* src, uint16_t src_size, Robot_4WD* dst)
{
	if(!src || src_size == 0 || !dst)
		return -1;

	uint16_t l = 0;

	int i;
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		orcp2::copy_int32( (uint8_t *)&(dst->US[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->US[i]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		orcp2::copy_int32( (uint8_t *)&(dst->IR[i]), *(uint32_t*)(src+l));
		l += sizeof(dst->IR[i]);
	}
	orcp2::copy_int32( (uint8_t *)&(dst->Voltage), *(uint32_t*)(src+l));
	l += sizeof(dst->Voltage);

	return l;
}
