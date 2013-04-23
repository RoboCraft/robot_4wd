// 
// Base struct for 4WD robot
//
//
// robocraft.ru
//

#ifndef _ROBOT_4WD_H_
#define _ROBOT_4WD_H_

#include "orcp2.h"

#define MOTORS_COUNT 	4
#define ENCODERS_COUNT 	4

// sensors
#define ULTRASONIC_COUNT	1
#define INFRARED_COUNT		4
#define BAMPER_COUNT		4

#define BAMPER_1 1
#define BAMPER_2 2
#define BAMPER_3 4
#define BAMPER_4 8

typedef struct Robot_4WD {
	uint8_t 	Bamper;
	int32_t 	Encoder [ENCODERS_COUNT];
	int16_t 	PWM [MOTORS_COUNT];
	uint32_t 	US [ULTRASONIC_COUNT];
	uint32_t 	IR [INFRARED_COUNT];
	uint32_t 	Voltage;
} Robot_4WD;

uint16_t serialize_robot_4wd(Robot_4WD* src, uint8_t* dst, uint16_t dst_size);
uint16_t deserialize_robot_4wd(uint8_t* src, uint16_t src_size, Robot_4WD* dst);

uint16_t serialize_robot_4wd_drive_part(Robot_4WD* src, uint8_t* dst, uint16_t dst_size);
uint16_t deserialize_robot_4wd_drive_part(uint8_t* src, uint16_t src_size, Robot_4WD* dst);

uint16_t serialize_robot_4wd_sensors_part(Robot_4WD* src, uint8_t* dst, uint16_t dst_size);
uint16_t deserialize_robot_4wd_sensors_part(uint8_t* src, uint16_t src_size, Robot_4WD* dst);

#endif //#ifndef _ROBOT_4WD_H_
