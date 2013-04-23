//
// test ORCP2
//
//
// robocraft.ru
//

#include "gtest/gtest.h"

#include "robot_4wd.h"

TEST(robot_4wd, serialization) 
{
	Robot_4WD r;
	memset(&r, 0, sizeof(r));

	int sz = sizeof(r);

	r.Bamper = BAMPER_1;

	int i;
	for(i=0; i<ENCODERS_COUNT; i++) {
		r.Encoder[i] = 0xAABBCCDD;
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		r.PWM[i] = (int16_t)0xEEFF;
	}
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		r.US[i] = 0xAABBCCDD;
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		r.IR[i] = 0xAABBCCDD;
	}
	r.Voltage = 13;

	//
	// serialization
	//

	uint8_t buf[256]={0};
	uint16_t buf_size = sizeof(buf);
	uint16_t res = serialize_robot_4wd(&r, buf, buf_size);

	// selected check
	EXPECT_GT(res, 0);

	EXPECT_EQ(BAMPER_1, buf[0]);

	EXPECT_EQ(0xDD, buf[5]);
	EXPECT_EQ(0xCC, buf[6]);
	EXPECT_EQ(0xBB, buf[7]);
	EXPECT_EQ(0xAA, buf[8]);

	EXPECT_EQ(0xDD, buf[5]);
	EXPECT_EQ(0xCC, buf[6]);
	EXPECT_EQ(0xBB, buf[7]);
	EXPECT_EQ(0xAA, buf[8]);

	EXPECT_EQ(13, buf[res-4]);
	EXPECT_EQ(0, buf[res-3]);
	EXPECT_EQ(0, buf[res-2]);
	EXPECT_EQ(0, buf[res-1]);

	// full check
	int l=0;
	EXPECT_EQ(BAMPER_1, buf[l++]);
	for(i=0; i<ENCODERS_COUNT; i++) {
		EXPECT_EQ(0xDD, buf[l++]);
		EXPECT_EQ(0xCC, buf[l++]);
		EXPECT_EQ(0xBB, buf[l++]);
		EXPECT_EQ(0xAA, buf[l++]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		EXPECT_EQ(0xFF, buf[l++]);
		EXPECT_EQ(0xEE, buf[l++]);
	}
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		EXPECT_EQ(0xDD, buf[l++]);
		EXPECT_EQ(0xCC, buf[l++]);
		EXPECT_EQ(0xBB, buf[l++]);
		EXPECT_EQ(0xAA, buf[l++]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		EXPECT_EQ(0xDD, buf[l++]);
		EXPECT_EQ(0xCC, buf[l++]);
		EXPECT_EQ(0xBB, buf[l++]);
		EXPECT_EQ(0xAA, buf[l++]);
	}
	EXPECT_EQ(0x0D, buf[l++]);
	EXPECT_EQ(0x00, buf[l++]);
	EXPECT_EQ(0x00, buf[l++]);
	EXPECT_EQ(0x00, buf[l++]);
	
	EXPECT_EQ(res, l);

	//
	// deserialization
	//
	Robot_4WD r2;
	memset(&r2, 0, sizeof(r2));
	res = deserialize_robot_4wd(buf, res, &r2);
	
	EXPECT_EQ(BAMPER_1, r2.Bamper);

	for(i=0; i<ENCODERS_COUNT; i++) {
		EXPECT_EQ(0xAABBCCDD, r2.Encoder[i]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		EXPECT_EQ((int16_t)0xEEFF, r2.PWM[i]);
	}
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		EXPECT_EQ(0xAABBCCDD, r2.US[i]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		EXPECT_EQ(0xAABBCCDD, r2.IR[i]);
	}
	EXPECT_EQ(13, r2.Voltage);

	EXPECT_EQ(res, l);
}

TEST(robot_4wd_drive_part, serialization) 
{
	Robot_4WD r;
	memset(&r, 0, sizeof(r));

	int sz = sizeof(r);

	r.Bamper = BAMPER_1;

	int i;
	for(i=0; i<ENCODERS_COUNT; i++) {
		r.Encoder[i] = 0xAABBCCDD;
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		r.PWM[i] = (int16_t)0xEEFF;
	}

	//
	// serialization
	//

	uint8_t buf[256]={0};
	uint16_t buf_size = sizeof(buf);
	uint16_t res = serialize_robot_4wd_drive_part(&r, buf, buf_size);

	// selected check
	EXPECT_GT(res, 0);

	EXPECT_EQ(BAMPER_1, buf[0]);

	// full check
	int l=0;
	EXPECT_EQ(BAMPER_1, buf[l++]);
	for(i=0; i<ENCODERS_COUNT; i++) {
		EXPECT_EQ(0xDD, buf[l++]);
		EXPECT_EQ(0xCC, buf[l++]);
		EXPECT_EQ(0xBB, buf[l++]);
		EXPECT_EQ(0xAA, buf[l++]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		EXPECT_EQ(0xFF, buf[l++]);
		EXPECT_EQ(0xEE, buf[l++]);
	}
	
	EXPECT_EQ(res, l);

	//
	// deserialization
	//
	Robot_4WD r2;
	memset(&r2, 0, sizeof(r2));
	res = deserialize_robot_4wd_drive_part(buf, res, &r2);
	
	EXPECT_EQ(BAMPER_1, r2.Bamper);

	for(i=0; i<ENCODERS_COUNT; i++) {
		EXPECT_EQ(0xAABBCCDD, r2.Encoder[i]);
	}
	for(i=0; i<MOTORS_COUNT; i++) {
		EXPECT_EQ((int16_t)0xEEFF, r2.PWM[i]);
	}

	EXPECT_EQ(res, l);
}

TEST(robot_4wd_sensors_part, serialization) 
{
	Robot_4WD r;
	memset(&r, 0, sizeof(r));

	int sz = sizeof(r);

	int i;
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		r.US[i] = 0xAABBCCDD;
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		r.IR[i] = 0xAABBCCDD;
	}
	r.Voltage = 13;

	//
	// serialization
	//

	uint8_t buf[256]={0};
	uint16_t buf_size = sizeof(buf);
	uint16_t res = serialize_robot_4wd_sensors_part(&r, buf, buf_size);

	// selected check
	EXPECT_GT(res, 0);

	// full check
	int l=0;
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		EXPECT_EQ(0xDD, buf[l++]);
		EXPECT_EQ(0xCC, buf[l++]);
		EXPECT_EQ(0xBB, buf[l++]);
		EXPECT_EQ(0xAA, buf[l++]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		EXPECT_EQ(0xDD, buf[l++]);
		EXPECT_EQ(0xCC, buf[l++]);
		EXPECT_EQ(0xBB, buf[l++]);
		EXPECT_EQ(0xAA, buf[l++]);
	}
	EXPECT_EQ(0x0D, buf[l++]);
	EXPECT_EQ(0x00, buf[l++]);
	EXPECT_EQ(0x00, buf[l++]);
	EXPECT_EQ(0x00, buf[l++]);
	
	EXPECT_EQ(res, l);

	//
	// deserialization
	//
	Robot_4WD r2;
	memset(&r2, 0, sizeof(r2));
	res = deserialize_robot_4wd_sensors_part(buf, res, &r2);
	
	for(i=0; i<ULTRASONIC_COUNT; i++) {
		EXPECT_EQ(0xAABBCCDD, r2.US[i]);
	}
	for(i=0; i<INFRARED_COUNT; i++) {
		EXPECT_EQ(0xAABBCCDD, r2.IR[i]);
	}
	EXPECT_EQ(13, r2.Voltage);

	EXPECT_EQ(res, l);
}
