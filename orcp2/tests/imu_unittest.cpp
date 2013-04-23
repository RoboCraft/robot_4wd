//
// test ORCP2
//
//
// robocraft.ru
//

#include "gtest/gtest.h"

#include "imu.h"

TEST(imu, serialization) 
{
	IMU3_data r;
	memset(&r, 0, sizeof(r));

	int sz = sizeof(r);

	int i;
	for(i=0; i<3; i++) {
		r.Accelerometer[i] = (int16_t)0xAABB;
	}
	for(i=0; i<3; i++) {
		r.Magnetometer[i] = (int16_t)0xCCDD;
	}
	for(i=0; i<3; i++) {
		r.Gyro[i] = (int16_t)0xEEFF;
	}

	//
	// serialization
	//

	uint8_t buf[256]={0};
	uint16_t buf_size = sizeof(buf);
	uint16_t res = serialize_imu3_data(&r, buf, buf_size);

	// selected check
	EXPECT_GT(res, 0);

	EXPECT_EQ(0xBB, buf[0]);
	EXPECT_EQ(0xAA, buf[1]);
	EXPECT_EQ(0xBB, buf[2]);
	EXPECT_EQ(0xAA, buf[3]);

	// full check
	int l=0;
	for(i=0; i<3; i++) {
		EXPECT_EQ(0xBB, buf[l++]);
		EXPECT_EQ(0xAA, buf[l++]);
	}
	for(i=0; i<3; i++) {
		EXPECT_EQ(0xDD, buf[l++]);
		EXPECT_EQ(0xCC, buf[l++]);
	}
	for(i=0; i<3; i++) {
		EXPECT_EQ(0xFF, buf[l++]);
		EXPECT_EQ(0xEE, buf[l++]);
	}
	
	EXPECT_EQ(res, l);

	//
	// deserialization
	//
	IMU3_data r2;
	memset(&r2, 0, sizeof(r2));
	res = deserialize_imu3_data(buf, res, &r2);
	
	for(i=0; i<3; i++) {
		EXPECT_EQ((int16_t)0xAABB, r2.Accelerometer[i]);
	}
	for(i=0; i<3; i++) {
		EXPECT_EQ((int16_t)0xCCDD, r2.Magnetometer[i]);
	}
	for(i=0; i<3; i++) {
		EXPECT_EQ((int16_t)0xEEFF, r2.Gyro[i]);
	}

	EXPECT_EQ(res, l);
}
