//
// test ORCP2
//
//
// robocraft.ru
//

#include "gtest/gtest.h"

#include "orcp2.h"

TEST(ORCP2, checksum) 
{
	uint8_t buff[]={ 0x41, 0x42, 0x43, 0x00 };
	uint32_t buff_size = sizeof(buff);

	uint8_t crc = orcp2::calc_checksum(buff, buff_size);
	EXPECT_EQ(0x40, crc);
}

TEST(ORCP2, serialization) 
{
	uint16_t s16 = 0xAABB;
	s16 = orcp2::serialize_int16(s16);
	EXPECT_EQ(0xBBAA, s16);
	s16 = orcp2::deserialize_int16(s16);
	EXPECT_EQ(0xAABB, s16);

	uint32_t s32 = 0xAABBCCDD;
	s32 = orcp2::serialize_int32(s32);
	EXPECT_EQ(0xDDCCBBAA, s32);
	s32 = orcp2::deserialize_int32(s32);
	EXPECT_EQ(0xAABBCCDD, s32);
}

TEST(ORCP2, copy) 
{
	uint16_t s16 = 0xAABB;
	uint8_t buf[256]={0};
	orcp2::copy_int16(buf, s16);
	EXPECT_EQ(0xBB, buf[0]);
	EXPECT_EQ(0xAA, buf[1]);

	uint32_t s32 = 0xCCDDEEFF;
	orcp2::copy_int32(buf, s32);
	EXPECT_EQ(0xFF, buf[0]);
	EXPECT_EQ(0xEE, buf[1]);
	EXPECT_EQ(0xDD, buf[2]);
	EXPECT_EQ(0xCC, buf[3]);
}

TEST(ORCP2, base)
{
	orcp2::packet pkt;
	//                ...  |         | MT        | lenght    | data                  | CRC | ...
	uint8_t buff[]={ 0x00, 0x0D, 0x0A, 0x01, 0x00, 0x04, 0x00, 0x41, 0x42, 0x43, 0x00, 0xFF, //0x11, 0x12,
					 0x00, 0x0D, 0x0A, 0x01, 0x02, 0x04, 0x00, 0x41, 0x42, 0x43, 0x00, 0x40, 0x13, 0x14
	};
	uint32_t buff_size = sizeof(buff);

	uint16_t res = 0;

	// get packet from data

	int i=0;
	while ( (res = orcp2::get_packet(buff, buff_size, &pkt)) > 0) {
//		printf("[i] res: %02d message: %04d size: %04d CRC: %02X buff_size: %02d\n", 
//			res, pkt.message_type, pkt.message_size, pkt.checksum, buff_size);

		uint8_t data_crc = orcp2::calc_checksum(pkt.message.data, pkt.message_size);
		uint8_t crc = pkt.calc_checksum(); 

#if 0
		if(crc == pkt.checksum) {
			printf("[i] Good message CRC: %02X\n", pkt.checksum);
		}
		else {
			printf("[!] Bad message CRC: %02X vs: %02X\n", crc, pkt.checksum);
		}
#endif

		EXPECT_EQ(0x40, data_crc);

		switch(i) {
			case 0:
				EXPECT_EQ(11, res);
				EXPECT_EQ(0x01, pkt.message_type);
				EXPECT_EQ(0x04, pkt.message_size);
				EXPECT_EQ(0x41, pkt.message.data[0]);
				EXPECT_EQ(0xFF, pkt.checksum);
				EXPECT_EQ(14, buff_size);
				break;
			case 1:
				EXPECT_EQ(11, res);
				EXPECT_EQ(0x201, pkt.message_type);
				EXPECT_EQ(0x04, pkt.message_size);
				EXPECT_EQ(0x41, pkt.message.data[0]);
				EXPECT_EQ(0x40, pkt.checksum);
				EXPECT_EQ(0x47, crc);
				EXPECT_EQ(2, buff_size);
				break;
		}

		i++;
	}

	// serialize packet to message

	TBuff<uint8_t> buf;
	res = orcp2::to_buffer(&pkt, &buf);

#if 0
	printf("[i] res: %02d buf size: %02d\n", res, buf.size);
	for(size_t i=0; i<buf.size; i++) {
		printf("%02X ", buf.data[i]);
	}
	printf("\n");
#endif

	EXPECT_EQ(11, res);
	EXPECT_EQ(11, buf.size);
	EXPECT_EQ(0x0D, buf.data[0]);
	EXPECT_EQ(0x0A, buf.data[1]);
	EXPECT_EQ(0x01, buf.data[2]);
	EXPECT_EQ(0x02, buf.data[3]);
	EXPECT_EQ(0x04, buf.data[4]);
	EXPECT_EQ(0x00, buf.data[5]);
	EXPECT_EQ(0x41, buf.data[6]);
	EXPECT_EQ(0x42, buf.data[7]);
	EXPECT_EQ(0x43, buf.data[8]);
	EXPECT_EQ(0x00, buf.data[9]);
	EXPECT_EQ(0x47, buf.data[10]); // chksum // 0x40
}
