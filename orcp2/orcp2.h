// 
// ORCP2 - OR Communication Protocol ver. 2
// for working with modules over serial link
//
//
// robocraft.ru
//

#ifndef _ORCP2_H_
#define _ORCP2_H_

#include "types.h"

#ifndef ARDUINO
#include "buffer.h"
#include "stream.h"
#endif //#ifndef ARDUINO

// default output buffer size
#define ORCP2_DEFAULT_BUFFER_SIZE 256
// default input buffer size
#define ORCP2_DEFAULT_INPUT_BUFFER_SIZE 512

// packet header and CRC length
#define ORCP2_PACKET_HEADER_LENGTH 6
#define ORCP2_PACKET_CRC_LENGTH 1

// smallest packet length (header + CRC)
#define ORCP2_MIN_PACKET_LENGTH 7

// messages
#define ORCP2_PIN_MODE			0xAC01
#define ORCP2_DIGITAL_READ		0xAC02
#define ORCP2_ANALOG_READ		0xAC03
#define ORCP2_DIGITAL_WRITE		0xAC04
#define ORCP2_ANALOG_WRITE		0xAC05

#define ORCP2_SEND_STRING		0xAC06

#define ORCP2_RESET_ENCODER		0xAC22
#define ORCP2_RESET_ENCODERS 	0xAC23
#define ORCP2_MOTOR_WRITE 		0xAC42
#define ORCP2_MOTORS_WRITE 		0xAC43

// robots messages
#define ORCP2_MESSAGE_IMU_ANGLES					0xB030
#define ORCP2_MESSAGE_IMU_RAW_DATA					0xB031
#define ORCP2_MESSAGE_ROBOT_4WD_DRIVE_TELEMETRY		0xB410
#define ORCP2_MESSAGE_ROBOT_4WD_SENSORS_TELEMETRY	0xB420
#define ORCP2_MESSAGE_ROBOT_4WD_TELEMETRY			0xB430

// pin modes
#define ORCP2_MODE_INPUT    0x00
#define ORCP2_MODE_OUTPUT   0x01
#define ORCP2_MODE_ANALOG   0x02
#define ORCP2_MODE_PWM      0x03
#define ORCP2_MODE_SERVO    0x04

// to write a high value
#define ORCP2_LOW	0
// to write a low value
#define ORCP2_HIGH	1

namespace orcp2 {

#ifndef ARDUINO
	// for store ORCP2 packet
	typedef struct packet {

		uint16_t receiver_id;
		uint16_t transmitter_id;

		uint16_t message_type;
		uint16_t message_size;
		
		TBuff<uint8_t> message;

		uint8_t checksum;

		packet() {
			receiver_id=transmitter_id=0;
			message_type=message_size=checksum=0;
		}

		void reset() {
			receiver_id=transmitter_id=0;
			message_type=message_size=checksum=0;
			message.reset();
		}

		uint8_t calc_checksum() {
			if(!message.data)
				return 0;
			uint8_t chsum=0;
			chsum ^= (uint8_t)(message_type & 0xFF);
			chsum ^= (uint8_t)(message_type >> 8);
			chsum ^= (uint8_t)(message_size & 0xFF);
			chsum ^= (uint8_t)(message_size >> 8);
			for(uint16_t i = 0; i < message_size; i++) {
				chsum ^= message.data[i];
			}
			return chsum;
		}

	} packet;
#endif //#ifndef ARDUINO

	// calc checksum 
	uint8_t calc_checksum (uint8_t *src, size_t src_size);

	// values serialization/deserialization
	uint16_t serialize_int16(uint16_t src);
	uint16_t deserialize_int16(uint16_t src);
	uint32_t serialize_int32(uint32_t src);
	uint32_t deserialize_int32(uint32_t src);
	void copy_int16(uint8_t *dst, uint16_t src);
	void copy_int32(uint8_t *dst, uint32_t src);

	// create message from data
	uint16_t to_buffer(uint16_t message_type, uint8_t* src, uint16_t src_size, uint8_t* dst, uint16_t dst_size);

#ifndef ARDUINO
	// get packet from data
	uint16_t get_packet(uint8_t* src, uint32_t& src_size, packet* pkt);

	// serialize packet to message
	uint16_t to_buffer(packet* pkt, uint8_t* dst, uint16_t dst_size);
	uint16_t to_buffer(packet* pkt, TBuff<uint8_t>* buff);

	class ORCP2
	{
	public:
		ORCP2(Stream& s);
		~ORCP2();

		void pinMode(int pin, int mode);
		int digitalRead(int pin);
		int analogRead(int pin);
		void digitalWrite(int pin, int value);

		int recv(uint8_t* dst, uint16_t dst_size);

		void motorWrite(int id, uint16_t value);
		void motorsWrite(uint16_t value);

	private:
		Stream &serial;

	};
#endif //#ifndef ARDUINO

}; // namespace orcp2 {

#endif //#ifndef _ORCP2_H_
