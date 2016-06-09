// 
// ORCP2 - OR Communication Protocol ver. 2
// for working with modules over serial link
//
//
// robocraft.ru
//

#include "orcp2.h"
#include <string.h>

uint8_t orcp2::calc_checksum (uint8_t *src, size_t src_size)
{
	if(!src || src_size == 0)
		return 0;

	uint8_t chsum = 0;
	for(uint16_t i = 0; i < src_size; i++) {
		chsum ^= src[i];
	}

	return chsum;
}

uint16_t orcp2::serialize_int16(uint16_t src)
{
	uint16_t res = ((src & 0xFF) << 8) | ((src >> 8) & 0xFF);
	return res;
}

uint16_t orcp2::deserialize_int16(uint16_t src)
{
	return serialize_int16(src);
}

uint32_t orcp2::serialize_int32(uint32_t src)
{
	uint32_t res = ( (src >> 24) | (((src>>16) & 0xFF) << 8) | (((src>>8) & 0xFF) << 16) | ((src & 0xFF)<<24) );
	return res;
}

uint32_t orcp2::deserialize_int32(uint32_t src)
{
	return serialize_int32(src);
}

void orcp2::copy_int16(uint8_t *dst, uint16_t src)
{
	if(!dst)
		return;

	dst[0] = (uint8_t)(src & 0xFF);
	dst[1] = (uint8_t)(src >> 8);
}

void orcp2::copy_int32(uint8_t *dst, uint32_t src)
{
	if(!dst)
		return;

	dst[0] = (uint8_t)( src & 0xFF );
	dst[1] = (uint8_t)( (src >> 8) & 0xFF );
	dst[2] = (uint8_t)( (src >> 16) & 0xFF );
	dst[3] = (uint8_t)( (src >> 24) & 0xFF );
}

uint16_t orcp2::to_buffer(uint16_t message_type, uint8_t* src, uint16_t src_size, uint8_t* dst, uint16_t dst_size)
{
	if(!src || src_size == 0 || !dst || dst_size < src_size + ORCP2_MIN_PACKET_LENGTH)
		return 0;

	uint16_t msg_size=0;

	// make header
	dst[msg_size++] = 0x0D;
	dst[msg_size++] = 0x0A;

	dst[msg_size++] = (uint8_t)(message_type & 0xFF);
	dst[msg_size++] = (uint8_t)(message_type >> 8);
	dst[msg_size++] = (uint8_t)(src_size & 0xFF);
	dst[msg_size++] = (uint8_t)(src_size >> 8);

	// copy data
	if(dst+msg_size != src) {
		memcpy(dst+msg_size, src, src_size);
    }
    msg_size += src_size;

	//for(uint16_t i=0; i<src_size; i++) {
	//	dst[msg_size++] = src[i];
	//}

	// set checksum
	dst[msg_size++] = orcp2::calc_checksum(dst+2, msg_size-2);

	return msg_size;
}

#ifndef ARDUINO

uint16_t orcp2::get_packet(uint8_t* src, size_t& src_size, packet* pkt)
{
	if(!src || src_size < ORCP2_MIN_PACKET_LENGTH || !pkt)
		return 0;

	bool packet_finded = false;
	size_t i, j, k, pkt_begin=0;
	uint16_t pkt_size = 0;

	for(i=0; i<src_size-1; i++) {
		// light packet
		if(src[i] == 0x0D && src[i+1] == 0x0A) {
			pkt_begin = i;
			// read header
			if(pkt_begin + ORCP2_MIN_PACKET_LENGTH < src_size ) {
				pkt->message_type = (uint16_t)src[i+2];
				pkt->message_type += ((uint16_t)src[i+3]) << 8;
				pkt->message_size = (uint16_t)src[i+4];
				pkt->message_size += ((uint16_t)src[i+5]) << 8;
				
				pkt->message.resize( pkt->message_size );

				// read data
				if(pkt->message.data && 
					(pkt_begin + ORCP2_MIN_PACKET_LENGTH + pkt->message_size) < src_size) {
					for(j=0, k=pkt_begin + ORCP2_PACKET_HEADER_LENGTH; j<pkt->message_size; j++, k++) {
						pkt->message.data[j] = src[k];
					}
					pkt->checksum = src[k];
					pkt->message.size = pkt->message_size;
					pkt_size = pkt->message_size + ORCP2_MIN_PACKET_LENGTH;
					packet_finded = true;
					break;
				}
				else
					break;
			}
			else
				break;
		}
	}

	if(!packet_finded) {
		pkt_size = 0;
	}
	else {
		// move data in source buffer
		for(i=pkt_begin+pkt_size, j=0; i<src_size; i++, j++) { 
			src[j] = src[i];
		}
		src_size = j;
	}

	return pkt_size;
}

uint16_t orcp2::to_buffer(packet* pkt, uint8_t* dst, uint16_t dst_size)
{
	if(!pkt || !dst || dst_size < pkt->message_size + ORCP2_MIN_PACKET_LENGTH)
		return 0;

	uint16_t msg_size = orcp2::to_buffer(pkt->message_type, pkt->message.data, pkt->message_size, dst, dst_size);

	return msg_size;
}

uint16_t orcp2::to_buffer(packet* pkt, TBuff<uint8_t>* buff)
{
	if(!pkt || !buff || pkt->message_size == 0)
		return 0;

	uint16_t msg_size=0;

	buff->resize( pkt->message_size + ORCP2_MIN_PACKET_LENGTH );

	if(!buff->data)
		return 0;

	msg_size = orcp2::to_buffer(pkt, buff->data, buff->real_size);
	buff->size = msg_size;

	return msg_size;
}

orcp2::ORCP2::ORCP2(Stream &s):
serial(s)
{
}

orcp2::ORCP2::~ORCP2()
{
}

void orcp2::ORCP2::pinMode(int pin, int mode)
{
	uint8_t msg[2];
	msg[0] = (uint8_t)pin;
	msg[1] = (uint8_t)mode;

	uint8_t buf[ ORCP2_MIN_PACKET_LENGTH + sizeof(msg) ];

	uint16_t msg_size = orcp2::to_buffer(ORCP2_PIN_MODE, msg, sizeof(msg), buf, sizeof(buf));

	int res = serial.write(buf, sizeof(buf));

}

int orcp2::ORCP2::digitalRead(int pin)
{
	int res = 0;

	uint8_t msg = (uint8_t)pin;

	uint8_t buf[ ORCP2_MIN_PACKET_LENGTH + sizeof(msg) ];

	uint16_t msg_size = orcp2::to_buffer(ORCP2_DIGITAL_READ, &msg, sizeof(msg), buf, sizeof(buf));

	int send_res = serial.write(buf, sizeof(buf));
	if(send_res) {
		
	}

	return res;
}

int orcp2::ORCP2::analogRead(int pin)
{
	int res = 0;
	return res;
}

void orcp2::ORCP2::digitalWrite(int pin, int value)
{
	uint8_t msg[2];
	msg[0] = (uint8_t)pin;
	msg[1] = (uint8_t)value;

	uint8_t buf[ ORCP2_MIN_PACKET_LENGTH + sizeof(msg) ];

	uint16_t msg_size = orcp2::to_buffer(ORCP2_DIGITAL_WRITE, msg, sizeof(msg), buf, sizeof(buf));

	int res = serial.write(buf, sizeof(buf));
}

int orcp2::ORCP2::recv(uint8_t* dst, uint16_t dst_size)
{
	if(!dst || dst_size == 0)
		return 0;

	int res = 0;
	if( (res = serial.waitInput(500))>0 ) {
		if( (res = serial.available())>0 ) {
			if( res = serial.read(dst, dst_size) ) {
				return res;
			}
		}
	}

	return 0;
}

void orcp2::ORCP2::motorWrite(int id, uint16_t value)
{
	uint8_t msg[3];
	msg[0] = (uint8_t)id;
	orcp2::copy_int16(msg+1, value);

	uint8_t buf[ ORCP2_MIN_PACKET_LENGTH + sizeof(msg) ];

	uint16_t msg_size = orcp2::to_buffer(ORCP2_MOTOR_WRITE, msg, sizeof(msg), buf, sizeof(buf));

	int res = serial.write(buf, sizeof(buf));
}

void orcp2::ORCP2::motorsWrite(uint16_t value)
{
	uint8_t msg[2];
	orcp2::copy_int16(msg, value);

	uint8_t buf[ ORCP2_MIN_PACKET_LENGTH + sizeof(msg) ];

	uint16_t msg_size = orcp2::to_buffer(ORCP2_MOTORS_WRITE, msg, sizeof(msg), buf, sizeof(buf));

	int res = serial.write(buf, sizeof(buf));
}

void orcp2::ORCP2::drive_4wd(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3)
{
	uint8_t msg[4*sizeof(uint16_t)];
	int l=0;
	
	orcp2::copy_int16(msg+l, pwm0);
	l += sizeof(uint16_t);
	orcp2::copy_int16(msg+l, pwm1);
	l += sizeof(uint16_t);
	orcp2::copy_int16(msg+l, pwm2);
	l += sizeof(uint16_t);
	orcp2::copy_int16(msg+l, pwm3);
	l += sizeof(uint16_t);

	uint8_t buf[ ORCP2_MIN_PACKET_LENGTH + sizeof(msg) ];

	uint16_t msg_size = orcp2::to_buffer(ORCP2_MESSAGE_ROBOT_4WD_DRIVE, msg, sizeof(msg), buf, sizeof(buf));

	int res = serial.write(buf, sizeof(buf));
}

#endif //#ifndef ARDUINO
