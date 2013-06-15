//
// ORCP2
//

#include <stdio.h>
#include <stdlib.h>

#include "orcp2.h"
#include "serial.h"
#include "times.h"

#include "imu.h"
#include "robot_4wd.h"

int main(int argc, char* argv[])
{
	printf("[i] Start...\n");

	//printf("[i] sizeof(Robot_4WD) = %d\n", sizeof(Robot_4WD));

#if 0

	orcp2::packet pkt;
	//                ...  |         | MT        | lenght    | data                  | CRC | ...
	uint8_t buff[]={ 0x00, 0x0D, 0x0A, 0x01, 0x00, 0x04, 0x00, 0x41, 0x42, 0x43, 0x00, 0xFF, //0x11, 0x12,
					 0x00, 0x0D, 0x0A, 0x01, 0x02, 0x04, 0x00, 0x41, 0x42, 0x43, 0x00, 0x40, 0x13, 0x14
	};
	uint32_t buff_size = sizeof(buff);

	uint16_t res = 0;

	// get packet from data

	while ( (res = orcp2::get_packet(buff, buff_size, &pkt)) > 0) {
		printf("[i] res: %02d message: %04d size: %04d CRC: %02X buff_size: %02d\n", 
			res, pkt.message_type, pkt.message_size, pkt.checksum, buff_size);

		uint8_t crc = orcp2::calc_checksum(pkt.message.data, pkt.message_size);
		if(crc == pkt.checksum) {
			printf("[i] Good message CRC: %02X\n", pkt.checksum);
		}
		else {
			printf("[!] Bad message CRC: %02X vs: %02X\n", crc, pkt.checksum);
		}
	}

	// serialize packet to message

	TBuff<uint8_t> buf;
	res = orcp2::to_buffer(&pkt, &buf);
	printf("[i] res: %02d buf size: %02d\n", res, buf.size);
	for(size_t i=0; i<buf.size; i++) {
		printf("%02X ", buf.data[i]);
	}
	printf("\n");

#endif

#if 1

#if defined(WIN32)
	char _port[]="COM11";
#elif defined(LINUX)
	char _port[]="/dev/ttyUSB0";
#endif
	int _rate = 57600;

	char* port = _port;
	int rate = _rate;

	if(argc >= 3) {
		port = argv[1];
		rate = atoi(argv[2]);
	}
	else if(argc > 1) {
		port = argv[1];
	}
	else if(argc <= 1) {
		printf("Usage: \n");
		printf("program <port name> <baud rate> \n\n");
	}

	printf("[i] port: %s\n", port);
	printf("[i] rate: %d\n", rate);

	Serial serial;
	if( serial.open(port, rate) ) {
		return -1;
	}

	int res = 0;
	TBuff<uint8_t> buff;
	buff.resize(2048);

	orcp2::packet pkt;
	pkt.message.resize(256);

	if(!buff.data || !pkt.message.data) {
		fprintf(stderr, "[!] Error: cant allocate memory!\n");
		serial.close();
		return -1;
	}

#if 1
	orcp2::ORCP2 orcp(serial);

	for(int i=0; i<7; i++) {
		int val = i%2;
		printf("%d\n", val);
		orcp.digitalWrite(13, val);
		orv::time::sleep(500);
	}
#endif

	//serial.discardInput();

	IMU3_data raw_imu_data;
	Robot_4WD robot_data;

	int val=0;
	int counter=0;
	while( 1 ) {

#if 0
		// send test data to serial
		uint8_t buf[] = { 0x00, 0x0D, 0x0A, 0x01, 0x00, 0x04, 0x00, 0x41, 0x42, 0x43, 0x00, 0xFF, //0x11, 0x12,
						  0x00, 0x0D, 0x0A, 0x01, 0x02, 0x04, 0x00, 0x41, 0x42, 0x43, 0x00, 0x40, 0x13, 0x14
		};
		//{  0x00, 0x0D, 0x0A, 0x01, 0x02, 0x04, 0x00, 0x41, 0x42, 0x43, 0x00, 0x40, 0x13, 0x14 };
		uint32_t buf_size = sizeof(buf);
		res = serial.write(buf, buf_size);
		printf("[i] write: (%d)\n", res);
#endif

		if( res = serial.waitInput(500) ) {
			//printf("[i] waitInput: %d\n", res);
			if( (res = serial.available()) > 0 ) {
				//printf("[i] available: %d\n", res);
				//res = res > (buff.real_size-buff.size) ? (buff.real_size-buff.size) : res ;
				if( (res = serial.read(buff.data+buff.size, buff.real_size-buff.size)) > 0 ) {
					buff.size += res;

#if 0
					if(buff.size > buff.real_size/3) {
						printf("[i] !!! CLEAN (%d) !!! \n", buff.size);
						buff.size = 0;
						serial.discardInput();
						continue;
					}
#endif

					// print data
					//printf("[i] read data(%d): \n", res);
#if 0
					for(int i=0; i<buff.size; i++) {
						printf("%02X ", buff.data[i]);
						if(i>0 && (i+1)%16 == 0) {
							printf("\t");
							for(int j=i-15; j<=i; j++) {
								printf("%c", buff.data[j]);
							}
							printf("\n");
						}
					}
					printf("\n");

					res = 0;
					//buff.size = 0;
#endif

					// get packet from data
					while ( (res = orcp2::get_packet(buff.data, buff.size, &pkt)) > 0) {
						printf("[i] res: %02d message: %04X size: %04d CRC: %02X buff.size: %02d\n", 
							res, pkt.message_type, pkt.message_size, pkt.checksum, buff.size);

						uint8_t crc = pkt.calc_checksum();
						if(crc == pkt.checksum) {
							printf("[i] Good message CRC: %02X\n", pkt.checksum);

							switch(pkt.message_type) {
								case ORCP2_SEND_STRING:
									pkt.message.data[pkt.message.size]=0;
									printf("[i] String: %s\n", pkt.message.data);
									//printf("[i] counter: %d\n", counter);
									if(++counter > 50) {
										//printf("[i] digitalWrite: %d\n", val);
										val = !val;
										orcp.digitalWrite(13, val);
										counter = 0;
									}
									break;
								case ORCP2_MESSAGE_IMU_RAW_DATA:
									deserialize_imu3_data(pkt.message.data, pkt.message.size, &raw_imu_data);
									printf( "[i] IMU: acc: [%d %d %d] mag: [%d %d %d] gyro: [%d %d %d]\n",
											raw_imu_data.Accelerometer[0], raw_imu_data.Accelerometer[1], raw_imu_data.Accelerometer[2], 
											raw_imu_data.Magnetometer[0], raw_imu_data.Magnetometer[1], raw_imu_data.Magnetometer[2], 
											raw_imu_data.Gyro[0], raw_imu_data.Gyro[1], raw_imu_data.Gyro[2] ); 
									break;
								case ORCP2_MESSAGE_ROBOT_4WD_DRIVE_TELEMETRY:
									deserialize_robot_4wd_drive_part(pkt.message.data, pkt.message.size, &robot_data);
									printf( "[i] Drive telemetry: bmp: %d enc: [%d %d %d %d] PWM: [%d %d %d %d]\n",
											robot_data.Bamper, 
											robot_data.Encoder[0], robot_data.Encoder[1], robot_data.Encoder[2], robot_data.Encoder[3],
											robot_data.PWM[0], robot_data.PWM[1], robot_data.PWM[2], robot_data.PWM[3] );
									break;
								case ORCP2_MESSAGE_ROBOT_4WD_SENSORS_TELEMETRY:
									deserialize_robot_4wd_sensors_part(pkt.message.data, pkt.message.size, &robot_data);
									printf( "[i] Sensors telemetry: US: %d IR: [%d %d %d %d] V: %d\n",
											robot_data.US[0], 
											robot_data.IR[0], robot_data.IR[1], robot_data.IR[2], robot_data.IR[3],
											robot_data.Voltage ); 
									break;
								case ORCP2_MESSAGE_ROBOT_4WD_TELEMETRY:
									deserialize_robot_4wd(pkt.message.data, pkt.message.size, &robot_data);
									printf( "[i] Drive Telemetry: bmp: %d enc: [%d %d %d %d] PWM: [%d %d %d %d]\n",
											robot_data.Bamper, 
											robot_data.Encoder[0], robot_data.Encoder[1], robot_data.Encoder[2], robot_data.Encoder[3],
											robot_data.PWM[0], robot_data.PWM[1], robot_data.PWM[2], robot_data.PWM[3] );
									printf( "[i] Sensors Telemetry: US: %d IR: [%d %d %d %d] V: %d\n",
											robot_data.US[0], 
											robot_data.IR[0], robot_data.IR[1], robot_data.IR[2], robot_data.IR[3],
											robot_data.Voltage ); 
									break;
								default:
									printf("[i] Unknown message type: %04X!\n", pkt.message_type);
									break;
							}

						}
						else {
							printf("[!] Bad message CRC: %02X vs: %02X\n", crc, pkt.checksum);
						}
					}
				}
				else {
					printf("[!] too much data: %d (%d)\n", res, buff.size);
					buff.size = 0;
				}
			}
			else {
				printf("[!] no available: %d\n", res);
			}
		}
		else {
			printf("[!] waitInput timeout: %d\n", res);
		}
	}

	serial.close();

#endif

	printf("[i] End.\n");

	return 0;
}
