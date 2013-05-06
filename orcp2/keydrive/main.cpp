//
// drive 4WD robot via ORCP2 protocol
//

#include <stdio.h>
#include <stdlib.h>

#include "orcp2.h"
#include "serial.h"
#include "times.h"

#include "imu.h"
#include "robot_4wd.h"
#include "console.h"

int main(int argc, char* argv[])
{
	printf("[i] Start...\n");

#if 1

#if defined(WIN32)
	char _port[]="COM9";
#elif defined(LINUX)
	char _port[]="/dev/ttyUSB0";
#endif
	int _rate = 57600;
	int _speed = 100;

	char* port = _port;
	int rate = _rate;
	int speed = _speed;

	if(argc >= 4) {
		port = argv[1];
		rate = atoi(argv[2]);
		speed = atoi(argv[3]);
	}
	else if(argc > 1) {
		port = argv[1];
	}
	else if(argc <= 1) {
		printf("Usage: \n");
		printf("program <port name> <baud rate> <speed>\n\n");
	}

	printf("[i] port: %s\n", port);
	printf("[i] rate: %d\n", rate);
	printf("[i] speed: %d\n", speed);

	Serial serial;
	if( serial.open(port, rate) ) {
		//return -1;
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

	orcp2::ORCP2 orcp(serial);

	IMU3_data raw_imu_data;
	Robot_4WD robot_data;

	int val=0;
	int counter=0;

	while( 1 ) {

		int key = console::waitKey(30);
		if(key != 0 ) printf( "[i] Key: %c (%d)\n", key ,key );
		if(key == 27) { //ESC
			break;
		}
		else if(key == 32) { // SPACE
			printf("[i] stop\n");
			orcp.motorsWrite(0);
			
		}
		else if(key == 'w' || key == 'W') {
			printf("[i] forward %d\n", speed);
			orcp.motorsWrite(speed);
		}
		else if(key == 's' || key == 'S') {
			printf("[i] backward %d\n", speed);
			orcp.motorsWrite(-speed);
		}
		else if(key == 'a' || key == 'A') {
			printf("[i] left %d\n", speed);
			
			orcp.drive_4wd(-speed, speed, -speed, speed);
			//orcp.motorWrite(1, speed);
			//orcp.motorWrite(3, speed);
			//orcp.motorWrite(0, -speed);
			//orcp.motorWrite(2, -speed);			
		}
		else if(key == 'd' || key == 'D') {
			printf("[i] right %d\n", speed);

			orcp.drive_4wd(speed, -speed, speed, -speed);
			//orcp.motorWrite(1, -speed);
			//orcp.motorWrite(3, -speed);
			//orcp.motorWrite(0, speed);
			//orcp.motorWrite(2, speed);
		}

		if( res = serial.waitInput(500) ) {
			//printf("[i] waitInput: %d\n", res);
			if( (res = serial.available()) > 0 ) {
				//printf("[i] available: %d\n", res);
				//res = res > (buff.real_size-buff.size) ? (buff.real_size-buff.size) : res ;
				if( (res = serial.read(buff.data+buff.size, buff.real_size-buff.size)) > 0 ) {
					buff.size += res;

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
