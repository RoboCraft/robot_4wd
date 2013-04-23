//
// ORCP2
//

#include <stdio.h>
#include <stdlib.h>

#include "orcp2.h"
#include "serial.h"
#include "times.h"

int main(int argc, char* argv[])
{
	printf("[i] Start...\n");

#if defined(WIN32)
	char _port[]="COM9";
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

	if(!buff.data) {
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

	int val=0;
	int counter=0;
	while( 1 ) {
		if( res = serial.waitInput(500) ) {
			if( (res = serial.available())>0 ) {
				if( res = serial.read(buff.data+buff.size, res) ) {
					buff.size += res;
					
					// print data
					printf("[i] read data(%d): \n", res);
#if 1
					for(int i=0; i<res; i++) {
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
					buff.size = 0;
#endif
					
				}
			}
		}
	}

	serial.close();
	
	printf("[i] End.\n");

	return 0;
}
