/*
 * ORCP2 test
 *
 * v.0.0.3
 *
 * http://robocraft.ru
 */

#include "orcp2.h"
#include "robot_4wd.h"
#include "robot_sensors.h"
#include "imu.h"
 
#define USE_TELEMETRY

#define BAUDRATE 57600
 
// telemetry rate (Hz)
#define TELEMETRY_RATE 		50

// Convert the rate into an interval
const int TELEMETRY_INTERVAL = 1000 / TELEMETRY_RATE;
  
// Track the next time we send telemetry
unsigned long nextTELEMETRY = TELEMETRY_INTERVAL;

int led = 13;

#define MODE_FIRST_0D       0
#define MODE_SECOND_0A      1
#define MODE_TOPIC_L        2   // waiting for topic id
#define MODE_TOPIC_H        3
#define MODE_SIZE_L         4   // waiting for message size
#define MODE_SIZE_H         5
#define MODE_MESSAGE        6
#define MODE_CHECKSUM       7

int mode_ = 0;
int bytes_ = 0;
int topic_ = 0;
int index_ = 0;
int checksum_ = 0;

unsigned char message_in[128];
unsigned char message_out[256];

IMU3_data raw_imu_data;

void make_message() {
  int i=0;
  int val = 0;
  switch(topic_) {
  case ORCP2_DIGITAL_WRITE:
    // digitalWrite
    digitalWrite(message_in[0], message_in[1]);
    break;
  default:
    break;
  }
}

int send_message(int id, unsigned char* src, int src_size) {
	int l = orcp2::to_buffer(id, src, src_size, message_out, sizeof(message_out));
	int res = Serial.write(message_out, l);	
	return res;
}

char buf[128]={"abcdefghijklmnopqru test_str_data:::::::: millis: "};
int buf_len = 50;

int send_test_string() {
  String str = String(millis(), DEC);
  str.toCharArray(buf+buf_len, 128-buf_len);
  send_message(ORCP2_SEND_STRING, (unsigned char*)buf, buf_len+str.length());
}

int send_telemetry() {
  
  uint16_t len=0;
  // raw IMU data
  len = serialize_imu3_data( &raw_imu_data, 
			message_out+ORCP2_PACKET_HEADER_LENGTH, 
			sizeof(message_out)-ORCP2_PACKET_HEADER_LENGTH );
  send_message(ORCP2_MESSAGE_IMU_RAW_DATA, message_out+ORCP2_PACKET_HEADER_LENGTH, len);
}

void setup() {                
  pinMode(led, OUTPUT);
  
  digitalWrite(led, LOW);

  Serial.begin(BAUDRATE);
  
  raw_imu_data.Accelerometer[0] = 100;
  raw_imu_data.Accelerometer[1] = 200;
  raw_imu_data.Accelerometer[2] = 300;
  raw_imu_data.Magnetometer[0] = 4000;
  raw_imu_data.Magnetometer[1] = 5000;
  raw_imu_data.Magnetometer[2] = 6000;
  raw_imu_data.Gyro[0] = 7000;
  raw_imu_data.Gyro[1] = 8000;
  raw_imu_data.Gyro[2] = 9000;
}

void loop() {
  if(Serial.available()) {
    int data = Serial.read();
    if(data < 0 ) {
      return;
    }
    
    if(mode_ != MODE_CHECKSUM)
      checksum_ ^= data;
    if( mode_ == MODE_MESSAGE ) {        /* message data being recieved */
      message_in[index_++] = data;
      bytes_--;
      if(bytes_ == 0) {                   /* is message complete? if so, checksum */
        mode_ = MODE_CHECKSUM;
      }
    }
    else if( mode_ == MODE_FIRST_0D ) {
      if(data == 0x0D) {
        mode_++;
      }
    }
    else if( mode_ == MODE_SECOND_0A ) {
      if(data == 0x0A) {
        checksum_ = 0;
        mode_++;
      }
      else {
        mode_ = MODE_FIRST_0D;
      }
    }
    else if( mode_ == MODE_TOPIC_L ) {  /* bottom half of topic id */
      topic_ = data;
      mode_++;
    }
    else if( mode_ == MODE_TOPIC_H ) {  /* top half of topic id */
      topic_ += data<<8;
      mode_++;
    }
    else if( mode_ == MODE_SIZE_L ) {   /* bottom half of message size */
      bytes_ = data;
      index_ = 0;
      mode_++;
    }
    else if( mode_ == MODE_SIZE_H ) {   /* top half of message size */
      bytes_ += data<<8;
      mode_ = MODE_MESSAGE;
      if(bytes_ == 0)
        mode_ = MODE_CHECKSUM;
    }
    else if( mode_ == MODE_CHECKSUM ) { /* do checksum */
      if(checksum_ == data) { // check checksum
        make_message();
      }
      mode_ = MODE_FIRST_0D;
    }
  } // if(Serial.available())
  
#if defined(USE_TELEMETRY)
	if (millis() > nextTELEMETRY) {
                send_test_string();
		send_telemetry();
		nextTELEMETRY += TELEMETRY_INTERVAL;
	}  
#endif  // #if USE_TELEMETRY


}



