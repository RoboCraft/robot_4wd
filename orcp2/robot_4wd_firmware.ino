/*
 * Robot 4WD Firmware (with ORCP2)
 *
 * v.0.0.2
 *
 * http://robocraft.ru
 */

// define type of sensors connected to the board

// IMU + US + IR
#define IMU_BOARD

// motors + encoders + bumper + voltage
#define DRIVE_BOARD

// debug
//#define DEBUG

#include "orcp2.h"
#include "robot_4wd.h"
#include "robot_sensors.h"
#include "imu.h"

#define BAUDRATE 57600

// telemetry rate (Hz)
#define TELEMETRY_RATE 		50

// Convert the rate into an interval
const int TELEMETRY_INTERVAL = 1000 / TELEMETRY_RATE;

// Track the next time we send telemetry
unsigned long nextTELEMETRY = TELEMETRY_INTERVAL;

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

#if defined(IMU_BOARD)
IMU3_data raw_imu_data;
#endif
Robot_4WD robot_data;

#if defined(IMU_BOARD)
#include <Wire.h>
#include <Ultrasonic.h>

Ultrasonic ultrasonic(13, 12); // Trig - 123, Echo - 12
int IRpin[4] = { 0, 1, 2, 3 };
int battVoltPin = A7;

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

int send_imu();
int send_sensors_telemetry();

#endif //#if defined(IMU_BOARD)

#if defined(DRIVE_BOARD)

#include <TimerOne.h>
int i=0;
int j=0;

int encoderPin[] = {13, 10, 9, 2}; 
int encoderState[] = {0, 0, 0, 0};
int encoderOldState[] = {0, 0, 0, 0};

int bumperPin[] = {14, 15, 16, 17}; 
int bumperState[] = {0, 0, 0, 0};
//int battVoltPin = A7;

MOTOR motors [MOTORS_COUNT] = { 
  { 12, 11 },
  { 8, 6   },
  { 7, 5   },
  { 4, 3   }
};

int FORWARD = HIGH;
int BACKWARD = LOW;

int send_drive_telemetry();
void motor_drive(int motor_id, int dir, int pwm);

void motor_drive(int motor_id, int pwm) {
  if(pwm >= 0) {
    motor_drive(motor_id, FORWARD, pwm);
  }
  else {
    motor_drive(motor_id, BACKWARD, (-1)*pwm);
  }
}

#endif //#if defined(DRIVE_BOARD)

#if defined(DEBUG)
#include <SoftwareSerial.h>

SoftwareSerial mySerial(19, 20); // RX, TX
#endif //#if defined(DEBUG)

int send_message(int id, unsigned char* src, int src_size) {
  int l = orcp2::to_buffer(id, src, src_size, message_out, sizeof(message_out));
  int res = Serial.write(message_out, l);	
  return res;
}

void make_message() {
  int i = 0;
  uint8_t val8 = 0;
  uint16_t val16 = 0;
  unsigned char* msg=0;
  uint16_t len = 0;
  
#if defined(DEBUG)
mySerial.print("[i] Message: ");
mySerial.println(topic_, HEX);
#endif //#if defined(DEBUG)  
  
  switch(topic_) {
  case ORCP2_PIN_MODE:
    pinMode(message_in[0], message_in[1]);
	break;
  case ORCP2_DIGITAL_READ:
	val8 = digitalRead ( message_in[0] );
	msg = message_out+ORCP2_PACKET_HEADER_LENGTH;
	msg[0] = message_in[0];
	msg[1] = val8;
	len = 2;
	send_message(ORCP2_DIGITAL_READ, message_out+ORCP2_PACKET_HEADER_LENGTH, len);
	break;
  case ORCP2_ANALOG_READ:
    val16 = analogRead(message_in[0]);
	msg = message_out+ORCP2_PACKET_HEADER_LENGTH;
	msg[0] = message_in[0];
	len = 1;
	orcp2::copy_int16(msg+1, val16);
	len += sizeof(val16);
	send_message(ORCP2_ANALOG_READ, message_out+ORCP2_PACKET_HEADER_LENGTH, len);
    break;
  case ORCP2_DIGITAL_WRITE:
    // digitalWrite
    digitalWrite(message_in[0], message_in[1]);
    break;
  case ORCP2_ANALOG_WRITE:
    val16 = message_in[1];
	val16 += message_in[2]<<8;
    analogWrite(message_in[0], val16);
    break;
  //------------------------------------------------
#if defined(IMU_BOARD) 
  case ORCP2_MESSAGE_IMU_RAW_DATA:
    send_imu();
  break;
  case ORCP2_MESSAGE_ROBOT_4WD_SENSORS_TELEMETRY:
    send_sensors_telemetry();
  break;
#endif //#if defined(IMU_BOARD)
#if defined(DRIVE_BOARD)  
  case ORCP2_RESET_ENCODER:
    val8 = message_in[0]; // encoder number
    if(val8 >= 0 && val8 < ENCODERS_COUNT) {
	  robot_data.Encoder[val8] = 0;
	}
    break;
  case ORCP2_RESET_ENCODERS:
    for(i = 0; i < ENCODERS_COUNT; i++) {
	  robot_data.Encoder[i] = 0;
	}
    break;
  case ORCP2_MOTOR_WRITE:
    val8 = message_in[0]; // motor number
    val16 = message_in[1]; // value
	val16 += message_in[2]<<8;
	motor_drive(val8, val16);
    break;
  case ORCP2_MOTORS_WRITE:
    val16 = message_in[0]; // value
	val16 += message_in[1]<<8;
	for(i=0; i<MOTORS_COUNT; i++) {
	  motor_drive(i, val16);
	}
  break;
  case ORCP2_MESSAGE_ROBOT_4WD_DRIVE_TELEMETRY:
    send_drive_telemetry();
  break;
#endif //if defined(DRIVE_BOARD)  
  //------------------------------------------------
  default:
    break;
  }
}

int send_string(char *src) {
  if(!src)
    return -1;
  return send_message(ORCP2_SEND_STRING, (unsigned char*)src, strlen(src));
}

#if defined(IMU_BOARD)
int send_imu() {
  // raw IMU data
  uint16_t len=0;
  len = serialize_imu3_data( &raw_imu_data, 
							message_out+ORCP2_PACKET_HEADER_LENGTH, 
							sizeof(message_out)-ORCP2_PACKET_HEADER_LENGTH );
  return send_message(ORCP2_MESSAGE_IMU_RAW_DATA, message_out+ORCP2_PACKET_HEADER_LENGTH, len);
}
#endif

int send_telemetry() {
  uint16_t len=0;
  len = serialize_robot_4wd( &robot_data, 
								message_out+ORCP2_PACKET_HEADER_LENGTH,  
								sizeof(message_out)-ORCP2_PACKET_HEADER_LENGTH );
  return send_message(ORCP2_MESSAGE_ROBOT_4WD_TELEMETRY, message_out+ORCP2_PACKET_HEADER_LENGTH, len);
}

#if defined(DRIVE_BOARD)
int send_drive_telemetry() {
  uint16_t len=0;
  len = serialize_robot_4wd_drive_part( &robot_data, 
										  message_out+ORCP2_PACKET_HEADER_LENGTH, 
										  sizeof(message_out)-ORCP2_PACKET_HEADER_LENGTH );
  return send_message(ORCP2_MESSAGE_ROBOT_4WD_DRIVE_TELEMETRY, message_out+ORCP2_PACKET_HEADER_LENGTH, len);
}
#endif

#if defined(IMU_BOARD)
int send_sensors_telemetry() {
  uint16_t len=0;
  len = serialize_robot_4wd_sensors_part( &robot_data, 
										  message_out+ORCP2_PACKET_HEADER_LENGTH, 
										  sizeof(message_out)-ORCP2_PACKET_HEADER_LENGTH );
  return send_message(ORCP2_MESSAGE_ROBOT_4WD_SENSORS_TELEMETRY, message_out+ORCP2_PACKET_HEADER_LENGTH, len);
}
#endif

#if defined(IMU_BOARD)

// Arduino backward compatibility macros
#if ARDUINO >= 100
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 
#else
#define WIRE_SEND(b) Wire.send(b)
#define WIRE_RECEIVE() Wire.receive() 
#endif

// I2C code to read the sensors

void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x08);  // Set to full resolution
  Wire.endTransmission();
  delay(5);

  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x09);  // Set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();

  if (i == 6)  // All bytes received?
  {
    // No multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    raw_imu_data.Accelerometer[0] = (((int) buff[3]) << 8) | buff[2];  // X axis (internal sensor y axis)
    raw_imu_data.Accelerometer[1] = (((int) buff[1]) << 8) | buff[0];  // Y axis (internal sensor x axis)
    raw_imu_data.Accelerometer[2] = (((int) buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
  }
  else
  {
    if (output_errors) send_string("!ERR: reading accelerometer");
  }
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();

  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();

  if (i == 6)  // All bytes received?
  {

    // HMC5883L magnetometer
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    raw_imu_data.Magnetometer[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
    raw_imu_data.Magnetometer[1] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
    raw_imu_data.Magnetometer[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)

  }
  else
  {
    if (output_errors) send_string("!ERR: reading magnetometer");
  }
}

void Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);

  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);

  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();

  if (i == 6)  // All bytes received?
  {
    raw_imu_data.Gyro[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);    // X axis (internal sensor -y axis)
    raw_imu_data.Gyro[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);    // Y axis (internal sensor -x axis)
    raw_imu_data.Gyro[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);    // Z axis (internal sensor -z axis)
  }
  else
  {
    if (output_errors) send_string("!ERR: reading gyroscope");
  }
}

void Read_US() {
  robot_data.US[0] = ultrasonic.Ranging(CM);       // get distance
  if ((robot_data.US[0] < 1) || (robot_data.US[0] > 350)) {
    robot_data.US[0] = 350;
  }
}

void Read_IR() {
  for(int j=0; j<INFRARED_COUNT; j++) {
    robot_data.IR[j] = 65*pow(analogRead(IRpin[j])*0.0048828125, -1.10);
  }
}

#endif //#if defined(IMU_BOARD)

#if defined(DRIVE_BOARD)

void motor_drive(int motor_id, int dir, int pwm)
{
  if(motor_id < 0 || motor_id >= MOTORS_COUNT)
    return;

  digitalWrite(motors[motor_id].in, dir);
  analogWrite(motors[motor_id].enable, pwm);

  robot_data.PWM[motor_id] = pwm;
  if(dir == BACKWARD) {
    robot_data.PWM[motor_id] *= (-1);
  }
}

int encoder_counter = 0;

void Read_Encoders()
{
  for(encoder_counter=0; encoder_counter<ENCODERS_COUNT; encoder_counter++) {
    encoderState[encoder_counter] = digitalRead (encoderPin[encoder_counter]);
    if(!encoderOldState[encoder_counter]) {
      if(encoderState[encoder_counter]) {
        if(robot_data.PWM[encoder_counter] > 0 ) {
          robot_data.Encoder[encoder_counter]++; 
        }
        else {
          robot_data.Encoder[encoder_counter]--; 
        }
      }
    }
    encoderOldState[encoder_counter] =  encoderState[encoder_counter];
  }
}

void Read_Voltage() {
  //$TODO

  // 4,883mV & divider 1/6
  float val = (float)analogRead(battVoltPin)*4.883*6.000;

  robot_data.Voltage = (uint32_t)(val*1000);
}

void Read_Bampers() {
  robot_data.Bamper = 0;
  for (int i=0; i<BAMPER_COUNT; i++) {
    bumperState[i] = digitalRead ( bumperPin[i]);
    if(bumperState[i]) {
      robot_data.Bamper = robot_data.Bamper | (1 << i);
    }
  }
}

#endif //#if defined(DRIVE_BOARD)

void setup() {                
  Serial.begin(BAUDRATE);

#if defined(IMU_BOARD)  
  memset(&raw_imu_data, 0 , sizeof(raw_imu_data));
#endif  
  memset(&robot_data, 0 , sizeof(robot_data));

#if defined(IMU_BOARD)    

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();

#endif //#if defined(IMU_BOARD) 

#if defined(DRIVE_BOARD)

  int i;
  for(i=0; i<MOTORS_COUNT; i++) {
    pinMode(motors[i].in, OUTPUT);
    pinMode(motors[i].in, OUTPUT);
    pinMode(motors[i].in, OUTPUT);
    pinMode(motors[i].in, OUTPUT);
  }

  for(i=0; i<MOTORS_COUNT; i++) {
    motor_drive(i, FORWARD, 0);
  } 

  for(i=0; i<BAMPER_COUNT; i++) {
    pinMode(bumperPin[i], INPUT); 
  }

  for(i=0; i<ENCODERS_COUNT; i++) {
    pinMode(encoderPin[i], INPUT); 
    encoderOldState[i] = digitalRead(encoderPin[i]);
  }

  Timer1.initialize(300); 
  Timer1.attachInterrupt(Read_Encoders); 
  
#endif //#if defined(DRIVE_BOARD)

#if defined(DEBUG)
mySerial.begin(9600);
mySerial.println("[i] Start");
#endif //#if defined(DEBUG)
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

  if (millis() > nextTELEMETRY) {

#if defined(IMU_BOARD)
    Read_Gyro(); // Read gyroscope
    Read_Accel(); // Read accelerometer
    Read_Magn(); // Read magnetometer
    Read_US();
    Read_IR();
	Read_Voltage();

    send_imu();
    send_sensors_telemetry();
#endif //#if defined(IMU_BOARD)
	
#if defined(DRIVE_BOARD)
    //Read_Voltage();
    Read_Bampers();

    send_drive_telemetry();
#endif	//#if defined(DRIVE_BOARD)

    nextTELEMETRY += TELEMETRY_INTERVAL;
  }  

}
