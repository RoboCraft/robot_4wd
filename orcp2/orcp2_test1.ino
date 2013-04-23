/*
 * ORCP2 test
 *
 * v.0.0.1
 *
 * http://robocraft.ru
 */

#define USE_TELEMETRY
 
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

unsigned char message_in[32];
unsigned char message_out[128];

void make_message() {
  //digitalWrite(red_led, HIGH);
  int i=0;
  int val = 0;
  switch(topic_) {
  case 0xAC04:
    // digitalWrite
    digitalWrite(message_in[0], message_in[1]);
    break;
  default:
    break;
  }
}

unsigned char calculate_checksum(unsigned char* message, int size) {
  unsigned char chk = 0;
  for(int i=0; i<size; i++) {
    chk ^= message[i];
  }
  return chk;
}

int send_message(int id, unsigned char* src, int src_size) {
	
	int l=0;
	
	/* setup the header */
	message_out[0] = 0x0D;
	message_out[1] = 0x0A;
	message_out[2] = (unsigned char) (id&255);
	message_out[3] = (unsigned char) (id>>8);
	message_out[4] = (unsigned char) (src_size&255);
	message_out[5] = (unsigned char) (src_size>>8);
	l=6;
	
        if(message_out+l != src) {
          memcpy(message_out+l, src, src_size);
        }
        l += src_size;
	//for(int i=0; i<src_size; i++, l++)
	//	message_out[l] = src[i];

	/* calculate checksum */
	message_out[l++] = calculate_checksum(message_out+2, l-2);

	int res = Serial.write(message_out, l);
	
	return res;
}

unsigned char test_string[]="Test\0";
char buf[64]={"test_str_data:::::::: millis: "};
int buf_len = 30;

int send_telemetry() {
  // test string
  //send_message(0xAC06, test_string, 5);
  String str = String(millis(), DEC);
  str.toCharArray(buf+buf_len, 64-buf_len);
  send_message(0xAC06, (unsigned char*)buf, buf_len+str.length());
}

void setup() {                
  pinMode(led, OUTPUT);
  
  digitalWrite(led, LOW);

  Serial.begin(57600);
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
		send_telemetry();
		nextTELEMETRY += TELEMETRY_INTERVAL;
	}  
#endif  // #if USE_TELEMETRY


}



