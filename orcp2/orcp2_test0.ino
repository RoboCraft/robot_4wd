/*
 * ORCP2 test
 *
 * v.0.0.0
 *
 * http://robocraft.ru
 */

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

int calculate_checksum(unsigned char* message, int size) {
  /* calculate checksum */
  int chk = 0;
  for(int i=0; i<size; i++) {
    chk ^= message[i];
  }
  return chk;
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
  } // if(Serial.available()) {
}



