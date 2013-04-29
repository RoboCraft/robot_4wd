#include <TimerOne.h>
int i=0; // счётчик в лупе
int j=0; //счётчик в прерывании
int encoderPin[] = {13, 10, 9, 2}; 
int encoderCount[] = {0, 0, 0, 0};  
int encoderState[] = {0, 0, 0, 0};  
int encoderOldState[] = {0, 0, 0, 0};  

int bumperPin[] = {14, 15, 16, 17}; 
int bumperState[] = {0, 0, 0, 0}; 
int batteryVoltage = 0; //в mV
int battVoltPin = A7;


struct MOTOR    // массив структур для хранения номеров pin-ов, к которым подключены моторчики
{
  int in;     // INVERTOR INPUT (dir)
  int enable; // ENABLE (pwm)
};


MOTOR MOTOR1 = { 12, 11 };
MOTOR MOTOR2 = { 8, 6 };
MOTOR MOTOR3 = { 7, 5 };
MOTOR MOTOR4 = { 4, 3 };

int FORWARD = HIGH;
int BACKWARD = LOW;

char val; 
int spd = 100;

void motor1(int dir, int pwm) // первый 
{
  digitalWrite(MOTOR1.in, dir);
  analogWrite(MOTOR1.enable, pwm);
}

void motor2(int dir, int pwm) // второй
{
  digitalWrite(MOTOR2.in, dir);
  analogWrite(MOTOR2.enable, pwm);
}

void motor3(int dir, int pwm) // третий
{
  digitalWrite(MOTOR3.in, dir);
  analogWrite(MOTOR3.enable, pwm);
}

void motor4(int dir, int pwm) // четвёртый
{
  digitalWrite(MOTOR4.in, dir);
  analogWrite(MOTOR4.enable, pwm);
}


void readEncoder()
{
  for (j=0; j<ENCODERS_COUNT; j++)
  {
   encoderState[j] = digitalRead (encoderPin[j]);
    if (!encoderOldState[j])
     {
      if (encoderState[j]) encoderCount[j]++; 
     }
   encoderOldState[j] =  encoderState[j] ;
 }
}

void setup() {
  
 Serial.begin(9600);
 pinMode(MOTOR1.in, OUTPUT); // настраиваем выводы
 pinMode(MOTOR2.in, OUTPUT);
 pinMode(MOTOR3.in, OUTPUT);
 pinMode(MOTOR4.in, OUTPUT);
 
  motor1(FORWARD, 0);   
  motor2(FORWARD, 0);
  motor3(FORWARD, 0);   
  motor4(FORWARD, 0);
 
  for (i=0; i<4; i++)
  {
  pinMode(bumperPin[i], INPUT); 
  }
 
 for (i=0; i<4; i++)
  {
  pinMode(encoderPin[i], INPUT); 
  encoderOldState[i] = digitalRead (encoderPin[i]);
  }
  Timer1.initialize(300); 
  Timer1.attachInterrupt( readEncoder ); 
}
 
void loop() {
 
  // !тут требуется подогнать при нормальном питании!
  batteryVoltage = (float)analogRead(battVoltPin)*4.883*6.000; // 4,883mV/деление * делитель на плате 1/6

  for (i=0; i<4 ; i++)
  {
   bumperState[i] = digitalRead ( bumperPin[i]); 
  }
 
 
  for (i=0; i<4 ; i++)
  {
   Serial.print ("enc");
   Serial.print (i+1);
   Serial.print (" ");
   Serial.println (encoderCount[i]);
  }
   Serial.println ();
   
  for (i=0; i<4 ; i++)
  { 
   Serial.print ("bump");
   Serial.print (i+1);
   Serial.print (" ");
   Serial.println (bumperState[i]);
  }
  Serial.println ();
  
   Serial.print ("Voltage ");
   Serial.print (batteryVoltage);
  Serial.println ();
  
 delay(500);
 
  if (Serial.available()) 
    {
    val = Serial.read();
    }
    

//************ Bluetooth RC Controller ******************  

   if (val == '0'){ 
     spd = 20;   
     } else if (val == '1') {   
     spd = 40;
     } else if (val == '2') {   
     spd = 60;
     } else if (val == '3') {   
     spd = 80;
     } else if (val == '4') {   
     spd = 100;
     } else if (val == '5') {   
     spd = 120;
     } else if (val == '6') {   
     spd = 140;
     } else if (val == '7') {   
     spd = 180;
     } else if (val == '8') {   
     spd = 200;
     } else if (val == '9') {   
     spd = 220;
     } else if (val == 'q') {   
     spd = 250;
     } 
   
   if (val == 'F') // Forward
    { 
       motor1(FORWARD, spd);   
       motor2(FORWARD, spd);
       motor3(FORWARD, spd);   
       motor4(FORWARD, spd);
    } 
    if (val == 'S') // Stop 
    { 
       motor1(FORWARD, 0);   
       motor2(FORWARD, 0);
       motor3(FORWARD, 0);   
       motor4(FORWARD, 0);
    } 
    if (val == 'B') // Backward
    { 
       motor1(BACKWARD, spd);   
       motor2(BACKWARD, spd);
       motor3(BACKWARD, spd);   
       motor4(BACKWARD, spd);
    } 
    if (val == 'L') // Right
    { 
       motor2(FORWARD, spd);   
       motor4(FORWARD, spd);   
       motor1(BACKWARD, spd);
       motor3(BACKWARD, spd);
       
    } 
    if (val == 'R') // Left
    { 
       motor1(FORWARD, spd); 
       motor3(FORWARD, spd); 
       motor2(BACKWARD, spd);
       motor4(BACKWARD, spd);
    }   
    if (val == 'I') // F-R
    { 
       motor1(FORWARD, spd);   
       motor3(FORWARD, spd);
       motor2(FORWARD, spd-100);
       motor4(FORWARD, spd-100);
    } 
    if (val == 'G') // F-L
    { 
       motor2(FORWARD, spd); 
       motor4(FORWARD, spd); 
       motor1(FORWARD, spd-100); 
       motor3(FORWARD, spd-100);
    }  
     if (val == 'J') // B-R
    { 
       motor1(BACKWARD, spd); 
       motor3(BACKWARD, spd); 
       motor2(BACKWARD, spd-100);
       motor4(BACKWARD, spd-100);
    } 
    if (val == 'H') // B-L
    { 
       motor2(BACKWARD, spd); 
       motor4(BACKWARD, spd); 
       motor1(BACKWARD, spd-100);  
       motor3(BACKWARD, spd-100);  
    }   
//*************************************************** 
}

