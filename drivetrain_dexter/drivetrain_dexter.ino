
#include <Wire.h>
#include "GP2Y0E03.h"
#include "filters.h"
#include <util/atomic.h>
#include <AutoPID.h>
#include <Servo.h>
Servo tiltservo;
Servo myservo;
Servo myservo2;

/**********************************************************************
**********************************************************************/
String command = "idk";

int M1VCC = 46;
int M2VCC = 9;
int M3VCC = 8;
int M4VCC = 47;

 
int M1DIR = 4;
int M2DIR = 11;
int M3DIR = 10; 
int M4DIR = 5;

int M1PWM = 6;
int M2PWM = 13;
int M4PWM = 7;
int M3PWM = 12;

int encoderead1 = 16;
int encoderead2 = 14;
int encoderead3 = 15;
int encoderead4 = 22;

// Encoder1 and Feedback Control 
#define ENCA_1 18
//#define ENCB_1 19
long prevT_1 = 0;
int posPrev_1 = 0;
volatile int pos_i_1 = 0;
//volatile float velocity_i_1 = 0;
volatile long prevT_i_1 = 0;
float v1Filt_1 = 0;
float v1Prev_1 = 0;
double my_vt1;
double my_v1;
double my_u1;
double Kp1=0.8, Ki1=1, Kd1=0;
AutoPID myPID_1(&my_v1, &my_vt1, &my_u1, 0, 230, Kp1, Ki1, Kd1); // PID for joint position

// Encoder2 and Feedback Control 
#define ENCA_2 3
//#define ENCB_2 21
long prevT_2 = 0;
int posPrev_2 = 0;
volatile int pos_i_2 = 0;
//volatile float velocity_i_2 = 0;
volatile long prevT_i_2 = 0;
float v1Filt_2 = 0;
float v1Prev_2 = 0;
double my_vt2;
double my_v2;
double my_u2;
double Kp2=0.8, Ki2=1, Kd2=0;
AutoPID myPID_2(&my_v2, &my_vt2, &my_u2, 0, 230, Kp2, Ki2, Kd2); // PID for joint position

// Encoder3 and Feedback Control 
#define ENCA_3 2
long prevT_3 = 0;
int posPrev_3 = 0;
volatile int pos_i_3 = 0;
//volatile float velocity_i_2 = 0;
volatile long prevT_i_3 = 0;
float v1Filt_3 = 0;
float v1Prev_3 = 0;
double my_vt3;
double my_v3;
double my_u3;
double Kp3=0.8, Ki3=1, Kd3=0;
AutoPID myPID_3(&my_v3, &my_vt3, &my_u3, 0, 230, Kp3, Ki3, Kd3); // PID for joint position


// Encoder4 and Feedback Control 
#define ENCA_4 19
long prevT_4 = 0;
int posPrev_4 = 0;
volatile int pos_i_4 = 0;
//volatile float velocity_i_2 = 0;
volatile long prevT_i_4 = 0;
float v1Filt_4 = 0;
float v1Prev_4 = 0;
double my_vt4;
double my_v4;
double my_u4;
double Kp4=0.8, Ki4=1, Kd4=0;
AutoPID myPID_4(&my_v4, &my_vt4, &my_u4, 0, 230, Kp4, Ki4, Kd4); // PID for joint position
/**********************************************************************
**********************************************************************/


/**********************************************************************
**********************************************************************/
const int GP2Y0E03_I2C_ADDR = 0x40;
float filterValue = 0.0;
int smoothedValue = 4;
const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

// Low-pass filter
Filter f(cutoff_freq, sampling_time, order);

GP2Y0E03 gp2y0e03( GP2Y0E03_I2C_ADDR );

float distance;

long currentTime;
long endTime1;
long endTime2;
int count1 = 0;
int count2 = 0;

long currentTimePath;
long endTimePath;

int check = 0;
/**********************************************************************
**********************************************************************/


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Wire.begin();
Serial.flush();
pinMode(ENCA_1,INPUT);
//pinMode(ENCB_1,INPUT);
pinMode(ENCA_2,INPUT);
//pinMode(ENCB_2,INPUT);
pinMode(encoderead1,INPUT);
pinMode(encoderead2,INPUT);

pinMode(ENCA_3,INPUT);
pinMode(encoderead3,INPUT);
pinMode(ENCA_4,INPUT);
pinMode(encoderead4,INPUT);
attachInterrupt(digitalPinToInterrupt(ENCA_1),readEncoder1,RISING);
attachInterrupt(digitalPinToInterrupt(ENCA_2),readEncoder2,RISING);
attachInterrupt(digitalPinToInterrupt(ENCA_3),readEncoder3,RISING);
attachInterrupt(digitalPinToInterrupt(ENCA_4),readEncoder4,RISING);
pinMode(M1DIR,OUTPUT);
pinMode(M2DIR,OUTPUT);
pinMode(M1PWM,OUTPUT);
pinMode(M2PWM,OUTPUT);
pinMode(M1VCC,OUTPUT);
pinMode(M2VCC,OUTPUT);
pinMode(M3DIR,OUTPUT);
pinMode(M4DIR,OUTPUT);
pinMode(M3PWM,OUTPUT);
pinMode(M4PWM,OUTPUT);
pinMode(M3VCC,OUTPUT);
pinMode(M4VCC,OUTPUT);
tiltservo.attach(30);
myservo.attach(40);
myservo2.attach(41);
tiltservo.write(95);
myservo.write(90);
myservo2.write(90);
}

void loop() {
  digitalWrite(M1VCC,HIGH);
  digitalWrite(M2VCC,HIGH);
  digitalWrite(M3VCC,HIGH);
  digitalWrite(M4VCC,HIGH);
  readIRSensor();

  if (Serial.available()){
    command = Serial.readStringUntil('\n'); 
    command.trim();
    //Serial.print("the command is ");
    //Serial.println(command);
    }
  if (command.equals("forward")){
    driveforward();
    }
  else if (command.equals("backward")){
    drivebackward();
    }
  else if(command.equals("stop")){
    fullstop();
    }
   else if(command.equals("ccw")){
    spinccw();
    }
   else if(command.equals("cw")){
    spincw();
    }
   else if(command.equals("up")){
    tiltservo.write(110);
   }
   else if(command.equals("down")){
    tiltservo.write(80);
   }
   else if(command.equals("center")){
    tiltservo.write(95);
   }
   else if(command.equals("open")){
    count2 = 0;
    
    if (count1 == 0)
    {
      currentTime = millis();

      endTime1 = millis() + 2000;

      count1++;
    }
    if ((endTime1 - currentTime) > 0)
    {
      myservo.write(77);
  
      myservo2.write(79);

      currentTime = millis();
    }
    else
    {
      myservo.write(90);
  
      myservo2.write(90);
    }
   }
   else if(command.equals("close")){
    count1 = 0;
    
    if (count2 == 0)
    {
      currentTime = millis();

      endTime2 = millis() + 3500;

      count2++;
    }
    if ((endTime2 - currentTime) > 0)
    {
      myservo.write(108);
  
      myservo2.write(110);

      currentTime = millis();
    }
    else
    {
      myservo.write(90);
  
      myservo2.write(90);
    }
   }
   else if(command.equals("hold")){
    myservo.write(90);
  
    myservo2.write(90);
   }
   else if(command.equals("path")){
    Path();
   }
   
   Serial.flush();
    
  //Serial.print(my_u1);
  //Serial.print("  ");
  //Serial.print(my_u2);
  //Serial.print("  ");
  //Serial.print(my_u3);
  //Serial.print("  ");
  //Serial.print(my_u4);
  //Serial.print("  ");
  //Serial.println(distance);
}

void driveforward(){
  cal_speed1();
  cal_speed2();
  cal_speed3();
  cal_speed4();
  motor1(-1,my_u1);
  motor2(1,my_u2);
  motor3(1,my_u3);
  motor4(-1,my_u4);
  }


void cal_speed1(){
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i_1;
    }

  long currT = micros();
  float deltaT = ((float) (currT-prevT_1))/1.0e6;
  float velocity1 = abs((pos - posPrev_1)/deltaT);
  posPrev_1 = pos;
  prevT_1 = currT;
  float v1 = velocity1/600.0*60.0;
  v1Filt_1 = 0.854*v1Filt_1 + 0.0728*v1 + 0.0728*v1Prev_1;
  v1Prev_1 = v1;

  float vt = 50;
  my_vt1 = vt;
  my_v1 = v1Filt_1;
  myPID_1.run();
  }

void cal_speed2(){
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i_2;
    }

  long currT = micros();
  float deltaT = ((float) (currT-prevT_2))/1.0e6;
  float velocity1 = abs((pos - posPrev_2)/deltaT);
  posPrev_2 = pos;
  prevT_2 = currT;
  float v1 = velocity1/600.0*60.0;
  v1Filt_2 = 0.854*v1Filt_2 + 0.0728*v1 + 0.0728*v1Prev_2;
  v1Prev_2 = v1;

  float vt = 50;
  my_vt2 = vt;
  my_v2 = v1Filt_2;
  myPID_2.run();
  }

void cal_speed3(){
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i_3;
    }

  long currT = micros();
  float deltaT = ((float) (currT-prevT_3))/1.0e6;
  float velocity1 = abs((pos - posPrev_3)/deltaT);
  posPrev_3 = pos;
  prevT_3 = currT;
  float v1 = velocity1/600.0*60.0;
  v1Filt_3 = 0.854*v1Filt_3 + 0.0728*v1 + 0.0728*v1Prev_3;
  v1Prev_3 = v1;

  float vt = 50;
  my_vt3 = vt;
  my_v3 = v1Filt_3;
  myPID_3.run();
  }
  
void cal_speed4(){
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i_4;
    }

  long currT = micros();
  float deltaT = ((float) (currT-prevT_4))/1.0e6;
  float velocity1 = abs((pos - posPrev_4)/deltaT);
  posPrev_4 = pos;
  prevT_4 = currT;
  float v1 = velocity1/600.0*60.0;
  v1Filt_4 = 0.854*v1Filt_4 + 0.0728*v1 + 0.0728*v1Prev_4;
  v1Prev_4 = v1;

  float vt = 50;
  my_vt4 = vt;
  my_v4 = v1Filt_4;
  myPID_4.run();
  }

void drivebackward(){
  cal_speed1();
  cal_speed2();
  cal_speed3();
  cal_speed4();
  motor1(1,my_u1);
  motor2(-1,my_u2);
  motor3(-1,my_u3);
  motor4(1,my_u4);
  }

void fullstop(){
  motor1(-1,0);
  motor2(-1,0);
  motor3(-1,0);
  motor4(-1,0);
  }

void spinccw(){
  cal_speed1();
  cal_speed2();
  cal_speed3();
  cal_speed4();
  motor1(1,my_u1);
  motor2(1,my_u2);
  motor3(1,my_u3);
  motor4(1,my_u4);
  }

void spincw(){
  cal_speed1();
  cal_speed2();
  cal_speed3();
  cal_speed4();
  motor1(-1,my_u1);
  motor2(-1,my_u2);
  motor3(-1,my_u3);
  motor4(-1,my_u4);
  }

void motor1(int dir, int pwmVal){
  if (dir == 1){
    digitalWrite(M1DIR,HIGH);
    }
  else if (dir == -1){
    digitalWrite(M1DIR,LOW);
    }
  analogWrite(M1PWM,pwmVal);
  }

void motor2(int dir, int pwmVal){
  if (dir == 1){
    digitalWrite(M2DIR,HIGH);
    }
  else if (dir == -1){
    digitalWrite(M2DIR,LOW);
    }
  analogWrite(M2PWM,pwmVal);
  }

void motor3(int dir, int pwmVal){
  if (dir == 1){
    digitalWrite(M3DIR,HIGH);
    }
  else if (dir == -1){
    digitalWrite(M3DIR,LOW);
    }
  analogWrite(M3PWM,pwmVal);
  }

void motor4(int dir, int pwmVal){
  if (dir == 1){
    digitalWrite(M4DIR,HIGH);
    }
  else if (dir == -1){
    digitalWrite(M4DIR,LOW);
    }
  analogWrite(M4PWM,pwmVal);
  }

void readEncoder1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(encoderead1);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i_1 = pos_i_1 + increment;
}

void readEncoder2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(encoderead2);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i_2 = pos_i_2 + increment;
}

void readEncoder3(){
  // Read encoder B when ENCA rises
  int b = digitalRead(encoderead3);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i_3 = pos_i_3 + increment;
}

void readEncoder4(){
  // Read encoder B when ENCA rises
  int b = digitalRead(encoderead4);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i_4 = pos_i_4 + increment;
}

void readIRSensor(){
  distance = gp2y0e03.get_length();
  //distance = f.filterIn(distance);
  if(distance<0){
    distance = 1000;
    }
}

void Path()
{
  readIRSensor();
  
  if (distance > 20)
  {
    driveforward();
  }
  else if (distance <= 20 && check == 0)
  {
    endTimePath = millis() + 750;

    for (currentTimePath = millis(); currentTimePath < endTimePath;)
    {
      spincw();
      currentTimePath = millis();
    }

    endTimePath = millis() + 500;

    for (currentTimePath = millis(); currentTimePath < endTimePath;)
    {
      driveforward();
      currentTimePath = millis();
    }

    endTimePath = millis() + 750;

    for (currentTimePath = millis(); currentTimePath < endTimePath;)
    {
      spincw();
      currentTimePath = millis();
    }

    check = 1;
    
  }
  else if (distance <= 20 && check == 1)
  {
    endTimePath = millis() + 750;

    for (currentTimePath = millis(); currentTimePath < endTimePath;)
    {
      spinccw();
      currentTimePath = millis();
    }

    endTimePath = millis() + 500;

    for (currentTimePath = millis(); currentTimePath < endTimePath;)
    {
      driveforward();
      currentTimePath = millis();
    }

    endTimePath = millis() + 750;

    for (currentTimePath = millis(); currentTimePath < endTimePath;)
    {
      spinccw();
      currentTimePath = millis();
    }

    check = 0;
    
  }
  else
  {
    fullstop();
  }
}
