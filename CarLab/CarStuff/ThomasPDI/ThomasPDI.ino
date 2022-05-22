#include <ECE3.h>

uint16_t sensorValues[8];
const uint16_t sensorMins[8] = {412, 550, 573, 643, 549, 527, 481, 527};
const uint16_t sensorMaxes[8] = {2500, 2494, 2399, 2351, 1707, 2257, 2375, 2500};
const int sensorWeights[8] = {8,4,2,1,-1,-2,-4,-8};   //{5.0,3.5,2.0,1.0,-1.0,-2.0,-3.5,-5.0}

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

uint16_t baseSpeed = 200;
float kP = 50;
float kD = 4;

int prevTime;
float prevError;
void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);

 
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
//  pinMode(PUSH2, INPUT_PULLUP);
//  attachInterrupt(PUSH2, button, FALLING);
  delay(2000);
  prevTime = micros();
  prevError = 0;
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance

  int currentTime = micros();
  int timeElapsed = currentTime - prevTime;
  prevTime = currentTime;
  float error = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    error += sensorWeights[i] * range(sensorMins[i],sensorMaxes[i],sensorValues[i]);
  }
  float derivative = (error-prevError)/timeElapsed*1000000.0;
  float turn = error * kP + derivative * kD;
  
  analogWrite(right_pwm_pin,baseSpeed - turn);
  analogWrite(left_pwm_pin,baseSpeed + turn);
  
  
  prevError = error;
}

float range(uint16_t rangeMin, uint16_t rangeMax, uint16_t number){
  return 1.0*(number - rangeMin) / (rangeMax-rangeMin);
}

float getEncoderCount(){
  return getEncoderCount_left() + getEncoderCount_right();
}
