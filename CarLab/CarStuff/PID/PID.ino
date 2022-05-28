#include <ECE3.h>

uint16_t sensorValues[8];
const uint16_t sensorMins[8] = {753, 616, 662, 523, 615, 592, 686, 757};
const uint16_t sensorMaxes[8] = {2500, 2370, 2500, 1570, 1790, 2120, 2500, 2500};
const int sensorWeights[8] = {8, 4, 2, 1, -1, -2, -4, -8}; //{5.0,3.5,2.0,1.0,-1.0,-2.0,-3.5,-5.0}

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const uint8_t turnSpeedDonut = 200;

uint16_t baseSpeed = 50;
float kP = 12;
float kD = 1;

uint16_t slowSpeed = 50;
float kPS = 7;
float kDS = .15;

int prevTime;
float prevError;

uint8_t numDonuts = 0;
uint8_t state = 0; //0 = follow line, 1 = perform donut
uint32_t initialEncoder;
uint32_t endEncoder;
uint32_t startEncoder;
uint32_t s_start1 = 1440;
uint32_t s_end1 = 3000;
uint32_t s_start2;
uint32_t s_end2;
uint32_t d_slow = 720;
void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);


  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);
  pinMode(PUSH2, INPUT_PULLUP);
  attachInterrupt(PUSH2, button, FALLING);
  delay(2000);
  prevTime = micros();
  prevError = 0;
  numDonuts = 0;
  startEncoder = getEncoderCount();
  
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  while(seesPhantomCrossbar(sensorValues)){
    ECE3_read_IR(sensorValues);
  }

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance

  int currentTime = micros();
  int timeElapsed = currentTime - prevTime;
  prevTime = currentTime;
  float error = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    error += sensorWeights[i] * range(sensorMins[i], sensorMaxes[i], sensorValues[i]);
  }
  float derivative = (error - prevError) / timeElapsed * 1000000.0;
  float turn = error * kP + derivative * kD;
  

  prevError = error;

////  if (sum_sensor_values < 2400 * 8) {
////    analogWrite(right_pwm_pin, baseSpeed - turn);
////    analogWrite(left_pwm_pin, baseSpeed + turn);
////  }
////  else {
////    digitalWrite(right_dir_pin, HIGH);
////
////    reset_EncoderCountLeft();
////
////    analogWrite(right_pwm_pin, 150);
////    analogWrite(left_pwm_pin, 150);
////
////    while (get_encoderCountLeft() < 360) {
////
////    }
//
//    analogWrite(right_pwm_pin, 0);
//    analogWrite(left_pwm_pin, 0);
//
//    digitalWrite(right_dir_pin, LOW);
//  }

    if(state == 0){
      bool endReached = range(sensorMins[0], sensorMaxes[0], sensorValues[0]) > 0.5 && range(sensorMins[7], sensorMaxes[7], sensorValues[7]) > 0.5;
      
      if(endReached){
        if(numDonuts == 0){
          Serial.println("state 1 will be triggered");
          state = 1;
        }
        else if(getEncoderCount() - endEncoder > 10000){
          Serial.println("state 2 will be triggered");
          state = 2;
        }
        initialEncoder = getEncoderCount();
        numDonuts++;
      } 
      else{  
        if((getEncoderCount()-startEncoder > s_start1 && getEncoderCount()-startEncoder < s_end1) ||
        (numDonuts >= 1 && getEncoderCount()-startEncoder > 2*endEncoder - s_end1 && getEncoderCount()-startEncoder < 2*endEncoder - s_start1) ||
        (numDonuts >=1 && getEncoderCount()-startEncoder < endEncoder+d_slow)){
//          float turnS = error * kPS + derivative * kDS;
//          analogWrite(right_pwm_pin,slowSpeed - turnS);
//          analogWrite(left_pwm_pin,slowSpeed + turnS);
            analogWrite(right_pwm_pin,baseSpeed - turn);
            analogWrite(left_pwm_pin,baseSpeed + turn);
        }
        else{
          analogWrite(right_pwm_pin,baseSpeed - turn);
          analogWrite(left_pwm_pin,baseSpeed + turn);
        } 
      }
    }
    else if(state == 1 && numDonuts == 1){
      if(getEncoderCount() - initialEncoder > 660){
        Serial.println("end of donut");
        state = 0;
        digitalWrite(left_dir_pin,LOW);
        analogWrite(right_pwm_pin, 0);
        analogWrite(left_pwm_pin,0);
        endEncoder = getEncoderCount();
      }
      else {
        //Serial.println(getEncoderCountDifference());
        //Serial.println(initialEncoder);
        digitalWrite(left_dir_pin,HIGH);
        analogWrite(right_pwm_pin, turnSpeedDonut);
        analogWrite(left_pwm_pin, turnSpeedDonut);
      }
    }
    else if(state == 2 && numDonuts > 1 && getEncoderCount() - endEncoder > 10000){
        Serial.println("DONE");
        analogWrite(right_pwm_pin, 0);
        analogWrite(left_pwm_pin,0);
    }

}

float range(uint16_t rangeMin, uint16_t rangeMax, uint16_t number) {
  return 1.0 * (number - rangeMin) / (rangeMax - rangeMin);
}

float getEncoderCount() {
  return getEncoderCount_left() + getEncoderCount_right();
}

uint16_t getEncoderCountDifference() {
  return getEncoderCount_right() - getEncoderCount_left();
}

bool seesPhantomCrossbar(uint16_t sensorValues[8]){
  bool seesPhantomCrossbar = true;
  for(uint8_t i = 0; i < 8; i++){
    if(sensorValues[i] != 2500){
      seesPhantomCrossbar = false;
    }
  }
  return seesPhantomCrossbar;
}

void button() {
  resetEncoderCount_left();
  resetEncoderCount_right();
}
