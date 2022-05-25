
#include <ECE3.h>

uint16_t sensorValues[8];


const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;



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
//  delay(2000);
//  prevTime = micros();
//  prevError = 0;
}


int minX[8]={412, 550, 573, 643, 549, 527, 481, 527};
int maxNormX[8]={  2088, 1944, 1826, 1708, 1158, 1730, 1894, 1973};
int weights[8]={8, 4, 2, 1,-1,-2,-4,-8};

float sensorFusion(){
  ECE3_read_IR(sensorValues);

  float error=0;
  for (unsigned char i = 0; i < 8; i++){
    error+=weights[i]*1000*(sensorValues[i]-minX[i])/maxNormX[i];
  }
  return error;
}

float k_p=-10;
float k_d=0;
float PID(float error,float old_error){
  return k_p*error+k_d*(error-old_error);
}

float error;
float old_error=0;

uint16_t base_speed=20;
float correction;

void loop()
{
  error=sensorFusion(); 
  Serial.println(error);
  correction=PID(error,old_error);

  analogWrite(left_pwm_pin,base_speed-correction);
  analogWrite(right_pwm_pin,base_speed+correction);

// 
  
//  ECE3_read_IR(sensorValues);
//
//  digitalWrite(LED_RF, HIGH);
//  delay(250);
//  digitalWrite(LED_RF, LOW);
//  delay(250);

  old_error=error;
}
