#include <ECE3.h>
uint16_t sensorValues[8];
const int bump_sw_0_pin = 24;
const int bump_sw_4_pin = 8;

const int LED_RF = 77;
const int LED_GREEN=76;
const int LED_RED = 75;

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

void setup() {
  // put your setup code here, to run once:
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  //delay(2000);

 
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
  pinMode(LED_RF, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}

//setup memory

struct memory_node{
  uint16_t sensor_values[8]; //the sensor Values at this time t
  int action; //the action taken
  int reward; //the reward recived
};

struct memorySample{
  uint16_t sensor_values[8]; //the sensor Values at this time t
  int action; //the action taken
  int reward; //the reward recived
  uint16_t next_sensor_values[8];
};

void copy_unit16(uint16_t* arr1,uint16_t* arr2){
  for (int i=0; i<8; i++){
    arr2[i]=arr1[i];
  }
}


//Memory memory;
struct memory_node memory[2000];
int observations_taken=0;
int max_observations=2000;

bool Add(uint16_t sensorValues[],int action,int reward, int & observations_taken,struct memory_node *memory){
    if (observations_taken<=max_observations){
      
      memory[observations_taken].action=action;
      observations_taken++;
      return true;
    }
    return false;
}

struct memorySample randomSample(){
    int index=random(observations_taken-1); //choose an observation, cannot be the last observation because it does not have a next value
    memorySample sampled;


    copy_unit16(memory[index].sensor_values,sampled.sensor_values);
    sampled.action=memory[index].action;
    sampled.reward=memory[index].reward;
    copy_unit16(memory[index+1].sensor_values,sampled.sensor_values);
    return sampled;
}


//possible actions
uint16_t actions[5][2]={{0,50},{50,0},{100,0},{0,100},{100,100}};
int n_actions=5;

void actions_to_motor(int i){
  //turns action i into motor movement
  analogWrite(left_pwm_pin,actions[i][0]);
  analogWrite(right_pwm_pin,actions[i][1]);

}


//stop function (incomplete)
bool stop=false;

//

int minX[8]={412, 550, 573, 643, 549, 527, 481, 527};
int maxNormX[8]={  2088, 1944, 1826, 1708, 1158, 1730, 1894, 1973};
int weights[8]={8, 4, 2, 1,-1,-2,-4,-8};

float sensorFusion(uint16_t sensorValues[]){

  float error=0;
  for (unsigned char i = 0; i < 8; i++){
    error+=weights[i]*1000*(sensorValues[i]-minX[i])/maxNormX[i];
  }
  return error;
}

//stop function (incomplete)
bool OffTrack=false;
int i;

bool reversing=false;

int reward;
memory_node new_memory_node;

void loop() {
  // put your main code here, to run repeatedly:
  //pick a random action
  if (!reversing){
    i=random(n_actions);
    //i=4;
//    Serial.println(i);
    //take the measurments from the sensors
    ECE3_read_IR(sensorValues);
    //calculate reward
    reward=0;
    //move motors
    actions_to_motor(i);
    //save values
    reversing=!Add(sensorValues,i,reward,observations_taken,memory);
    delay(0.005);
    if (reversing){
      analogWrite(left_pwm_pin,0);
      analogWrite(right_pwm_pin,0);
      i=max_observations-1;
      digitalWrite(right_dir_pin,HIGH);
      digitalWrite(left_dir_pin,HIGH);
      digitalWrite(left_nslp_pin,HIGH);
      digitalWrite(right_nslp_pin,HIGH);
      digitalWrite(LED_RF, HIGH);
    }
  }
//      Serial.println(i);
  else{
    Serial.println(i);
    if (i==max_observations-1){digitalWrite(LED_RED, HIGH);}
    actions_to_motor(memory[i].action);
    i--;
    delay(0.005);
    if (i==-1){
      reversing=true;
      digitalWrite(LED_RF, LOW);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(right_dir_pin,LOW);
      digitalWrite(left_dir_pin,LOW);
      observations_taken=0;
    }
  }
    

}
