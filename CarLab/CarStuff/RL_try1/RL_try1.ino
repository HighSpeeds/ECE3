#include <ECE3.h>
uint16_t sensorValues[8];
const int bump_sw_0_pin = 24;
const int bump_sw_4_pin = 8;

const int LED_BLUE = 77;
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
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}

//memory simple as possible
uint16_t states[200][8];
int actions[200];
float rewards[200];
int max_obs=200;
int n_obs=0;


bool add_to_memory(uint16_t state[],int action,float reward, 
        uint16_t States[][8]=states, int* Actions=actions, float* Reward=rewards, int &n_obs=n_obs){
        // adds to the memory arrays, assuming that n_obs is not greater than or equalt to
        // max observation
        // if it is return false, otherswise return true
        if (n_obs<max_obs){
          //copy the values of state into States
          for (int i=0; i<8; i++){
            States[n_obs][i]=state[i];
          }
          Actions[n_obs]=action;
          Reward[n_obs]=reward;
          n_obs++;
          return true;
        }
        n_obs--;
        return false;
}


//possible actions
uint16_t possible_actions[5][2]={{0,50},{50,0},{100,0},{0,100},{100,100}};
int n_actions=5;

void actions_to_motor(int action,bool reversing=false){
  //turns action i into motor movement
  if (reversing){
    analogWrite(left_pwm_pin,possible_actions[action][1]);
  analogWrite(right_pwm_pin,possible_actions[action][0]);
    }
  else{
  analogWrite(left_pwm_pin,possible_actions[action][0]);
  analogWrite(right_pwm_pin,possible_actions[action][1]);
  }

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
int action;

bool reversing=false;
bool done=false;
float reward;
bool started=false;
int time_steps=10;
int loopStartTime;

void loop() {
    loopStartTime=micros();
    if (reversing && !done){
      Serial.print("reversing");
      //pass the values back
      action=actions[n_obs];
      Serial.print(n_obs);
      n_obs--;
      actions_to_motor(action,false);
      delay(5);
      if (n_obs<0){
        //finished
        Serial.print("done!");
        done=true;
        //so zero motors
        analogWrite(left_pwm_pin,0);
        analogWrite(right_pwm_pin,0);
        analogWrite(LED_GREEN, HIGH);
        analogWrite(LED_RED, LOW);
      }
    }
    else if (!done)
    {
      //moving forward
      if (!started){
        //right blue led on
        analogWrite(LED_BLUE, HIGH);
        started=true;
      }
      //randomly pick an action
      action=random(n_actions);
      Serial.print(n_obs);
      ECE3_read_IR(sensorValues);
      reward=0;
      reversing=!add_to_memory(sensorValues, action, reward);
      delay(5);
      if (reversing){
        //setup for reversing
        //zero out blue led
        analogWrite(LED_BLUE,LOW);
        //turn on red led
        analogWrite(LED_RED,HIGH);
        //reverse motors
        digitalWrite(right_dir_pin,HIGH);
        digitalWrite(right_nslp_pin,HIGH);
        digitalWrite(left_dir_pin,HIGH);
        digitalWrite(left_nslp_pin,HIGH);
      }
      actions_to_motor(action);
    }
    if (!done){
      Serial.println();
    }
    

}
