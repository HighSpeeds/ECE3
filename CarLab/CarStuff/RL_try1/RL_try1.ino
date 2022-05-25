#include <ECE3.h>

const int bump_sw_0_pin = 24;
const int bump_sw_4_pin = 8;

const int LED_RF = 77;


void setup() {
  // put your setup code here, to run once:

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


class Memory{
  public:
    void Memory(){};

    bool Add(memory_node newObservation){
      if (observations_taken<=max_observations){
        memory_node[observations_taken]=newObservation;
        observations_taken++;
        return true;
        }
      return false;
    }

    memorySample sample(){
      int index=random(observations_taken-1); //choose an observation, cannot be the last observation because it does not have a next value

      memorySample sampled;
      sampled.sensor_values=Memory[index].sensor_values;
      sampled.action=Memory[index].action;
      sampled.reward=Memory[index].reward;
      sampled.next_sensor_values=Memory[index+1].sensor_values;
      return sampled
   }

     

    
  private: 
    memory_node Memory[5000];
    int observations_taken=0;
    int max_observations=5000;
    
};






void loop() {
  // put your main code here, to run repeatedly:

}
