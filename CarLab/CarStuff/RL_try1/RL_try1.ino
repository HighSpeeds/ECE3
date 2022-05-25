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

void copy_sensor_values(uint16_t* sensor_values,uint16_t* memory_sensor_values){
  for (int i=0; i<8; i++){
    memory_sensor_values[i]=sensor_values[i];
  }
}

class Memory{
  public:
    Memory(){}

    bool Add(memory_node newObservation){
      if (observations_taken<=max_observations){
        memory[observations_taken]=newObservation;
        observations_taken++;
        return true;
        }
      return false;
    }

    memorySample randomSample(){
      int index=random(observations_taken-1); //choose an observation, cannot be the last observation because it does not have a next value

      memorySample sampled;


      copy_sensor_values(memory[index].sensor_values,sampled.sensor_values);
      sampled.action=memory[index].action;
      sampled.reward=memory[index].reward;
      copy_sensor_values(memory[index+1].sensor_values,sampled.sensor_values);
      return sampled;
   }

   int nObs(){
    return observations_taken;
   }

   memory_node getSample(int index){
    return memory[index];
    
   } 
  private: 
    memory_node memory[1000];
    int observations_taken=0;
    int max_observations=1000;
    
};

//Memory memory;
memory_node memory[4000];
int observations_taken=0;
int max_observations=4000;

bool Add(memory_node newObservation){
      if (observations_taken<=max_observations){
        memory[observations_taken]=newObservation;
        observations_taken++;
        return true;
        }
      return false;
}

memorySample randomSample(){
    int index=random(observations_taken-1); //choose an observation, cannot be the last observation because it does not have a next value

    memorySample sampled;


    copy_sensor_values(memory[index].sensor_values,sampled.sensor_values);
    sampled.action=memory[index].action;
    sampled.reward=memory[index].reward;
    copy_sensor_values(memory[index+1].sensor_values,sampled.sensor_values);
    return sampled;
}




void loop() {
  // put your main code here, to run repeatedly:

}
