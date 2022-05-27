#include <ECE3.h>

const int bump_sw_0_pin = 24;
const int bump_sw_1_pin = 25;
const int bump_sw_2_pin = 6;
const int bump_sw_3_pin = 27;
const int bump_sw_4_pin = 8;
const int bump_sw_5_pin = 28;

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;


const int LED_RF = 77;
const int LED_GREEN=76;

void setup() {
  // put your setup code here, to run once:

  ECE3_Init();
  Serial.begin(9600);
  Serial.setTimeout(1000);
  
  pinMode(LED_RF, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(bump_sw_0_pin,INPUT_PULLUP);
  pinMode(bump_sw_1_pin,INPUT_PULLUP);
  pinMode(bump_sw_2_pin,INPUT_PULLUP);
  pinMode(bump_sw_3_pin,INPUT_PULLUP);
  pinMode(bump_sw_4_pin,INPUT_PULLUP);
  pinMode(bump_sw_5_pin,INPUT_PULLUP);
//  fillArray();

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

  //set the green led on
  digitalWrite(LED_GREEN,HIGH);
}




bool bump_sw_0=true;
bool bump_sw_1=true;
bool bump_sw_2=true;
bool bump_sw_3=true;
bool bump_sw_4=true;
bool bump_sw_5=true;

void read_bumpers(){
  bump_sw_0 = digitalRead(bump_sw_0_pin);
  bump_sw_1 = digitalRead(bump_sw_1_pin);
  bump_sw_2 = digitalRead(bump_sw_2_pin);
  bump_sw_3 = digitalRead(bump_sw_3_pin);
  bump_sw_4 = digitalRead(bump_sw_4_pin);
  bump_sw_5 = digitalRead(bump_sw_5_pin);
};

//handshake
//a handshake function to check if both sides are reciving/getting
char char1;
char testMsg[4]={'t','e','s','t'};
bool handShake(){
  Serial.print("test");
  Serial.println();
  delay(500);
  for (int i=0; i<4;i++){
    char1 = Serial.read();
    if (char1!=testMsg[i]){
      return false;  
    }
  }
  
  return true;
}


//sendArray
template<typename Value>
void send1dArray(Value Array[],int array_size){
  for (int i=0; i<array_size; i++){
    Serial.print(Array[i]);
    Serial.print(",");
  }
  Serial.println();
  
}

template<typename Value>
void send2dArray(Value Array[][2],int dim1,int dim2){
  //assume second dim is 2
  Serial.print(dim1);
  Serial.print(",");
  Serial.print(dim2);
  Serial.println();
  for (int i=0; i<dim1; i++){
    send1dArray(Array[i],dim2);  
  }
  Serial.println();
  
}


//read in arrays
//we will only need float arrays so

void read1dArray(float Array[],int s){
  for (int i=0; i<s; i++){
    Array[i]=Serial.parseFloat();
  }
}


void read2dArray(float **Array,int rows,int cols){
  for (int i=0; i<rows; i++){
    for (int j=0; j<cols; j++){
      Array[i][j]=Serial.parseFloat();
    }
  }
}
//set the actions

uint16_t possibleActions[8][2]={{0,0},
              {50,50},
              {50,0},
              {0,50},
              {100,100},
              {100,50},
              {50,100},
              {200,200}};
static const int num_actions=8;

//convert states to motor speeds
void setMotor(int n_action){
  analogWrite(left_pwm_pin,possibleActions[n_action][1]);
  analogWrite(right_pwm_pin,possibleActions[n_action][0]);
  
}

//random pick a state
int random_action(){
  return random(num_actions);
}
//linear layer class
class LinearyLayer{
  public:
    LinearyLayer(int input_size,int output_size){
      input_size_=input_size;
      output_size_=output_size;
      weights_=new float*[output_size];
      for (int i=0; i<output_size; i++){
        weights_[i]=new float[input_size];
      }
      biases_=new float[output_size];
      for (int i=0; i<output_size; i++){
        biases_[i]=0;
      }
      for (int i=0; i<output_size; i++){
        for (int j=0; j<input_size; j++){
          weights_[i][j]=0;
        }
      }
    }
    ~LinearyLayer(){
      for (int i=0; i<output_size_; i++){
        delete[] weights_[i];
      }
      delete[] weights_;
      delete[] biases_;
    }
    void setWeights(){
      read2dArray(weights_,output_size_,input_size_);
    }
    void setBiases(float *biases){
      read1dArray(biases_,output_size_);
    }
    void printWeights(){
      send2dArray(weights_,output_size_,input_size_);
    }
    void printBiases(){
      send1dArray(biases_,output_size_);
    }
    float ** getWeights(){
      return weights_;
    }

    float * getBiases(){
      return biases_;
    }
    void forward(float input[],float output[]){
      for (int i=0; i<output_size_; i++){
        output[i]=0;
        for (int j=0; j<input_size_; j++){
          output[i]+=input[j]*weights_[i][j];
        }
        output[i]+=biases_[i];
      }
    }
    private:
      int input_size_;
      int output_size_;
      float **weights_;
      float *biases_;
};


//RELU function
void relu(float input[],float output[],int size){
  for (int i=0; i<size; i++){
    if (input[i]>0){
      output[i]=input[i];
    }
    else{
      output[i]=0;
    }
  }
}

class Model{
  public:
    Model(int input_size, int output_size, int num_layers, int layer_sizes[]){
      input_size_=input_size;
      output_size_=output_size;
      num_layers_=num_layers;

      layers_=new LinearyLayer*[num_layers_];
      layer_sizes_=new int[num_layers_-1];
      
      layers_[0]=new LinearyLayer(input_size_,layer_sizes[0]);
      layer_sizes_[0]=layer_sizes[0];
      for (int i=0; i<num_layers_-2; i++){
        layers_[i+1]=new LinearyLayer(layer_sizes[i],layer_sizes[i+1]);
        layer_sizes_[i+1]=layer_sizes[i+1];
      }
      layers_[num_layers_-1]=new LinearyLayer(layer_sizes[num_layers_-2],output_size_);
      layer_sizes_[num_layers_-1]=output_size_;
    }

    ~Model(){
      for (int i=0; i<num_layers_; i++){
        delete layers_[i];
      }
      delete[] layers_;
      delete[] layer_sizes_;
    }

    int num_layers(){
      return num_layers_;
    }

    void forward(float input[],float output[]){
      for (int i=0; i<num_layers_; i++){
        layers_[i]->forward(input,output);
        relu(output,output,layer_sizes_[i]);
      }
    }

    LinearyLayer* getLinearLayer(int i){
      return layers_[i];
    }

  private:
    int input_size_;
    int output_size_;
    int num_layers_;
    int *layer_sizes_;
    LinearyLayer **layers_;

};

//define our model, which we will first set as a 2 layer linear model
//with 8 inputs and num_actions outputs
int layer_sizes[num_actions-1]={4};
Model model(8,num_actions,2,layer_sizes);

//get the action the model chose
float output[num_actions];
int model_pick_action(){
  model.forward(state,output);
  int max_index=0;
  float max_value=output[0];
  for (int i=1; i<num_actions; i++){
    if (output[i]>max_value){
      max_value=output[i];
      max_index=i;
    }
  }
  return max_index;
}

//function to decide whether the model will pick the next action,
// or the action will be randomly selected
int SelectAction(int p_random){
  if (p_random*100>random(100)){
    return random_action();
  }
  else{
    return model_pick_action();
  }
}


//setup memory
static const int max_obs=1000;
uint16_t states[max_obs][8];
int actions[max_obs];
float rewards[max_obs];
int num_obs=0;

bool AddValue(int state[],int action,float reward){
  if (num_obs<max_obs){
    for (int i=0; i<8; i++){
      states[num_obs][i]=state[i];
    }
    actions[num_obs]=action;
    rewards[num_obs]=reward;
    num_obs++;
    return true;
  }
  else{
    return false;
  }
}


//dumping the memory to serial
void dump_memory(){
  Serial.print("memory dump")
  Serial.println();
  Serial.print(num_obs);
  Serial.println("");
  Serial.println("actions:")
  send1dArray(actions,num_obs);
  Serial.println("rewards:")
  send1dArray(rewards,num_obs);
  Serial.println("states:")
  send2dArray(states,num_obs,8);
  Serial.println("done")
}

//dumping the model to serial
void dump_model(){
  for (int i=0; i<model.num_layers();i++){
    Serial.print("layer ");
    Serial.print(i);
    Serial.println("");
    Serial.println("weights:")
    model.getLinearLayer(0)->printWeights();
    Serial.println("biases:")
    model.getLinearLayer(0)->printBiases();
  }
  Serial.println("done")
}




uint16_t sensorValues[8];

bool mem_full=false;
int action_selected;
bool val=false;
float p_random=0.5;
float reward;

void loop() {
  // put your main code here, to run repeatedly:
  read_bumpers();
  
  if (!mem_full && !val){
    //read sensor values
    ECE3_read_IR(sensorValues);
    //pick an action
    action=SelectAction(p_random);
    //set motor speeds based of this action
    set_motors(action);
    //wait 50 ms
    delay(50);
    //calculate reward (not implemented so set reward to 0 for now)
    reward=0;
    //add to memory
    mem_full=AddValue(sensorValues,action,reward);
  }
  
  //if the memory is full and still in train mode
  //turn off the green led
  if (mem_full && !val){
    digitalWrite(LED_GREEN,LOW);
    //and turn on the RF led
    digitalWrite(LED_RF,HIGH);
  }
  }
  if(!bump_sw_4){
    //if the 4th bumper is pressed
    
    //turn off the RF led
    digitalWrite(LED_RF, LOW);
    //and turn on the green led
    digitalWrite(LED_GREEN, HIGH);
    //set the num_obs to 0
    num_obs=0;
    //set mem_full to false
    mem_full=false;
  }

}
