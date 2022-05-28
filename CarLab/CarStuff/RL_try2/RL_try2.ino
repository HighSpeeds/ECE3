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
const int LED_RED=75;

void setup() {
  // put your setup code here, to run once:

  ECE3_Init();
  Serial.begin(9600);
  Serial.setTimeout(1000);
  
  pinMode(LED_RF, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED,OUTPUT);
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

//  //set the green led on
//  digitalWrite(LED_GREEN,HIGH);
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
void send2dArray(Value** Array,int dim1,int dim2){
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
        biases_[i]=random(5);
      }
      for (int i=0; i<output_size; i++){
        for (int j=0; j<input_size; j++){
          weights_[i][j]=random(5);
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

    //assigment operator
    LinearyLayer& operator=(const LinearyLayer& other){
      input_size_=other.input_size_;
      output_size_=other.output_size_;
      //delete weights and biases first
      for (int i=0; i<output_size_; i++){
        delete[] weights_[i];
      }
      delete[] weights_;
      delete[] biases_;
      weights_=new float*[output_size_];
      for (int i=0; i<output_size_; i++){
        weights_[i]=new float[input_size_];
      }
      biases_=new float[output_size_];
      for (int i=0; i<output_size_; i++){
        biases_[i]=other.biases_[i];
      }
      for (int i=0; i<output_size_; i++){
        for (int j=0; j<input_size_; j++){
          weights_[i][j]=other.weights_[i][j];
        }
      }
      return *this;
    }
    //copy constructor
    LinearyLayer(const LinearyLayer& other){
      input_size_=other.input_size_;
      output_size_=other.output_size_;
      weights_=new float*[output_size_];
      for (int i=0; i<output_size_; i++){
        weights_[i]=new float[input_size_];
      }
      biases_=new float[output_size_];
      for (int i=0; i<output_size_; i++){
        biases_[i]=other.biases_[i];
      }
      for (int i=0; i<output_size_; i++){
        for (int j=0; j<input_size_; j++){
          weights_[i][j]=other.weights_[i][j];
        }
      }
    }
    void serialSetWeights(){
      read2dArray(weights_,output_size_,input_size_);
    }
    void serialSetBiases(float *biases){
      read1dArray(biases_,output_size_);
    }

    void setWeights(float **weights){
      for (int i=0; i<output_size_; i++){
        for (int j=0; j<input_size_; j++){
          weights_[i][j]=weights[i][j];
        }
      }
    }
    void setBiases(float *biases){
      for (int i=0; i<output_size_; i++){
        biases_[i]=biases[i];
      }
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

    void backwards(float dLdY[],float dLdX[]){

      //output dL/dX
      for (int i=0; i<input_size_; i++){
        dLdX[i]=0;
        for (int j=0; j<output_size_; j++){
          dLdX[i]+=dLdY[j]*weights_[j][i];
        }
      }
    }
    void initializeUpdate(){
      dLdW_=new float*[output_size_];
      for (int i=0; i<output_size_; i++){
        dLdW_[i]=new float[input_size_];
      }
      dLdB_=new float[output_size_];
      //set both to 0
      for (int i=0; i<output_size_; i++){
        for (int j=0; j<input_size_; j++){
          dLdW_[i][j]=0;
        }
        dLdB_[i]=0;
      }
    }
    void updateWeights(float input[],float dLdY[],float learning_rate){
      for (int i=0; i<output_size_; i++){
        for (int j=0; j<input_size_; j++){
          dLdW_[i][j]+=learning_rate*dLdY[i]*input[j];
        }
        dLdB_[i]+=learning_rate*dLdY[i];
      }
    }

    void update(){
      for (int i=0; i<output_size_; i++){
        for (int j=0; j<input_size_; j++){
          weights_[i][j]+=dLdW_[i][j];
        }
        biases_[i]+=dLdB_[i];
      }
      //delete both
      for (int i=0; i<output_size_; i++){
        delete[] dLdW_[i];
      }
      delete[] dLdW_;
      delete[] dLdB_;
    }
    private:
      int input_size_;
      int output_size_;
      float **weights_;
      float *biases_;
      float **dLdW_;
      float *dLdB_;
};


//RELU function
void relu(float input[],int size){
  for (int i=0; i<size; i++){
    if (input[i]<0){
      input[i]=0;
    }
  }
}

void relu_derivative(float input[],float input_derivative[],int size){
  for (int i=0; i<size; i++){
    if (input[i]<0){
      input_derivative[i]=0;
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
//      layer_sizes_[num_layers_-1]=output_size_;

      // send1dArray(layer_sizes_,num_layers_-1);
    }

    //assigment operator
    Model& operator=(const Model& other){
      input_size_=other.input_size_;
      output_size_=other.output_size_;
      num_layers_=other.num_layers_;
      //delete layer sizes
      delete[] layer_sizes_;
      layer_sizes_=new int[num_layers_-1];
      for (int i=0; i<num_layers_-1; i++){
        layer_sizes_[i]=other.layer_sizes_[i];
      }
      for (int i=0; i<num_layers_; i++){
        delete layers_[i];
      }
      delete[] layers_;
      layers_=new LinearyLayer*[num_layers_];
      for (int i=0; i<num_layers_-1; i++){
        layers_[i]=new LinearyLayer(layer_sizes_[i],layer_sizes_[i+1]);
      }
      layers_[num_layers_-1]=new LinearyLayer(layer_sizes_[num_layers_-2],output_size_);
      return *this;
    }

    //copy constructor
    Model(const Model& other){
      input_size_=other.input_size_;
      output_size_=other.output_size_;
      num_layers_=other.num_layers_;
      layer_sizes_=new int[num_layers_-1];
      for (int i=0; i<num_layers_-1; i++){
        layer_sizes_[i]=other.layer_sizes_[i];
      }
      layers_=new LinearyLayer*[num_layers_];
      for (int i=0; i<num_layers_-1; i++){
        layers_[i]=new LinearyLayer(layer_sizes_[i],layer_sizes_[i+1]);
      }
      layers_[num_layers_-1]=new LinearyLayer(layer_sizes_[num_layers_-2],output_size_);
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

    void SerialPrintLayerSizes(){
      send1dArray(layer_sizes_,num_layers_-1);
    }

    void SetWeights(float ***weights, float **biases){
      for (int i=0; i<num_layers_-1; i++){
        layers_[i]->setWeights(weights[i]);
        layers_[i]->setBiases(biases[i]);
      }
    }

    void forward(float input[],float output[]){
      float *temp_output;
      for (int i=0; i<num_layers_-1; i++){
          temp_output=new float[layer_sizes_[i]];
          
          layers_[i]->forward(input,temp_output);
          relu(output,layer_sizes_[i]);
          input=temp_output;
          Serial.print("layer");
          Serial.println(i);
          send1dArray(temp_output,layer_sizes_[i]);
          delete[] temp_output;

      }
      //pass through last layer
      layers_[num_layers_-1]->forward(input,output);
    }
    void set_train(){
      //initializes the updates for each layer
      for (int i=0; i<num_layers_-1; i++){
        layers_[i]->initializeUpdate();
      }

    }

    void backwards(float input[], float dLdY[],float learning_rate){
      //Serial.println("forward passing");
      float **hiddenLayerOutputs=new float*[num_layers_-1];
      //Serial.println("dynamically created a 2d array");
      //forward pass through, but save the outputs of each layer
      for (int i=0; i<num_layers_-1; i++){
        //Serial.print("passing through layer: ");
        //Serial.println(i);
        hiddenLayerOutputs[i]=new float[layer_sizes_[i]];
        //Serial.println("dynamically alocated a hidden layer");
        layers_[i]->forward(input,hiddenLayerOutputs[i]);
        //pass through relu
        relu(hiddenLayerOutputs[i],layer_sizes_[i]);
        //Serial.println("forwad passed");
        input=hiddenLayerOutputs[i];
        //Serial.print("passed through layer: ");
        //Serial.println(i);
      }
      //Serial.println("finished the forward pass");

      float * dLdX_temp=new float[layer_sizes_[num_layers_-2]];
      float * dLdX_temp_old;

      //backprop through last layer
      layers_[num_layers_-1]->backwards(dLdY,dLdX_temp);
      //update last layer parameters
      layers_[num_layers_-1]->updateWeights(hiddenLayerOutputs[num_layers_-2],dLdY,learning_rate);
      dLdX_temp_old=dLdX_temp;
      //Serial.print("updated weights for layer");
      //Serial.println(num_layers_-1);
      for (int i=num_layers_-2; i>0; i--){
        //Serial.print("trying to update the weights for layer");
        //Serial.println(i);
        //delete dLdX
        delete[] dLdX_temp;
        
        dLdX_temp=new float[layer_sizes_[i]];
        //pass dLdX_temp_old through relu
        relu_derivative(hiddenLayerOutputs[i-1],dLdX_temp_old,layer_sizes_[i]);
        //backprop through hidden layers
        layers_[i]->backwards(dLdX_temp_old,dLdX_temp);
        //update hidden layer parameters
        layers_[i]->updateWeights(hiddenLayerOutputs[i-1],dLdX_temp,learning_rate);
        //Serial.print("updated weights for layer");
        //Serial.println(i);
        dLdX_temp_old=dLdX_temp;
      }
      //update the first layers weights
      relu_derivative(hiddenLayerOutputs[i-1],dLdX_temp_old,layer_sizes_[i]);
      layers_[0]->updateWeights(input,dLdX_temp_old,learning_rate);
      delete[] dLdX_temp;
      for (int i=0; i<num_layers_-1; i++){
        delete[] hiddenLayerOutputs[i];
      }
      delete[] hiddenLayerOutputs;
    }
    void update(){
      //update layers
      for (int i=0; i<num_layers_; i++){
        layers_[i]->update();
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


class SmoothL1Loss{
  public:
    SmoothL1Loss(float beta)
    {
      beta_=beta;
    }
    float CalculateLossForOneSample(float output, float target){
      float loss=0;
      float diff=abs(output-target);
      if (diff<=beta_){
        loss=0.5*diff*diff/beta_;
      }
      else{
        loss=diff-0.5*beta_;
      }
      return loss;
    }

    float CalculateLossDerivativeForOneSample(float output, float target){
      float diff=output-target;
      if (abs(diff)<=beta_){
        return diff/beta_;
      }
      else{
        if (diff>0){
          return 1;
        }
        else{
          return -1;
        }
      }
    }

    float CalculateLoss(float output[], float target[], int size){
      float loss=0;
      for (int i=0; i<size; i++){
        loss+=CalculateLossForOneSample(output[i],target[i]);
      }
      return loss;
    }

    void CalculateLossDerivative(float output[], float target[], float dLdY[], int size){
      for (int i=0; i<size; i++){
        dLdY[i]=CalculateLossDerivativeForOneSample(output[i],target[i]);
      }
    }
  private:
    float beta_;
};
float beta=0.5;
SmoothL1Loss loss(beta);

//define our model, which we will first set as a 2 layer linear model
//with 8 inputs and num_actions outputs
int layer_sizes[2]={4};
Model target_model(8,num_actions,2,layer_sizes);

Model policy_model(8,num_actions,2,layer_sizes);

//get the action the model chose

float output[num_actions];
float float_state[8];
int model_pick_action(uint16_t state[]){
  for (int i=0; i<8; i++){
    float_state[i]=state[i];
  }
  target_model.forward(float_state,output);
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
int SelectAction(float p_random, uint16_t state[]){
  if (p_random*100>random(100)){
    digitalWrite(LED_RED,HIGH);
    return random_action();
  }
  else{
    digitalWrite(LED_RED,LOW);
    return model_pick_action(state);
  }
}


//setup memory

//set up as a class
static const int max_obs=1000;

class Memory{
  public:
    Memory(int max_obs,int state_size=8){
      max_obs_=max_obs;
      obs_count_=0;
      state_size_=state_size;
      states_=new uint16_t*[max_obs_];
      for (int i=0; i<max_obs_; i++){
        states_[i]=new uint16_t[state_size];
      }
      actions_=new int[max_obs_];
      rewards_=new float[max_obs_];
    }
    ~Memory(){
      for (int i=0; i<max_obs_; i++){
        delete[] states_[i];
      }
      delete[] states_;
      delete[] actions_;
      delete[] rewards_;
    }

    bool AddValue(uint16_t state[],int action,float reward){
      if (obs_count_<max_obs_){
        for (int i=0; i<8; i++){
          states_[obs_count_][i]=state[i];
        }
        actions_[obs_count_]=action;
        rewards_[obs_count_]=reward;
        obs_count_+=1;
        // Serial.println("Adding Value to memory");
        // Serial.print("max obs");
        // Serial.println(max_obs_);
        // Serial.print("obs_count_");
        // Serial.println(obs_count_);
        return true;
      }
      else{
        return false;
      }
    }

    void Sample(uint16_t state[],uint16_t next_state[],int &action,float &reward){
      //random samples an index
      int index=random(obs_count_-1);
      //get the state
      for (int i=0; i<8; i++){
        state[i]=states_[index][i];
      }
      //get the action
      action=actions_[index];
      //get the reward
      reward=rewards_[index];
      //get the next state
      for (int i=0; i<8; i++){
        next_state[i]=states_[index+1][i];
      }

    }

    void Reset(){
      obs_count_=0;
    }

    int num_obs(){return obs_count_;}

    void dumpToSerial(){
      Serial.print("memory dump");
      Serial.println();
      Serial.print(obs_count_);
      Serial.println("");
      Serial.println("actions:");
      send1dArray(actions_,obs_count_);
      Serial.println("rewards:");
      send1dArray(rewards_,obs_count_);
      Serial.println("states:");
      send2dArray(states_,obs_count_,state_size_);
      Serial.println("done");
    }

  private:
    int max_obs_;
    int state_size_;
    uint16_t **states_;
    int *actions_;
    float *rewards_;
    int obs_count_;
};

Memory memory(max_obs);

//dumping the model to serial
void dump_model(){
  for (int i=0; i<policy_model.num_layers();i++){
    Serial.print("layer ");
    Serial.print(i);
    Serial.println("");
    Serial.println("weights:");
    policy_model.getLinearLayer(i)->printWeights();
    Serial.println("biases:");
    policy_model.getLinearLayer(i)->printBiases();
  }
  Serial.println("done");
}

float Gamma=0.9;
//train function
void train(int n_samples,float learning_rate=0.01){
  //set policy model to train
  policy_model.set_train();
  for (int i=0; i<n_samples; i++){
    float state[8];
    float next_state[8];
    int action;
    float reward;
    //sample from memory
    memory.Sample(state,next_state,action,reward);
    //pass through policy model
    float output[num_actions];
    policy_model.forward(state,output);
    //get the predicted Q values for the action taken state
    float Q_predicted=output[action];
    //get the Q values for the next state
    float Q_next_state[num_actions];
    //pass through target model
    target_model.forward(next_state,Q_next_state);
    //get the max Q value for the next state
    float max_Q_next_state=Q_next_state[0];
    for (int j=1; j<num_actions; j++){
      if (Q_next_state[j]>max_Q_next_state){
        max_Q_next_state=Q_next_state[j];
      }
    }
    //get the target Q value
    float target_Q=reward+Gamma*max_Q_next_state;
    //get the deriative of the loss
    float dlossdY[8]={0};
    dlossdY[action]=loss.CalculateLossDerivativeForOneSample(Q_predicted,target_Q);
    //backpropagate
    policy_model.backward(state,dlossdY,learning_rate);
  }
  //update policy model
  policy_model.update();
}
//calculated the reward function
float getEncoderCount() {
  return getEncoderCount_left() + getEncoderCount_right();
}

uint16_t getEncoderCountDifference() {
  return getEncoderCount_right() - getEncoderCount_left();
}
int minX[8]={412, 550, 573, 643, 549, 527, 481, 527};
int maxNormX[8]={  2088, 1944, 1826, 1708, 1158, 1730, 1894, 1973};
int weights[8]={8, 4, 2, 1,1,2,4,8};

float calculate_reward() {
  uint16_t currSensorValues[8];
  ECE3_read_IR(currSensorValues);
  float reward = getEncoderCount();
  //sensor fusion
  for (int i=0; i<8; i++){
    reward-=1000*weights[i]*(currSensorValues[i]-minX[i])/maxNormX[i];
  }
  return reward;
}

//transfer the policy model to the target model
void transferModels(){
  target_model=policy_model;
}

//helpful function to flash the led
void flash_led(const int LED_PIN,int n_times,int delay_ms){
  for (int i=0; i<n_times; i++){
    digitalWrite(LED_PIN,HIGH);
    delay(delay_ms);
    digitalWrite(LED_PIN,LOW);
    delay(delay_ms);
  }
}

uint16_t sensorValues[8];

bool mem_full=false;
int action_selected;
bool val=false;
float p_random=0;
float reward;

int batch_size=100;
float lr=0.01;

int n_runs=0;
int transfer_every=5;

bool run_car=false;
void loop() {
  // put your main code here, to run repeatedly:
  read_bumpers();
  if (run_car){
    if (!mem_full && !val){
      //read sensor values
      ECE3_read_IR(sensorValues);
      //pick an action
      action_selected=SelectAction(p_random,sensorValues);
      //set motor speeds based of this action
      setMotor(action_selected);
      //wait 50 ms
      delay(50);
      //calculate reward (not implemented so set reward to 0 for now)
      reward=calculate_reward();
      //add to memory
      mem_full=!memory.AddValue(sensorValues,action_selected,reward);
      if (mem_full){
        run_car=false;
      }
    }
    if (val){
      //read the sensor values
      ECE3_read_IR(sensorValues);
      //select action with the model
      action_selected=model_pick_action(sensorValues);
      //set motor speeds based of this action
      setMotor(action_selected);
      //wait 50 ms
      delay(50);
      //add to memory
      mem_full=!memory.AddValue(sensorValues,action_selected,0.0);
      if (mem_full){
        //restart the memory
        memory.Reset();
      }
    }
  }
  
  //if the memory is full and still in train mode
  //turn off the green led
  if (mem_full && !val){
    digitalWrite(LED_GREEN,LOW);
    //and turn on the RF led
    digitalWrite(LED_RF,HIGH);
    //train the model
    digitalWrite(LED_RED,HIGH);
    train(batch_size,lr);
    digitalWrite(LED_RED,LOW);
  }

  if (mem_full && !bump_sw_2){
    //reset the memory
    
    //turn off the RF led
    digitalWrite(LED_RF,LOW);
    //and turn on the GREEN led
    digitalWrite(LED_GREEN,HIGH);
    //reset the memory
    memory.Reset();
    //set mem_full to false
    mem_full=false;
    //incrment the number of runs
    n_runs++;
    //if the number of runs is divisible by transfer_every
    //transfer the models
    if (n_runs%transfer_every==0){
      digitalWrite(LED_RF,HIGH);
      transferModels();
      delay(1000);
      digitalWrite(LED_RF,LOW);
    }
  }
  if (!bump_sw_4 && !bump_sw_2){
    //dump the model over serial
    dump_model();
    flash_led(LED_GREEN,5,100);
    flash_led(LED_RF,5,100);
  }

  if (!bump_sw_0 && !bump_sw_5){
    //switch to val mode
    val=true;
    flash_led(LED_RF,5,100);
  }

  if (!bump_sw_5 && !bump_sw_4){
    //switch to train mode
    val=false;
    flash_led(LED_RED,5,100);
  }

  if (!bump_sw_0 && !bump_sw_1){
    //switch to run mode
    run_car=true;
    flash_led(LED_GREEN,5,100);
  }
  
}
