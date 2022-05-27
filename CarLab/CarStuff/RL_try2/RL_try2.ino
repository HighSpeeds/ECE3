#include <ECE3.h>

const int bump_sw_0_pin = 24;
const int bump_sw_1_pin = 25;
const int bump_sw_2_pin = 6;
const int bump_sw_3_pin = 27;
const int bump_sw_4_pin = 8;
const int bump_sw_5_pin = 28;


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
  fillArray();
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
  analogWrite(left_pwm_pin,possible_actions[n_action][1]);
  analogWrite(right_pwm_pin,possible_actions[n_action][0]);
  
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
    void ~LinearyLayer(){
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
      
      layers[0]=new LinearyLayer(input_size_,layer_sizes[0]);
      layer_sizes_[0]=layer_sizes[0];
      for (int i=0; i<num_layers_-2; i++){
        layers_[i+1]=new LinearyLayer(layer_sizes[i],layer_sizes[i+1]);
        layer_sizes_[i+1]=layer_sizes[i+1];
      }
      layers[num_layers_-1]=new LinearyLayer(layer_sizes[num_layers_-2],output_size_);
      layer_sizes_[num_layers_-1]=output_size_;
    }

    ~Model(){
      for (int i=0; i<num_layers_; i++){
        delete layers_[i];
      }
      delete[] layers_;
      delete[] layer_sizes_;
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

//function to decide whether the model will pick the next action,
// or the action will be randomly selected
int SelectAction(int p_random){

}







void loop() {
  // put your main code here, to run repeatedly:
  read_bumpers();
  //fillArray();
//  digitalWrite(LED_RF, HIGH);
//  delay(250);
//  digitalWrite(LED_RF, LOW);
//  delay(250);
  if(!bump_sw_4){
    digitalWrite(LED_RF, HIGH);
    delay(800);
    read2dArray(testArray2d,5,2);
    delay(2500);
    
    digitalWrite(LED_RF, LOW);
  }

}
