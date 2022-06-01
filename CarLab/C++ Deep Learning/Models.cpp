#include <iostream>     // std::cout
#include <cmath>        // std::abs
using namespace std;


//sendArray
template<typename Value>
void send1dArray(Value Array[],int array_size){
  for (int i=0; i<array_size; i++){
    cerr<<Array[i]<<",";
  }
  cerr<<endl;
  
}

template<typename Value>
void send2dArray(Value** Array,int dim1,int dim2){
  //assume second dim is 2
  cerr<<dim1<<","<<dim2<<endl;
  for (int i=0; i<dim1; i++){
    send1dArray(Array[i],dim2);  
  }
  
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
        biases_[i]=random()%5;
      }
      for (int i=0; i<output_size; i++){
        for (int j=0; j<input_size; j++){
          weights_[i][j]=random()%5;
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
//      Serial.println("initialized update");
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
        layer_input_sizes_=new int[num_layers_];
        layer_output_sizes_=new int [num_layers_];

        layers_[0]=new LinearyLayer(input_size_,layer_sizes[0]);
        layer_input_sizes_[0]=input_size_;
        layer_output_sizes_[0]=layer_sizes[0];
        for (int i=0; i<num_layers_-2; i++){
            layers_[i+1]=new LinearyLayer(layer_sizes[i],layer_sizes[i+1]);
            layer_input_sizes_[i+1]=layer_sizes[i];
            layer_output_sizes_[i+1]=layer_sizes[i+1];
        }
        layers_[num_layers_-1]=new LinearyLayer(layer_sizes[num_layers_-2],output_size_);
    //      layer_sizes_[num_layers_-1]=output_size_;
        layer_input_sizes_[num_layers_-1]=layer_sizes[num_layers_-2];
        layer_output_sizes_[num_layers_-1]=output_size_;

        // send1dArray(layer_sizes_,num_layers_-1);
    }

    //assigment operator
    Model& operator=(const Model& other){
        input_size_=other.input_size_;
        output_size_=other.output_size_;
        num_layers_=other.num_layers_;
        //delete layer sizes
        delete[] layer_input_sizes_;
        delete[] layer_output_sizes_;
        layer_input_sizes_=new int[num_layers_];
        layer_output_sizes_=new int[num_layers_];
        for (int i=0; i<num_layers_; i++){
            layer_input_sizes_[i]=other.layer_input_sizes_[i];
            layer_output_sizes_[i]=other.layer_output_sizes_[i];
        }
    //      Serial.println("transfered layer sizes");
        for (int i=0; i<num_layers_; i++){
            delete layers_[i];
        }
        delete[] layers_;
    //      Serial.println("transfering layers");
        layers_=new LinearyLayer*[num_layers_];
        for (int i=0; i<num_layers_; i++){
    //        Serial.print("transfering layer ");
    //        Serial.println(i);
            layers_[i]=other.layers_[i];
        }
    //      layers_[num_layers_-1]=new LinearyLayer(layer_sizes_[num_layers_-2],output_size_);
        return *this;
    }

    //copy constructor
    Model(const Model& other){
        input_size_=other.input_size_;
        output_size_=other.output_size_;
        num_layers_=other.num_layers_;
        layer_input_sizes_=new int[num_layers_];
        layer_output_sizes_=new int[num_layers_];
        for (int i=0; i<num_layers_-1; i++){
            layer_input_sizes_[i]=other.layer_input_sizes_[i];
            layer_output_sizes_[i]=other.layer_output_sizes_[i];
        }
        layers_=new LinearyLayer*[num_layers_];
        for (int i=0; i<num_layers_; i++){
            layers_[i]=new LinearyLayer(layer_input_sizes_[i],layer_output_sizes_[i]);
            layers_[i]=other.layers_[i];
        }
    }


    ~Model(){
      for (int i=0; i<num_layers_; i++){
        delete layers_[i];
      }
      delete[] layers_;
      delete[] layer_input_sizes_;
      delete[] layer_output_sizes_;
    }

    int num_layers(){
      return num_layers_;
    }

    void SetWeights(float ***weights, float **biases){
      for (int i=0; i<num_layers_; i++){
        layers_[i]->setWeights(weights[i]);
        layers_[i]->setBiases(biases[i]);
      }
    }

    void forward(float input[],float output[]){
      float *temp_output;
      for (int i=0; i<num_layers_; i++){
          temp_output=new float[layer_output_sizes_[i]];
          
          layers_[i]->forward(input,temp_output);
          relu(output,layer_output_sizes_[i]);
          input=temp_output;
//          Serial.print("layer");
//          Serial.println(i);
//          send1dArray(temp_output,layer_sizes_[i]);
          delete[] temp_output;

      }
      //pass through last layer
      output=input;
    }
    void set_train(){
      //initializes the updates for each layer
//      Serial.println("setting to train");
      for (int i=0; i<num_layers_; i++){
//        Serial.println(i);
        layers_[i]->initializeUpdate();
      }

    }

    void backwards(float input[], float dLdY[],float learning_rate){
//      Serial.println("forward passing");
      float **hiddenLayerOutputs=new float*[num_layers_];
      
//      Serial.println("dynamically created a 2d array");
      //forward pass through, but save the outputs of each layer
      for (int i=0; i<num_layers_; i++){
        //Serial.print("passing through layer: ");
        //Serial.println(i);
        hiddenLayerOutputs[i]=new float[layer_output_sizes_[i]];
        //Serial.println("dynamically alocated a hidden layer");
        layers_[i]->forward(input,hiddenLayerOutputs[i]);
        //pass through relu
        relu(hiddenLayerOutputs[i],layer_output_sizes_[i]);
        //Serial.println("forwad passed");
        input=hiddenLayerOutputs[i];
        //Serial.print("passed through layer: ");
        //Serial.println(i);
      }
//      Serial.println("finished the forward pass");

      float * dLdX_temp=new float[layer_input_sizes_[num_layers_-1]];
      float * dLdX_temp_old;

      //backprop through last layer
      layers_[num_layers_-1]->backwards(dLdY,dLdX_temp);
      //update last layer parameters
      layers_[num_layers_-1]->updateWeights(hiddenLayerOutputs[num_layers_-2],dLdY,learning_rate);
      dLdX_temp_old=dLdX_temp;
//      Serial.print("updated weights for layer");
//      Serial.println(num_layers_-1);
      for (int i=num_layers_-2; i>0; i--){
//        Serial.print("trying to update the weights for layer");
//        Serial.println(i);
        //delete dLdX
        delete[] dLdX_temp;
        
        dLdX_temp=new float[layer_input_sizes_[i]];
        //pass dLdX_temp_old through relu
        relu_derivative(hiddenLayerOutputs[i+1],dLdX_temp_old,layer_output_sizes_[i]);
        //backprop through hidden layers
        layers_[i]->backwards(dLdX_temp_old,dLdX_temp);
        //update hidden layer parameters
        layers_[i]->updateWeights(hiddenLayerOutputs[i-1],dLdX_temp,learning_rate);
//        Serial.print("updated weights for layer");
//        Serial.println(i);
        dLdX_temp_old=dLdX_temp;
      }
      //update the first layers weights
      relu_derivative(hiddenLayerOutputs[0],dLdX_temp_old,layer_output_sizes_[0]);
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

    void dump(){
        for (int i=0; i<num_layers_;i++){
            cerr<<"layer "<<i<<endl;
            cerr<<"weights:"<<endl;
            layers_[i]->printWeights();
            cerr<<"biases:"<<endl;
            layers_[i]->printBiases();
        }
        cerr<<"done"<<endl;
    }

  private:
    int input_size_;
    int output_size_;
    int num_layers_;
    
    int * layer_output_sizes_;
    int* layer_input_sizes_;
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


int main(){
    float beta=0.5;
    SmoothL1Loss loss(beta);
    int layer_sizes[2]={4};
    Model model(8,8,2,layer_sizes);

    cerr<<"running"<<endl;
    model.dump();
    float input[8];
    float target[8]={2,2,2,2,2,2,2,2};
    float output[8];
    float dLdY[8];
    for (int i=0; i<10; i++){
      float total_loss=0;
      cerr<<"running epoch"<<i<<endl;
      model.set_train();
      for (int k=0; k<5; k++){
        //initialize input
        for (int j=0; j<8; j++){input[j]=random()%10; target[j]=input[j]*10;}
//        flash_led(LED_RF,5,100);
        model.forward(input,output);
//        flash_led(LED_RF,5,100);
        total_loss+=loss.CalculateLoss(target,output,8);
        //calcualte dL/dY
//        flash_led(LED_GREEN,5,100);
        loss.CalculateLossDerivative(target,output,dLdY,8);
        //model backwards
        model.backwards(input,dLdY,0.01);
//        flash_led(LED_GREEN,5,100);
      }
      cerr<<"total loss for epoch "<<i<<" is "<<total_loss<<endl;
      model.update();
      
    }
    model.forward(input,output);
    cerr<<"input"<<endl;
    send1dArray(input,8);
    cerr<<"output"<<endl;
    send1dArray(output,8);
}