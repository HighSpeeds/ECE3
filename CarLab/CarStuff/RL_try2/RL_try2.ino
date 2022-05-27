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


void read2dArray(float Array[][2],int rows){
  for (int i=0; i<rows; i++){
    for (int j=0; j<2; j++){
      Array[i][j]=Serial.parseFloat();
    }
  }
}



//uint16_t testArray[5]={1,2,3,4,5};

float testArray[5]={1,1,1,1,1};
float testArray2d[5][2];




void loop() {
  // put your main code here, to run repeatedly:
  read_bumpers();
//  digitalWrite(LED_RF, HIGH);
//  delay(250);
//  digitalWrite(LED_RF, LOW);
//  delay(250);
  if(!bump_sw_4){
    digitalWrite(LED_RF, HIGH);
    delay(800);
    read2dArray(testArray2d,5);
    delay(2500);
    
    digitalWrite(LED_RF, LOW);
  }

  if (!bump_sw_5){
    digitalWrite(LED_GREEN, HIGH);
    delay(800);
    for (int i=0; i<5; i++){
      for (int j=0; j<2; j++){
        Serial.print(testArray2d[i][j]);
        Serial.print(",");
      }
      Serial.println();
    }
    digitalWrite(LED_GREEN,LOW);
    
  }
}
