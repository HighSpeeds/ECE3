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

  for (int i=0; i<4;i++){
    char1 = Serial.read();
    if (char1!=testMsg[i]){
      return false;  
    }
  }
  
  return true;
}


void loop() {
  // put your main code here, to run repeatedly:
  read_bumpers();
//  digitalWrite(LED_RF, HIGH);
//  delay(250);
//  digitalWrite(LED_RF, LOW);
//  delay(250);
  if(!bump_sw_4){
    digitalWrite(LED_RF, HIGH);
    digitalWrite(LED_GREEN, LOW);
    delay(800);
    if (handShake()){
      digitalWrite(LED_GREEN, HIGH);
    }
    delay(2500);
    digitalWrite(LED_RF, LOW);
  }
}
