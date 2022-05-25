

#include <ECE3.h>

const int bump_sw_0_pin = 24;
const int bump_sw_4_pin = 8;

const int LED_RF = 77;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_RF, OUTPUT);
  pinMode(bump_sw_0_pin,INPUT_PULLUP);
  pinMode(bump_sw_4_pin,INPUT_PULLUP);
}
bool bump_sw_0_reading;
bool bump_sw_4_reading;
void loop() {
  // put your main code here, to run repeatedly:
  bump_sw_0_reading = digitalRead(bump_sw_0_pin);
  bump_sw_4_reading = digitalRead(bump_sw_4_pin);
//  digitalWrite(LED_RF, HIGH);
//  delay(250);
//  digitalWrite(LED_RF, LOW);
//  delay(250);
  if(!bump_sw_0_reading && !bump_sw_4_reading){
    digitalWrite(LED_RF, HIGH);
    delay(250);
    digitalWrite(LED_RF, LOW);
  }
}
