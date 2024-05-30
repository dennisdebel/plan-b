#include "Arduino.h"

float readMux;

//define mux pins
const int PIN_VALUE_ONE = D6; // IO read pin mux 1
const int PIN_VALUE_TWO = D7; // IO read pin mux 2
// const int PIN_INHIBIT_1 =  // maybe you need to connect the inhibit pins to the WEMOS as well, or atleas to ground!
// const int PIN_INHIBIT_2 =  // maybe you need to connect the inhibit pins to the WEMOS as well, or atleas to ground!
const int PIN_A = D1;
const int PIN_B = D2;
const int PIN_C = D5;


void setup()
{
  pinMode(PIN_VALUE_ONE, INPUT);
  pinMode(PIN_VALUE_TWO, INPUT);
  //pinMode(PIN_INH, OUTPUT);
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);

  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  digitalWrite(PIN_C, LOW);

  Serial.begin(115200);
  Serial.println("Hello I am alive!");
}

void loop()
{
  // read pin zero from mux one:
  Serial.println(digitalRead(PIN_VALUE_ONE)); // this gives me '1' without anything being connected...(the reverse wemos pins, low is high, high is low?)
  delay(200);
  Serial.println("Hello I am alive!");
}
