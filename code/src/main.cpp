#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
// TODO
// -add debug flag
//


// declare custom functions
void muxOne(); 
void muxTwo(); 

// declare mux pins
const int PIN_VALUE_ONE = D6; // IO read pin mux 1 (COMmon InputOutput pin)
const int PIN_VALUE_TWO = D7; // IO read pin mux 2 (COMmon InputOutput pin)
const int PIN_A = D1;
const int PIN_B = D2;
const int PIN_C = D5;

int buttonValue[8] = {0,1,2,3,4,5,6,7}; // array/counter for pin numbers

//software button debounce 
int lastButtonStateOne = LOW;           // the previous reading from the input pin, mux one
unsigned long lastDebounceTimeOne = 0;  // the last time the output pin was toggled, mux one
int lastButtonStateTwo = LOW;           // the previous reading from the input pin, mux two
unsigned long lastDebounceTimeTwo = 0;  // the last time the output pin was toggled, mux two
unsigned long debounceDelay = 200;      // the debounce time; increase if the output flickers

int b0 = 0; // channel storage
int b1 = 0;
int b2 = 0;

void setup()
{
  pinMode(PIN_VALUE_ONE, INPUT);
  pinMode(PIN_VALUE_TWO, INPUT);

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
  muxOne(); // function to loop over mux 1
  muxTwo(); // function to loop over mux 2
}

void muxOne() {

  for (int buttonCount = 0; buttonCount < 8; buttonCount++) {

      b0 = bitRead(buttonCount,0); // convert buttonCount integer to bits and assign the first bit to the variable b0
      b1 = bitRead(buttonCount,1); // convert buttonCount integer to bits and assign the second bit to the variable b1
      b2 = bitRead(buttonCount,2); // convert buttonCount integer to bits and assign the last bit to the variable b2

      digitalWrite(PIN_A,b0);
      digitalWrite(PIN_B,b1);
      digitalWrite(PIN_C,b2);

      int reading = digitalRead(PIN_VALUE_ONE);
      
      if(reading == HIGH && lastButtonStateOne == LOW && millis() - lastDebounceTimeOne > debounceDelay)
      {
        lastDebounceTimeOne = millis();
            Serial.print("Mux 1 Pin: "); // DEBUG
            Serial.print(buttonValue[buttonCount]); // DEBUG
            Serial.println(" Pressed!"); // DEBUG
            // Serial.print("Value: "); // DEBUG
            // Serial.print(" "); // DEBUG
            // Serial.println(digitalRead(PIN_VALUE_ONE)); // DEBUG
      }
      lastButtonStateOne = reading;
  }    
}
  
void muxTwo() {

  for (int buttonCount = 0; buttonCount < 8; buttonCount++) {

      b0 = bitRead(buttonCount,0); // convert buttonCount integer to bits and assign the first bit to the variable b0
      b1 = bitRead(buttonCount,1); // convert buttonCount integer to bits and assign the second bit to the variable b1
      b2 = bitRead(buttonCount,2); // convert buttonCount integer to bits and assign the last bit to the variable b2

      digitalWrite(PIN_A,b0);
      digitalWrite(PIN_B,b1);
      digitalWrite(PIN_C,b2);

      int reading = digitalRead(PIN_VALUE_TWO);

      if(reading == HIGH && lastButtonStateTwo == LOW && millis() - lastDebounceTimeTwo > debounceDelay)
      {
        lastDebounceTimeTwo = millis();
            Serial.print("Mux 2 Pin: "); // DEBUG
            Serial.print(buttonValue[buttonCount]); // DEBUG
            Serial.println(" Pressed!"); // DEBUG
            // Serial.print("Value: "); // DEBUG
            // Serial.print(" "); // DEBUG
            // Serial.println(digitalRead(PIN_VALUE_TWO)); // DEBUG
      }
      lastButtonStateTwo = reading;
  }
}
