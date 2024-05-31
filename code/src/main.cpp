#include "Arduino.h"

// TODO
// -add debug flag
//


// declare custom functions
void muxOne(); 
void muxTwo(); 

// declare mux pins
const int PIN_VALUE_ONE = D6; // IO read pin mux 1 (COMmon InputOutput pin)
const int PIN_VALUE_TWO = D7; // IO read pin mux 2 (COMmon InputOutput pin)
// const int PIN_INHIBIT_1 =  // maybe you need to connect the inhibit pins to the WEMOS as well, or atleas to ground!
// const int PIN_INHIBIT_2 =  // maybe you need to connect the inhibit pins to the WEMOS as well, or atleas to ground!
const int PIN_A = D1;
const int PIN_B = D2;
const int PIN_C = D5;

int buttonValue[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // debounce
int lastButtonValue[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // debounce
int b0 = 0; // channel storage
int b1 = 0;
int b2 = 0;

void setup()
{
  pinMode(PIN_VALUE_ONE, INPUT);
  pinMode(PIN_VALUE_TWO, INPUT);
  //pinMode(PIN_INH, OUTPUT); // not used as of now
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
  //muxTwo(); // function to loop over mux 2
}

void muxOne() {

for (int buttonCount = 0; buttonCount < 8; buttonCount++) {

    Serial.print(" "); // DEBUG
    Serial.print("Reading mux 1 pin: "); // DEBUG
    Serial.println(buttonCount); // DEBUG
    delay(1000); // DEBUG slow down for debuging

    b0 = bitRead(buttonCount,0); // convert buttonCount integer to bits and assign the first bit to the variable b0
    b1 = bitRead(buttonCount,1); // convert buttonCount integer to bits and assign the second bit to the variable b1
    b2 = bitRead(buttonCount,2); // convert buttonCount integer to bits and assign the last bit to the variable b2

    Serial.print(" "); // DEBUG
    Serial.print("Setting Channel: "); // DEBUG
    Serial.print(bitRead(buttonCount,0)); // DEBUG
    Serial.print(bitRead(buttonCount,1)); // DEBUG
    Serial.print(bitRead(buttonCount,2)); // DEBUG

    digitalWrite(PIN_A,b0);
    digitalWrite(PIN_B,b1);
    digitalWrite(PIN_C,b2);
    buttonValue[buttonCount] = digitalRead(PIN_VALUE_ONE);

    Serial.print(" "); // DEBUG
    Serial.print("Value: "); // DEBUG
    Serial.print(buttonValue[buttonCount] = digitalRead(PIN_VALUE_ONE)); // DEBUG
  


    // if(buttonValue[buttonCount] == LOW && lastButtonValue[buttonCount] == HIGH) {
    //   // do something (send NoteOn)
    // } 

    // if(buttonValue[buttonCount] == HIGH && lastButtonValue[buttonCount] == LOW) {
    //   // do something (send NoteOff) 
    //   }    

    //lastButtonValue[buttonCount] = buttonValue[buttonCount];
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

    buttonValue[buttonCount] = digitalRead(PIN_VALUE_TWO);
    
    // if(buttonValue[buttonCount] == LOW && lastButtonValue[buttonCount] == HIGH) {
    //   // do something (send NoteOn)
    // } 

    // if(buttonValue[buttonCount] == HIGH && lastButtonValue[buttonCount] == LOW) {
    //   // do something (send NoteOff) 
    //   }    
    // lastButtonValue[buttonCount] = buttonValue[buttonCount];
    // }
  
  }
}