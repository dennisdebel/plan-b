#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Wifi settings
const char* ssid = "OtaNew";
const char* password = "1234567890";

// UDP variables
WiFiUDP Udp;
unsigned int udpPort = 4000;  // udp port 
byte noteOnBuffer[16] = {0x2F, 0x6D, 0x69, 0x64, 0x69, 0x00, 0x00, 0x00, 0x2C, 0x6D, 0x00, 0x00, 0x00, 0x64, 0x24, 0x90};

// Debugging switches and macros
#define DEBUG // Comment out for no debugging messages

#ifdef DEBUG
#define DEBUG_PRINT(str)    \
    Serial.print(millis());     \
    Serial.print(": ");    \
    Serial.print(__PRETTY_FUNCTION__); \
    Serial.print(' ');      \
    Serial.print(__FILE__);     \
    Serial.print(':');      \
    Serial.print(__LINE__);     \
    Serial.print(' ');      \
    Serial.println(str);
#else
#define DEBUG_PRINT(str)
#endif


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
  WiFi.begin(ssid, password);             // Connect to the network
  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINT(ssid);

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    DEBUG_PRINT(++i); 
    DEBUG_PRINT(' ');
  }
  DEBUG_PRINT('\n');
  DEBUG_PRINT("Connection established!");  
  DEBUG_PRINT("WEMOS Local IP address:\t");
  DEBUG_PRINT(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
  Udp.begin(udpPort);
  //DEBUG_PRINT("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), udpPort);


  pinMode(PIN_VALUE_ONE, INPUT);
  pinMode(PIN_VALUE_TWO, INPUT);

  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);

  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  digitalWrite(PIN_C, LOW);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
  DEBUG_PRINT("\nHello I am alive!");
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

      digitalWrite(PIN_A,b0); // actually set the registers
      digitalWrite(PIN_B,b1); // actually set the registers
      digitalWrite(PIN_C,b2); // actually set the registers

      int reading = digitalRead(PIN_VALUE_ONE); // read the mux IO pin
      
      if(reading == HIGH && lastButtonStateOne == LOW && millis() - lastDebounceTimeOne > debounceDelay)
      {
        lastDebounceTimeOne = millis();
            DEBUG_PRINT("Mux 1 Pin: "); 
            DEBUG_PRINT(buttonValue[buttonCount]);
            DEBUG_PRINT("Pressed!"); 
            // DEBUG_PRINT("Value: "); 
            // DEBUG_PRINT(" "); 
            // DEBUG_PRINT(digitalRead(PIN_VALUE_ONE)); 
            
            // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
            Udp.beginPacket("172.20.10.4", udpPort); 
            Udp.write(noteOnBuffer,16); // send noteOn message
            Udp.endPacket();
      }
      lastButtonStateOne = reading;
  }    
}
  
void muxTwo() {

  for (int buttonCount = 0; buttonCount < 8; buttonCount++) {

      b0 = bitRead(buttonCount,0); // convert buttonCount integer to bits and assign the first bit to the variable b0
      b1 = bitRead(buttonCount,1); // convert buttonCount integer to bits and assign the second bit to the variable b1
      b2 = bitRead(buttonCount,2); // convert buttonCount integer to bits and assign the last bit to the variable b2

      digitalWrite(PIN_A,b0); // actually set the registers
      digitalWrite(PIN_B,b1); // actually set the registers
      digitalWrite(PIN_C,b2); // actually set the registers

      int reading = digitalRead(PIN_VALUE_TWO); // read the mux IO pin

      if(reading == HIGH && lastButtonStateTwo == LOW && millis() - lastDebounceTimeTwo > debounceDelay)
      {
        lastDebounceTimeTwo = millis();
            DEBUG_PRINT("Mux 2 Pin: "); 
            DEBUG_PRINT(buttonValue[buttonCount]);
            DEBUG_PRINT("Pressed!"); 
            // DEBUG_PRINT("Value: "); 
            // DEBUG_PRINT(" "); 
            // DEBUG_PRINT(digitalRead(PIN_VALUE_ONE)); 

            // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
            Udp.beginPacket("172.20.10.4", udpPort); 
            Udp.write(noteOnBuffer,16); // send noteOn message
            Udp.endPacket();
      }
      lastButtonStateTwo = reading;
  }
}
