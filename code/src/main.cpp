#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Debugging switches and macros
#define DEBUG // Comment out for no debugging messages
//#define WIFI // enable wifi or not

// Wifi settings 
const char* ssid = "OtaNew";
const char* password = "1234567890";

// UDP variables
WiFiUDP Udp;
const char* udpIP = "172.20.10.4";
unsigned int udpPort = 4000;  // udp port 

// Button config (user variables)
int buttonConfig[16][3] = {
  {/* BUTTON NUMBER */ 0,  /* NOTE NUMBER */ 36, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 1,  /* NOTE NUMBER */ 37, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 2,  /* NOTE NUMBER */ 38, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 3,  /* NOTE NUMBER */ 39, /* CHANNEL NUMBER */ 3 },
  {/* BUTTON NUMBER */ 4,  /* NOTE NUMBER */ 40, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 5,  /* NOTE NUMBER */ 41, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 6,  /* NOTE NUMBER */ 42, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 7,  /* NOTE NUMBER */ 43, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 8,  /* NOTE NUMBER */ 44, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 9,  /* NOTE NUMBER */ 45, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 10, /* NOTE NUMBER */ 46, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 11, /* NOTE NUMBER */ 47, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 12, /* NOTE NUMBER */ 48, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 13, /* NOTE NUMBER */ 49, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 14, /* NOTE NUMBER */ 50, /* CHANNEL NUMBER */ 1 },
  {/* BUTTON NUMBER */ 15, /* NOTE NUMBER */ 51, /* CHANNEL NUMBER */ 1 },
};

int channelLUT[17][2] = { // noteOff/noteOn Channel look up table. Format: [0][0] = noteOff channel 1, [0][1] noteOn channel 1
  {0,0},      // bogus entry to subtly offset the look up table by one, since there is no midi channel 0
  {128,144},  // Midi Channel 1 Note Off, Note On
  {129,145},
  {130,146},
  {131,147},
  {132,148},
  {133,149},
  {134,150},
  {135,151},
  {136,152},
  {137,153},
  {138,154},
  {139,155},
  {140,156},
  {141,157},
  {142,158},
  {143,159},
};

// MIDI settings
int noteOnChannel  = 144; //channel 0 noteon
int noteOffChannel = 128; //channel 0 noteoff

// Packet buffer
uint8_t noteOnBuffer[16] = {0x2F, 0x6D, 0x69, 0x64, 0x69, 0x00, 0x00, 0x00, 0x2C, 0x6D, 0x00, 0x00, 0x00, 0x64, 0x24, 0x90};
uint8_t noteOffBuffer[16] = {0x2F, 0x6D, 0x69, 0x64, 0x69, 0x00, 0x00, 0x00, 0x2C, 0x6D, 0x00, 0x00, 0x00, 0x00, 0x24, 0x80};

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
void muxOneTest();
void muxOne(); 
void muxTwo(); 
void printHex();

// declare mux pins
const int PIN_VALUE_ONE = D6; // IO read pin mux 1 (COMmon InputOutput pin)
const int PIN_VALUE_TWO = D7; // IO read pin mux 2 (COMmon InputOutput pin)
const int PIN_A = D1;
const int PIN_B = D2;
const int PIN_C = D5;

// declare button states
int mux0ne_lastState[8] = {1, 1, 1, 1, 1, 1, 1, 1};
int mux0ne_currentState[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int muxTwo_lastState[8] = {1, 1, 1, 1, 1, 1, 1, 1};
int muxTwo_currentState[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int buttonValue[8] = {0,1,2,3,4,5,6,7}; // array/counter for pin numbers // TODO remove this

// TODO currently no debounce
int lastButtonStateOne = LOW;           // the previous reading from the input pin, mux one
unsigned long lastDebounceTimeOne = 0;  // the last time the output pin was toggled, mux one
int lastButtonStateTwo = LOW;           // the previous reading from the input pin, mux two
unsigned long lastDebounceTimeTwo = 0;  // the last time the output pin was toggled, mux two
unsigned long debounceDelay = 200;      // the debounce time; increase if you get double presses, or place a 100nF capacitor (or larger) from the input pin to ground. Note that this requires a 10K (or larger) resistance in series with the button circuit in order for the capacitor to charge/discharge.

int b0 = 0; // channel storage
int b1 = 0;
int b2 = 0;

void setup()
{

  // debugging LED 
  #ifdef DEBUG
    pinMode(LED_BUILTIN, OUTPUT);
  #endif

  #ifdef WIFI
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
  #endif
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn off debug LED at start of code


  //   int baseNoteOn = 108;
  // int baseNoteOff = 92;
  // for(int i=0;i<256;i++){
  //   Serial.print("{");   
  //   Serial.print(i);
  //   Serial.print(",");
  //   Serial.print(i+baseNoteOn);  
  //   Serial.println("},");  
  //   Serial.print("{"); 
  //   Serial.print(i);
  //   Serial.print(",");
  //   Serial.print(i+baseNoteOff); 
  //   Serial.println("}");
  // };
}


//---------------------------------------------------  Main loop -----------------------------------------------------------
void loop()
{


// int buttonConfig[16][3] = {
// {/* BUTTON NUMBER */ 0, /* NOTE NUMBER */ 36, /* CHANNEL NUMBER */ 0 },
// };
// Serial.print("Note number: ");
// Serial.print(buttonConfig[0][1]); //TODO in mux loop buttonConfig[ COUNTER 0-15 ][1]);
// Serial.print(" , Note ON: ");
// Serial.print(channelLUT[buttonConfig[0][1]-36][1]); //substract base number of 36....
// Serial.print(" , Note OFF: ");
// Serial.println(channelLUT[buttonConfig[0][1]-36][0]); //substract base number of 36....
// delay(100);
  

  muxOne(); // function to loop over mux 1 TODO mux must buttonvalue count 0 < 8
  muxTwo(); // function to loop over mux 2 TODO mux must buttonvlaue count 8 < 16
}

//---------------------------------------------------  Helper functions ----------------------------------------------------

void printHex(uint8_t num) { //print hex values
  char hexCar[3];
  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

//--------------------------------------------------- Reading the muxes ---------------------------------------------------
void muxOne() {

  for (int buttonCount = 0; buttonCount < 8; buttonCount++) {
      #ifdef DEBUG
        delay(20); // debug delay for readability
      #endif

      b0 = bitRead(buttonCount,0); // convert buttonCount integer to bits and assign the first bit to the variable b0
      b1 = bitRead(buttonCount,1); // convert buttonCount integer to bits and assign the second bit to the variable b1
      b2 = bitRead(buttonCount,2); // convert buttonCount integer to bits and assign the last bit to the variable b2

      digitalWrite(PIN_A,b0); // actually set the registers
      digitalWrite(PIN_B,b1); // actually set the registers
      digitalWrite(PIN_C,b2); // actually set the registers

      int noteMuxOne = buttonConfig[buttonCount][1]; // Look up the note number
      noteOnBuffer[14] = noteMuxOne; // modify position 14 (0-15) of the noteOnBuffer to the currently pressed button
      noteOffBuffer[14] = noteMuxOne; // modify noteOffBuffer to turn off the played note
      noteOnBuffer[15]  = channelLUT[buttonConfig[buttonCount][1]-36][1]; // Set the Note on + Channel
      noteOffBuffer[15] = channelLUT[buttonConfig[buttonCount][2]][0]; // Set the Note off + Channel   

      int reading = digitalRead(PIN_VALUE_ONE); // read the mux IO pin

      // read the state of the pushbutton and set a flag if it is low:
        if (reading == LOW && mux0ne_lastState[buttonCount] == 0)  {
            mux0ne_lastState[buttonCount] = 1;
            mux0ne_currentState[buttonCount] = 1;
            DEBUG_PRINT("button:"); DEBUG_PRINT(buttonCount); DEBUG_PRINT("released"); 
            //Send noteOff here
            DEBUG_PRINT("Midi Note:"); 
            DEBUG_PRINT(noteOffBuffer[14]); 
            DEBUG_PRINT("Channel:"); 
            DEBUG_PRINT(noteOffBuffer[15]); 
            DEBUG_PRINT("Send note OFF"); 
            // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
            Udp.beginPacket(udpIP, udpPort); 
            Udp.write(noteOffBuffer,16); // send noteOff message
            Udp.endPacket();
            digitalWrite(LED_BUILTIN, HIGH); // debug LED
        }

        // This if statement will only fire on the rising edge of the button input
        if (reading == HIGH && mux0ne_lastState[buttonCount] == 1)  {
            // reset the button low flag
            mux0ne_lastState[buttonCount] = 0;
            DEBUG_PRINT("Button:"); DEBUG_PRINT(buttonCount); DEBUG_PRINT("Pressed");
            DEBUG_PRINT("Note:"); DEBUG_PRINT(noteMuxOne); DEBUG_PRINT("send!");  
            // Send noteOn here:
            DEBUG_PRINT("Packet send: "); 
            for(int i=0; i<sizeof(noteOnBuffer); i++){ //debug the note on message
              printHex(noteOnBuffer[i]);
            }
            // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
            Udp.beginPacket(udpIP, udpPort); 
            Udp.write(noteOnBuffer,16); // send noteOn message
            Udp.endPacket();
            #ifdef DEBUG
              digitalWrite(LED_BUILTIN, LOW); // debug LED
            #endif
        }
  }    
}

// void muxOne() {

//   for (int buttonCount = 0; buttonCount < 8; buttonCount++) {
//       #ifdef DEBUG
//         delay(20); // debug delay for readability
//       #endif

//       b0 = bitRead(buttonCount,0); // convert buttonCount integer to bits and assign the first bit to the variable b0
//       b1 = bitRead(buttonCount,1); // convert buttonCount integer to bits and assign the second bit to the variable b1
//       b2 = bitRead(buttonCount,2); // convert buttonCount integer to bits and assign the last bit to the variable b2

//       digitalWrite(PIN_A,b0); // actually set the registers
//       digitalWrite(PIN_B,b1); // actually set the registers
//       digitalWrite(PIN_C,b2); // actually set the registers

//       int noteMuxOne = buttonValue[buttonCount]+36; // offset button count to represent note numbers, TODO make more flexible (lookuptable)
//       noteOnBuffer[14]= noteMuxOne; // modify position 14 (0-15) of the noteOnBuffer to the currently pressed button TODO: add note off
//       noteOffBuffer[14] = noteMuxOne; // modify noteOffBuffer to turn off the played note
//       noteOnBuffer[15]  = noteOnChannel; // Set the Note on + Channel
//       noteOffBuffer[15] = noteOffChannel; // Set the Note off + Channel

//       int reading = digitalRead(PIN_VALUE_ONE); // read the mux IO pin

//       // read the state of the pushbutton and set a flag if it is low:
//         if (reading == LOW && mux0ne_lastState[buttonCount] == 0)  {
//             mux0ne_lastState[buttonCount] = 1;
//             mux0ne_currentState[buttonCount] = 1;
//             DEBUG_PRINT("button:"); DEBUG_PRINT(buttonCount); DEBUG_PRINT("released"); 
//             //Send noteOff here
//             DEBUG_PRINT("Send note OFF"); 
//             // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
//             Udp.beginPacket(udpIP, udpPort); 
//             Udp.write(noteOffBuffer,16); // send noteOff message
//             Udp.endPacket();
//             digitalWrite(LED_BUILTIN, HIGH); // debug LED
//         }

//         // This if statement will only fire on the rising edge of the button input
//         if (reading == HIGH && mux0ne_lastState[buttonCount] == 1)  {
//             // reset the button low flag
//             mux0ne_lastState[buttonCount] = 0;
//             DEBUG_PRINT("button:"); DEBUG_PRINT(buttonCount); DEBUG_PRINT("pressed"); 
//             // Send noteOn here:
//             DEBUG_PRINT("Packet send: "); 
//             for(int i=0; i<sizeof(noteOnBuffer); i++){ //debug the note on message
//               printHex(noteOnBuffer[i]);
//             }
//             // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
//             Udp.beginPacket(udpIP, udpPort); 
//             Udp.write(noteOnBuffer,16); // send noteOn message
//             Udp.endPacket();
//             #ifdef DEBUG
//               digitalWrite(LED_BUILTIN, LOW); // debug LED
//             #endif
//         }
//   }    
// }
  
void muxTwo() {
   for (int buttonCount = 0; buttonCount < 8; buttonCount++) {  //   for (int buttonCount = 7; buttonCount < 15; buttonCount++) {
      #ifdef DEBUG
        delay(20); // debug delay for readability
      #endif

      b0 = bitRead(buttonCount,0); // convert buttonCount integer to bits and assign the first bit to the variable b0
      b1 = bitRead(buttonCount,1); // convert buttonCount integer to bits and assign the second bit to the variable b1
      b2 = bitRead(buttonCount,2); // convert buttonCount integer to bits and assign the last bit to the variable b2

      digitalWrite(PIN_A,b0); // actually set the registers
      digitalWrite(PIN_B,b1); // actually set the registers
      digitalWrite(PIN_C,b2); // actually set the registers

      int noteMuxTwo = buttonConfig[buttonCount+7][1]; // Look up the note number
      noteOnBuffer[14]= noteMuxTwo; // modify position 14 (0-15) of the noteOnBuffer to the currently pressed button
      noteOffBuffer[14] = noteMuxTwo; // modify noteOffBuffer to turn off the played note
      noteOnBuffer[15]  = channelLUT[buttonConfig[buttonCount+7][1]-36][1]; // Set the Note on + Channel
      noteOffBuffer[15] = channelLUT[buttonConfig[buttonCount+7][2]][0]; // Set the Note off + Channel   // channelLUT[buttonConfig[note][2]][0]


      int reading = digitalRead(PIN_VALUE_TWO); // read the mux IO pin

      // read the state of the pushbutton and set a flag if it is low:
        if (reading == LOW && muxTwo_lastState[buttonCount] == 0)  {
            muxTwo_lastState[buttonCount] = 1;
            muxTwo_currentState[buttonCount] = 1;
            DEBUG_PRINT("button:"); DEBUG_PRINT(buttonCount); DEBUG_PRINT("released"); 
            //Send noteOff here
            DEBUG_PRINT("Midi Note:"); 
            DEBUG_PRINT(noteOffBuffer[14]); 
            DEBUG_PRINT("Channel:"); 
            DEBUG_PRINT(noteOffBuffer[15]); 
            DEBUG_PRINT("Send note OFF"); 
            // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
            Udp.beginPacket(udpIP, udpPort); 
            Udp.write(noteOffBuffer,16); // send noteOff message
            Udp.endPacket();
            digitalWrite(LED_BUILTIN, HIGH); // debug LED
        }

        // This if statement will only fire on the rising edge of the button input
        if (reading == HIGH && muxTwo_lastState[buttonCount] == 1)  {
            // reset the button low flag
            muxTwo_lastState[buttonCount] = 0;
            DEBUG_PRINT("button:"); DEBUG_PRINT(buttonCount); DEBUG_PRINT("pressed"); 
            // Send noteOn here:
            DEBUG_PRINT("Packet send: "); 
            for(int i=0; i<sizeof(noteOnBuffer); i++){ //debug the note on message
              printHex(noteOnBuffer[i]);
            }
            // send udp packet to the IP address (broadcast ip 255.255.255.255 doesnt seem to work in my setup)
            Udp.beginPacket(udpIP, udpPort); 
            Udp.write(noteOnBuffer,16); // send noteOn message
            Udp.endPacket();
            #ifdef DEBUG
              digitalWrite(LED_BUILTIN, LOW); // debug LED
            #endif
        }
  }    
}

