//====== latest code 9jun 20:38 pm ==== nog 1 bug en geen failsafe

// TODO
// [x] wifi reconnect on disconnect (esp native function). No reconnect debug messages to save time/cpu cycles.
// [x] start mode motors on specific midi message (note on/off) via Udp (https://docs.arduino.cc/retired/library-examples/wifi-library/WiFiUdpSendReceiveString/)
// [x] stop mode (when all 16 buttons are pressed "simultaniously" (aka long debounce! 200ms?))
      // https://forum.arduino.cc/t/concatenate-states-of-inputs/1176595 & https://www.reddit.com/r/arduino/comments/4gv2zp/detecting_two_buttons_clicked_at_the_same_time/
      // https://forum.arduino.cc/t/array-content-fastest-means-to-verify-whether-all-variables-contain-an-identical-value/968783/15
      // https://arduino.stackexchange.com/questions/49153/how-can-i-record-a-push-button-sequence-and-store-it-in-an-array 
// [x] init mode: LED goes on when succesfully connect to wifi hotspot
// [x] wait mode: after setup() go into "wait mode", after stop go into wait mode...
// [x] refactor the mux function (future: into a class/struct)
// [ ] refactor the code to use enum and case/switch between wait, stop and start mode.
// [ ] motor does not start again after stop + new start command
// [ ] timing of 16 button presses is TOO crucial, it wont trigger if you sequentially hit the buttons
// [ ] if button 16 ...0 ...


#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <AccelStepper.h>


// Starting State 
bool State; // TODO maybe implement a more desciptive with (enum and case/switch) state machine (https://forum.arduino.cc/t/state-machines-a-short-tutorial/580593)
//#define STEPPER // Only test stepper 
#define FULLMODE // Run full code 

// Hotfix for button 0 (the 'stop' channel..)
unsigned long startTime;
boolean timing;
unsigned long timeOut = 10000; // ten seconds

// Wifi definitions 
#define WIFI // uncomment to enable wifi, comment to disable wifi
const char* ssid = "TP-LINK_E0D9";
const char* password = "13402018667";

// UDP definitions
WiFiUDP Udp;
const char* udpIP = "192.168.69.1"; // IP to send the Midi notes to
unsigned int udpPort = 12101;  // udp port 
char udpPacketBuffer[16]; //buffer to hold incoming packet, same size as midi message
char udpReplyBuffer[] = "acknowledged";       // a string to send back
void udpListen();

// Stepper definitions
const int stepperSpeed = 500; // to reverse directions set to -200 for example
const float stepperMaxSpeed = 1000.0;
//const float stepperAccel = 50.0;
#define HALFSTEP 8

// Stepper pin definitions
#define motorPin1  D5     // IN1 on the ULN2003 driver 1
#define motorPin2  D6     // IN2 on the ULN2003 driver 1
#define motorPin3  D8     // IN3 on the ULN2003 driver 1
#define motorPin4  D7     // IN4 on the ULN2003 driver 1

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

// Button config (user variables)  
int buttonConfig[16][3] = {
  {/* BUTTON NUMBER */ 0,  /* NOTE NUMBER */ 49, /* CHANNEL NUMBER */ 5 }, 
  {/* BUTTON NUMBER */ 1,  /* NOTE NUMBER */ 46, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 2,  /* NOTE NUMBER */ 44, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 3,  /* NOTE NUMBER */ 42, /* CHANNEL NUMBER */ 5 }, 
  {/* BUTTON NUMBER */ 4,  /* NOTE NUMBER */ 39, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 5,  /* NOTE NUMBER */ 37, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 6,  /* NOTE NUMBER */ 34, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 7,  /* NOTE NUMBER */ 32, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 8,  /* NOTE NUMBER */ 30, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 9,  /* NOTE NUMBER */ 27, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 10, /* NOTE NUMBER */ 25, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 11, /* NOTE NUMBER */ 22, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 12, /* NOTE NUMBER */ 20, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 13, /* NOTE NUMBER */ 18, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 14, /* NOTE NUMBER */ 15, /* CHANNEL NUMBER */ 5 },
  {/* BUTTON NUMBER */ 15, /* NOTE NUMBER */ 13, /* CHANNEL NUMBER */ 5 },
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

// MIDI definitions
int noteOnChannel  = 144; //channel 0 noteon
int noteOffChannel = 128; //channel 0 noteoff

// Packet buffer definitions
uint8_t noteOnBuffer[16] = {0x2F, 0x6D, 0x69, 0x64, 0x69, 0x00, 0x00, 0x00, 0x2C, 0x6D, 0x00, 0x00, 0x00, 0x64, 0x24, 0x90};
uint8_t noteOffBuffer[16] = {0x2F, 0x6D, 0x69, 0x64, 0x69, 0x00, 0x00, 0x00, 0x2C, 0x6D, 0x00, 0x00, 0x00, 0x00, 0x24, 0x80};
uint8_t allNotesOff[16] = {0x2F, 0x6D, 0x69, 0x64, 0x69, 0x00, 0x00, 0x00, 0x2C, 0x6D, 0x00, 0x00, 0x00, 0x64, 0x00, 0x94}; // All notes off packet
                    
// Custom functions definitions
void readMux(int pinValue, int length, int* lastState, int* currentState);
int countPressed(int* stateArray, int length);
void muxOneTest();
void muxOne(); 
void muxTwo(); 
void printHex();
//void moveSteppers();

// declare mux pins
const int PIN_VALUE_ONE = D1; // IO read pin mux 1 (COMmon InputOutput pin) // TODO change to..
const int PIN_VALUE_TWO = D2; // IO read pin mux 2 (COMmon InputOutput pin) // TODO change to..
const int PIN_A = D0;
const int PIN_B = D4;
const int PIN_C = D3;                                                       // TODO change to..

// declare button states
int mux0ne_lastState[8] = {1, 1, 1, 1, 1, 1, 1, 1}; // TODO rename 0 to O
int mux0ne_currentState[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // TODO rename 0 to O
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
  // Start serial
  pinMode(LED_BUILTIN, OUTPUT); // TODO this might conflict with the MUX
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off when setup is starting (on the wemos d1 mini board, LOW = HIGH and vice versa)
  Serial.begin(115200);
  delay(3000); // give us some time
  Serial.println(" "); // give us some space
  Serial.println("Entering Setup...");
  Serial.println(" "); // give us some space


  #ifdef WIFI
    WiFi.begin(ssid, password);             // Connect to the network
    Serial.println("Trying to connect to wifi network:");
    Serial.println(ssid);
    Serial.println(" ");
  
    int i = 0; // TODO is this ok here???
    while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
      delay(1000);
      Serial.print("Still trying to connect...");
      Serial.print(++i); 
      Serial.println(" second(s) elapsed"); 
      
    }
    Serial.println('\n');
    Serial.println("Connection established!");  
    Serial.println("WEMOS Local IP address:\t");
    Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
    delay(2000); // give us some time to see the IP address
    WiFi.setAutoReconnect(true); // Auto reconnect on disconnect (https://randomnerdtutorials.com/solved-reconnect-esp8266-nodemcu-to-wifi/)
    WiFi.persistent(true); // Keep alive (https://randomnerdtutorials.com/solved-reconnect-esp8266-nodemcu-to-wifi/)
    Udp.begin(udpPort); // start UDP
  #endif
      
  Serial.println("Setup finished, entering wait mode");
  State = 0; // Start in Wait State
  // start timer for to ignore stop button the first 10 seconds
  startTime = millis();

  // Mux pin settings
  pinMode(PIN_VALUE_ONE, INPUT);
  pinMode(PIN_VALUE_TWO, INPUT);

  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);

  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW); // PIN_B is connected to D4, the built in LED....this turn on the led...
  digitalWrite(PIN_C, LOW);
}

void loop()
{
  #ifdef STEPPER
   stepper.runSpeed();
  #endif

  #ifdef FULLMODE
    if(State == 1){ //if state == 1, start!
      // Serial.print("Starting! | ");
      // Serial.print("State: ");
      // Serial.println(State);
      
      // start timer for to ignore stop button the first 10 seconds
      timing = true;

      stepper.setMaxSpeed(stepperMaxSpeed);
      stepper.setSpeed(stepperSpeed);  
      stepper.enableOutputs(); // Reconnect power to the stepper motors
      stepper.runSpeed();
      readMux(PIN_VALUE_ONE, 0, mux0ne_lastState, mux0ne_currentState);
      readMux(PIN_VALUE_TWO, 8, muxTwo_lastState, muxTwo_currentState);
      
      #ifdef DEGUG
        Serial.print("Mux1 states: ");
        for (int i = 0; i < 8; i++) {
          Serial.print(mux0ne_currentState[i]);
          Serial.print(" ");
        }
        Serial.println(); delay(50);

        Serial.print("Mux2 states: ");
        for (int i = 0; i < 8; i++) {
          Serial.print(muxTwo_currentState[i]);
          Serial.print(" ");
        }
        Serial.println();
      #endif

      // Count presses
      //int muxOnePressed = countPressed(mux0ne_currentState, 8);
      //int muxTwoPressed = countPressed(muxTwo_currentState, 8);
      
      // if button 0 is pressed, stop everything, but ignore this the first 10 seconds of running:
      if (timing == true){
        if (millis() - startTime >= timeOut){ //if current time - startTime is bigger than time out, listen to button 0
          if(mux0ne_currentState[4] == 1){ //this was button 0 but aparently mine is connected to button 4?!?!? CONFIG
            //Serial.println("Two or more buttons pressed in total!");
            Serial.println("Stopping");
            stepper.stop(); // Stop the stepper motors
            stepper.disableOutputs(); // Reconnect power to the stepper motors

            //Send ALL notes off
            Udp.beginPacket(udpIP, udpPort);
            Udp.write(allNotesOff, 16);
            Udp.endPacket();

            State = 0; 
            Udp.begin(udpPort); // start UDP (again) to prevent ghost buffer (prevents buffering multiple start packages) NEW 20.7.2025 
          }
        }
      }
      // if ((muxOnePressed + muxTwoPressed) >= 2) {
      //   Serial.println("Two or more buttons pressed in total!");
      //   Serial.println("Stopping");
      //   stepper.stop(); // Stop the stepper motors
      //   stepper.disableOutputs(); // Reconnect power to the stepper motors

      //   //Send ALL notes off
      //   Udp.beginPacket(udpIP, udpPort);
      //   Udp.write(allNotesOff, 16);
      //   Udp.endPacket();

      //   State = 0; 
      // }

    } else { // Enter WAIT MODE, if state is 0, wait for udp start command (noteOn/noteOff)

      // Serial.println("Waiting for start command..."); // TODO remove
      // delay(100); // this messes with udp receive...duh
      // Serial.print("State: ");
      // Serial.println(State); 
      //Listen for incoming udp packet
      udpListen();
    }
  #endif
}

//buja!!! ---------------------------------------------------  Helper functions ----------------------------------------------------

void printHex(uint8_t num) { //print hex values
  char hexCar[3];
  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}
int countPressed(int* stateArray, int length) {
  int count = 0;
  for (int i = 0; i < length; i++) {
    if (stateArray[i] == 1) count++;
  }
  return count;
}
//--------------------------------------------------- Reading the muxes ---------------------------------------------------
void readMux(
  int valuePin,
  int startIndex,
  int lastState[],
  int currentState[]
) {

  // Reset current state to zero before reading new button states
  for (int i = 0; i < 8; i++) {
    currentState[i] = 0;
  }

  for (int i = 0; i < 8; i++) {
    int muxIndex = startIndex + i;

    b0 = bitRead(i, 0);
    b1 = bitRead(i, 1);
    b2 = bitRead(i, 2);

    digitalWrite(PIN_A, b0);
    digitalWrite(PIN_B, b1);
    digitalWrite(PIN_C, b2);

    int note = buttonConfig[muxIndex][1];
    int channel = buttonConfig[muxIndex][2];

    noteOnBuffer[14]  = note;
    noteOffBuffer[14] = note;
    noteOnBuffer[15]  = channelLUT[channel][1];
    noteOffBuffer[15] = channelLUT[channel][0];

    int reading = digitalRead(valuePin);

    if (reading == LOW && lastState[i] == 0) { // note OFF (so reset state here?)
      lastState[i] = 1; 
      currentState[i] = 1;

      // for (int j = 0; j < sizeof(noteOffBuffer); j++) {
      //   printHex(noteOffBuffer[j]);
      //   Serial.print(" ");
      // }
      // Serial.println();


      Udp.beginPacket(udpIP, udpPort);
      Udp.write(noteOffBuffer, 16);
      Udp.endPacket();
    }

    else if (reading == HIGH && lastState[i] == 1) { // note ON
      Serial.print("Button pressed: ");
      Serial.println(i);
      lastState[i] = 0;
      currentState[i] = 0;  

      //debug TODO needs to go
      // for (int j = 0; j < sizeof(noteOnBuffer); j++) {
      //   printHex(noteOnBuffer[j]);
      //   Serial.print(" ");
      // }
      // Serial.println();

      Udp.beginPacket(udpIP, udpPort);
      Udp.write(noteOnBuffer, 16);
      Udp.endPacket();
    }

    else if (reading == HIGH) {
      currentState[i] = 0;  //  ALSO HANDLE NO CHANGE CASE // TODO THIS DOESNT WORK
    }
  }
  
}

void udpListen(){
  // TODO when two top packets are send, you also need to press stop two times...
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    // read the packet into packetBufffer
    Udp.read(udpPacketBuffer, 16);

  if ((udpPacketBuffer[13] == 0x00) && (udpPacketBuffer[14] == 0x7F) && (udpPacketBuffer[15] == 0x84)){  
    Serial.println("starting (from udp)!");
    State = 1; // set PCB in action mode
    Udp.stop(); // Prevent 'ghost' buffer issues (prevents buffering multiple start packages) NEW 20.7.2025
  }
    
  }
}