#include <Wire.h>
#include "SevSeg.h"
#include "MAX31855.h"
#include "PID_v1.h"
#include "ClickEncoder.h"
#include "TimerOne.h"

// thermopair
const int doPin = 7;
const int csPin = 8;
const int clPin = 9;
MAX31855 tc(clPin, csPin, doPin);
int status;
double internal, celsius;
unsigned long tempTime;
int tempLapse = 1500;

//7Segment 3 digit thing
SevSeg myDisplay;
int displayType = COMMON_ANODE;
char tempString[10];

//This pinout is for a 3361BS (common anode)
int digit1 = 21; //Pin 1
int digit2 = 14; //Pin 10
int digit3 = 10; //Pin 4
int digit4 = 34; //Pin 6 (ignored)

//Declare what pins are connected to the segments (cathodes)
int segA = 19; //Pin 12
int segB = 3; //Pin 11
int segC = 16; //Pin 3
int segD = 18; //Pin 8
int segE = 20; //Pin 2
int segF = 15; //Pin 9
int segG = 2; //Pin 7
int segDP = 34; //Pin 5 (ignored)
int numberOfDigits = 3; //Do you have a 1, 2 or 4 digit display?

//Rotary Encoder
int rotaryA = 0;
int rotaryB = 1;
int rotaryP = 5;
ClickEncoder *encoder;

//Relay and PID
#define RelayPin 6
int WindowSize = 500;
unsigned long windowStartTime;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE);

// Variables
int newPos = 0;
int pos = 80;
double coolTemp = 80;
double maxTemp = 450;
double setTemperature = coolTemp;

// ISR for ClickEncoder
void rotary() {
  encoder->service();
}

void setup() {

  //turn the PID on
  windowStartTime = millis();
  Setpoint = setTemperature;
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

  // initialize 7 segment
  myDisplay.Begin(displayType, numberOfDigits, digit1, digit2, digit3, digit4, segA, segB, segC, segD, segE, segF, segG, segDP);
  myDisplay.SetBrightness(80); //Set the display to 100% brightness level

  // Rotary Encoder pins
  encoder = new ClickEncoder(rotaryA, rotaryB, rotaryP);
  encoder -> setAccelerationEnabled(false);
  encoder -> setDoubleClickEnabled(false);

  // Timer for Rotary Encoder
  Timer1.initialize(1000);
  Timer1.attachInterrupt(rotary);

  // Timer for temperature reading
  tempTime = millis();
  tc.begin();

  // set relay pin
  pinMode(RelayPin, OUTPUT);

  // setup serial
  Serial.begin(115200);

//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB
//  }

}

void loop() {

  // Get RotaryEncoder value
  newPos += encoder->getValue();
  if (newPos < pos) {
    setTemperature = setTemperature - 10;
    if (setTemperature <= coolTemp) {
      setTemperature = coolTemp;
    }
  }
  if (newPos > pos) {
    setTemperature = setTemperature + 10;
    if (setTemperature >= maxTemp) {
      setTemperature = maxTemp;
    }
  }
  pos = newPos;

  if (millis() >= tempTime) {
    tempTime = millis() + tempLapse;

    // read Temperature
    status = tc.read();
    internal = tc.getInternal();
    celsius = tc.getTemperature();
    Input = celsius;
    Setpoint = setTemperature;
  }

  myPID.Compute();

//  sprintf(tempString, "%03d", (int)setTemperature);
  sprintf(tempString, "%03d", (int)Input);
//  sprintf(tempString, "%03d", (int)Output);
  myDisplay.DisplayString(tempString, 4);

  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) {
    digitalWrite(RelayPin, HIGH);
        Serial.print ("Temp: ");
        Serial.print (Input);
        Serial.println(" Relay ON");
  }
  else {
    digitalWrite(RelayPin, LOW);
        Serial.print ("Temp: ");
        Serial.print (Input);
        Serial.println(" Relay OFF");
  }
}
