#include <Wire.h>
#include "PID_v1.h"
#include "SevSeg.h"
#include "MAX31855.h"
#include "TimerOne.h"
#include "ClickEncoder.h"

// thermopair
const int doPin = 7;
const int csPin = 8;
const int clPin = 9;
MAX31855 tc(clPin, csPin, doPin);
int tcstatus;
double internal, celsius;

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

//PWM and PID
#define PWMPin 6
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 4, 0.2, 1, REVERSE);

// Variables
int newPos = 0;
int pos = 80;
double coolTemp = 80;
double maxTemp = 450;
double setTemperature = coolTemp;
int pwm = 0;

// ISR for ClickEncoder
void rotary() {
  encoder->service();
}

void setup() {

  //turn the PID on
  Setpoint = setTemperature;
  
  //  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetOutputLimits(0, 255);
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

  // Temperature reading
  tc.begin();

  // set PWM pin
  pinMode(PWMPin, OUTPUT);

  // setup serial
  Serial.begin(115200);

  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB
  //  }

}

void loop() {

  // Get RotaryEncoder value
  newPos += encoder->getValue();

  // Prevent rotary values to get outside possible values
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

  // read Temperature
  tcstatus = tc.read();
  celsius = tc.getTemperature();
  Input = celsius;
  Setpoint = setTemperature;

  // Compute PID
  myPID.Compute();

  // Show set temperature in 7 segment display
  sprintf(tempString, "%03d", (int)setTemperature);
  myDisplay.DisplayString(tempString, 4);

  // Write PWM value to MOSFET
  pwm = Output;
  analogWrite(PWMPin, pwm);

  // for debug only
  Serial.print ("Temp: ");
  Serial.println (Input);

}
