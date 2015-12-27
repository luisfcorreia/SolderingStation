#include <Wire.h>
#include "LiquidCrystal_I2C.h"

#include "Adafruit_MAX31855.h"
#include "PID_v1.h"
#include "ClickEncoder.h"
#include "TimerOne.h"

// thermopair
int thermoCS1 = 8;
int thermoCLK = 9;
int thermoDO  = 7;
Adafruit_MAX31855 tc(thermoCLK, thermoCS1, thermoDO);

//LCD address is 0x27 for a 16 chars and 2 line display
int lcdAddress = 0x27;
LiquidCrystal_I2C lcd(lcdAddress, 16, 2);

//Rotary Encoder
int rotaryA = 14;
int rotaryB = 15;
int rotaryP = 16;
ClickEncoder *encoder;

//Relay and PID
#define RelayPin 6
int WindowSize = 500;
unsigned long windowStartTime;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

// Variables
int newPos = 0;
int pos = 30;
int coolTemp = 30;
int maxTemp = 450;
int setTemperature = coolTemp;
String texto = "";

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

  // initialize the lcd
  lcd.init();
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("SolderingStation");
  lcd.setCursor(0, 1);
  lcd.print("v 0.1");

  // Rotary Encoder pins
  encoder = new ClickEncoder(rotaryA, rotaryB, rotaryP);
  encoder -> setAccelerationEnabled(false);
  encoder -> setDoubleClickEnabled(false);

  // Timer for Rotary Encoder
  Timer1.initialize(1000);
  Timer1.attachInterrupt(rotary);

  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  texto = "Soldering Station v0.1";
  Serial.println(texto);

}

void loop() {
  texto = "";

  // Get RotaryEncoder value
  newPos += encoder->getValue();
  if (newPos < pos) {
    setTemperature = setTemperature - 10;
    if (setTemperature <= coolTemp) {
      setTemperature = coolTemp;
    }
    Serial.print("New Temperature ");
    Serial.println(setTemperature);
  }
  if (newPos > pos) {
    setTemperature = setTemperature + 10;
    if (setTemperature >= maxTemp) {
      setTemperature = maxTemp;
    }
    Serial.print("New Temperature ");
    Serial.println(setTemperature);
  }
  pos = newPos;

  // read Temperature
  Input = tc.readCelsius();
  Setpoint = setTemperature;
  
  myPID.Compute();


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
