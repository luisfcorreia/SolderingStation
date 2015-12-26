#include <Wire.h>
#include "Adafruit_MAX31855.h"
#include "LiquidCrystal_I2C.h"
#include "PID_v1.h"
#include "RotaryEncoder.h"

// thermopair
int thermoCS1 = 2;
int thermoCLK = 3;
int thermoDO  = 4;
Adafruit_MAX31855 tc(thermoCLK, thermoCS1, thermoDO);

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

//LCD address is 0x27 for a 16 chars and 2 line display
int lcdAddress = 0x27;
LiquidCrystal_I2C lcd(lcdAddress,16,2);  

//Rotary Encoder
int rotaryA = 9;
int rotaryB = 10;
RotaryEncoder encoder(rotaryA, rotaryB);

// Variables
static int pos = 0;
int newPos = 0;
int setTemperature = 25;

void setup() {

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // initialize the lcd 
  lcd.init();                      
  lcd.backlight();
  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("SolderingStation");
  lcd.setCursor(0,1);
  lcd.print("v 0.1");

  // pullup for Rotary Encoder pins
  pinMode(rotaryA,INPUT_PULLUP);
  pinMode(rotaryB,INPUT_PULLUP);
  
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("Soldering Station v0.1");


}

void loop() {

  // Get RotaryEncoder movement
  // This sets new temperature setpoint
  encoder.tick();
  newPos = encoder.getPosition();
  if (newPos < pos){
    setTemperature++;
    pos = newPos;
  }
  if (newPos > pos){
    if (setTemperature >= 25){
      setTemperature--;
    }
    pos = newPos;
  }
  Serial.println(setTemperature);
  
}
