#include "Adafruit_MAX31855.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// thermopair
int thermoCS1 = 2;
int thermoCLK = 3;
int thermoDO  = 4;
Adafruit_MAX31855 tc(thermoCLK, thermoCS1, thermoDO);

//PID
#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

//LCD address is 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);  

//Rotary Encoder
#include <RotaryEncoder.h>
RotaryEncoder encoder(9, 10);

void setup() {

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // initialize the lcd 
  lcd.init();                      
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("SolderingStation");
  lcd.setCursor(0,1);
  lcd.print("v 0.1");


}

void loop() {

  // Get RotaryEncoder movement
  static int pos = 0;
  encoder.tick();

  int newPos = encoder.getPosition();
  if (pos != newPos) {
    lcd.setCursor(0, 1); 
    lcd.print(newPos);
    lcd.print(" ");
    pos = newPos;
  } // if



}
