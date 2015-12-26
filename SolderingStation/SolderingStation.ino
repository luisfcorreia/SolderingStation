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
