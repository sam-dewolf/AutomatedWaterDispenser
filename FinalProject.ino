#include <Wire.h>
#include "Adafruit_VCNL4010.h"
#include <Adafruit_LiquidCrystal.h>
#include <EEPROM.h>

/*
 * 1 gallon is 128 fl oz
 * normal water flow is 200g/h in a 1/2" pipe
 * This is .055555 g/s
 * This is 7.11111 oz/s
 */

#define gate 21 //port 21
#define range 2300 //distance for proximity sensor(4.5-5 in)
#define flow 7.111111111111111
#define bottleSize 16.9 //oz
#define button 26 //A0
#define led 25 //A1
#define rs 13
#define en 12
#define d4 27
#define d5 33
#define d6 15
#define d7 32
#define EEPROM_SIZE 1

Adafruit_VCNL4010 vcnl;
uint64_t startMillis;
uint64_t currentMillis;
float ozSaved;
float bottlesSaved;
Adafruit_LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("VCNL4010 test");
  pinMode(gate, OUTPUT); //sets MOSFET connection pin to output
  pinMode(button, INPUT_PULLUP); //sets on/off button connection pin to input with pullup
  pinMode(led, OUTPUT); //sets on/off button connection pin to output to power it
  lcd.begin(16, 2); //starts lcd connection
  lcd.clear();
  lcd.setCursor(1,0); //centers text
  lcd.print("Bottles Saved:");

  if (! vcnl.begin()){ //waits until connected to the proximity sensor
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Found VCNL4010");
  digitalWrite(gate, LOW); //sets MOSFET to off
  ozSaved = EEPROM.read(0); //initializes saved volume to value saved in flash memory
  ozSaved *= (flow*100); //corrects it from bottles saved to ozSaved
  bottlesSaved = ozSaved/bottleSize;
  String bottles = (String) bottlesSaved;
  lcd.setCursor(6,1); //sets cursor to second line
  lcd.print(bottles);
}

//opens solenoid valve and gets start time
void opener(){
  startMillis = millis();
  digitalWrite(gate, HIGH);
}

//closes solenoid valve and calculates bottles used based on time open
void closer(){
  currentMillis = millis();
  uint64_t tmp = currentMillis-startMillis;
  ozSaved = ozSaved + ((float(tmp)/100.0)/flow);
  lcd.setCursor(6,1); //sets cursor to second line
  bottlesSaved = ozSaved/bottleSize;
  EEPROM.write(0, trunc(bottlesSaved));
  String bottles = (String) bottlesSaved;
  lcd.print(bottles);
  EEPROM.commit();
  digitalWrite(gate, LOW);
}


void loop() {
  //if button is pressed and light is off, turn on light.
  if(digitalRead(button) == LOW){
    if(digitalRead(led) == LOW){
      digitalWrite(led, HIGH);
    }
  //button is not pressed, so turn off light if it's on
  }else if(digitalRead(led) == HIGH){
    digitalWrite(led, LOW);
  }
  uint16_t prox = vcnl.readProximity(); //gets proximity reading from sensor
  //if button is pressed and valve is closed, call opener
  if(digitalRead(button) == LOW){
    if (digitalRead(gate) == LOW){
      opener();
    }
  //button is not pressed, so take proximity readings and gate status into consideration
  }else {
    //if proximity reading is within range and valve is closed, call opener
    if(prox>range and digitalRead(gate) == LOW){
      opener();
    //if proximity reading is out of range and valve is open, call closer
    }else if(prox<range and digitalRead(gate) == HIGH){
      closer();
    }
  }
}
