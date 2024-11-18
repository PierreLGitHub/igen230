#include <Arduino.h>
const int analogInPin1 = 34; 
const int analogInPin2 = 35;
const int analogInPin3 = 32;
const int analogInPin4 = 33;
const int analogInPin5 = 25;  

int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;
int sensorValue5 = 0;

void setup() {
  Serial.begin(115200);

}

void loop() {
  sensorValue1 = analogRead(analogInPin1);
  sensorValue2 = analogRead(analogInPin2);
  sensorValue3 = analogRead(analogInPin3);
  sensorValue4 = analogRead(analogInPin4);
  sensorValue5 = analogRead(analogInPin5);

  Serial.print("sensor 1 = ");
  Serial.print(sensorValue1);

  Serial.print("   sensor 2 = ");
  Serial.print(sensorValue2);
  
  Serial.print("   sensor 3 = ");
  Serial.print(sensorValue3);

  Serial.print("   sensor 4 = ");
  Serial.print(sensorValue4);

  Serial.print("   sensor 5 = ");
  Serial.println(sensorValue5);

  delay(100);
}

