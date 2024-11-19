#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial ESP_BT;
int incoming;

//input pins for the IR sensors
const int analogInPin1 = 34; 
const int analogInPin2 = 35;
const int analogInPin3 = 32;
const int analogInPin4 = 33;
const int analogInPin5 = 25;  

//values read form the IR sensors
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;
int sensorValue5 = 0;

void setup() {
  Serial.begin(115200);

  ESP_BT.begin("ESP32_Control");

}

void loop() {
     if (ESP_BT.available())
    {
      incoming = ESP_BT.read(); //Read what we receive

      // separate button ID from button value -> button ID is 10, 20, 30, etc, value is 1 or 0
      int button = floor(incoming / 10);
      int value = incoming % 10;

    }
    //reading sensor values
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

