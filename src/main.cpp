#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

const int pwmChannel = 0;    // PWM channel (0-15)
const int pwmFreq = 5000;    // Frequency in Hz 
const int pwmResolution = 8; // Resolution in bits (2^8 = 256, values from 0 to 255)

//output pins for the motors
const int rfPin = 26;
const int rbPin = 27;
const int lfPin = 14;
const int lbPin = 12;


//input pins for the IR sensors
const int analogInPin1 = 34; 
const int analogInPin2 = 35;
const int analogInPin3 = 32;
const int analogInPin4 = 33;
const int analogInPin5 = 25;

int input[] = {analogInPin1, analogInPin2, analogInPin3, analogInPin4, analogInPin5};
int sensor[] = {0, 0, 0, 0, 0};
int sensorWeight[] = {-2, -1, 0, 1, 2};

int sensorAvg = 0;
int sensorSum = 0;

//values read form the IR sensors
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;
int sensorValue5 = 0;

void lineCalc();
void motorControl();

void setup() {
    Serial.begin(115200);

    // Configure the PWM channels for right motor forward and backward motion
    ledcSetup(pwmChannel, pwmFreq, pwmResolution); // channel 0
    ledcAttachPin(rfPin, pwmChannel);

    ledcSetup(pwmChannel + 1, pwmFreq, pwmResolution); // channel 1
    ledcAttachPin(rbPin, pwmChannel + 1);

    // Configure the PWM channels for left motor forward and backward motion
    ledcSetup(pwmChannel + 2, pwmFreq, pwmResolution); // channel 2
    ledcAttachPin(lfPin, pwmChannel + 2);

    ledcSetup(pwmChannel + 3, pwmFreq, pwmResolution); // channel 3
    ledcAttachPin(lbPin, pwmChannel + 3);


    pinMode(analogInPin1, INPUT);
    pinMode(analogInPin2, INPUT);
    pinMode(analogInPin3, INPUT);
    pinMode(analogInPin4, INPUT);
    pinMode(analogInPin5, INPUT);

    sensorAvg = 0;
    sensorSum = 0;

    for (int i = 0; i <= 4; i++){
        sensor[i] = analogRead(input[i]);
        sensorAvg += sensorWeight[i] * sensor[i] * 1000;
        sensorSum += sensor[i];
        
    }

}

void loop() {
    lineCalc();
    motorControl();

}

void lineCalc(){
    sensorAvg = 0;
    sensorSum = 0;

    for (int i = 0; i <= 4; i++){
        sensor[i] = analogRead(input[i]);
        sensorAvg += sensorWeight[i] * sensor[i] * 1000;
        sensorSum += sensor[i];

    }

}

void motorControl(){


}
