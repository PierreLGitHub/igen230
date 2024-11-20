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
const int base_speed = 170;
const int max_speed = 255;

int right_speed = 0;
int left_speed = 0;

const int sensor_history = 100;
const int threshold = 2000;
const int sensor_on = 1500;
const int sensor_off = 0;

//input pins for the IR sensors
const int analogInPin1 = 34; 
const int analogInPin2 = 35;
const int analogInPin3 = 32;
const int analogInPin4 = 33;
const int analogInPin5 = 25;

int input[] = {analogInPin1, analogInPin2, analogInPin3, analogInPin4, analogInPin5};
int sensor[] = {0, 0, 0, 0, 0};
int sensorWeight[] = {-2, -1, 0, 1, 2};

const int Kp = 5;
const int Ki = 0;
const int Kd = 40;

int correction;
int new_correction;
int sensorAvg = 0;
int sensorSum = 0;

float p;
float pp;
float intg = 0;
float d;

int sensorHistory[sensor_history][5];

void lineCalc();
void motorControl();
void printSensor();
void printData();
int truncate(int);

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
        
        if(sensor[i] >= threshold){
            sensorAvg += sensorWeight[i] * sensor_on;
            sensorSum += sensor[i];

        }
        else {
            sensorAvg += sensorWeight[i] * sensor_off;
            sensorSum += sensor[i];
        }       
    }

    pp = sensorAvg / sensorSum;

    Serial.print("Intial Correction = ");
    Serial.println(pp);

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

        printSensor();

        if(sensor[i] >= threshold){
            sensorAvg += sensorWeight[i] * sensor_on;
            sensorSum += sensor[i];

        }
        else {
            sensorAvg += sensorWeight[i] * sensor_off;
            sensorSum += sensor[i];
        }
    }

    p = (sensorAvg / sensorSum) * 5;
    intg += p;
    d = p - pp;
    pp = p;

    correction = Kp * p + Ki * intg + Kd * d;

    new_correction = truncate(correction);

    printData();

}

void motorControl(){
    right_speed = base_speed + correction;
    left_speed = base_speed - correction;

    if(new_correction > 0){
        ledcWrite(rfPin, right_speed);
        ledcWrite(lfPin, base_speed);

        Serial.print("Turning right with speed ");
        Serial.println(new_correction);

    }
    else if(new_correction < 0){
        ledcWrite(rfPin, base_speed);
        ledcWrite(lfPin, left_speed);

        Serial.print("Turning left with speed ");
        Serial.println(new_correction);
    }
    else{
        ledcWrite(rfPin, base_speed);
        ledcWrite(lfPin, base_speed);

        Serial.println("Maintaining forward direction, correction = 0");
        
    }

}

int truncate(){
    if(correction > 255){
        correction = 255;

    }
    else if(correction < -255){
        correction = -255;

    }

    return correction;

}

void printData(){
    Serial.print("p = ");
    Serial.print(p);

    Serial.print("  i = ");
    Serial.print(intg);

    Serial.print("  d = ");
    Serial.println(d);

    Serial.print("correction =  ");
    Serial.print(correction);
}
