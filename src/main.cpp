#include <Arduino.h>

const int pwmChannel = 0;    // PWM channel (0-15)
const int pwmFreq = 30000;   // Frequency in Hz
const int pwmResolution = 8; // Resolution in bits (2^12 = 4096, values from 0 to 4095)

// output pins for the motors
const int rfPin = 14;
const int rbPin = 12;
const int enableR = 26;

const int lfPin = 13;
const int lbPin = 2;
const int enableL = 27;

const int base_speed = 165;
const int max_speed = pow(2, pwmResolution) - 1;
const int min_speed = 160;
const int reverse_speed = 160;

const int sensor_history = 8;
const int threshold = 1000;
const int onValue = 2000;
const int offValue = 0;

// input pins for the IR sensors
const int analogInPin1 = 34;
const int analogInPin2 = 35;
const int analogInPin3 = 32;
const int analogInPin4 = 33;
const int analogInPin5 = 25;

const int input[] = {analogInPin1, analogInPin2, analogInPin3, analogInPin4, analogInPin5};

int sensor[] = {0, 0, 0, 0, 0};
int binSensor[] = {0, 0, 0, 0, 0};
int sensorWeight[] = {-2, -1, 0, 1, 2};

const int Kp = 2;
const int Ki = 0.02;
const int Kd = 10;

int last_extreme;
int reverse_direction;
int correction;
int sensorAvg = 0;
int sensorSum = 0;

float p;
float pp = 0;
float intg = 0;
float d;

int sensorHistory[sensor_history][5];

int checkReverse();

void lineCalc();
void reverse();
void motorControl();
void recordHistory();
void printSensor();
void printData();

void setup()
{
    Serial.begin(115200);

    // Configure the PWM channels for right motor forward and backward motion
    pinMode(enableR, OUTPUT);
    pinMode(rfPin, OUTPUT);
    pinMode(rbPin, OUTPUT);
    ledcAttachChannel(enableR, pwmFreq, pwmResolution, pwmChannel); // channel 0

    pinMode(enableL, OUTPUT);
    pinMode(lfPin, OUTPUT);
    pinMode(lbPin, OUTPUT);
    ledcAttachChannel(enableL, pwmFreq, pwmResolution, pwmChannel + 1); // channel 1

    for (int i = 0; i <= 4; i++){
        pinMode(input[i], INPUT);

    }

    for (int j = 0; j <= sensor_history - 1; j++)
    {

        for (int i = 0; i <= 4; i++)
        {
            sensorHistory[j][i] = 1;
        }
    }
}

void loop()
{
    lineCalc();
}

void lineCalc()
{
    sensorAvg = 0;
    sensorSum = 0;

    for (int i = 0; i <= 4; i++)
    {
        sensor[i] = analogRead(input[i]);
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(" = ");
        Serial.print(sensor[i]);
        Serial.print("  ");

        if (sensor[i] >= threshold)
        {
            sensorAvg += sensorWeight[i] * onValue;
            sensorSum += onValue;
            binSensor[i] = 1;
        }

        else
        {
            sensorSum += offValue;
            binSensor[i] = 0;
        }
    }

    Serial.println(" ");

    if (sensorSum != 0)
    {
        p = (sensorAvg / sensorSum);
    }
    else{
        p = 0;
    }

    intg += p;

    d = p - pp;

    correction = Kp * p + Ki * intg + Kd * d;

    recordHistory();


    if (checkReverse() != 0)
    {
        reverse();
    }

    else{
        motorControl();

    }

    //delay(500);

}

void motorControl(){
    int right_speed = 0;
    int left_speed = 0;

    printData();

    if (correction > 0){
        right_speed = constrain(base_speed + correction, min_speed, max_speed);
        left_speed = constrain(base_speed - correction, min_speed, max_speed);

        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, right_speed);
        ledcWrite(enableL, left_speed);

        Serial.print("Left speed: ");
        Serial.print(abs(left_speed));

        Serial.print("      Right speed: ");
        Serial.println(abs(right_speed));
    }

    else if (correction < 0){
        left_speed = constrain(-base_speed + correction, -max_speed, -min_speed);
        right_speed = constrain(-base_speed - correction, -max_speed, -min_speed);

        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, abs(right_speed));
        ledcWrite(enableL, abs(left_speed));

        Serial.print("Left speed: ");
        Serial.print(abs(left_speed));

        Serial.print("      Right speed: ");
        Serial.println(abs(right_speed));
    }

    else{
        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, base_speed);
        ledcWrite(enableL, base_speed);

        Serial.print("Left speed: ");
        Serial.print(base_speed);

        Serial.print("      Right speed: ");
        Serial.println(base_speed);
    }
}

void reverse(){
    int right_reverse;
    int left_reverse;

    right_reverse = reverse_speed + last_extreme * 10;
    left_reverse = -reverse_speed + last_extreme * 10;

    digitalWrite(rfPin, LOW);
    digitalWrite(rbPin, HIGH);
    digitalWrite(lfPin, LOW);
    digitalWrite(lbPin, HIGH);
    ledcWrite(enableR, right_reverse);
    ledcWrite(enableL, abs(left_reverse));

    Serial.println("Reversing...");

}

int checkReverse(){
    int sensors_on = 0;
    bool extreme;

    for (int j = 0; j <= sensor_history - 1; j++){

        for (int i = 0; i <= 4; i++){
            sensors_on += sensorHistory[j][i];

            if(!extreme){
                if (sensorHistory[j][i] = 1){
                    if (i = 0){
                        last_extreme = -1;
                    }

                    if (i = 4){
                        last_extreme = 1;
                    }
                }
            }
        }   
    
    }



    if (sensors_on > 0){

        return 0;
    }

    else{
        return last_extreme;

    }

}

void recordHistory(){
    for (int j = sensor_history - 1; j >= 1; j--){

        for (int i = 0; i <= 4; i++){
            sensorHistory[j][i] = sensorHistory[j - 1][i];
            
        }
    }

    for (int i = 0; i <= 4; i++){
        sensorHistory[0][i] = binSensor[i];
    }
}

void printData(){
    Serial.print("p = ");
    Serial.print(p);

    Serial.print("  i = ");
    Serial.print(intg);

    Serial.print("  d = ");
    Serial.println(d);

    Serial.print("sensor average = ");
    Serial.print(sensorAvg);

    Serial.print("  sensor sum =  ");
    Serial.print(sensorSum);

    Serial.print("  correction =  ");
    Serial.println(correction);
}
