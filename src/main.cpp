#include <Arduino.h>

// declaring trigger output pin and echo input pin
const int trigPin = 16;
const int echoPin = 4;

// declaring PWM channel, frequency and resolution
const int pwmChannel = 0;    // PWM channel (0-15)
const int pwmFreq = 30000;   // Frequency in Hz
const int pwmResolution = 8; // Resolution in bits (2^12 = 4096, values from 0 to 4095)

// declaring output pins and PWM enabling pins for the motors
const int rfPin = 14;
const int rbPin = 12;
const int enableR = 26;

const int lfPin = 13;
const int lbPin = 2;
const int enableL = 27;

// declaring values for how fast the motor will be able to go
// 255 is the maximum speed of the motor, and 160 is the minimum voltage at which the motor will activate
const int base_speed = 167;
const int max_speed = pow(2, pwmResolution) - 1;
const int min_speed = 150;
const int reverse_speed = 210;

// declaring how many previous iterations the ESP will remember, as well as threshold values for the sensors to be on/off
const int sensor_history = 5;
const int threshold = 1000;
const int onValue = 1500;
const int offValue = 0;

// input pins for the IR sensors
const int analogInPin1 = 34;
const int analogInPin2 = 35;
const int analogInPin3 = 32;
const int analogInPin4 = 33;
const int analogInPin5 = 25;

const int input[] = {analogInPin1, analogInPin2, analogInPin3, analogInPin4, analogInPin5};

// declaring the sensor array that will store sensor readings from the current iteration
// declaring the binary sensor array that stores sensor readings as on/off
// declaring the sensor weights for the PID control algorithm
int sensor[] = {0, 0, 0, 0, 0};
int binSensor[] = {0, 0, 0, 0, 0};
int sensorWeight[] = {-2, -1, 0, 1, 2};

// declaring Kp, Ki and Kd values for the PID control algorithm
const int Kp = 2;
const int Ki = 0;
const int Kd = 10;

// integers for storing values while the program is running
int last_extreme = 0;
int reverse_direction;
int correction;
int sensorAvg = 0;
int sensorSum = 0;

// current error, previous error, integral error and derivative error
float p;
float pp = 0;
float intg = 0;
float d;

// declaring sensor history array
int sensorHistory[sensor_history][5];

// declaring functions
bool checkReverse();

void lineCalc();
void reverse();
void motorControl();
void recordHistory();
void printSensor();
void printData();
void findOutermostSensor();
void checkObstacle();
void avoidObstacle();
void motorReset();

void setup()
{
    Serial.begin(115200);

    // initializing trigger and echo pins
    pinMode(trigPin,OUTPUT);
    pinMode(echoPin,INPUT);


    // configuring the PWM channels for right and left motor forward and backward motion
    pinMode(enableR, OUTPUT);
    pinMode(rfPin, OUTPUT);
    pinMode(rbPin, OUTPUT);
    ledcAttachChannel(enableR, pwmFreq, pwmResolution, pwmChannel); // channel 0

    pinMode(enableL, OUTPUT);
    pinMode(lfPin, OUTPUT);
    pinMode(lbPin, OUTPUT);
    ledcAttachChannel(enableL, pwmFreq, pwmResolution, pwmChannel + 1); // channel 1

    // initializes sensor input pins
    for (int i = 0; i <= 4; i++){
        pinMode(input[i], INPUT);

    }

    // fills sensor history array with empty values
    for (int j = 0; j <= sensor_history - 1; j++)
    {

        for (int i = 0; i <= 4; i++)
        {
            sensorHistory[j][i] = 0;
        }
    }
}

// repeats the obstacle checking code and the error calculation algorithm
void loop()
{
    checkObstacle();
    lineCalc();
}

void checkObstacle()
{
    int distance;

    // activates trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // converting ms to cm using the speed of sound
    distance = pulseIn(echoPin, HIGH) / 29/ 2;

    if (distance < 8){
        avoidObstacle();
        Serial.println("Obstacle detected.");
    }

    else{
        Serial.println("No obstacle detected.");

    }

}

// error calculation algorithm
void lineCalc()
{
    sensorAvg = 0;
    sensorSum = 0;

    for (int i = 0; i <= 4; i++)
    {
        // reads and prints sensor values
        sensor[i] = analogRead(input[i]);
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(" = ");
        Serial.print(sensor[i]);
        Serial.print("  ");

        // determines if the sensor is detecting a black line and declares the sensor as on/off
        // calculates the error as the sum of all the binary sensor values multiplied by each sensors weight
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

    // ensures the proportional error is 0 if no sensors are on
    if (sensorSum != 0)
    {
        p = (sensorAvg / sensorSum);
    }
    else{
        p = 0;
    }

    // calculates integral and derivative error
    intg += p;

    d = p - pp;

    // calculates necessary correction
    correction = Kp * p + Ki * intg + Kd * d;

    // records the sensor history and determines most recent extreme sensor that was activated
    recordHistory();
    findOutermostSensor();

    // activates the function to check the if the history array is empty i.e. the line hasn't been detected in the last x iterations
    // if no line has been detected in the last iterations, activates the reverse function
    if (checkReverse())
    {
        reverse();

    }

    else{
        motorControl();

    }

    // delay to make debugging easier
    //delay(500);

}

// calculates motor speeds based on previous error calculations and activates motors with the necessary speeds
void motorControl(){
    int right_speed = 0;
    int left_speed = 0;

    printData();

    // for turning right
    if (correction > 0){
        right_speed = constrain(base_speed + correction, min_speed, max_speed);
        left_speed = constrain(base_speed - correction, min_speed, max_speed);

        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, LOW);
        digitalWrite(lbPin, HIGH);
        ledcWrite(enableR, abs(right_speed));
        ledcWrite(enableL, abs(left_speed));

        Serial.print("Left speed: ");
        Serial.print(abs(left_speed));

        Serial.print("      Right speed: ");
        Serial.println(abs(right_speed));
    }

    // for turning left
    else if (correction < 0){
        left_speed = constrain(-base_speed + correction, -max_speed, -min_speed);
        right_speed = constrain(-base_speed - correction, -max_speed, -min_speed);

        digitalWrite(rfPin, LOW);
        digitalWrite(rbPin, HIGH);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, abs(right_speed));
        ledcWrite(enableL, abs(left_speed));

        Serial.print("Left speed: ");
        Serial.print(abs(left_speed));

        Serial.print("      Right speed: ");
        Serial.println(abs(right_speed));
    }

    // for going straight ahead
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

    delay(200);

    motorReset();

}

// reverses back onto line while correcting the reverse speeds depending on which side of the line the robot is on
void reverse(){
    int right_reverse;
    int left_reverse;

    right_reverse = reverse_speed;
    left_reverse = reverse_speed;

    digitalWrite(rfPin, LOW);
    digitalWrite(rbPin, HIGH);
    digitalWrite(lfPin, LOW);
    digitalWrite(lbPin, HIGH);
    ledcWrite(enableR, right_reverse);
    ledcWrite(enableL, left_reverse + 10);

    Serial.println("Reversing...");

    delay(10);
}

// parses through the sensor history array and checks if the line has been detected in the last x iterations
bool checkReverse(){
    int sensors_on = 0;
    bool extreme = false;

    for (int j = 0; j <= sensor_history - 1; j++){

        for (int i = 0; i <= 4; i++){
            sensors_on += sensorHistory[j][i];

        }   
    
    }



    if (sensors_on > 0){

        return false;
    }

    else{
        
        return true;

    }

}

// uses the sensor history array to determine which side of the line the robot is on when the robot is no longer detecting the line
void findOutermostSensor() {
    bool extremeFound = false;
    
    for (int i = 0; i < sensor_history; i++) {
        
        if (sensorHistory[i][0] == 1) {
            last_extreme = -1; // Leftmost sensor activated
            extremeFound = true;
            break; 
        }

        if (sensorHistory[i][4] == 1) {
            last_extreme = 1; // Rightmost sensor activated
            extremeFound = true;
            break;
        }
    }
}

// records history using on/off values for sensor activation state
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

// activates motors to avoid obstacle
void avoidObstacle(){
        digitalWrite(rfPin, LOW);
        digitalWrite(rbPin, HIGH);
        digitalWrite(lfPin, LOW);
        digitalWrite(lbPin, HIGH);
        ledcWrite(enableR, base_speed + 30);
        ledcWrite(enableL, base_speed + 30);

        delay(100);

        digitalWrite(rfPin, LOW);
        digitalWrite(rbPin, HIGH);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, base_speed + 30);
        ledcWrite(enableL, base_speed + 30);

        delay(200);

        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, base_speed + 30);
        ledcWrite(enableL, base_speed + 30);

        delay(700);

        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, LOW);
        digitalWrite(lbPin, HIGH);
        ledcWrite(enableR, base_speed + 30);
        ledcWrite(enableL, base_speed + 30);

        delay(400);

        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, base_speed);
        ledcWrite(enableL, base_speed);

        delay(2000);

        digitalWrite(rfPin, LOW);
        digitalWrite(rbPin, HIGH);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, base_speed);
        ledcWrite(enableL, base_speed);

        delay(200);

}

// prints values used in calculating correction for debugging purposes
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

void motorReset(){
        digitalWrite(rfPin, LOW);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, LOW);
        digitalWrite(lbPin, LOW);

        delay(300);

}
