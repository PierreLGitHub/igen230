    #include <Arduino.h>

    const int pwmChannel = 0;    // PWM channel (0-15)
    const int pwmFreq = 5000;    // Frequency in Hz 
    const int pwmResolution = 9; // Resolution in bits (2^9 = 512, values from 0 to 511)

    //output pins for the motors
    const int rfPin = 14;
    const int rbPin = 12;
    const int enableR = 26;
    const int lfPin = 13;
    const int lbPin = 2;
    const int enableL = 27;
    const int base_speed = 300;
    const int max_speed = 511;

    int right_speed = 0;
    int left_speed = 0;

    const int sensor_history = 100;
    const int threshold = 2000;
    const int sensor_on = 2000;
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
    int truncate();

    void setup() {
        Serial.begin(115200);

        // Configure the PWM channels for right motor forward and backward motion   
        ledcAttachChannel(enableR, pwmFreq, pwmResolution, pwmChannel); // channel 0
        pinMode(enableR, OUTPUT);
        pinMode(rfPin, OUTPUT);
        pinMode(rbPin, OUTPUT);

        ledcAttachChannel(enableL, pwmFreq, pwmResolution, pwmChannel + 1); // channel 1
        pinMode(enableL, OUTPUT);
        pinMode(lfPin, OUTPUT);
        pinMode(lbPin, OUTPUT);        


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
            Serial.print("Sensor ");
            Serial.print(i + 1);
            Serial.print(" = ");
            Serial.print(sensor[i]);
            Serial.print("  ");

            if(sensor[i] >= threshold){
                sensorAvg += sensorWeight[i] * sensor_on * 100;
                sensorSum += sensor[i];

            }
            else {
                sensorAvg += sensorWeight[i] * sensor_off;
                sensorSum += sensor[i];
            }
        }

        Serial.println(" ");

        p = (sensorAvg / sensorSum);
        intg += p;
        d = p - pp;
        pp = p;

        correction = Kp * p + Ki * intg + Kd * d;

        Serial.print("correction = ");
        Serial.println(correction);

        new_correction = truncate();

        motorControl();

        printData();
        delay(500);

    }

    void motorControl(){
        right_speed = base_speed + new_correction;
        left_speed = base_speed - new_correction;

        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        ledcWrite(enableR, right_speed); 
        Serial.print("Right speed: ");
        Serial.print(right_speed);

        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableL, left_speed);
        Serial.print("     Left speed: ");
        Serial.println(left_speed);
    }

    int truncate(){
        if(correction >= max_speed){
            correction = max_speed;

        }
        else if(correction <= -max_speed){
            correction = -max_speed;

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
