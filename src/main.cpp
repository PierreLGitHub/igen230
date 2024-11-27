    #include <Arduino.h>

    const int pwmChannel = 0;    // PWM channel (0-15)
    const int pwmFreq = 30000;    // Frequency in Hz 
    const int pwmResolution = 9; // Resolution in bits (2^9 = 512, values from 0 to 511)

    //output pins for the motors
    const int rfPin = 14;
    const int rbPin = 12;
    const int enableR = 26;

    const int lfPin = 13;
    const int lbPin = 2;
    const int enableL = 27;

    const int base_speed = 300;
    const int max_speed = pow(2, pwmResolution) - 1;

    int right_speed = 0;
    int left_speed = 0;

    const int sensor_history = 100;
    const int threshold = 1000;
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
    const int Ki = 0.03;
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
    void recordHistory();
    void printSensor();
    void printData();

    void setup() {
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

        pinMode(analogInPin1, INPUT);
        pinMode(analogInPin2, INPUT);
        pinMode(analogInPin3, INPUT);
        pinMode(analogInPin4, INPUT);
        pinMode(analogInPin5, INPUT);


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
                sensorAvg += sensorWeight[i] * sensor_on;
                sensorSum += sensor_on;

            }
            else {
                sensorSum += sensor_off;
            }
        }

        Serial.println(" ");
        
        if(sensorSum != 0){
            p = (sensorAvg / sensorSum);

        }
        else{
            p = 0;

        }

        intg += p;
        d = p - pp;
        pp = p;

        correction = Kp * p + Ki * intg + Kd * d;

        new_correction = constrain(correction, -max_speed, max_speed);

        printData();

    }

    void motorControl(){
        right_speed = base_speed + new_correction;
        left_speed = base_speed - new_correction;



        digitalWrite(rfPin, HIGH);
        digitalWrite(rbPin, LOW);
        digitalWrite(lfPin, HIGH);
        digitalWrite(lbPin, LOW);
        ledcWrite(enableR, abs(right_speed));
        ledcWrite(enableL, abs(left_speed));

        Serial.print("Right speed: ");
        Serial.print(abs(right_speed));
        
        Serial.print("     Left speed: ");
        Serial.println(abs(left_speed));
        
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
