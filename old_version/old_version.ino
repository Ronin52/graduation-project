#define PID_INTEGER
#include <GyverPID.h>

#define SERIAL_SPEED 9600
#define READ_SENSOR_INTERVAL 500UL
#define LEFT 0
#define RIGHT 1

int wantedSpeed[] = {1,2,3,4,5};

int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENA = 9;
int ENB = 3;

GyverPID leftWheelRegulator;
GyverPID rightWheelRegulator;

char command = 'S';
char prevCommand = 'A';
int pwmFilling = (4 + 1) * 10 + 100;

unsigned long prevSensorTime = 0;
unsigned long lostConnectionTimer = 0;

long randNumber;
bool stopFlag = false;

const int numReadings = 3;

int readings[2][numReadings];
int sensorReading[2] = {0, 0};
int readIndex[2] = {0, 0};
int total[2] = {0, 0};
int average[2] = {0, 0};
int saveP[2] = {0, 0};
int cntSec[2] = {0 ,0};

const int wheelDiametr = 128;//mm

float velocity[2] = {0.0, 0.0};

void clearReading(int sensor) {
    for (int i = 0; i < numReadings; i++) {
        readings[sensor][i] = 0;
    }
}

void conditionForIncreaseCnsSec(int sensor) {
    if (saveP[sensor] != sensorReading[sensor]) {
        cntSec[sensor] = cntSec[sensor] + 1;
    }
    saveP[sensor] = sensorReading[sensor];
}

void doSomethingWithAnyFields(int sensor) {
    total[sensor] = total[sensor] - readings[sensor][readIndex[sensor]];
        readings[sensor][readIndex[sensor]] = cntSec[sensor];
        total[sensor] = total[sensor] + readings[sensor][readIndex[sensor]];
        readIndex[sensor] = readIndex[sensor] + 1;
        if (readIndex[sensor] >= numReadings) {
            readIndex[sensor] = 0;
        }
        average[sensor] = total[sensor] / numReadings;
}

float calculateVelocity(int sensor) {
    // msec = 60 / 10;
    // rpm = average[sensor] * msec;
    // velocity = rpm * 3.1416 * wheelDiametr * 60 / 1000000;
    return (average[sensor] * (60 / 10)) * 3.1416 * wheelDiametr * 60 / 1000000;
}

void stopEngine() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void runEngine(int pwmFilling) {
    analogWrite(ENA, pwmFilling);
    analogWrite(ENB, pwmFilling);
}

void setup() {
    
    Serial.begin(SERIAL_SPEED);
    pinMode (ENA, OUTPUT);
    pinMode (IN1, OUTPUT);
    pinMode (IN2, OUTPUT);
    pinMode (ENB, OUTPUT);
    pinMode (IN4, OUTPUT);
    pinMode (IN3, OUTPUT);

    pinMode (A0, INPUT);
    pinMode (A1, INPUT);
    
    clearReading(LEFT);
    clearReading(RIGHT);

    leftWheelRegulator.setDirection(NORMAL);
    leftWheelRegulator.setLimits(0, 255);

    rightWheelRegulator.setDirection(NORMAL);
    rightWheelRegulator.setLimits(0, 255);
}

void loop() {
  delay(500);
    sensorReading[LEFT] = digitalRead(A0);
    sensorReading[RIGHT] = digitalRead(A1);
    
    conditionForIncreaseCnsSec(LEFT);
    conditionForIncreaseCnsSec(RIGHT);

    if (millis() - prevSensorTime > READ_SENSOR_INTERVAL) {
        prevSensorTime = millis();

        doSomethingWithAnyFields(LEFT);
        doSomethingWithAnyFields(RIGHT);

        velocity[LEFT] = calculateVelocity(LEFT);
        velocity[RIGHT] = calculateVelocity(RIGHT);

        cntSec[LEFT] = 0;
        cntSec[RIGHT] = 0;

        if (stopFlag == true) {
            //Serial.print("0.00:0.00");
        }

        if (stopFlag == false) {
//            Serial.print(velocity[LEFT]);
//            Serial.print(":");
//            Serial.print(velocity[RIGHT]);
        }
    }

   if (Serial.available() > 0) {
        lostConnectionTimer = millis();

        if (prevCommand != 'x') {
            prevCommand = command;
        }
            
        command = Serial.read();

        Serial.println(pwmFilling);
        Serial.println(sensorReading[LEFT]);
        Serial.println(sensorReading[RIGHT]);
        
        if (command != prevCommand) {
            switch (command) {
                case 'W':
                    stopFlag = false;
                    
                    stopEngine();
                    delay(20);
                    
                    digitalWrite (IN2, LOW);
                    digitalWrite (IN1, HIGH);

                    digitalWrite (IN4, LOW);
                    digitalWrite (IN3, HIGH);

                    leftWheelRegulator.input = velocity[LEFT];
                    leftWheelRegulator.setpoint = 2;

                    rightWheelRegulator.input = velocity[RIGHT];
                    rightWheelRegulator.setpoint = 2;
                    
                    analogWrite(ENA, leftWheelRegulator.getResult());
                    analogWrite(ENB, rightWheelRegulator.getResult());
                    break;
                case 'A':
                    stopFlag = false;
                    
                    stopEngine();
                    delay(20);
                    
                    digitalWrite (IN2, LOW);
                    digitalWrite (IN1, HIGH);
                    
                    digitalWrite (IN4, HIGH);
                    digitalWrite (IN3, LOW);

                    runEngine(pwmFilling);

                    break;
                case 'S':
                    stopFlag = false;
                    
                    stopEngine();
                    delay(20);

                    digitalWrite (IN2, HIGH);
                    digitalWrite (IN1, LOW);

                    digitalWrite (IN4, HIGH);
                    digitalWrite (IN3, LOW);

                    leftWheelRegulator.input = sensorReading[LEFT];
                    leftWheelRegulator.setpoint = pwmFilling;

                    rightWheelRegulator.input = sensorReading[RIGHT];
                    rightWheelRegulator.setpoint = pwmFilling;
                    
                    Serial.print("left: ");
                    Serial.println(leftWheelRegulator.getResult());
                    Serial.print("rigth: ");
                    Serial.println(rightWheelRegulator.getResult());
                    
                    analogWrite(ENA, leftWheelRegulator.getResult());
                    analogWrite(ENB, rightWheelRegulator.getResult());
                    break;
                case 'D':
                    stopFlag = false;
                    
                    stopEngine();
                    delay(20);
                    
                    digitalWrite (IN2, HIGH);
                    digitalWrite (IN1, LOW);
                    
                    digitalWrite (IN4, LOW);
                    digitalWrite (IN3, HIGH);

                    runEngine(pwmFilling);
                    break;
                case ' ':
                    stopFlag = true;
                    stopEngine();
                    break;
                default:
                    stopFlag = false;
                    // Символ '0' - '9' по таблице ASCII имеет код 48 - 57.
                    if ((command >= 48) && (command <= 57)) {
                        if (command == 48) {
                            pwmFilling = 0;
                        } else {
                            pwmFilling = (command - 48 + 1) * 10 + 100;
                        }
                        analogWrite(ENA, pwmFilling);
                        analogWrite(ENB, pwmFilling);             
                    }
            }
        }
    }
    Serial.print("left: ");
    Serial.println(leftWheelRegulator.getResult());
    Serial.print("rigth: ");
    Serial.println(rightWheelRegulator.getResult());

//    else {
//        if ((unsigned long)(millis() - lostConnectionTimer) > 20000) {
//            stopEngine();
//            prevCommand = 'x';
//        }
//    }
}
