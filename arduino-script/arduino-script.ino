#define PID_INTEGER
#include <GyverPID.h>

#define SERIAL_SPEED 9600
#define READ_SENSOR_INTERVAL 20000
#define LEFT 0
#define RIGHT 1

#define MIN_SPEED 0
#define MAX_SPEED 255

#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3

int encoderCount[2];
int realSpeed[2];

unsigned long encoderTimer = 0;

void setup() {
    pinMode(LEFT_ENCODER, INPUT);
    pinMode(RIGHT_ENCODER, INPUT);
}

void loop() {
    if (millis() - encoderTimer > READ_SENSOR_INTERVAL) {
        encoderTimer = millis();
        writeRealSpeed();
    }
}

void doEncodeLeft(void) {
    static byte pred_e = 0;
    byte e = digitalRead(LEFT_ENCODER);
    if(e != pred_e) {
        pred_e = e;
        encoderCount[LEFT_ENCODER]++;
    }
}

void doEncodeRigth(void) {
    static byte pred_e = 0;
    byte e = digitalRead(RIGHT_ENCODER);
    if(e != pred_e) {
        pred_e = e;
        encoderCount[RIGHT_ENCODER]++;
    }
}

void writeRealSpeed() {
    realSpeed[LEFT_ENCODER] = encoderCount[LEFT_ENCODER];
    encoderCount[LEFT_ENCODER] = 0;
    realSpeed[RIGHT_ENCODER] = encoderCount[RIGHT_ENCODER];
    encoderCount[RIGHT_ENCODER] = 0;
}