#include <GyverPID.h>

#define SERIAL_SPEED 9600 // скорость работы последовательного порта
#define READ_SENSOR_INTERVAL 500UL  // периодичность вывода времени в Serial (1 cекунда)
#define LEFT 0
#define RIGHT 1
// 2 что за магические имена переменных?
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENA = 9;
int ENB = 3;

char command = 'S';
char prevCommand = 'A';
int pwmFilling = (4 + 1) * 10 + 100; // коэффициент заполнения ШИМ

unsigned long lostConnectionTimer = 0;    //Stores the time when the last command was received from the phone

long randNumber;
bool stopFlag = false;

// 1 Необходимы пояснения по поводу этих вещей
const int numReadings = 3;

int readings[2][numReadings];
int sensorReading[2] = {0, 0};
int readIndex[2] = {0, 0};
int total[2] = {0, 0};
int average[2] = {0, 0};
int saveP[2] = {0, 0};
int cntSec[2] = {0 ,0};
// 1

const int wheel_diameter = 128;   // Диаметр колеса в мм

float velocity[2] = {0.0, 0.0};
// 1
void clearReading(int sensor) {
    for (int i = 0; i < numReadings; i++) {
        readings[sensor][i] = 0;
    }
}
// 1
void conditionForIncreaseCnsSec(int sensor) {
    if (saveP[sensor] != sensorReading[sensor]) {
        cntSec[sensor] = cntSec[sensor] + 1;
    }
    saveP[sensor] = sensorReading[sensor];
}
// 1
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
    // velocity = rpm * 3.1416 * wheel_diameter * 60 / 1000000;
    return (average[sensor] * (60 / 10)) * 3.1416 * wheel_diameter * 60 / 1000000;
}
void setup() {
    // задаем скорость работы ком-порта
    Serial.begin(SERIAL_SPEED);
    pinMode (ENA, OUTPUT);
    pinMode (IN1, OUTPUT);
    pinMode (IN2, OUTPUT);
    pinMode (ENB, OUTPUT);
    pinMode (IN4, OUTPUT);
    pinMode (IN3, OUTPUT);
    // обнуление массивов для хранения показаний с датчиков
    clearReading(LEFT);
    clearReading(RIGHT);  
}

void loop() {
    sensorReading[LEFT] = digitalRead(A0);
    sensorReading[RIGHT] = digitalRead(A1);

    conditionForIncreaseCnsSec(LEFT);
    conditionForIncreaseCnsSec(RIGHT);

    // периодически выводим millis() в Serial
    static unsigned long prevSensorTime = 0;
    if (millis() - prevSensorTime > READ_SENSOR_INTERVAL) {
        prevSensorTime = millis();

        doSomethingWithAnyFields(LEFT);
        doSomethingWithAnyFields(RIGHT);

        velocity[LEFT] = calculateVelocity(LEFT);
        velocity[RIGHT] = calculateVelocity(RIGHT);

        cntSec[LEFT] = 0;
        cntSec[RIGHT] = 0;

        if (stopFlag == true) {
            Serial.print("0.00:0.00");
        }

        if (stopFlag == false) {
            Serial.print(velocity[LEFT]);
            Serial.print(":");
            Serial.print(velocity[RIGHT]);
        }
    }

    if (Serial.available() > 0) {
        lostConnectionTimer = millis();

        if (prevCommand != 'x') {
            prevCommand = command;
        }
            
        command = Serial.read();
        // Выполняем условие, только если новая команда, отличается от предыдущей

        // для отладки:
        /*Serial.print(command);
        Serial.print(":");
        Serial.println(prevCommand);*/

        if ((command == 'W') || (command == 'A') || (command == 'S') || (command == 'D') || (command == ' ') ) {
            stopFlag = 1;
        }
          
        if (command == ' ') {
            // тут наверное нужен код, который отправляет 0 в виде скорости
            stopFlag = 2;
        }

        if (command != prevCommand) {
            switch (command) {
                case 'W':
                    stopFlag = true;
                    // Вперёд
                    analogWrite(ENA, 0);
                    analogWrite(ENB, 0);
                    delay(20);
                    // Смена направления вращения колёс
                    digitalWrite (IN2, LOW);
                    digitalWrite (IN1, HIGH);

                    digitalWrite (IN4, LOW);
                    digitalWrite (IN3, HIGH);
                     // Установка скважности ШИМ
                    analogWrite(ENA, pwmFilling);
                    analogWrite(ENB, pwmFilling);
                    break;
                case 'A':
                    stopFlag = false;
                    // Налево
                    analogWrite(ENA, 0);
                    analogWrite(ENB, 0);
                    delay(20);
                    // Мотор A
                    digitalWrite (IN2, LOW);
                    digitalWrite (IN1, HIGH);
                    // Мотор B
                    digitalWrite (IN4, HIGH);
                    digitalWrite (IN3, LOW);

                    analogWrite(ENA, pwmFilling);
                    analogWrite(ENB, pwmFilling);

                    break;
                case 'S':
                    stopFlag = false;
                    // Назад
                    analogWrite(ENA, 0);
                    analogWrite(ENB, 0);
                    delay(20);

                    digitalWrite (IN2, HIGH);
                    digitalWrite (IN1, LOW);

                    digitalWrite (IN4, HIGH);
                    digitalWrite (IN3, LOW);

                    analogWrite(ENA, pwmFilling);
                    analogWrite(ENB, pwmFilling);

                    break;
                case 'D':
                    stopFlag = false;
                    // Направо

                    analogWrite(ENA, 0);
                    analogWrite(ENB, 0);
                    delay(20);
                    // A
                    digitalWrite (IN2, HIGH);
                    digitalWrite (IN1, LOW);
                    // B
                    digitalWrite (IN4, LOW);
                    digitalWrite (IN3, HIGH);

                    analogWrite(ENA, pwmFilling);
                    analogWrite(ENB, pwmFilling);

                    break;
                case ' ': // Остановка робота
                    stopFlag = true;
                    //velocity = 0;
                    analogWrite(ENA, 0);
                    analogWrite(ENB, 0);

                    break;
                default:  // Обработка полученного значения мощности
                    stopFlag = false;
                    // Символ '0' - '9' по таблице ASCII имеет код 48 - 57.
                    if ((command >= 48) && (command <= 57)) {
                        // Вычитаем 48 из полученного кода символа и
                        // получаем число в диапазоне от 0 до 9.
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
    } else {
        // Проверка на время отправки последней команды:
        // если прошло более секунды, робот останавливается
        if ((unsigned long)(millis() - lostConnectionTimer) > 20000) {
            analogWrite(ENA, 0);
            analogWrite(ENB, 0);
            prevCommand = 'x';
        }
    }
}
