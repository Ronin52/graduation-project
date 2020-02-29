#define SERIAL_SPEED 9600
#define READ_SENSOR_INTERVAL 500UL
#define LEFT 1
#define RIGHT 0

class PID {
  float setpoint = 0.0;   // заданная величина, которую должен поддерживать регулятор
  float input = 0.0;      // сигнал с датчика (например температура, которую мы регулируем)
  int output = 0;     // выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
  int pidMin = 0;     // минимальный выход с регулятора
  int pidMax = 255;   // максимальный выход с регулятора
  // коэффициенты
  float Kp = 1.0;
  float Ki = 1.0;
  float Kd = 1.0;
  float _dt_s = 1; // время итерации в секундах
  // вспомогательные переменные
  int prevInput = 0;
  float integral = 0.0;
  // ПИД
  // функция расчёта выходного сигнала
  public:
  int compute(float setpoint, float input) {
    float error = setpoint - input;           // ошибка регулирования
    float delta_input = prevInput - input;    // изменение входного сигнала
    prevInput = input;
    output = 0;
    output += (float)error * Kp;                  // пропорционально ошибке регулирования
    output += (float)delta_input * Kd / _dt_s;    // дифференциальная составляющая
    integral += (float)error * Ki * _dt_s;        // расчёт интегральной составляющей
    // тут можно ограничить интегральную составляющую!
    output += integral;                           // прибавляем интегральную составляющую
    output = constrain(output, pidMin, pidMax);   // ограничиваем выход
    return output;
  }
};

int wantedSpeed[] = {1,2,3,4,5};

int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENA = 9;
int ENB = 3;

PID leftWheelRegulator;
PID rightWheelRegulator;

char command = 'S';
char prevCommand = 'A';
int pwmFilling = (4 + 1) * 10 + 100;

unsigned long prevSensorTime = 0;
unsigned long engineTime = 0;

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
float velocity[2] = {0.0, 0.0};

const int wheelDiametr = 128;//mm

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
}

void loop() {
    
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
    }
    if (millis() - engineTime > 1000) {
      engineTime=millis();                
      digitalWrite (IN2, LOW);
      digitalWrite (IN1, HIGH);
  
      digitalWrite (IN4, LOW);
      digitalWrite (IN3, HIGH);
      
      analogWrite(ENA, leftWheelRegulator.compute(2,velocity[LEFT]));
      analogWrite(ENB, rightWheelRegulator.compute(2,velocity[RIGHT]));
                  
      Serial.print("regulator left: ");
      Serial.println(leftWheelRegulator.compute(2,velocity[LEFT]));
      Serial.print("regulator rigth: ");
      Serial.println(rightWheelRegulator.compute(2,velocity[RIGHT]));
      Serial.print("velocity left: ");
      Serial.println(velocity[LEFT]);
      Serial.print("velocity rigth: ");
      Serial.println(velocity[RIGHT]);
    }
}
