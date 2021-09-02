#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeOrion.h>
#include "math.h"
#include "GyverTimer.h"
#include <BlynkSimpleSerialBLE.h>
//#include <TroykaIMU.h>

#define BLYNK_USE_DIRECT_CONNECT
#define BALANCING_LIMITS 40
#define BLACK_LEFT_LINE_S 480 // Сырые значения с датчика линии
#define WHITE_LEFT_LINE_S 45
#define BLACK_RIGHT_LINE_S 480
#define WHITE_RIGHT_LINE_S 45
#define MAX_VAL_POT 976
#define N_MEASURE_SPEED 15
#define N_MEASURE_ULTRASONIC 15
//#define K 0.98 // Коэффициент комплементарного фильтра

#define AUTH "0smfD4CC7z2KMFQyonHKAnzWH5nbtoAT"

//Accelerometer accel; Gyroscope gyro;
MeGyro gyroAccel;
MeUltrasonicSensor ultrasonic(3);
MePotentiometer potentiometer(8);
MePort lineSensors(PORT_7);
Me7SegmentDisplay seg7(4);
MeEncoderMotor lMotor(0x09, 1), rMotor(0x09, 2);

GTimer_ms myTimer1(10), myTimer2(10);
SoftwareSerial DebugSerial(0, 1); // RX, TX

#define BLYNK_PRINT DebugSerial

//float accelY, accelZ, accAngle, gyroX, gyroAngle;
float cfAngle, cfAngle_old, angleY;
float potS, oldValPotS;
bool isBalanced = false, showBalanceSetPoint = false;

float vKp = 0, vKi = 0.0001, vKd = 0, vKpOld = 0, vKdOld = 0, vKiOld = 0;
float bKp = 17, bKi = 0.15, bKd = 400, bKpOld = 0, bKdOld = 0, bKiOld = 0; // float bKp = 17, bKi = 0.09, bKd = 600, bKpOld = 0, bKdOld = 0, bKiOld = 0;

unsigned long currTime, prevTime = 0, loopTime, showSetPointTime;
float balanceSetPoint = -1.0;
int u_left, u_right;
float u_speed = 0, u_balance = 0, u_lineFollower = 0;
int speeds[N_MEASURE_SPEED] = {};
int currentSpeed = 0;
int distance[N_MEASURE_ULTRASONIC] = {};
int currentUltasonicDist = 0;
int target_speed = 10;

// PID Variables
float I[3] = {0, 0, 0}, error_old[3] = {0, 0, 0}, maxI[3] = {50, 50, 50};

void PID_reset(short i) {
  I[i] = 0;
  error_old[i] = 0;
}

int PID_Control(short i, float error, float Kp, float Ki, float Kd, bool needInvert) {
  float P = error * Kp;
  P = constrain(P, -255, 255);
  I[i] += error * loopTime * Ki;
  I[i] = constrain(I[i], -maxI[i], maxI[i]);
  float D = ((error - error_old[i]) / loopTime) * Kd;
  int u = (P + I[i] + D) * (needInvert ? -1 : 1);
  u = constrain(u, -255, 255);
  error_old[i] = error;
  Serial.print(error); Serial.print(" "); Serial.print(P); Serial.print(" "); Serial.print(I[i]); Serial.print(" "); Serial.print(D); Serial.print(" "); Serial.print(u); Serial.print("\t\t");
  return u;
}

void setup() {
  Serial.begin(9600);
  Blynk.begin(Serial, AUTH);
  /*
  // Amperka Troyka модули
  //accel.begin(); // Инициализация акселерометра
  //accel.setRange(RANGE_2G); // Устанавливаем чувствительность акселерометра, 2g — по умолчанию, 4g, 8g
  //gyro.begin(); // Инициализация гиро
  //gyro.setRange(RANGE_250DPS); // Устанавливаем чувствительность гироскопа, 250dps — по умолчанию, 500dps, 2000dps
  */
  gyroAccel.begin();
  Serial.println();
  Serial.println("Initialization completed");
  buzzerOff();
  lMotor.begin(); rMotor.begin();
  lMotor.runSpeed(0); rMotor.runSpeed(0);
}

void loop()
{
  Blynk.run();
  if (myTimer1.isReady())
  {
    currTime = millis();
    loopTime = currTime - prevTime;
    prevTime = currTime;
    /*
    // Amperka Troyka модули
    accelY = accel.readAY(); // величины ускорения в м/с² по оси Y
    accelZ = accel.readAZ(); // величины ускорения в м/с² по оси Z
    accAngle = atan2(accelY, accelZ) * RAD_TO_DEG;
    gyroX = gyro.readDegPerSecX();
    gyroAngle = (float)gyroX * loopTime / 1000;
    cfAngle = K * (cfAngle_old + gyroAngle) + (1 - K) * accAngle; // Комплементарный фильтр от двух датчиков
    cfAngle_old = cfAngle;
    Serial.print("cfAng: "); Serial.print(cfAngle); Serial.print("\t");
    */
    gyroAccel.update();
    angleY = gyroAccel.getAngleY();
    Serial.print("angleY: "); Serial.print(angleY); Serial.print("  ");
    potS = potentiometer.read();
    potS = map(potS, 0, MAX_VAL_POT, 30, -40) / 10.0;
    if (potS != oldValPotS) {
      PID_reset(0); PID_reset(1);
      balanceSetPoint = potS; // Устанавливаем позицию баланса по потенцометру
      showBalanceSetPoint = true;
      showSetPointTime = 0; // Сбросить таймер вывода нового balanceSetPoint
      oldValPotS = potS;
    }
    if (showBalanceSetPoint && showSetPointTime <= 2000) { // Вывод на дисплей
      showSetPointTime += loopTime;
      seg7.display(balanceSetPoint);
    } else {
      seg7.display(angleY);
      if (showBalanceSetPoint == true) showBalanceSetPoint == false;
    }
    if (abs(angleY) < BALANCING_LIMITS) {
      if (currTime >= 10) {
        // Speed
        for (int i = 0; i < N_MEASURE_SPEED - 1; i++) speeds[i] = speeds[i + 1];
        currentSpeed = (lMotor.getCurrentSpeed() + -rMotor.getCurrentSpeed()) / 2;
        speeds[N_MEASURE_SPEED - 1] = currentSpeed;
        currentSpeed = 0;
        for (int i = 0; i < N_MEASURE_SPEED; i++) currentSpeed += speeds[i];
        currentSpeed = currentSpeed / N_MEASURE_SPEED;
        //Serial.print("currentSpeedL: "); Serial.print(lMotor.getCurrentSpeed()); Serial.print("\t"); Serial.print("currentSpeedR: "); Serial.print(-rMotor.getCurrentSpeed()); Serial.print("\t");
        Serial.print("speed: "); Serial.print(currentSpeed); Serial.print("  ");
        
        // Ultrasonic distance
        for (int i = 0; i < N_MEASURE_ULTRASONIC - 1; i++) distance[i] = distance[i + 1];
        currentUltasonicDist = ultrasonic.distanceCm();
        distance[N_MEASURE_ULTRASONIC - 1] = currentUltasonicDist;
        currentUltasonicDist = 0;
        for (int i = 0; i < N_MEASURE_ULTRASONIC; i++) currentUltasonicDist += distance[i];
        currentUltasonicDist = currentUltasonicDist / N_MEASURE_ULTRASONIC;
        //Serial.print("ultra: "); Serial.print(currentUltasonicDist); Serial.print("\t");
        
        // Speed PID
        float errorSpeed = currentSpeed - target_speed;
        if (abs(angleY) > 4) { PID_reset(0); PID_Control(0, errorSpeed, 0, 0, 0, true); }
        else u_speed = PID_Control(0, errorSpeed, vKp, vKi, vKd, true);
        //u_speed = constrain(u_speed, -255, 255);
        u_speed = 0;
  
        // Balance PID
        float targetAngle = balanceSetPoint - u_speed;
        float errorAngle = angleY - targetAngle;
        u_balance = PID_Control(1, errorAngle, bKp, bKi, bKd, true); // PID_Control(1, errorAngle, 13, 0.09, 550, true) // int u = PID_Control(error, 25, 0.25, 500); // int u_balance = PID_Control(1, errorAngle, 30, 0.3, 600, false);
        u_balance = constrain(u_balance, -255, 255);
  
        // Line Sensors
        int leftLineS = lineSensors.aRead1(); //Serial.print("l: "); Serial.print(leftLineS); Serial.print("\t"); // Для вывода сырых значений левого
        int rightLineS = lineSensors.aRead2(); //Serial.print("r: "); Serial.print(rightLineS); Serial.print("\t"); // Для вывода сырых значений правого
        leftLineS = map(leftLineS, BLACK_LEFT_LINE_S, WHITE_LEFT_LINE_S, 0, 255);
        leftLineS = constrain(leftLineS, 0, 255); //Serial.print("leftLineS: "); Serial.print(leftLineS); Serial.print("\t");
        rightLineS = map(rightLineS, BLACK_RIGHT_LINE_S, WHITE_RIGHT_LINE_S, 0, 255);
        rightLineS = constrain(rightLineS, 0, 255); //Serial.print("rightLineS: "); Serial.print(rightLineS); Serial.print("\t");
        
        // LineFollower PID
        float errorLineFollower = leftLineS - rightLineS;
        u_lineFollower = PID_Control(2, errorLineFollower, 0.05, 0, 0, false);
        u_lineFollower = constrain(u_lineFollower, -25, 25);
        u_lineFollower = 0; // ВРЕМЕННАЯ ЗАГЛУШКА
      }
      if (myTimer2.isReady()) { // Меньше 10 мсек - НЕЛЬЗЯ!
        if (currentSpeed > 5 && angleY < -1.5) { // Вперёд
          u_left = u_balance + u_lineFollower;
          u_right = -u_balance - u_lineFollower;
        } else { // Назад
          u_left =  u_balance;// + u_lineFollower * 0.2;
          u_right = -u_balance;// + u_lineFollower * 0.2;
        }
        u_left = constrain(u_left, -255, 255); u_right = constrain(u_right, -255, 255);
        lMotor.runSpeed(u_left); rMotor.runSpeed(u_right);
      }
    } else {
      PID_reset(0); PID_reset(1); PID_reset(2);
      lMotor.runSpeed(0); rMotor.runSpeed(0);
    }
    Serial.println();
  }
}

BLYNK_WRITE(V0) {
  bKp = param.asFloat();
  Serial.println(bKp);
}

BLYNK_WRITE(V1) {
  bKi = param.asFloat();
  Serial.println(bKi);
}

BLYNK_WRITE(V2) {
  bKd = param.asFloat();
  Serial.println(bKd);
}

BLYNK_WRITE(V3) {
  vKp = param.asFloat();
}

BLYNK_WRITE(V4) {
  vKi = param.asFloat();
}

BLYNK_WRITE(V5) {
  vKd = param.asFloat();
}

BLYNK_READ(V0)
{
  Blynk.virtualWrite(V0, bKp);
}

BLYNK_READ(V1)
{
  Blynk.virtualWrite(V1, bKi);
}

BLYNK_READ(V2)
{
  Blynk.virtualWrite(V2, bKd);
}

BLYNK_READ(V3)
{
  Blynk.virtualWrite(V3, vKp);
}

BLYNK_READ(V4)
{
  Blynk.virtualWrite(V4, vKi);
}

BLYNK_READ(V5)
{
  Blynk.virtualWrite(V5, vKd);
}
