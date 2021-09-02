#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeOrion.h>
#include <math.h>
#include "GyverTimer.h"
#include <BlynkSimpleSerialBLE.h>

#define BLYNK_USE_DIRECT_CONNECT
#define BALANCING_LIMITS 40 // Границы баланса
#define BLACK_LEFT_LINE_S 480 // Сырые значения с датчика линии
#define WHITE_LEFT_LINE_S 45
#define BLACK_RIGHT_LINE_S 480
#define WHITE_RIGHT_LINE_S 45
#define MAX_VAL_POT 976
#define N_MEASURE_SPEED 15
#define N_MEASURE_ULTRASONIC 15

#define BLYNK_AUTH "0smfD4CC7z2KMFQyonHKAnzWH5nbtoAT"

MeGyro gyroAccel;
MeUltrasonicSensor ultrasonic(3);
MePort lineSensors(PORT_7);
Me7SegmentDisplay seg7(4);
MeEncoderMotor lMotor(0x09, 1), rMotor(0x09, 2);

GTimer_ms myTimer1(10), myTimer2(10);
SoftwareSerial BluetoothSerial(0, 1); // RX, TX

#define BLYNK_PRINT BluetoothSerial

float angleY;
float sKp = 0, sKi = 0, sKd = 0;
float bKp = 20, bKi = 0.1, bKd = 0;

unsigned long currTime, prevTime, loopTime;
float balanceSetPoint = -1.0;
int targetSpeed = 0;
int u_left, u_right;
float u_speed, u_balance, u_lineFollower;
int speeds[N_MEASURE_SPEED] = {};
int currentSpeed;
int distance[N_MEASURE_ULTRASONIC] = {};
int currentUltasonicDist;

// PID Variables
float I[3] = {0, 0, 0}, error_old[3] = {0, 0, 0}, maxI[3] = {25, 25, 25};

void PID_reset(short i) {
  I[i] = 0;
  error_old[i] = 0;
}

int PID_Control(short i, float error, float Kp, float Ki, float Kd, float dt, bool invert) {
  float P = error;
  I[i] += error * dt;
  I[i] = constrain(I[i], -maxI[i], maxI[i]);
  float D = (error - error_old[i]) / dt;
  int u = (P * Kp + I[i] * Ki + D * Kd) * (invert ? -1 : 1);
  u = constrain(u, -255, 255);
  error_old[i] = error;
  if (i == 1) {
    Serial.print(error); Serial.print(" "); Serial.print(P * Kp); Serial.print(" "); Serial.print(I[i] * Ki); Serial.print(" "); Serial.print(D * Kd); Serial.print(" "); Serial.print(u); Serial.print("\t\t");
  }
  return u;
}

void setup() {
  Serial.begin(115200);
  gyroAccel.begin();
  lMotor.begin(); rMotor.begin();
  lMotor.runSpeed(0); rMotor.runSpeed(0);
  BluetoothSerial.begin(115200);
  //BluetoothSerial.println("Waiting for connections...");
  Serial.println();
  Blynk.begin(BluetoothSerial, BLYNK_AUTH);
  Serial.println();
  Serial.println("Initialization completed");
  buzzerOff();
}

void loop()
{
  Blynk.run();
  if (myTimer1.isReady())
  {
    currTime = millis();
    loopTime = currTime - prevTime;
    prevTime = currTime;
    //
    gyroAccel.update();
    angleY = gyroAccel.getAngleY();
    seg7.display(angleY);
    Serial.print("angleY: "); Serial.print(angleY); Serial.print("\t");
    if (abs(angleY) < BALANCING_LIMITS) {
      if (currTime >= 10) {
        // Speed
        for (int i = 0; i < N_MEASURE_SPEED - 1; i++) speeds[i] = speeds[i + 1];
        currentSpeed = (lMotor.getCurrentSpeed() + -rMotor.getCurrentSpeed()) / 2;
        speeds[N_MEASURE_SPEED - 1] = currentSpeed;
        currentSpeed = 0;
        for (int i = 0; i < N_MEASURE_SPEED; i++) currentSpeed += speeds[i];
        currentSpeed = currentSpeed / N_MEASURE_SPEED;
        Serial.print("speed: "); Serial.print(currentSpeed); Serial.print("\t");

        // Speed PID
        float errorSpeed = currentSpeed - targetSpeed;
        /*if (abs(angleY) > 4) { PID_reset(0); PID_Control(0, errorSpeed, 0, 0, 0, true); }
        else u_speed = PID_Control(0, errorSpeed, sKp, sKi, sKd, true);*/
        u_speed = PID_Control(0, errorSpeed, sKp, sKi, sKd, loopTime, true);
  
        // Balance PID
        float targetAngle = balanceSetPoint - u_speed;
        float errorAngle = angleY - targetAngle;
        u_balance = PID_Control(1, errorAngle, bKp, bKi, bKd, loopTime, true);
  
        // Line Sensors
        int leftLineS = lineSensors.aRead1(); //Serial.print("l: "); Serial.print(leftLineS); Serial.print("\t"); // Для вывода сырых значений левого
        int rightLineS = lineSensors.aRead2(); //Serial.print("r: "); Serial.print(rightLineS); Serial.print("\t"); // Для вывода сырых значений правого
        leftLineS = map(leftLineS, BLACK_LEFT_LINE_S, WHITE_LEFT_LINE_S, 0, 255);
        leftLineS = constrain(leftLineS, 0, 255); //Serial.print("leftLineS: "); Serial.print(leftLineS); Serial.print("\t");
        rightLineS = map(rightLineS, BLACK_RIGHT_LINE_S, WHITE_RIGHT_LINE_S, 0, 255);
        rightLineS = constrain(rightLineS, 0, 255); //Serial.print("rightLineS: "); Serial.print(rightLineS); Serial.print("\t");
        
        // LineFollower PID
        float errorLineFollower = leftLineS - rightLineS;
        u_lineFollower = PID_Control(2, errorLineFollower, 0.05, 0, 0, loopTime, false);
        u_lineFollower = constrain(u_lineFollower, -25, 25);

        // Ultrasonic distance
        for (int i = 0; i < N_MEASURE_ULTRASONIC - 1; i++) distance[i] = distance[i + 1];
        currentUltasonicDist = ultrasonic.distanceCm();
        distance[N_MEASURE_ULTRASONIC - 1] = currentUltasonicDist;
        currentUltasonicDist = 0;
        for (int i = 0; i < N_MEASURE_ULTRASONIC; i++) currentUltasonicDist += distance[i];
        currentUltasonicDist = currentUltasonicDist / N_MEASURE_ULTRASONIC;
        //Serial.print("ultra: "); Serial.print(currentUltasonicDist); Serial.print("\t");
      }
      if (myTimer2.isReady()) { // Меньше 10 мсек - НЕЛЬЗЯ!
        /*if (currentSpeed > 5 && angleY < -1.5) { // Вперёд
          u_left = u_balance + u_lineFollower;
          u_right = -u_balance - u_lineFollower;
        } else { // Назад
          u_left =  u_balance; // + u_lineFollower * 0.2;
          u_right = -u_balance; // + u_lineFollower * 0.2;
        }*/
        u_left = u_balance + u_lineFollower; u_right = -u_balance - u_lineFollower;
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
  PID_reset(0); PID_reset(1);
  bKp = param.asFloat();
}

BLYNK_WRITE(V1) {
  PID_reset(0); PID_reset(1);
  bKi = param.asFloat();
}

BLYNK_WRITE(V2) {
  PID_reset(0); PID_reset(1);
  bKd = param.asFloat();
}

BLYNK_WRITE(V3) {
  PID_reset(0); PID_reset(1);
  sKp = param.asFloat();
}

BLYNK_WRITE(V4) {
  PID_reset(0); PID_reset(1);
  sKi = param.asFloat();
}

BLYNK_WRITE(V5) {
  PID_reset(0); PID_reset(1);
  sKd = param.asFloat();
}

BLYNK_WRITE(V6) {
  PID_reset(0); PID_reset(1);
  balanceSetPoint = param.asFloat();
}

BLYNK_WRITE(V7) {
  PID_reset(0); PID_reset(1);
  targetSpeed = param.asFloat();
}

BLYNK_CONNECTED() {
  Blynk.virtualWrite(V0, bKp);
  Blynk.virtualWrite(V1, bKi);
  Blynk.virtualWrite(V2, bKd);
  Blynk.virtualWrite(V3, sKp);
  Blynk.virtualWrite(V4, sKi);
  Blynk.virtualWrite(V5, sKd);
  Blynk.virtualWrite(V6, balanceSetPoint);
  Blynk.virtualWrite(V7, targetSpeed);
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
  Blynk.virtualWrite(V3, sKp);
}

BLYNK_READ(V4)
{
  Blynk.virtualWrite(V4, sKi);
}

BLYNK_READ(V5)
{
  Blynk.virtualWrite(V5, sKd);
}

BLYNK_READ(V6)
{
  Blynk.virtualWrite(V6, balanceSetPoint);
}

BLYNK_READ(V7)
{
  Blynk.virtualWrite(V7, targetSpeed);
}
