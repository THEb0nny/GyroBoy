#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MeOrion.h>
#include <math.h>
#include "GyverTimer.h"

#define BALANCING_LIMITS 40 // Границы баланса
#define BLACK_LEFT_LINE_S 480 // Сырые значения с датчика линии
#define WHITE_LEFT_LINE_S 45
#define BLACK_RIGHT_LINE_S 480
#define WHITE_RIGHT_LINE_S 45
#define MAX_VAL_POT 976
#define N_MEASURE_SPEED 15
#define N_MEASURE_ULTRASONIC 15

MeGyro gyroAccel;
MeUltrasonicSensor ultrasonic(3);
MePort lineSensors(PORT_7);
Me7SegmentDisplay seg7(4);
MeEncoderMotor lMotor(0x09, 1), rMotor(0x09, 2);

GTimer_ms myTimer1(10), myTimer2(10);

float angleY;
float vKp = 0, vKi = 0, vKd = 0;
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
  Serial.println("Initialization...");
  gyroAccel.begin();
//  lMotor.begin(); rMotor.begin();
//  lMotor.runSpeed(0); rMotor.runSpeed(0);
  Serial.println();
  Serial.println("Initialization completed");
  buzzerOff();
}

void loop()
{
  gyroAccel.update();
  Serial.println(gyroAccel.getAngleX());
//  if (myTimer1.isReady())
//  {
//    currTime = millis();
//    loopTime = currTime - prevTime;
//    prevTime = currTime;
//    //
//    gyroAccel.update();
//    angleY = gyroAccel.getAngleY();
//    seg7.display(angleY);
//    Serial.print("angleY: "); Serial.print(angleY); Serial.print("\t");
//    if (abs(angleY) < BALANCING_LIMITS) {
//      if (currTime >= 10) {
//        // Speed
//        for (int i = 0; i < N_MEASURE_SPEED - 1; i++) speeds[i] = speeds[i + 1];
//        currentSpeed = (lMotor.getCurrentSpeed() + -rMotor.getCurrentSpeed()) / 2;
//        speeds[N_MEASURE_SPEED - 1] = currentSpeed;
//        currentSpeed = 0;
//        for (int i = 0; i < N_MEASURE_SPEED; i++) currentSpeed += speeds[i];
//        currentSpeed = currentSpeed / N_MEASURE_SPEED;
//        Serial.print("speed: "); Serial.print(currentSpeed); Serial.print("\t");
//
//        // Speed PID
//        float errorSpeed = currentSpeed - targetSpeed;
//        /*if (abs(angleY) > 4) { PID_reset(0); PID_Control(0, errorSpeed, 0, 0, 0, true); }
//        else u_speed = PID_Control(0, errorSpeed, vKp, vKi, vKd, true);*/
//        u_speed = PID_Control(0, errorSpeed, vKp, vKi, vKd, loopTime, true);
//  
//        // Balance PID
//        float targetAngle = balanceSetPoint - u_speed;
//        float errorAngle = angleY - targetAngle;
//        u_balance = PID_Control(1, errorAngle, bKp, bKi, bKd, loopTime, true);
//  
//        // Line Sensors
//        int leftLineS = lineSensors.aRead1(); //Serial.print("l: "); Serial.print(leftLineS); Serial.print("\t"); // Для вывода сырых значений левого
//        int rightLineS = lineSensors.aRead2(); //Serial.print("r: "); Serial.print(rightLineS); Serial.print("\t"); // Для вывода сырых значений правого
//        leftLineS = map(leftLineS, BLACK_LEFT_LINE_S, WHITE_LEFT_LINE_S, 0, 255);
//        leftLineS = constrain(leftLineS, 0, 255); //Serial.print("leftLineS: "); Serial.print(leftLineS); Serial.print("\t");
//        rightLineS = map(rightLineS, BLACK_RIGHT_LINE_S, WHITE_RIGHT_LINE_S, 0, 255);
//        rightLineS = constrain(rightLineS, 0, 255); //Serial.print("rightLineS: "); Serial.print(rightLineS); Serial.print("\t");
//        
//        // LineFollower PID
////        float errorLineFollower = leftLineS - rightLineS;
////        u_lineFollower = PID_Control(2, errorLineFollower, 0.05, 0, 0, loopTime, false);
////        u_lineFollower = constrain(u_lineFollower, -25, 25);
//
//        // Ultrasonic distance
//        for (int i = 0; i < N_MEASURE_ULTRASONIC - 1; i++) distance[i] = distance[i + 1];
//        currentUltasonicDist = ultrasonic.distanceCm();
//        distance[N_MEASURE_ULTRASONIC - 1] = currentUltasonicDist;
//        currentUltasonicDist = 0;
//        for (int i = 0; i < N_MEASURE_ULTRASONIC; i++) currentUltasonicDist += distance[i];
//        currentUltasonicDist = currentUltasonicDist / N_MEASURE_ULTRASONIC;
//        //Serial.print("ultra: "); Serial.print(currentUltasonicDist); Serial.print("\t");
//      }
//      if (myTimer2.isReady()) { // Меньше 10 мсек - НЕЛЬЗЯ!
//        /*if (currentSpeed > 5 && angleY < -1.5) { // Вперёд
//          u_left = u_balance + u_lineFollower;
//          u_right = -u_balance - u_lineFollower;
//        } else { // Назад
//          u_left =  u_balance; // + u_lineFollower * 0.2;
//          u_right = -u_balance; // + u_lineFollower * 0.2;
//        }*/
//        u_left = u_balance + u_lineFollower; u_right = -u_balance - u_lineFollower;
//        u_left = constrain(u_left, -255, 255); u_right = constrain(u_right, -255, 255);
////        lMotor.runSpeed(u_left); rMotor.runSpeed(u_right);
//      }
//    } else {
//      PID_reset(0); PID_reset(1); PID_reset(2);
//      lMotor.runSpeed(0); rMotor.runSpeed(0);
//    }
//    Serial.println();
//  }
}
