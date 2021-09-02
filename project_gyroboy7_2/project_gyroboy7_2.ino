#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeOrion.h>
#include <TroykaIMU.h>
#include "math.h"

#define K 0.98

#define blackLeftLineS 480
#define whiteLeftLineS 45
#define blackRightLineS 480
#define whiteRightLineS 45

Me7SegmentDisplay seg7(6);
MeDCMotor lMotor(9);
MeDCMotor rMotor(10);
Accelerometer accel; // акселерометр
Gyroscope gyro; // гироскоп
MeUltrasonicSensor ultrasonic(4);
//MePotentiometer potentiometer(8);
//MePort lineSensors(PORT_7);

float accelY, accelZ;
float accAngle;
float gyroX, gyroAngle;
float cfAngle, cfAngle_old;

void setup() {
  Serial.begin(115200);
  Serial.println("Initialization...");
  accel.begin(); // инициализация акселерометра
  // устанавливаем чувствительность акселерометра
  accel.setRange(RANGE_2G); // 2g — по умолчанию, 4g, 8g
  ///
  gyro.begin(); // инициализация гиро
  // устанавливаем чувствительность гироскопа
  gyro.setRange(RANGE_250DPS); // 250dps — по умолчанию, 500dps, 2000dps

  //pinMode(ANALOG, INPUT);
  Serial.println("Initialization completed");
}

unsigned long currTime, prevTime = 0, loopTime;
float dt = 0;
float balanceSetPoint = -1.5;
float balansir = 0;

void loop()
{
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  dt = loopTime;
  
  accelY = accel.readAY(); // величины ускорения в м/с² по оси X
  accelZ = accel.readAZ(); // величины ускорения в м/с² по оси Z
  accAngle = atan2(accelY, accelZ) * RAD_TO_DEG;

  gyroX = gyro.readDegPerSecX();
  gyroAngle = (float)gyroX * loopTime / 1000;

  cfAngle = K * (cfAngle_old + gyroAngle) + (1 - K) * accAngle;
  Serial.print("cfAngle: ");
  Serial.print(cfAngle);
  Serial.print("\t\t");
  seg7.display((int)cfAngle);
  /*
  // Корректировка balanceSetPoint
  if (cfAngle < 1) balansir++;
  else if (cfAngle > -1) balansir--;
  if (balansir > 0 && balanceSetPoint < 3) {
    balanceSetPoint += 0.05;
    balansir = 0;
  }
  if (balansir < -1 && balanceSetPoint > -3) {
    balanceSetPoint -= 0.05;
    balansir = 0;
  }*/
  /*Serial.print("setPoint: ");
  Serial.print(balanceSetPoint);
  Serial.print("\t\t");*/
  float error = cfAngle - balanceSetPoint;
  int u_balance = PID_Control(0, error, 10, 0.25, 500); // int u = PID_Control(error, 10, 0.25, 500);
  
  //int leftLineS = lineSensors.aRead1();
  /*Serial.print("l: "); // Для вывода сырых значений левого
  Serial.print(leftLineS);
  Serial.print("\t");*/
  //int rightLineS = lineSensors.aRead2();
  /*Serial.print("r: "); // Для вывода сырых значений правого
  Serial.print(rightLineS);
  Serial.print("\t");*/
  /*leftLineS = map(leftLineS, blackLeftLineS, whiteLeftLineS, 0, 255);
  leftLineS = constrain(leftLineS, 0, 255);
  rightLineS = map(rightLineS, blackRightLineS, whiteRightLineS, 0, 255);
  rightLineS = constrain(rightLineS, 0, 255);
  Serial.print("l: ");
  Serial.print(leftLineS);
  Serial.print("\t");
  Serial.print("r: ");
  Serial.print(rightLineS);
  Serial.print("\t\t");*/
  //Serial.print(potentiometer.read());
  //Serial.print("\t\t");
  
  //float error2 = leftLineS - rightLineS;
  //int u_line_follower = PID_Control(1, error2, 0.4, 0, 0);

  int u_line_follower = 0;
  
  rMotor.run(0 + u_balance + u_line_follower);
  lMotor.run(-0 - u_balance + u_line_follower);
  cfAngle_old = cfAngle;
  Serial.println();
  delay(5);
}

// PID Variables
float I[2] = {0, 0}, error_old[2] = {0, 0}, maxI[2] = {175, 50};
//float I_sum[2] = {0, 0};

void PID_reset(short i) {
  //I_sum[i] = 0;
  I[i] = 0;
  error_old[i] = 0;
}

int PID_Control(short i, float error, float Kp, float Ki, float Kd) {
  float P = error * Kp;
  /*
  I_sum[i] += error * dt;
  float I = I_sum[i] * Ki;
  I = constrain(I, -maxI[i], maxI[i]);
  I_sum[i] = constrain(I_sum[i], -maxI[i], maxI[i]); // Ограничиваем сумму ошибок I
  */
  I[i] += error * dt * Ki;
  I[i] = constrain(I[i], -maxI[i], maxI[i]);
  float D = ((error - error_old[i]) / dt) * Kd;
  int u = P + I[i] + D;
  u = constrain(u, -255, 255);
  error_old[i] = error;
  Serial.print(error);
  Serial.print("\t");
  Serial.print(P);
  Serial.print("\t");
  Serial.print(I[i]);
  Serial.print("\t");
  Serial.print(D);
  Serial.print("\t");
  Serial.print(u);
  Serial.print("\t\t");
  return u;
}
