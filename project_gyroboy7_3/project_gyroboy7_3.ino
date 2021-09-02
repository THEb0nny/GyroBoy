#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeOrion.h>
#include <TroykaIMU.h>
#include "math.h"

#define K 0.98

#define blackLeftLineS 480 // Сырые значения с датчика линии
#define whiteLeftLineS 45
#define blackRightLineS 480
#define whiteRightLineS 45

#define maxPotentiometer 976

#define targetSpeed 10

Me7SegmentDisplay seg7(6);
MeEncoderMotor lMotor(0x09, 1);
MeEncoderMotor rMotor(0x09, 2);
Accelerometer accel; // акселерометр
Gyroscope gyro; // гироскоп
MeUltrasonicSensor ultrasonic(5);
MePotentiometer potentiometer(8);
MePort lineSensors(PORT_7);

float accelY, accelZ, accAngle;
float gyroX, gyroAngle;
float cfAngle, cfAngle_old;
float potentiometerS, oldValPotentiometerS;

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
  Serial.println("Initialization completed");
  buzzerOff();
  lMotor.begin(); rMotor.begin();
  lMotor.runSpeed(0); rMotor.runSpeed(0);
}

unsigned long currTime, prevTime = 0, loopTime, showSetPointTime;
float balanceSetPoint = -1.6;
bool showBalanceSetPoint = false;

void loop()
{
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  accelY = accel.readAY(); // величины ускорения в м/с² по оси X
  accelZ = accel.readAZ(); // величины ускорения в м/с² по оси Z
  accAngle = atan2(accelY, accelZ) * RAD_TO_DEG;

  gyroX = gyro.readDegPerSecX();
  gyroAngle = (float)gyroX * loopTime / 1000;

  cfAngle = K * (cfAngle_old + gyroAngle) + (1 - K) * accAngle; // Комплементарный фильтр от двух датчиков
  Serial.print("cfAngle: ");
  Serial.print(cfAngle);
  Serial.print("\t\t");
  
  if (currTime >= 200) {
    // Speed PID
    float speedNow = lMotor.getCurrentSpeed() + -rMotor.getCurrentSpeed()/ 2;
    //float errorSpeed = speedNow - targetSpeed;
    //float u_speed = PID_Control(0, errorSpeed, 0.5, 0.2, 0, true);
    //u_speed = constrain(u_speed, -20, 20);
    int u_speed = 0;
    Serial.print("speedNow: "); Serial.print(speedNow); Serial.print("\t\t");

    // Balance PID
    float targetAngle = balanceSetPoint - u_speed;
    float errorAngle = cfAngle - targetAngle;
    float u_balance = PID_Control(1, errorAngle, 20, 0.25, 500, false);
    
    int leftLineS = lineSensors.aRead1();
    int rightLineS = lineSensors.aRead2();
    //Serial.print("l: "); Serial.print(leftLineS); Serial.print("\t"); // Для вывода сырых значений левого
    //Serial.print("r: "); Serial.print(rightLineS); Serial.print("\t"); // Для вывода сырых значений правого
    leftLineS = map(leftLineS, blackLeftLineS, whiteLeftLineS, 0, 300);
    leftLineS = constrain(leftLineS, 0, 300);
    rightLineS = map(rightLineS, blackRightLineS, whiteRightLineS, 0, 300);
    rightLineS = constrain(rightLineS, 0, 300);
    /*Serial.print("leftLineS: "); Serial.print(leftLineS); Serial.print("\t");
    Serial.print("rightLineS: "); Serial.print(rightLineS); Serial.print("\t\t");*/

    // LineFollow PID
    float errorFollow = leftLineS - rightLineS;
    float u_follow = PID_Control(2, errorFollow, 1, 0, 0, false);
    
    potentiometerS = potentiometer.read();
    potentiometerS = map(potentiometerS, 0, maxPotentiometer, 30, -40) / 10.0;
    if (potentiometerS != oldValPotentiometerS) {
      balanceSetPoint = potentiometerS; // Устанавливаем позицию баланса по потенцометру
      showBalanceSetPoint = true;
      showSetPointTime = 0; // Сбросить таймер вывода нового balanceSetPoint
      oldValPotentiometerS = potentiometerS;
    }
    
    //Serial.print(ultrasonic.distanceCm()); Serial.print("\t\t");

    int u = u_balance + u_follow;
    int u_left = -u; int u_right = u;
    //lMotor.runSpeed(u_left); rMotor.runSpeed(u_right);
    lMotor.move(10); rMotor.move(10);
  
    if (showBalanceSetPoint && showSetPointTime <= 2000) { // Вывод на дисплей
      showSetPointTime += loopTime;
      seg7.display(balanceSetPoint);
    } else {
      seg7.display(cfAngle);
      if (showBalanceSetPoint == true) showBalanceSetPoint == false;
    }
  }
  
  cfAngle_old = cfAngle;
  Serial.println();
  delay(5);
}

// PID Variables
float I[3] = {0, 0, 0}, error_old[3] = {0, 0, 0}, maxI[3] = {50, 100, 50};
//float I_sum[3] = {0, 0, 0};

void PID_reset(short i) {
  //I_sum[i] = 0;
  I[i] = 0;
  error_old[i] = 0;
}

int PID_Control(short i, float error, float Kp, float Ki, float Kd, bool needInvert) {
  float P = error * Kp;
  P = constrain(P, -300, 300);
  /*
  I_sum[i] += error * dt;
  float I = I_sum[i] * Ki;
  I = constrain(I, -maxI[i], maxI[i]);
  I_sum[i] = constrain(I_sum[i], -maxI[i], maxI[i]); // Ограничиваем сумму ошибок I
  */
  I[i] += error * loopTime * Ki;
  I[i] = constrain(I[i], -maxI[i], maxI[i]);
  float D = ((error - error_old[i]) / loopTime) * Kd;
  int u = (P + I[i] + D) * (needInvert? 1 : -1);
  u = constrain(u, -300, 300);
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
  /*if (i == 0) {
    Serial.print(P);
    Serial.print(" ");
    Serial.print(I[i]);
    Serial.print(" ");
    Serial.print(D);
    Serial.print(" ");
    Serial.print(u);
  }*/
  return u;
}
