
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

const int BTLE_REQ = 10;
const int BTLE_RDY = 2;
const int BTLE_RST = 9;

const int MOTOR_ENB = 6;
const int MOTOR_ENA = 5;
const int MOTOR_FWA = 7;
const int MOTOR_BWA = 8;
const int MOTOR_FWB = 4;
const int MOTOR_BWB = 3;

const int CIRCLE_SIZE = 0x4000;

const int LENGTH_CM = 69;
const int WEIGHT_CENTER_CM = 22;
const int MAX_WHEEL_SPEED_CMPS = 100; /* this is an approximation */

//const float PI = 3.14159265358979323

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(BTLE_REQ, BTLE_RDY, BTLE_RST);
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> euler;
imu::Vector<3> gyro;
imu::Vector<3> grav;
float fwd_speed;
float speed_cmps;
bool disable;
float target_rotation;
float current_rotation;
float target_angle;

void initMotors() {
  pinMode(MOTOR_FWA, OUTPUT);
  pinMode(MOTOR_BWA, OUTPUT);
  pinMode(MOTOR_FWB, OUTPUT);
  pinMode(MOTOR_BWB, OUTPUT);
}

void initSensor() {
  if (!bno.begin()) {
    Serial.print("No BNO055 detected.");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void motorAFastStop() {
  analogWrite(MOTOR_ENA, 255);
  digitalWrite(MOTOR_FWA, LOW);
  digitalWrite(MOTOR_BWA, LOW);
}

void motorBFastStop() {
  analogWrite(MOTOR_ENB, 255);
  digitalWrite(MOTOR_FWB, LOW);
  digitalWrite(MOTOR_BWB, LOW);
}

void motorASlowStop() {
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_FWA, LOW);
  digitalWrite(MOTOR_BWA, LOW);
}

void motorBSlowStop() {
  analogWrite(MOTOR_ENB, 0);
  digitalWrite(MOTOR_FWB, LOW);
  digitalWrite(MOTOR_BWB, LOW);
}


//bool stopped;
int fix_speed(float speed) {
  if (speed < 0.05f && speed > -0.05f) {
    //speed *= 5;
    speed = 0;
  } else {
    speed *= 0.80f;
    if (speed > 0.0f) {
      speed += 0.20f;
    } else {
      speed -= 0.20f;
    }
  }
  int si = speed * 255;
  if (si > 255) { si = 255; }
  if (si < -255) { si = -255; }
  return si;
}

void motorA(float speed_f) {
  int speed = fix_speed(speed_f);
  Serial.print(speed);
  Serial.print("\t");
  if (speed > 0) {
    analogWrite(MOTOR_ENA, speed);
    digitalWrite(MOTOR_FWA, HIGH);
    digitalWrite(MOTOR_BWA, LOW);
  } else if (speed < 0) {
    analogWrite(MOTOR_ENA, -speed);
    digitalWrite(MOTOR_FWA, LOW);
    digitalWrite(MOTOR_BWA, HIGH);
  } else {
    motorASlowStop();
  }
}

void motorB(float speed_f) {
  int speed = fix_speed(speed_f);
  Serial.println(speed);
  if (speed > 0) {
    analogWrite(MOTOR_ENB, speed);
    digitalWrite(MOTOR_FWB, HIGH);
    digitalWrite(MOTOR_BWB, LOW);
  } else if (speed < 0) {
    analogWrite(MOTOR_ENB, -speed);
    digitalWrite(MOTOR_FWB, LOW);
    digitalWrite(MOTOR_BWB, HIGH);
  } else {
    motorBSlowStop();
  }
}

void initBtooth() {
  //BTLEserial.setDeviceName("WHEELIE"); // max 7 chars
  BTLEserial.begin();
}

void setup()
{
  initMotors();
  initSensor();
  initBtooth();
  Serial.begin(9600);
}

void btLoop() {
  BTLEserial.pollACI();
  if (BTLEserial.available() && BTLEserial.getState() == ACI_EVT_CONNECTED) {
    while (BTLEserial.available()) {
      byte b = BTLEserial.read();
      btHandle(b);
    }
  }
}

const int RF_TARGET_ANGLE = 2;
const int RF_TARGET_ROTATION = 1;

byte buffer[4];
byte bufferSize;
void btHandle(byte b) {
  if (bufferSize < 3) {
    buffer[bufferSize++] = b;
  }
  if (bufferSize < 3) {
    return;
  }
  bufferSize = 0;
  int n = (int)(((short)buffer[1]) << 8 | ((short)buffer[2]));
  if (buffer[0] == RF_TARGET_ROTATION) {
    target_rotation = ((float)n) / 0x4000 * 360;
  }
  if (buffer[0] == RF_TARGET_ANGLE) {
    target_angle = ((float)n) / 0x4000 / 4;
  }
}

float square(float x) {
  if (x > 0) {
    return x * x;
  } else {
    return -(x * x);
  }
}

void sensorLoop() {
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  float wheel_speed_cmps = fwd_speed * MAX_WHEEL_SPEED_CMPS;
  float top_speed_cmps = wheel_speed_cmps - gyro.z() * LENGTH_CM * 2;
  float cur_speed_cmps = (top_speed_cmps * (LENGTH_CM - WEIGHT_CENTER_CM) + \
          wheel_speed_cmps * WEIGHT_CENTER_CM) / LENGTH_CM;
  //speed_cmps = 0.8 * speed_cmps + 0.2 * cur_speed_cmps;
  speed_cmps = cur_speed_cmps;
  float fwd_angle = (-euler.z() - 90) / 90 - 0.045;

  disable = fwd_angle < -0.5 || fwd_angle > 0.5;
  current_rotation = euler.x();

  fwd_speed = square(fwd_angle - target_angle) * 128;

  if (fwd_speed > 1) { fwd_speed = 1; }
  if (fwd_speed < -1) { fwd_speed = -1; }
}

void motorLoop() {
  if (disable) {
    motorA(0);
    motorB(0);
    return;
  }
  float todo_rotation;
  if (fwd_speed > 0.75f || fwd_speed < -0.75f) {
    todo_rotation = 0;
  } else {
    todo_rotation = target_rotation - current_rotation + 540;
    while (todo_rotation > 360) { todo_rotation -= 360; }
    todo_rotation = todo_rotation / 180 - 1;
    if (todo_rotation > 0.2f) { todo_rotation = 0.2f; }
    if (todo_rotation < -0.2f) { todo_rotation = -0.2f; }
  }
  motorA(fwd_speed + todo_rotation);
  motorB(fwd_speed - todo_rotation);
}

void loop()
{
  sensorLoop();
  btLoop();
  motorLoop();
}
