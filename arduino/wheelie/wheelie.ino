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

const int LENGTH_CM = 69;
const int WEIGHT_CENTER_CM = 22;
const int MAX_WHEEL_SPEED_CMPS = 100; /* this is an approximation */


//If it leans forward too much, increase, else decrease;
const float null_angle = -0.0115;

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(BTLE_REQ, BTLE_RDY, BTLE_RST);
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> euler;
imu::Vector<3> gyro;
float fwd_speed;
bool disable;
float target_rotation;
float current_rotation;
float target_angle;
bool controlled;
float recent_speed;

double P[2][2] = {{1, 0}, {0, 1}};
double Pdot[4] = {0, 0, 0, 0};
static const double Q_ANGLE = 0.001;
static const double Q_GYRO = 0.003;
static const double R_ANGLE = 0.5;
static const double DTT = 0.01;
static const double C_0 = 1.0;
double q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
double angle, angle_dot, aaxdot, aax;
double position_dot, position_dot_filter, position;

void kalman() {
  double angle_m = -euler.z() - 90;
  double gyro_m = gyro.x() / PI * 180;
  angle += (gyro_m - q_bias) * DTT;
  Pdot[0] = Q_ANGLE - P[0][1] - P[1][0];
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_GYRO;
  P[0][0] += Pdot[0] * DTT;
  P[0][1] += Pdot[1] * DTT;
  P[1][0] += Pdot[2] * DTT;
  P[1][1] += Pdot[3] * DTT;
  angle_err = angle_m - angle;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_ANGLE + C_0 + PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias;
}

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
  if (speed == 0) { return 0; }
  const int NULL_ZONE = 0.05f;
  if (speed < NULL_ZONE && speed > -NULL_ZONE) {
    speed *= 5;
    //speed = 0;
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
  controlled = false;
}

void setup()
{
  recent_speed = 0;
  initMotors();
  initSensor();
  initBtooth();
  Serial.begin(9600);
}

void btLoop() {
  BTLEserial.pollACI();
  if (BTLEserial.available() && BTLEserial.getState() == ACI_EVT_CONNECTED) {
    controlled = true;
    while (BTLEserial.available()) {
      byte b = BTLEserial.read();
      btHandle(b);
    }
  } else if (!BTLEserial.getState() == ACI_EVT_CONNECTED) {
    controlled = false;
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
  kalman();

  float fwd_angle = (-euler.z() - 90) / 90 + null_angle;

  const float factor_rs = 0.95f;
  recent_speed = factor_rs * recent_speed + (1 - factor_rs) * fwd_speed;

  disable = fwd_angle > 0.5 || fwd_angle < -0.5;
  current_rotation = euler.x();

  Serial.print(fwd_angle);
  Serial.print('\t');
  Serial.print(angle);
  Serial.print('\t');
  Serial.print(angle_dot);
  Serial.print('\n');

  //fwd_speed = square(fwd_angle - target_angle) * 128;
  fwd_speed = angle * 0.25; //13;
  fwd_speed -= angle_dot * 0.005;
  if (controlled) {
    fwd_speed -= target_angle * 8;
  }
  //fwd_speed += gyro.z() * 0.2f;
  fwd_speed += recent_speed * 0.4f;

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
  if (true || abs(fwd_speed) > 0.25f || !controlled) {
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
