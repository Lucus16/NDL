#include <Adafruit_BNO055.h>
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
float target_speed;
bool controlled;
float recent_speed;
float speed_compensation;

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

const float NULL_ZONE = 0.02f;
const float MIN_SPEED = 0.30f;
int fix_speed(float speed) {
  if (speed == 0) { return 0; }
  if (speed < NULL_ZONE && speed > -NULL_ZONE) {
    //speed *= MIN_SPEED / NULL_ZONE;
    speed = 0;
  } else {
    speed *= 1.0f - MIN_SPEED + NULL_ZONE;
    if (speed > 0.0f) {
      speed += MIN_SPEED - NULL_ZONE;
    } else {
      speed -= MIN_SPEED - NULL_ZONE;
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
  BTLEserial.setDeviceName("WHEELIE"); // max 7 chars
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
    target_speed = 0;
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
    target_speed = ((float)n) / 0x4000;
  }
}

// ALTERNATIVE IDEAS
// work out the math formulas and use integrals to exactly calculate how much power
// results in standing still straight up.

// Assuming gyro.z() measures the rate at which the robot falls forward

void sensorLoop() {
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  current_rotation = euler.x();

  //If it leans forward too much, increase, else decrease;
  float null_angle = 0.015f; //-0.0115;

  float fwd_angle = (-euler.z() - 90) / 90 + null_angle;
  disable = fwd_angle > 0.5 || fwd_angle < -0.5;
  float fwd_accel = gyro.z();

  // Time in seconds to look into the future.
  const float ANGLE_NEXT_S = 0.2f;
  
  // What part of the speed should be based on the next angle.
  const float ANGLE_NEXT_PART = 0.0f;

  // What part of the speed compensation should be based on the next speed.
  const float SPEED_NEXT_PART = 1.0f;
  
  // How strongly the motors react to angles.
  const float ANGLE_MULTIPLIER = 7.5f;

  // Maximum compensation amount from speed
  // Assuming max speed is approx 1m/s
  const float MAX_SPEED_COMPENSATION = 0.0f;

  // Height of the center of mass in meters.
  const float COM_HEIGHT = 0.27f;

  float next_angle = fwd_angle + 4 / PI * ANGLE_NEXT_S * fwd_accel;

  fwd_speed = ANGLE_MULTIPLIER * (1 - ANGLE_NEXT_PART) * fwd_angle;
  fwd_speed += ANGLE_MULTIPLIER * ANGLE_NEXT_PART * next_angle;
  fwd_speed += speed_compensation;

  if (fwd_speed > 1) { fwd_speed = 1; }
  if (fwd_speed < -1) { fwd_speed = -1; }

  float cur_speed = recent_speed + 2 * COM_HEIGHT * gyro.x();
  float next_speed = fwd_speed; // TODO: Account for rotational velocity

  Serial.print(fwd_angle);
  Serial.print('\t');
  Serial.print(next_angle);
  Serial.print('\t');
  Serial.print(fwd_speed);
  Serial.print('\t');

  speed_compensation = (1 - SPEED_NEXT_PART) * cur_speed + SPEED_NEXT_PART * (next_speed - target_speed);

  Serial.print(speed_compensation);

  if (speed_compensation > MAX_SPEED_COMPENSATION) {
    speed_compensation = MAX_SPEED_COMPENSATION;
  }

  if (speed_compensation < -MAX_SPEED_COMPENSATION) {
    speed_compensation = -MAX_SPEED_COMPENSATION;
  }

  const float factor_rs = 0.8f; // TODO
  recent_speed = factor_rs * recent_speed + (1 - factor_rs) * fwd_speed;

  Serial.print('\n');
}

void motorLoop() {
  if (disable) {
    motorA(0);
    motorB(0);
    return;
  }
  const float MAX_ROTATION_SPEED = 0.5f;
  
  float todo_rotation;
  if (!controlled) {
    todo_rotation = 0;
  } else {
    todo_rotation = target_rotation - current_rotation + 540;
    while (todo_rotation > 360) { todo_rotation -= 360; }
    todo_rotation = todo_rotation / 180 - 1;

    if (todo_rotation + fwd_speed > MAX_ROTATION_SPEED) {
      todo_rotation = MAX_ROTATION_SPEED - fwd_speed;
    }
    if (todo_rotation + fwd_speed < -MAX_ROTATION_SPEED) {
      todo_rotation = -MAX_ROTATION_SPEED - fwd_speed;
    }
    if (-todo_rotation + fwd_speed > MAX_ROTATION_SPEED) {
      todo_rotation = MAX_ROTATION_SPEED - fwd_speed;
    }
    if (-todo_rotation + fwd_speed < -MAX_ROTATION_SPEED) {
      todo_rotation = -MAX_ROTATION_SPEED - fwd_speed;
    }
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
