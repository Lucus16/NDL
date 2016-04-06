// Provided by library: Adafruit BNO055
#include <Adafruit_BNO055.h>
// Provided by library: Adafruit nRF8001
#include "Adafruit_BLE_UART.h"
// These libraries are available from the Manage Libraries screen in Arduino IDE

// Wire connections

const int BTLE_REQ = 10;
const int BTLE_RDY = 2;
const int BTLE_RST = 9;

const int MOTOR_ENB = 6;
const int MOTOR_ENA = 5;
const int MOTOR_FWA = 7;
const int MOTOR_BWA = 8;
const int MOTOR_FWB = 4;
const int MOTOR_BWB = 3;

// Robot properties

const int LENGTH_CM = 69;
const int WEIGHT_CENTER_CM = 22;
const int MAX_WHEEL_SPEED_CMPS = 100; /* this is an approximation */

// Globals

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(BTLE_REQ, BTLE_RDY, BTLE_RST);
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> euler;
imu::Vector<3> gyro;
float fwd_speed;
float speed_cmps;
bool disable;
float target_rotation;
float current_rotation;
float target_angle;
bool controlled;
float recent_speed;

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
  delay(100);
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

// Change a -1.0...1.0 speed to a corresponding motor power.
// The NULL_ZONE makes sure the robot doesn't overreact for low values.
// The motors don't react properly below about 25% power, which may cause one to
// activate while the other doesn't, which can cause imbalance. This is why we
// make sure the speed always above this or 0.
const float NULL_ZONE = 0.02f;
const float MIN_SPEED = 0.30f;
int fix_speed(float speed) {
  if (speed == 0) { return 0; }
  if (speed < NULL_ZONE && speed > -NULL_ZONE) {
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

// Apply a -1.0...1.0 speed to motor A
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

// Apply a -1.0...1.0 speed to motor B
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

// Check for bluetooth signals and handle any found.
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

// This is where the bluetooth signals are handled. Currently, the robot tries
// to match the rotation of the phone, and the zero angle it targets is
// proportional to the pitch of the phone.

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

// A helper function to square a number but keep its sign.
float square(float x) {
  if (x > 0) {
    return x * x;
  } else {
    return -(x * x);
  }
}

// This is where the actual balancing calculations are done.
void sensorLoop() {
  // Get sensor data.
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Adjust for sensor imperfection in angle measurement.
  // If it leans forward too much, increase, else decrease;
  float null_angle = 0.01;

  // Calculate the -1.0...1.0 angle where 1.0 is flat on the ground forward and
  // -1.0 is flat on the ground backward.
  float fwd_angle = (-euler.z() - 90) / 90 + null_angle;

  // Approximate the current speed, this tries to account for delay in
  // acceleration
  const float factor_rs = 0.9f;
  recent_speed = factor_rs * recent_speed + (1 - factor_rs) * fwd_speed;

  // Disable the robot if it falls over too far, so it doesn't run away
  disable = fwd_angle > 0.5 || fwd_angle < -0.5;

  // Used for rotating left and right
  current_rotation = euler.x();

  // Set the speed based on the current angle. This seems to work decently,
  // although what value works depends on how well the batteries are charged.
  //fwd_speed = fwd_angle * 9;

  fwd_angle += recent_speed * 0.005 + gyro.z() * 0.00;

  Serial.print(fwd_angle);
  Serial.print('\t');

  // We were considering using the square instead, to account for the nonlinear
  // relation between angle and motor power it takes to normalize it.
  fwd_speed = square(fwd_angle) * 150;

  Serial.println(fwd_speed);

  // If we have bluetooth control, adjust the speed based on it.
  if (controlled) {
    fwd_speed -= target_angle * 8;
  }

  // These are attempts to have other variables affect the motor speed, although
  // they haven't worked out too well so far. gyro.z() is the angular speed in
  // radians per second. recent_speed is an approximation of the recent speed as
  // set by this function.
  //fwd_speed += gyro.z() * 0.2f;
  //fwd_speed += recent_speed * 0.2f;

  // Make sure these don't go out of bounds for the purposes of calculating
  // recent_speed.
  if (fwd_speed > 1) { fwd_speed = 1; }
  if (fwd_speed < -1) { fwd_speed = -1; }
}

// Set the motor power based on speed and rotation. Rotation is disabled if no
// controlling phone is connected or the following boolean is set.
const bool DISABLE_ROTATION = true;
const float MAX_ROTATION_SPEED = 0.25f;
void motorLoop() {
  if (disable) {
    // Motors disabled because we've toppled over too far, balance can't be
    // recovered anyway at this point.
    motorA(0);
    motorB(0);
    return;
  }
  float todo_rotation;
  if (DISABLE_ROTATION || fwd_speed > MAX_ROTATION_SPEED ||
      fwd_speed < -MAX_ROTATION_SPEED || !controlled) {
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
  btLoop();
  sensorLoop();
  motorLoop();
}
