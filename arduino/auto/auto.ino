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
const int MOTOR_FWB = 3;
const int MOTOR_BWB = 4;

// Globals

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(BTLE_REQ, BTLE_RDY, BTLE_RST);
float fwd_speed;
float target_rotation;

void initMotors() {
  pinMode(MOTOR_FWA, OUTPUT);
  pinMode(MOTOR_BWA, OUTPUT);
  pinMode(MOTOR_FWB, OUTPUT);
  pinMode(MOTOR_BWB, OUTPUT);
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
const float NULL_ZONE = 0.00f;
const float MIN_SPEED = 0.35f;
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
  BTLEserial.setDeviceName("AUTO"); // max 7 chars
  BTLEserial.begin();
}

void setup()
{
  initMotors();
  initBtooth();
  Serial.begin(9600);
}

// Check for bluetooth signals and handle any found.
void btLoop() {
  BTLEserial.pollACI();
  if (BTLEserial.available() && BTLEserial.getState() == ACI_EVT_CONNECTED) {
    while (BTLEserial.available()) {
      byte b = BTLEserial.read();
      btHandle(b);
    }
  } else if (BTLEserial.getState() != ACI_EVT_CONNECTED) {
    fwd_speed = 0;
    target_rotation = 0;
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
    target_rotation = ((float)n) / 0x4000 * 2;
  }
  if (buffer[0] == RF_TARGET_ANGLE) {
    fwd_speed = (((float)n) / 0x4000 + 0.5f) * 2;
  }
}

// Set the motor power based on speed and rotation. Rotation is disabled if no
// controlling phone is connected or the following boolean is set.
void motorLoop() {
  if (fwd_speed < 0.1f && fwd_speed > -0.1f) {
    fwd_speed = 0.0f;
  }
  if (target_rotation < 0.1f && target_rotation > -0.1f) {
    target_rotation = 0.0f;
  }
  Serial.print(fwd_speed);
  Serial.print('\t');
  Serial.println(target_rotation);
  motorA(fwd_speed + target_rotation);
  motorB(fwd_speed - target_rotation);
}

void loop()
{
  btLoop();
  motorLoop();
}
