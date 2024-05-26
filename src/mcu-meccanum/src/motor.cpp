#include <stdio.h>
#include <Arduino.h>
#include "motor.h"

static const uint8_t D0 = 26;
static const uint8_t D1 = 22;
static const uint8_t D2 = 21;
static const uint8_t D3 = 17;
static const uint8_t D4 = 16;
static const uint8_t D5 = 18;
static const uint8_t D6 = 19;
static const uint8_t D7 = 23;
static const uint8_t D8 = 5;
static const uint8_t RXD = 3;
static const uint8_t TXD = 1;

int rpmAtSpeed(int speed)
{
  // Based at 160rpm at 100% speed
  return (speed * 160) / 255;
}

MotorPin motorPins[4] = {
    {fwd : D1, rev : D2, currentSpeed : 0, position : 0}, // FL
    {fwd : D4, rev : D3, currentSpeed : 0, position : 0}, // FR
    {fwd : D8, rev : D7, currentSpeed : 0, position : 0}, // BL
    {fwd : D5, rev : D6, currentSpeed : 0, position : 0}, // BR
};

#define WHEEL_SEPERATION_WIDTH 0.14
#define WHEEL_SEPERATION_LENGTH 0.12
#define WHEEL_GEOMETRY ((WHEEL_SEPERATION_WIDTH + WHEEL_SEPERATION_LENGTH) / 2)
#define WHEEL_RADIUS 0.03

void setupMotors()
{
  for (int i = 0; i < 4; i++)
  {
    pinMode(motorPins[i].fwd, OUTPUT);
    analogWrite(motorPins[i].fwd, 0);

    pinMode(motorPins[i].rev, OUTPUT);
    analogWrite(motorPins[i].rev, 0);
  }
}

void handleMovement(float x, float y, float rz)
{
  // Based on documentation here - https://ecam-eurobot.github.io/Tutorials/software/mecanum/mecanum.html

  float speeds[4] = {
      (x - y - rz), // Front left
      (x + y + rz), // Front right
      (x + y - rz), // Rear left
      (x - y + rz), // Rear right
  };

  // Loop over the motors and apply
  for (int i = 0; i < 4; i++)
  {
    float speed = speeds[i] * 255;
    motorPins[i].currentSpeed = speed;

    analogWrite(motorPins[i].fwd, speed > 0 ? speed : 0);
    analogWrite(motorPins[i].rev, speed < 0 ? abs(speed) : 0);
  }
}

int32_t *handleIntergration()
{
  static int32_t positions[4];
  for (int i = 0; i < 4; i++)
  {
    motorPins[i].position += motorPins[i].currentSpeed;
    positions[i] = motorPins[i].position;
  }

  return positions;
}

float *getSpeeds()
{
  static float speeds[4];
  for (int i = 0; i < 4; i++)
  {
    speeds[i] = motorPins[i].currentSpeed;
  }

  return speeds;
}