#include <stdio.h>
#include <Arduino.h>

#include <nav_msgs/msg/odometry.h>

enum MotorSelection
{
  FrontLeft = 0,
  FrontRight = 1,
  BackLeft = 2,
  BackRight = 3,
};

struct MotorPin
{
  int fwd;
  int rev;
  float currentSpeed;
  int32_t position;
};

extern MotorPin motorPins[4];

void setupMotors();

void handleMovement(float x, float y, float rz);
void handleIntergration(nav_msgs__msg__Odometry *odom, geometry_msgs__msg__Twist *twist);

float* getSpeeds();