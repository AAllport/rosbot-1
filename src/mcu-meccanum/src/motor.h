#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>

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
int32_t *handleIntergration();
float* getSpeeds();