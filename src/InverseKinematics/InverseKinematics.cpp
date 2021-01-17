#include "InverseKinematics.h"
#include <Arduino.h>
#include <Encoder.h>
#include <time.h>

InverseKinematics::InverseKinematics() {}

const float radiansToDegrees(float radians)
{
  return radians * 180 / PI;
}

const float degreesToRadians(float degrees)
{
  return degrees / 180 * PI;
}

void InverseKinematics::calcAngles(float x, float y, float z, float gripperAngle)
{
  if (gripperAngle < -130 || gripperAngle > 130)
  {
    errorMessage("Gripper angle out of range");
    return;
  }

  float x2 = x - gripperLength * cos(degreesToRadians(gripperAngle));
  float y2 = y - gripperLength * sin(degreesToRadians(gripperAngle));
  float distanceToGripper = sqrt(square(x2) + square(y2));

  if (distanceToGripper > armLength1 + armLength2)
  {
    errorMessage("x, y coordinates out of range");
    return;
  }

  float alpha = acos((square(armLength1) - square(distanceToGripper) - square(armLength2)) / (-2 * distanceToGripper * armLength1));
  float beta = atan(y2 / x2);

  //TODO: add arm angles limitations
  // Axis 1: Base Rotation  310°

  theta0 = radiansToDegrees(PI / 2 - alpha - beta);

  if (theta0 > 310 || theta0 < -35)
  {
    errorMessage("Arm1 angle is out of range");
    return;
  }

  float x1 = armLength1 * cos(alpha + beta);
  float y1 = armLength1 * sin(alpha + beta);

  theta1 = radiansToDegrees(atan((y2 - y1) / (x2 - x1)));

  if (theta1 > 130 || theta1 < -130)
  {
    errorMessage("Arm2 angle is out of range");
    return;
  }
}

void InverseKinematics::errorMessage(String message)
{
  theta0 = NAN;
  theta1 = NAN;
  possibleLocation = false;
  error = message;
  if (!error.equals("No error"))
  {
    Serial.println(error);
  }
}