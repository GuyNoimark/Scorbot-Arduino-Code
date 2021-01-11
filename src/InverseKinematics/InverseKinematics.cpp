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

  float x2 = x - gripperLength * cos(degreesToRadians(gripperAngle));
  float y2 = y - gripperLength * sin(degreesToRadians(gripperAngle));
  // Serial.print(x2);
  // Serial.print(", ");
  // Serial.println(y2);

  float distanceToGripper = sqrt(square(x2) + square(y2));
  // Serial.println(distanceToGripper, 10);

  float alpha = acos((square(armLength1) - square(distanceToGripper) - square(armLength2)) / (-2 * distanceToGripper * armLength1));
  float beta = atan(y2 / x2);
  // Serial.print(alpha, 10);
  // Serial.print(", ");
  // Serial.println(beta, 10);

  angles[0] = radiansToDegrees(PI / 2 - alpha - beta);

  float x1 = armLength1 * cos(alpha + beta);
  float y1 = armLength1 * sin(alpha + beta);

  angles[1] = radiansToDegrees(atan((y2 - y1) / (x2 - x1)));
}

float InverseKinematics::getAngle(int angleIndex)
{

  return angles[angleIndex];
}
