#include "InverseKinematics.h"
#include <Arduino.h>
#include <Encoder.h>
#include <time.h>

InverseKinematics::InverseKinematics() {}


void InverseKinematics::calcAngles(double x, double y, double z, double gripperAngle)
{
  double x2 = x - gripperLength * cos(gripperAngle);
  double y2 = y - gripperLength * sin(gripperAngle);

  double distanceToGripper = sqrt(square(x2) + square(y2));

  double alpha = acos((square(armLength1) + square(distanceToGripper) - square(armLength2)) / (2 * distanceToGripper * armLength1));
  double beta = atan(y2 / x2);

  angles[0] = PI/2 - alpha - beta;

  double x1 = armLength1 * cos(alpha + beta);
  double y1 = armLength1 * sin(alpha + beta);

  angles[1] = atan((y2 - y1) / (x2 - x1));
}



double InverseKinematics::getAngle(int angleIndex)
{


  return angles[angleIndex];
}
