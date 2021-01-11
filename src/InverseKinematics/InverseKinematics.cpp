#include "InverseKinematics.h"
#include <Arduino.h>
#include <Encoder.h>
#include <time.h>

InverseKinematics::InverseKinematics() {}


void InverseKinematics::calcAngles(double x, double y, double z, double gripperAngle)
{



}



double InverseKinematics::getAngle(int angleIndex)
{


  return angles[angleIndex];
}
