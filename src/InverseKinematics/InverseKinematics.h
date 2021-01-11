#ifndef kinematics
#define kinematics

#include <Arduino.h>

class InverseKinematics
{
private:
  double armLength1 = 22.5;
  double armLength2 = 22.5;
  double gripperLength = 13.5;
  double angles[];


public:
  InverseKinematics();

  void calcAngles(double x, double y, double z, double gripperAngle);

  double getAngle(int angle);

};

#endif
