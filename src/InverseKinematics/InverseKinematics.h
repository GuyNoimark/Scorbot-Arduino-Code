#ifndef kinematics
#define kinematics

#include <Arduino.h>

class InverseKinematics
{
private:
  float armLength1 = 22.2;
  float armLength2 = 22.2;
  float gripperLength = 13.5;
  float angles[3];


public:
  InverseKinematics();

  void calcAngles(float x, float y, float z, float gripperAngle);

  float getAngle(int angle);

};

#endif
