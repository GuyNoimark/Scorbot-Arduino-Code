#ifndef kinematics
#define kinematics

#include <Arduino.h>

class InverseKinematics
{
private:
  float armLength1 = 22.2;
  float armLength2 = 22.2;
  float gripperLength = 13.5;

  void errorMessage(String message);

public:
  float theta0;
  float theta1;
  bool possibleLocation;
  String error = "No error";

  InverseKinematics();

  void calcAngles(float x, float y, float z, float gripperAngle);
};

#endif
