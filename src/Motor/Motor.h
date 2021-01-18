#ifndef scorobtMotor
#define scorobtMotor

#include <Arduino.h>
#include <Encoder.h>

class Motor
{
private:
  int powerPin;
  int directionPin;
  int encoder1;
  int encoder2;
  int encoderPosition;
  long currentTime = 0;
  Encoder encoder;

public:
  Motor() : encoder(0, 1) {}
  Motor(
      int _powerPin,
      int _dir,
      int _encoder1,
      int _encoder2);

  void setPower(double power);

  void setDir(int dir);

  void joystickControl(float value);

  void buttonControl(int button1, int button2);

  void moveByTime(double timeLimit, double power, int dir);

  long getPosition();

  void setPosition(float wantedTicks);

  void resetEncoder();

  // void configControlVariables();
};

#endif
