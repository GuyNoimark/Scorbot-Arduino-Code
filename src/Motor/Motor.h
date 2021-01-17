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
  float Kp;
  float Ki;
  float Kd;
  float lastError = 0;
  float lastTime = 0;
  long currentTime = 0;
  Encoder encoder;
  int i = 0;

public:
  Motor() : encoder(0, 1) {}
  Motor(
      int _powerPin,
      int _dir,
      int _encoder1,
      int _encoder2,
      float Kp,
      float Ki,
      float Kd);

  void setPower(double power);

  void setDir(int dir);

  void joystickControl(float value);

  void buttonControl(int button1, int button2);

  void moveByTime(double timeLimit, double power, int dir);

  long getPosition();

  void setPosition(float wantedTicks);

  void setPIDGains(
      float Kp,
      float Ki,
      float Kd);

  void resetEncoder();
};

#endif
