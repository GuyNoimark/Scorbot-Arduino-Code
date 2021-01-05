#ifndef scorobtMotor
#define scorobtMotor

#include <Arduino.h>
#include <Encoder.h>

class Motor
{
private:
  int powerPin;
  int direction;
  int encoder1;
  int encoder2;
  int encoderPosition;
  float Kp;
  float Ki;
  float Kd;
  float lastError = 0;
  float lastTime = 0;
  Encoder encoder;

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

  long getPosition();

  long setPosition(double wantedTicks);

  void setPIDGains(
      float Kp,
      float Ki,
      float Kd);

  void resetEncoder();
};

#endif
