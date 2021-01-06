#include "Motor.h"
#include <Arduino.h>
#include <Encoder.h>

Motor::Motor(
    int _powerPin,
    int _dir,
    int _encoder1,
    int _encoder2,
    float Kp,
    float Ki,
    float Kd)
    : encoder(_encoder1, _encoder2)
{
  this->powerPin = _powerPin;
  this->directionPin = _dir;
  this->encoder1 = _encoder1;
  this->encoder2 = _encoder2;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  pinMode(powerPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
}

void Motor::setPower(double power)
{
  double mappedNumber = abs(power * 255);
  analogWrite(powerPin, mappedNumber);
}

void Motor::setDir(int dir)
{

  digitalWrite(directionPin, dir);
}

void Motor::joystickControl(float value)
{
  float sensitivityFactor = 5;
  // float stuckFactor = 0.1;
  float joystickCenterValue = 127.5;

    if (value < joystickCenterValue - sensitivityFactor)
  {
    setDir(1);
    setPower(1 - value / joystickCenterValue);
  }
  else if (value > joystickCenterValue + sensitivityFactor)
  {
    setDir(0);
    setPower(value / joystickCenterValue - 1);
  }
  else
  {
    setDir(0);
    setPower(0);
  }
}

void Motor::buttonControl(int button1, int button2)
{
  int power = button1 + button2;
  int dir = button1;

  setPower(power);
  setDir(dir);
}

long Motor::getPosition()
{
  encoderPosition = encoder.read();
  return encoderPosition;
}

void Motor::setPosition(double wantedTicks)
{
  float error = wantedTicks - float(getPosition());
  long time = millis();

  //last used Kp = 0.05

  Serial.println(error);
  setDir(error > 0);
  // if (error > 0)
  // {
  //   setDir(1);
  // }
  // else
  // {
  //   setDir(0);
  // }

  //  setPower(110);
  setPower(Kp * error + Kd * (error - lastError) / (time - lastTime));
  lastError = error;
  lastTime = time;
}

void Motor::setPIDGains(
    float Kp,
    float Ki,
    float Kd)
{
}

void Motor::resetEncoder()
{

  encoder.write(0);
}
