#include "Motor.h"
#include <Arduino.h>
#include <Encoder.h>
#include <time.h>

Motor::Motor(
    int _powerPin,
    int _dir,
    int _encoder1,
    int _encoder2)
    : encoder(_encoder1, _encoder2)
{
  this->powerPin = _powerPin;
  this->directionPin = _dir;
  this->encoder1 = _encoder1;
  this->encoder2 = _encoder2;

  pinMode(powerPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
}

void Motor::setPower(double power)
{
  double mappedNumber = abs(power * 255);
  if (mappedNumber > 255)
  {
    mappedNumber = 255;
  }
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
  Serial.println(dir);
  setPower(power);
  setDir(dir);
}

void Motor::moveByTime(double timeLimit, double power, int dir)
{
  long currentTime = millis();

  if (currentTime < timeLimit * 1000)
  {
    setPower(power);
    setDir(dir);
  }
  else
  {
    setPower(0);
    setDir(dir);
  }

  // lastTimeMillis = currentTime;
}

long Motor::getPosition()
{
  encoderPosition = encoder.read();
  return encoderPosition;
}

void Motor::setPosition(float wantedTicks)
{
  float torcLimit = 0.14;
  float startDecelerationTicks = 150;

  float error = wantedTicks - float(getPosition());

  setDir(error < 0);

  float prop = abs(error) / startDecelerationTicks + torcLimit;

  Serial.print(abs(error));
  Serial.print("\t");
  Serial.println(prop);
  if (abs(error) > 5)
  {
    setPower(prop);
  }
  else
  {
    setPower(0);
  }
}

void Motor::resetEncoder()
{
  encoder.write(0);
}
