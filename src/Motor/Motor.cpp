#include "Motor.h"
#include <Arduino.h>
#include <Encoder.h>

//Encoder encoder(int temp1, int temp2);

Motor::Motor(int _powerPin, int _dir, int _encoder1, int _encoder2) : encoder(_encoder1, _encoder2)
{
  this->powerPin = _powerPin;
  this->dir1 = _dir;
  // this->dir2 = _dir2;
  this->encoder1 = _encoder1;
  this->encoder2 = _encoder2;

  pinMode(powerPin, OUTPUT);
  pinMode(dir1, OUTPUT);
  // pinMode(dir2, OUTPUT);
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
  // digitalWrite(dir1, 1 - dir);
  // digitalWrite(dir2, dir);
  digitalWrite(dir1, dir);
}

void Motor::joystickControl(double value)
{
  //  Serial.println(value);
  if (value < 128)
  {
    setDir(1);
    setPower(1 - value / 128);
    //    Serial.println(1 - (value/128));
  }
  else
  {
    setDir(0);
    setPower(value / 128 - 1);
    Serial.println(value / 128 - 1);
  }
}

void Motor::buttonControl(int button1, int button2) {
  int power = button1 + button2;
  int direction = button1;

  setPower(power);
  setDir(direction);

}

long Motor::getPosition()
{
  return encoder.read();
}

long Motor::setPosition(double wantedTicks)
{

  double kp = 0.05;
  double ki = 1;
  double kd = 1;
  double error;
  double speedFactor;
  double basePower = 0.5;

  error = wantedTicks - float(getPosition());

  //  speedFactor = error;
  //  Serial.print(255 - error * kp);
  //  Serial.print("\t");

  Serial.println(error);
  if (error > 0)
  {
    setDir(1);
  }
  else
  {
    setDir(0);
  }

  //  setPower(110);
  setPower(kp * error);
}

void Motor::resetEncoder()
{

  encoder.write(0);
}
