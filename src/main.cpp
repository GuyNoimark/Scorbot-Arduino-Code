#include <Arduino.h>

#include <Encoder.h>
#include "Motor/Motor.h"
#include <lf310/lf310.h>

Motor arm(2, 23, 18, 19);
Motor base(3, 22, 18, 19);

USB Usb;
LF310 gamepad(&Usb);

void setup()
{

}

void loop()
{
  arm.setPower(0.5);
  arm.setDir(1);
  base.setPower(0.5);
  base.setDir(1);
  delay(1000);
  arm.setPower(0.5);
  arm.setDir(0);
  base.setPower(0.5);
  base.setDir(0);
  delay(1000);
}