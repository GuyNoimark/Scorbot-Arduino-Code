#include <Arduino.h>

#include <Encoder.h>
#include "Motor/Motor.h"
#include <lf310/lf310.h>

Motor arm(2, 54, 55, 18, 19);
//Motor g(10, 22, 23);

USB Usb;
LF310 gamepad(&Usb);

void setup() {
  arm.setPower(0.8);
}

void loop() {
  // put your main code here, to run repeatedly:
}