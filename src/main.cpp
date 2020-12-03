#ifdef dobogusinclude
//#include <spi4teensy3.h>
#endif

#include <Arduino.h>

#include <Encoder.h>
#include "Motor/Motor.h"
#include <lf310/lf310.h>

#include <SPI.h>

// Motor arm(2, 23, 18, 19);
Motor base(3, 22, 18, 19);

USB Usb;
LF310 gamepad(&Usb);

// const LF310Data joystick = controller.lf310Data;
// const LF310DataButtons buttons = gamepad.buttonClickState;

/*
gamepad x = left x
gamepad y = left y
gamepad z = right x
gamepad Rz = right y
*/

void setup()
{
    Serial.begin(9600);
    Usb.Init();
}

void loop()
{
    Usb.Task();
    // base.joystickControl(gamepad.lf310Data.X);

    //Controls the base movment of the robot
    base.buttonControl(gamepad.buttonClickState.RTbutton, gamepad.buttonClickState.LTbutton);
}