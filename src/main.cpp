#ifdef dobogusinclude
//#include <spi4teensy3.h>
#endif

#include <Arduino.h>

#include <Encoder.h>
#include <Motor/Motor.h>
#include <lf310/lf310.h>
#include <Gripper/Gripper.h>

#include <SPI.h>

Motor base(3, 22, 18, 19);
Motor arm(2, 23, 18, 19);
Motor arm2(4, 24, 18, 19);
Gripper gripper(5, 6, 7, 40, 41, 42, 0, 0, 0, 0, 0, 0);

USB Usb;
LF310 gamepad(&Usb);

// const LF310Data joystick = controller.lf310Data;
// const LF310DataButtons buttons = gamepad.buttonClickState;

void setup()
{
    Serial.begin(9600);
    Usb.Init();
}

boolean up;
boolean down;
boolean left;
boolean right;

void loop()
{
    Usb.Task();
    // base.joystickControl(gamepad.lf310Data.X);

    //Controls the base movment of the robot
    base.buttonControl(gamepad.buttonClickState.RTbutton, gamepad.buttonClickState.LTbutton);
    arm.joystickControl(gamepad.lf310Data.Y);
    arm2.joystickControl(gamepad.lf310Data.Rz);
    gripper.dPadControl(gamepad.lf310Data.btn.dPad);
    gripper.clewState(gamepad.buttonClickState.Abutton, gamepad.buttonClickState.Bbutton);
    // Serial.println(gamepad.lf310Data.Y);
}