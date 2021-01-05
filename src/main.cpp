#ifdef dobogusinclude
//#include <spi4teensy3.h>
#endif

#include <Arduino.h>

#include <Encoder.h>
#include <Motor/Motor.h>
#include <lf310/lf310.h>
#include <Gripper/Gripper.h>

#include <SPI.h>

//Pins by order of base, arm, arm2, gripper rotation 1,  gripper rotation 2, and gripper
int motorPwmPins[6] = {7, 6, 5, 4, 3, 2};
int directionPins[6] = {14, 15, 16, 17, 18, 19};

Motor base(motorPwmPins[0], directionPins[0], 0, 0, 0, 0, 0);
Motor arm(motorPwmPins[1], directionPins[1], 0, 0, 0, 0, 0);
Motor arm2(motorPwmPins[2], directionPins[2], 0, 0, 0, 0, 0);
Gripper gripper(motorPwmPins[3], motorPwmPins[4], motorPwmPins[5], directionPins[3], directionPins[4], directionPins[5], 0, 0, 0, 0, 0, 0);

USB Usb;
LF310 gamepad(&Usb);

// const LF310Data joystick = controller.lf310Data;
// const LF310DataButtons buttons = gamepad.buttonClickState;

void setup()
{
    Serial.begin(9600);
    Usb.Init();
    // pinMode(51, INPUT_PULLUP);
}

boolean up;
boolean down;
boolean left;
boolean right;

void loop()
{
    Usb.Task();

    if (!gamepad.connected())
    {
        Serial.println("Gamepad not connected!");
    }

    // base.joystickControl(gamepad.lf310Data.X);

    // Controls the base movment of the robot
    base.buttonControl(gamepad.buttonClickState.RTbutton, gamepad.buttonClickState.LTbutton);
    arm.joystickControl(gamepad.lf310Data.Y);
    arm2.joystickControl(gamepad.lf310Data.Rz);
    gripper.dPadControl(gamepad.lf310Data.btn.dPad);
    gripper.clewState(gamepad.buttonClickState.Abutton, gamepad.buttonClickState.Bbutton);
    // Serial.println(digitalRead(46));

}