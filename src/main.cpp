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
int encoderP0Pins[6] = {22, 24, 26, 28, 30, 32};
int encoderP1Pins[6] = {23, 25, 27, 29, 31, 33};

Motor base(
    motorPwmPins[0],
    directionPins[0],
    encoderP0Pins[0],
    encoderP1Pins[0], 0, 0, 0);
Motor arm(
    motorPwmPins[1],
    directionPins[1],
    encoderP0Pins[1],
    encoderP1Pins[1], 0, 0, 0);
Motor arm2(
    motorPwmPins[2],
    directionPins[2],
    encoderP0Pins[2],
    encoderP1Pins[2], 0, 0, 0);
Gripper gripper(
    motorPwmPins[3],
    motorPwmPins[4],
    motorPwmPins[5],
    directionPins[3],
    directionPins[4],
    directionPins[5],
    encoderP0Pins[3],
    encoderP0Pins[4],
    encoderP0Pins[5],
    encoderP1Pins[3],
    encoderP1Pins[4],
    encoderP1Pins[5]);

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

    Serial.println(gamepad.buttonClickState.Xbutton);

    // base.buttonControl(gamepad.buttonClickState.RTbutton, gamepad.buttonClickState.LTbutton);
    // arm.joystickControl(gamepad.lf310Data.Y);
    // arm2.joystickControl(gamepad.lf310Data.Rz);
    // gripper.dPadControl(gamepad.lf310Data.btn.dPad);
    // gripper.clewState(gamepad.buttonClickState.Abutton, gamepad.buttonClickState.Bbutton);

    //base 0 - left
    //arm 0 - down
    //arm2 0 - down
    //gripper pitch 0 - up
    //gripper roll 0 - right
    //gripper claw 0 - open

    base.moveByTime(5, 0.3, 0);
    arm.moveByTime(4.8, 1, 0);
    arm2.moveByTime(5, 0.5, 1);
    gripper.moveByTime(5, 0.1, 1, "pitch");
    gripper.moveByTime(5, 0.2, 0, "claw");
}