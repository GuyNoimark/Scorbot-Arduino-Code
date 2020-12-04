#include "Motor/Motor.h"
#include "Gripper.h"
#include <Arduino.h>
#include <Encoder.h>
#include <lf310/lf310.h>

//Encoder encoder(int temp1, int temp2);

Gripper::Gripper(int _powerPinA, int _powerPinB, int _powerPinC, int _dirA, int _dirB, int _dirC, int _encoder1A, int _encoder2A, int _encoder1B, int _encoder2B, int _encoder1C, int _encoder2C)
    : rightMotor(_powerPinA, _dirA, _encoder1A, _encoder2A),
      leftMotor(_powerPinB, _dirB, _encoder1B, _encoder2B),
      clawMotor(_powerPinC, _dirC, _encoder1C, _encoder2C)
{
    // Motor right(_powerPinA, _dirA, _encoder1A, _encoder2A);
    // Motor left(_powerPinB, _dirB, _encoder1B, _encoder2B);
    // Motor claw(_powerPinC, _dirC, _encoder1C, _encoder2C);
}

void Gripper::dPadControl(uint8_t dPad)
{

    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;

    switch (dPad)
    {
    case DPAD_UP:
        up = true;
        break;
    case DPAD_RIGHT:
        right = true;
        break;
    case DPAD_DOWN:
        down = true;
        break;
    case DPAD_LEFT:
        left = true;
        break;
    case DPAD_OFF:
        break;
    }

    boolean pitch = up + down;
    boolean roll = left + right;

    rightMotor.setPower(pitch + roll);
    leftMotor.setPower(pitch + roll);

    rightMotor.setDir(down - left);
    leftMotor.setDir(down - right);
}

void Gripper::clewState(bool buttonA, bool buttonB)
{

    clawMotor.setPower(buttonA + buttonB);
    clawMotor.setDir(buttonA);
    // if (buttonA)
    // {
    //     clawMotor.setDir(1);
    // }else
    // {
    //    clawMotor.setDir(0);
    // }
    
}
// long Gripper::getPosition()
// {
//     return encoder.read();
// }

// long Gripper::setPosition(double wantedTicks)
// {

//     double kp = 0.05;
//     double ki = 1;
//     double kd = 1;
//     double error;
//     double speedFactor;
//     double basePower = 0.5;

//     error = wantedTicks - float(getPosition());

//     //  speedFactor = error;
//     //  Serial.print(255 - error * kp);
//     //  Serial.print("\t");

//     Serial.println(error);
//     if (error > 0)
//     {
//         setDir(1);
//     }
//     else
//     {
//         setDir(0);
//     }

//     //  setPower(110);
//     setPower(kp * error);
// }

// void Gripper::resetEncoder()
// {

//     encoder.write(0);
// }
