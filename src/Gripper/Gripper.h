#ifndef scorobtGripper
#define scorobtGripper

#include <Arduino.h>
#include <Encoder.h>
#include "Motor/Motor.h"
    

class Gripper : public Motor {
  private:
    int powerPin;
    int dir1;
    int dir2;
    int encoder1;
    int encoder2;
    Motor rightMotor;
    Motor leftMotor;
    Motor clawMotor;
    // Encoder encoder;

    
  public:
    
    Gripper(int _powerPinA, int _powerPinB, int _powerPinC, int _dirA, int _dirB, int _dirC, int _encoder1A, int _encoder2A, int _encoder1B, int _encoder2B, int _encoder1C, int _encoder2C);

    void dPadControl(uint8_t dPad);

    void clewState(bool buttonA, bool buttonB);

    // void setGripperPosition(int roll, int pitch, boolean gipperState);


};

#endif
