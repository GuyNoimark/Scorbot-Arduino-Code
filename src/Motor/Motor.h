#ifndef scorobtMotor
#define scorobtMotor

#include <Arduino.h>
#include <Encoder.h>

    

class Motor {
  private:
    int powerPin;
    int dir1;
    // int dir2;
    int encoder1;
    int encoder2;
    Encoder encoder;

    
  public:
    Motor(int _powerPin, int _dir, int _encoder1, int _encoder2);

    void setPower(double power);

    void setDir(int dir);

    void joystickControl(double value);

    long getPosition();

    long setPosition(double wantedTicks);

    void resetEncoder();
};

#endif
