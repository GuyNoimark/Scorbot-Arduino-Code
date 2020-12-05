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
    Motor() : encoder(0, 1){}
    Motor(int _powerPin, int _dir, int _encoder1, int _encoder2);

    void setPower(double power);

    void setDir(int dir);

    void joystickControl(float value);

    void buttonControl(int button1, int button2);

    long getPosition();

    long setPosition(double wantedTicks);

    void resetEncoder();
};

#endif
