#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "Arduino.h"

class DCMotor{
  private:
    const unsigned char _pinDir1;
    const unsigned char _pinDir2;
    const unsigned char _pinSpeed;
  public:
    DCMotor(const unsigned char pinDir1, const unsigned char pinDir2, const unsigned char pinSpeed);
    ~DCMotor();
    void drive(int speed);
};

#endif
