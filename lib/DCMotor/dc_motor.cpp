#include "dc_motor.h"

DCMotor::
DCMotor(const unsigned char pinDir1, const unsigned char pinDir2, const unsigned char pinSpeed)
  : _pinDir1(pinDir1), _pinDir2(pinDir2), _pinSpeed(pinSpeed)
{
  pinMode(_pinDir1, OUTPUT);
  pinMode(_pinDir2, OUTPUT);
  pinMode(_pinSpeed, OUTPUT);
}

DCMotor::
~DCMotor(){}


void
DCMotor::
drive(int speed){
  if(speed > 0){
    digitalWrite(_pinDir1, HIGH);
    digitalWrite(_pinDir2, LOW);
    analogWrite(_pinSpeed, speed);
  }
  else if(speed < 0){
    digitalWrite(_pinDir1, LOW);
    digitalWrite(_pinDir2, HIGH);
    analogWrite(_pinSpeed, -speed);
  }
  else{
    analogWrite(_pinSpeed, 0);
  }
}
