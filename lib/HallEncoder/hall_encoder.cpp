#include "hall_encoder.h"

HallEncoder::
HallEncoder(const unsigned char pinSigA, const unsigned char pinSigB)
  : _pinSigA(pinSigA), _pinSigB(pinSigB), _count(0), PPR(408)
{
  pinMode(_pinSigA, INPUT_PULLUP);
  pinMode(_pinSigB, INPUT);
}


HallEncoder::
~HallEncoder(){}



void
HallEncoder::
update_count(){
  char sig = digitalRead(_pinSigB);
  if(sig) _count++;
  else _count--;
}


int 
HallEncoder::
count(){
  return _count;
}

double
HallEncoder::
rev_per_sec(const double dt){
  return ((double)_count / (double)PPR) * M_PI / dt ;
}


void
HallEncoder::
reset(){
  _count = 0;
}


unsigned char
HallEncoder::
pinINT(){
  return _pinSigA;
}
