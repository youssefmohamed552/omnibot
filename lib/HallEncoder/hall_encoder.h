#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include <math.h>

#include "Arduino.h"

class HallEncoder{
  private:
    const unsigned char _pinSigA;
    const unsigned char _pinSigB;
    volatile int _count;
    const unsigned int PPR;
  public:
    HallEncoder(const unsigned char pinSigA, const unsigned char pinSigB);
    ~HallEncoder();
    void update_count();
    int count();
    double rev_per_sec(const double dt);
    void reset();
    unsigned char pinINT();
};

#endif
