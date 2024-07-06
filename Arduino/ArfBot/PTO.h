// PTO.h
#ifndef PTO_h
#define PTO_h

#include <Arduino.h>
#include <digitalWriteFast.h>

#define THROW_ERROR_IF_NOT_FAST 

class PTO {
  private:
    bool Direction, Direction_Last;
    uint32_t _SysTime, _SysTime_Last;
    bool DriveEnabled;
    int dirPin;
    int pulsePin;
    bool bOffOneshot;

  public:
    PTO(int In_DirPin, int In_PulsePin);
    void init();
    void run(int Frequency);
    int turnON();
    int turnOFF();
};

#endif