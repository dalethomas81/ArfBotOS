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
    int enablePin;
    bool bOffOneshot;

  public:
    PTO(int In_DirPin, int In_PulsePin, int In_EnablePin);
    void init();
    void run(int16_t Frequency);
    int turnON();
    int turnOFF();
};

#endif