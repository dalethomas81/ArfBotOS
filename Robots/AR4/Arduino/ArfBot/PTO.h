// PTO.h
#ifndef PTO_h
#define PTO_h

#include <Arduino.h>
#include <digitalWriteFast.h>

#define THROW_ERROR_IF_NOT_FAST 

class PTO {
  private:
    bool _direction;
    bool _directionLast;
    uint32_t _sysTime;
    uint32_t _sysTimeLast;
    int _dirPin;
    int _pulsePin;
    int _enablePin;
    bool _offOneshot;

  public:
    PTO(int dirPin, int pulsePin, int enablePin = -1);
    void init();
    void run(int16_t frequency);
    int enable();
    int disable();
    bool enabled;

};
#endif
