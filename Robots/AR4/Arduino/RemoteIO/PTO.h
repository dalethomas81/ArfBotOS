// PTO.h
#ifndef PTO_h
#define PTO_h

#include <Arduino.h>
#include <digitalWriteFast.h>

#define THROW_ERROR_IF_NOT_FAST

// Max half-period edges to emit per run() call. Limits blocking if the
// main loop falls far behind a high commanded frequency.
#ifndef PTO_MAX_EDGES_PER_CALL
#define PTO_MAX_EDGES_PER_CALL 8
#endif

class PTO {
  private:
    bool _direction;              // true = HIGH (negative frequency)
    uint32_t _sysTimeLast;        // cycle count at last edge (phase reference)
    uint32_t _halfPeriodCycles;   // CPU cycles per half-period (50% duty)
    int16_t _frequencyLast;       // last applied frequency (0 = stopped)
    int _dirPin;
    int _pulsePin;
    int _enablePin;
    bool _offOneshot;
    bool _pulseHigh;              // tracks step pin state for DIR setup

    void applyFrequency(int16_t frequency);

  public:
    PTO(int dirPin, int pulsePin, int enablePin = -1);
    void init();
    void run(int16_t frequency);
    int enable();
    int disable();
    bool enabled;

};
#endif
