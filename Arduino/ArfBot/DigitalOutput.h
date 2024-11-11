#ifndef DIGITALOUTPUT_H
#define DIGITALOUTPUT_H

#include <Arduino.h>
#include <digitalWriteFast.h>

enum FailMode {
    FAIL_HIGH,
    FAIL_LOW,
    HOLD_LAST
};

class DigitalOutput {
  private:
    int _pin, _pinMode;
    FailMode _failMode;
    bool _lastState;
  
  public:
    DigitalOutput(int Pin, int PinMode, FailMode FailMode = HOLD_LAST);
    void init();
    void on();
    void off();
    void fail();
};

#endif