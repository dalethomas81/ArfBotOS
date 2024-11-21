#ifndef INPUTDEBOUNCED_h
#define INPUTDEBOUNCED_h

#include <Arduino.h>

class InputDebounced {
  
  private:
    int _pin, _mode;
    unsigned long _debounce;
    bool  _read, _read_last, _read_out;
    unsigned long _timer, _timer_last;

  public:
    InputDebounced(int pin, int mode, unsigned long debounce);
    bool read();
    void init();

};

#endif