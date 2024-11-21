#ifndef DIGITALINPUT_H
#define DIGITAL_INPUT_H

#include <Arduino.h>

class DigitalInput {
  
  private:
    int _pin, _mode;
    unsigned long _debounce;
    bool  _read, _read_last, _read_out;
    unsigned long _timer, _timer_last;

  public:
    DigitalInput(int pin, int mode, unsigned long debounce);
    bool read();
    void init();

};

#endif