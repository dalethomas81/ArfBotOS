#ifndef STATUSLED_h
#define STATUSLED_h

#include <Arduino.h>

class StatusLed {
  
  private:
    int _pin, _mode;
    unsigned long _timer, _timer_last;

  public:
    StatusLed(uint8_t pin);
    void run();
    void init();
    void setMode();

};

#endif