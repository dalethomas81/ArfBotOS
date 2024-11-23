#ifndef R_TRIG_h
#define R_TRIG_h

#include <Arduino.h>

class R_TRIG {
  
  private:
    bool _clk_last;

  public:
    R_TRIG();
    bool read(bool clk);

};

#endif