#include "R_TRIG.h"

R_TRIG::R_TRIG(){
  _clk_last = false;
}

bool R_TRIG::read(bool clk){
  if (clk != _clk_last){
    _clk_last = clk;
    if (clk){
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}