#include "DigitalInput.h"

DigitalInput::DigitalInput(int pin, int mode, unsigned long debounce){
  _pin = pin;
  _mode = mode;
  _debounce = debounce;
}

bool DigitalInput::read(){
  _read = (digitalReadFast(_pin) ? true : false);
  _timer = millis();
  if (_read == _read_last){
    if (_timer - _timer_last > _debounce){
      _read_out = _read;
    }
  } else {
    _read_last = _read;
    _timer_last = _timer;
  }
  return _read_out;
}

void DigitalInput::init(){
  _read = _read_last = _read_out = false;
  _timer_last = _timer = millis();

  pinMode(_pin, _mode);
}