// DigitalOutput.cpp

#include "DigitalOutput.h"

DigitalOutput::DigitalOutput(int Pin, int PinMode, FailMode FailMode) {
  _pin = Pin;
  _pinMode = PinMode;
  _failMode = FailMode;
}

void DigitalOutput::init(){
  pinModeFast(_pin, _pinMode);
}

void DigitalOutput::on(){
  digitalWriteFast(_pin, HIGH);
  _lastState = HIGH;
}

void DigitalOutput::off(){
  digitalWriteFast(_pin, LOW);
  _lastState = LOW;
}

void DigitalOutput::fail(){
  switch (_failMode) {
    case FAIL_HIGH:
      on();
      break;

    case FAIL_LOW:
      off();
      break;

    case HOLD_LAST:
      _lastState ? on() : off();
      break;

  }
}