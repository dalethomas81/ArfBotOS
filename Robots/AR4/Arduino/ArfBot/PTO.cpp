
#include "PTO.h"

PTO::PTO(int dirPin, int pulsePin, int enablePin){

  _dirPin = dirPin;
  _pulsePin = pulsePin;
  _enablePin = enablePin;
}

void PTO::init() {

  pinModeFast(_dirPin, OUTPUT);
  pinModeFast(_pulsePin, OUTPUT);
  if (_enablePin >= 0) {
    pinModeFast(_enablePin, OUTPUT);
  }
  
  digitalWriteFast(_dirPin, HIGH);
  digitalWriteFast(_pulsePin, HIGH);
  if (_enablePin >= 0) {
    digitalWriteFast(_enablePin, LOW);
  }
}

void PTO::run(int16_t frequency) {

  if (frequency != 0 && enabled) {

    _offOneshot = true;

    float Period, PulseTime;
    Period = 1 / float(abs(frequency)); // seconds
    PulseTime = (Period * 1000 * 1000) / 2; // microseconds divide by 2 for 50% duty cycle

    _sysTime = ARM_DWT_CYCCNT;
    if (_sysTime - _sysTimeLast > (F_CPU_ACTUAL / 1000000 * PulseTime)) {
      _sysTimeLast = _sysTime;
      digitalToggleFast(_pulsePin);
    }

    _direction = (frequency < 0 ? HIGH : LOW);
    digitalWriteFast(_dirPin, _direction);

  } else {
    if (_offOneshot){
      _offOneshot = false;
      digitalWriteFast(_dirPin, false);
      digitalWriteFast(_pulsePin, false);
    }
  }
}

int PTO::enable() {

  enabled = true;
  if (_enablePin >= 0) {
    digitalWriteFast(_enablePin, HIGH);
  }
  return enabled;
}

int PTO::disable() {

  enabled = false;
  if (_enablePin >= 0) {
    digitalWriteFast(_enablePin, LOW);
  }
  return enabled;
}

