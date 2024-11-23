// PTO.cpp

#include "PTO.h"

PTO::PTO(int In_DirPin, int In_PulsePin, int In_EnablePin) {
  dirPin = In_DirPin;
  pulsePin = In_PulsePin;
  enablePin = In_EnablePin;
}

void PTO::run(int16_t Frequency){
  if (Frequency != 0 && DriveEnabled) {

    bOffOneshot = true;

    float Period, PulseTime;
    Period = 1 / float(abs(Frequency)); // seconds
    PulseTime = (Period * 1000 * 1000) / 2; // microseconds divide by 2 for 50% duty cycle

    _SysTime = ARM_DWT_CYCCNT;
    if (_SysTime - _SysTime_Last > (F_CPU_ACTUAL / 1000000 * PulseTime)) {
      _SysTime_Last = _SysTime;
      digitalToggleFast(pulsePin);
    }

    Direction = (Frequency < 0 ? HIGH : LOW);
    digitalWriteFast(dirPin, Direction);

  } else {
    if (bOffOneshot){
      bOffOneshot = false;
      digitalWriteFast(dirPin, false);
      digitalWriteFast(pulsePin, false);
    }
  }
}

void PTO::init() {
  pinModeFast(dirPin, OUTPUT);
  pinModeFast(pulsePin, OUTPUT);
  if (enablePin >= 0) {
    pinModeFast(enablePin, OUTPUT);
  }
  
  digitalWriteFast(dirPin, HIGH);
  digitalWriteFast(pulsePin, HIGH);
  if (enablePin >= 0) {
    digitalWriteFast(enablePin, LOW);
  }
}

int PTO::turnON() {
  DriveEnabled = true;
  if (enablePin >= 0) {
    digitalWriteFast(enablePin, HIGH);
  }
  return DriveEnabled;
}

int PTO::turnOFF() {
  DriveEnabled = false;
  if (enablePin >= 0) {
    digitalWriteFast(enablePin, LOW);
  }
  return DriveEnabled;
}