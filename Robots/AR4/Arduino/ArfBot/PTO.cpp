
#include "PTO.h"

PTO::PTO(int dirPin, int pulsePin, int enablePin){

  _dirPin = dirPin;
  _pulsePin = pulsePin;
  _enablePin = enablePin;
  _direction = true;       // matches init() default DIR = HIGH
  _sysTimeLast = 0;
  _halfPeriodCycles = 0;
  _frequencyLast = 0;
  _offOneshot = false;
  _pulseHigh = false;
  enabled = false;
}

void PTO::init() {

  pinModeFast(_dirPin, OUTPUT);
  pinModeFast(_pulsePin, OUTPUT);
  if (_enablePin >= 0) {
    pinModeFast(_enablePin, OUTPUT);
  }
  
  digitalWriteFast(_dirPin, HIGH);
  digitalWriteFast(_pulsePin, HIGH);
  _direction = true;
  _pulseHigh = true;
  if (_enablePin >= 0) {
    digitalWriteFast(_enablePin, LOW);
  }

  _sysTimeLast = ARM_DWT_CYCCNT;
  _frequencyLast = 0;
  _halfPeriodCycles = 0;
}

// Recompute timing and direction when the commanded frequency changes.
void PTO::applyFrequency(int16_t frequency) {

  bool newDirHigh = (frequency < 0);

  if (newDirHigh != _direction) {
    // Most stepper drivers need DIR stable before the next active edge.
    // Force the step line idle (LOW) before flipping DIR.
    if (_pulseHigh) {
      digitalWriteFast(_pulsePin, LOW);
      _pulseHigh = false;
    }
    _direction = newDirHigh;
    digitalWriteFast(_dirPin, _direction ? HIGH : LOW);
    // Re-base phase after a forced edge so we do not double-step immediately.
    _sysTimeLast = ARM_DWT_CYCCNT;
  } else if (_frequencyLast == 0) {
    // Starting from stop: ensure DIR is driven and phase is fresh.
    digitalWriteFast(_dirPin, _direction ? HIGH : LOW);
    _sysTimeLast = ARM_DWT_CYCCNT;
  }

  // half-period in CPU cycles: F_cpu / (2 * |f|)
  // Integer path avoids float rounding jitter on the compare threshold.
  uint32_t absFreq = (uint32_t)abs(frequency);
  uint32_t half = (uint32_t)((uint64_t)F_CPU_ACTUAL / (2ULL * (uint64_t)absFreq));
  if (half == 0) {
    half = 1; // clamp to maximum achievable software rate
  }
  _halfPeriodCycles = half;
  _frequencyLast = frequency;
}

void PTO::run(int16_t frequency) {

  if (frequency != 0 && enabled) {

    _offOneshot = true;

    if (frequency != _frequencyLast) {
      applyFrequency(frequency);
    }

    // Phase-locked edge generation: advance the deadline by a fixed period
    // instead of snapping to "now". This preserves pulse phase across variable
    // main-loop latency (SPI, encoder IRQs, etc.).
    uint8_t edges = 0;
    uint32_t now = ARM_DWT_CYCCNT;
    while ((int32_t)(now - _sysTimeLast) >= (int32_t)_halfPeriodCycles
           && edges < PTO_MAX_EDGES_PER_CALL) {
      _sysTimeLast += _halfPeriodCycles;
      digitalToggleFast(_pulsePin);
      _pulseHigh = !_pulseHigh;
      edges++;
      now = ARM_DWT_CYCCNT;
    }

  } else {
    if (_offOneshot){
      _offOneshot = false;
      _frequencyLast = 0;
      _halfPeriodCycles = 0;
      _pulseHigh = false;
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
