// PTO.cpp
/*                    DMT542T Stepper Driver
	Pulse and Direction Connection:
	(1) Optically isolated, high level 4.5-5V or 24V, low voltage 0-0.5V
	(2) Maximum 200 KHz input frequency
	(3) The width of PUL signal is at least 2.5μs, duty cycle is recommended 50%
	(4) DIR signal requires advance PUL signal minimum 5 μs in single pulse mode
	(5) The factory setting of control signal voltage is 24V, must need to set S2 (figure 2) if it is 5V
	
	Enable Connection: (default no connection)
	(1) Optically isolated, differential.
	(2) Disable the drive by 4.5- 24V input connection; enable the drive by 0-0.5V connection (3) ENA signal requires advance DIR signal minimum 5μs in single pulse mode
	(4) Enable time to be at least 200ms
*/
/*
	J1 - 400 steps per revolution | 10:1 gear ratio | 60tooth to 15tooth pulley
	J2 - 400 steps per revolution | 50:1 gear ratio
	J3 - 400 steps per revolution | 50:1 gear ratio | 15tooth to 15tooth pulley
	J4 - 400 steps per revolution | 13+212/289 (3969/289) : 1 gear ratio | 28tooth to 10tooth pulley
	J5 - 800 steps per revolution | travel per step 0.04mm 3015/2048 : 1
	J6 - 400 steps per revolution | 19+38/187 (3591/187) : 1 gear ratio

  J5 calculations  
  diameter of pulley = 25mm
  steps per revolution = 200 (dont count 1/4 stepping here)
  travel per step = 0.04mm
  linear travel of pulley (circumference) = 2 * 3.14 * (25/2) = 78.5
  linear travel per revolution = 200 * 0.04 = 8mm
  gear output turns per units in application = 360 <> 78.5 / 8 = 9.8125
  final formula : (2 * pi * (25/2)) / (200 * 0.04) = (25*pi) / 8 ~ 
  1 rev = 200 steps = 8mm = 120/pi ~ 38.1972 degrees

  J1 360 = 60/15 * 10 * 4000 = 160000
  J2 360 = 50 * 4000 = 200000
  J3 360 = 50 * 4000 = 200000
  J4 360 = 28/10 * 13+212/289 * 4000 = 44452800/289 ~ 153816.0
  J5 360 = 25pi / 8 * 4000 = 12500pi ~ 39269.9
  J6 360 = 19+38/187 * 4000 = 14364000/187 ~ 76812.8

*/

#include "PTO.h"

PTO::PTO(int In_DirPin, int In_PulsePin) {
  dirPin = In_DirPin;
  pulsePin = In_PulsePin;
}

// unit revolutions per second so we take in motor steps per revolution
void PTO::run(float RevolutionsPerSecond, int StepsPerRevolution){
  if (RevolutionsPerSecond != 0 && StepsPerRevolution != 0 && DriveEnabled) {

    bOffOneshot = true;

    float Frequency, Period, PulseTime;
    Frequency = abs(RevolutionsPerSecond) * StepsPerRevolution; // Hz
    Period = 1 / Frequency; // seconds
    PulseTime = (Period * 1000 * 1000) / 2; // microseconds

    _SysTime = ARM_DWT_CYCCNT;
    if (_SysTime - _SysTime_Last > (F_CPU_ACTUAL / 1000000 * PulseTime)) {
      _SysTime_Last = _SysTime;
      digitalToggleFast(pulsePin);
    }

    Direction = (RevolutionsPerSecond < 0 ? HIGH : LOW);

    //if (Direction != Direction_Last){
    //  Direction_Last = Direction;
      digitalWriteFast(dirPin, Direction);
    //}

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
  
  digitalWriteFast(dirPin, HIGH);
  digitalWriteFast(pulsePin, HIGH);
}

int PTO::turnON() {
  DriveEnabled = true;
  return DriveEnabled;
}

int PTO::turnOFF() {
  DriveEnabled = false;
  return DriveEnabled;
}