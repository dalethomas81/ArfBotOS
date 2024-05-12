#include "StatusLed.h"

StatusLed::StatusLed(uint8_t pin){
  _pin = pin;
}

void StatusLed::run(){

}

void StatusLed::setMode(){

}

void StatusLed::init(){
  pinMode(_pin, OUTPUT);
}