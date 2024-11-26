#include <PWMServo.h>
#include "DigitalInput.h"
#include "DigitalOutput.h"
#include "R_TRIG.h"

#include "EasyCAT.h"
#include <SPI.h>

EasyCAT EASYCAT(10);
bool CommsOK;
#define WATCHDOG 0x80 // this value is returned from EasyCAT.MainTask(). for some reason they dont define it there. i wont fox with it.

uint8_t DEVICE_TYPE = 2; // 1-Robot 2-IO

unsigned long SerialOutputTimer, SerialOutputTimer_last;
uint8_t TxBuffer[32];
uint8_t RxBuffer[32];

// https://www.pjrc.com/teensy/pinout.html
const int input0Pin = 0; // D0 | PWM | RX1
const int input1Pin = 1; // D1 | PWM | TX1
const int input2Pin = 2; // D2 | PWM
const int input3Pin = 3; // D3 | PWM
const int input4Pin = 4; // D4 | PWM
const int input5Pin = 5; // D5 | PWM
const int output0Pin = 6; // D6 | PWM
const int output1Pin = 7; // D7 | PWM | RX2
const int output2Pin = 8; // D8 | PWM | TX2
const int output3Pin = 9; // D9 | PWM
const int output4Pin = 14; // D14 | A0 | PWM | TX3
const int output5Pin = 15; // D15 | A1 | PWM | RX3
//const int  = 16; // D16 | A2 | SCL1 | RX4
const int StatusLed = 17; // D17 | A3 | SDA1 | TX4
const int pwm0Pin = 18; // D18 | A4 | PWM | SDA
const int pwm1Pin = 19; // D19 | A5 | PWM | SCL
//const int  = 20; // D20 | A6 | TX5
//const int  = 21; // D21 | A7 | RX5
const int pwm2Pin = 22; // D22 | A8 | PWM
const int pwm3Pin = 23; // D23 | A9 | PWM
const int pwm4Pin = 24; // D24 | A10 | PWM | SCL2 | TX6
const int pwm5Pin = 25; // D25 | A11 | PWM | SDA2 | RX6
//const int  = 26; // D26 | A12 |
//const int  = 27; // D27 | A13 | 
//const int  = 28; // D28 | PWM | RX7
//const int  = 29; // D29 | PWM | TX7
//const int  = 30; // D30 | 
//const int  = 31; // D31 | 
//const int  = 32; // D32 | 
//const int  = 33; // D33 | PWM
//const int  = 34; // D34 | RX8
//const int  = 35; // D35 | TX8
//const int  = 36; // D36 | PWM
//const int  = 37; // D37 | PWM
//const int  = 38; // D38 | A14
//const int  = 39; // D39 | A15
//const int  = 40; // D40 | A16
//const int  = 41; // D41 | A17

// UNUSED / SPARES / RESERVED
//const int  = 10; // D10 | PWM | CS | RESERVED FOR SPI
//const int  = 11; // D11 | PWM | MOSI | RESERVED FOR SPI
//const int  = 12; // D12 | PWM | MISO | RESERVED FOR SPI
//const int  = 13; // D13 | PWM | LED | SCK | RESERVED FOR SPI

union u_crc {
  uint8_t b[2];
  uint16_t u;
} TxCrc, RxCrc;

// 
DigitalInput DIn[] = {    DigitalInput(input0Pin, INPUT_PULLUP, 1),
                          DigitalInput(input1Pin, INPUT_PULLUP, 1),
                          DigitalInput(input2Pin, INPUT_PULLUP, 1),
                          DigitalInput(input3Pin, INPUT_PULLUP, 1),
                          DigitalInput(input4Pin, INPUT_PULLUP, 1),
                          DigitalInput(input5Pin, INPUT_PULLUP, 1)};
// 
DigitalOutput DOut[] = {  DigitalOutput(output0Pin, OUTPUT, HOLD_LAST),
                          DigitalOutput(output1Pin, OUTPUT, HOLD_LAST),
                          DigitalOutput(output2Pin, OUTPUT, HOLD_LAST),
                          DigitalOutput(output3Pin, OUTPUT, HOLD_LAST),
                          DigitalOutput(output4Pin, OUTPUT, HOLD_LAST),
                          DigitalOutput(output5Pin, OUTPUT, HOLD_LAST)};
						  
//
PWMServo Pwm[6];
int PwmPin[6];

//
void setup() {

  //Serial.begin(115200);                                           // start serial for output

                                                                  //---- initialize the EasyCAT board -----
                                                                  
  if (EASYCAT.Init() == true)                                     // initialization
  {                                                               // succesfully completed
    CommsOK = true;
    //Serial.print ("initialized");                                 //
  }                                                               //
  
  else                                                            // initialization failed   
  {                                                               // the EasyCAT board was not recognized
    CommsOK = false;
    //Serial.print ("initialization failed");                       //     
                                                                  // The most common reason is that the SPI 
                                                                  // chip select choosen on the board doesn't 
                                                                  // match the one choosen by the firmware
                                                                  
    pinMode(StatusLed, OUTPUT);                                   // stay in loop for ever
                                                                  // with the Arduino led blinking
    while(1)                                                      //
    {                                                             //   
      digitalWrite (StatusLed, LOW);                              // 
      delay(300);                                                 //   
      digitalWrite (StatusLed, HIGH);                             //  
      delay(300);                                                 // 
    }                                                             // 
  }

  //
  SerialOutputTimer = millis();
  SerialOutputTimer_last = SerialOutputTimer;

  // initialize i/o
  PwmPin[0] = pwm0Pin;
  PwmPin[1] = pwm1Pin;
  PwmPin[2] = pwm2Pin;
  PwmPin[3] = pwm3Pin;
  PwmPin[4] = pwm4Pin;
  PwmPin[5] = pwm5Pin;
  for(int i=0;i<6;i++){
	DIn[i].init();
	DOut[i].init();
    Pwm[i].attach(PwmPin[i]);
  }

  //
  pinMode(StatusLed, OUTPUT); digitalWriteFast(StatusLed, LOW);
  
}

bool InputState[6];
bool OutputState[6];
uint8_t PwmState[6];
bool Enable;
bool Heartbeat, HeartbeatLast, HeartbeatLost;
unsigned long HeartbeatWatchDog, HeartbeatWatchDogLast;
void loop() {

    handleEtherCAT();
    checkHeartbeat();
    handleTx();
    handleRx();
    handleInputs();
    handleOutputs();
    handleSerial();

}

void handleEtherCAT(){
  // WATCHDOG                   0x80
  // ESM_INIT                   0x01          // state machine control
  // ESM_PREOP                  0x02          // (state request)
  // ESM_BOOT                   0x03          // 
  // ESM_SAFEOP                 0x04          // safe-operational
  // ESM_OP                     0x08          // operational
  unsigned char Status = EASYCAT.MainTask();
  if (Status & WATCHDOG || !(Status & ESM_OP)){
    CommsOK = false;
  } else {
    CommsOK = true;
  }
}

void checkHeartbeat(){
  HeartbeatWatchDog = millis();
  if (Heartbeat != HeartbeatLast) {
    // reset watchdog timer
    HeartbeatLast = Heartbeat;
    HeartbeatWatchDogLast = HeartbeatWatchDog;
    HeartbeatLost = false;
  }
  if (HeartbeatWatchDog - HeartbeatWatchDogLast > 2000){
    // heartbeat from master controller was lost
    HeartbeatLost = true;
  }
}

void handleTx(){

    // reserved for future control use
    TxBuffer[0] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
    TxBuffer[0] = TxBuffer[0] | (Heartbeat ? B00000001 : B00000000);
    TxBuffer[0] = TxBuffer[0] | (Enable ? B00000010 : B00000000);
    //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00000100 : B00000000);
    //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00001000 : B00000000);
    //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00010000 : B00000000);
    //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00100000 : B00000000);
    //TxBuffer[0] = TxBuffer[0] | (tx_ ? B01000000 : B00000000);
    //TxBuffer[0] = TxBuffer[0] | (tx_ ? B10000000 : B00000000);
  
    // byte 1 is used for inputs
    TxBuffer[1] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
    TxBuffer[1] = TxBuffer[1] | (InputState[0] ? B00000001 : B00000000);
    TxBuffer[1] = TxBuffer[1] | (InputState[1] ? B00000010 : B00000000);
    TxBuffer[1] = TxBuffer[1] | (InputState[2] ? B00000100 : B00000000);
    TxBuffer[1] = TxBuffer[1] | (InputState[3] ? B00001000 : B00000000);
    TxBuffer[1] = TxBuffer[1] | (InputState[4] ? B00010000 : B00000000);
    TxBuffer[1] = TxBuffer[1] | (InputState[5] ? B00100000 : B00000000);
    //TxBuffer[1] = TxBuffer[1] | (tx_ ? B01000000 : B00000000);
    //TxBuffer[1] = TxBuffer[1] | (tx_ ? B10000000 : B00000000);

    // bytes 2 through 28 are unused
	  //

    // byte 29 will be used to transmit the device type
    TxBuffer[29] = DEVICE_TYPE;

    // bytes 30 and 31 will be for crc
    // calculate checksum and append to tx buffer
    TxCrc.u = calculateChecksum(TxBuffer,30);
    TxBuffer[30] = TxCrc.b[0];
    TxBuffer[31] = TxCrc.b[1];
  
    // copy bytes to ethercat
    for (uint i = 0; i < sizeof(EASYCAT.BufferIn.Byte); i++){
    EASYCAT.BufferIn.Byte[i] = TxBuffer[i];
    }

}

void handleRx(){

    // copy bytes from ethercat
    for (uint i = 0; i < sizeof(RxBuffer); i++){
      RxBuffer[i] = EASYCAT.BufferOut.Byte[i];
    }
    
    // calculate crc of rx buffer
    RxCrc.u = calculateChecksum(RxBuffer, 30);
    
    // compare calc crc with rx crc
    if (RxCrc.b[0] == RxBuffer[30] && RxCrc.b[1] == RxBuffer[31]) {
          
        // reserved for future control use
        Heartbeat   = RxBuffer[0] & B00000001;
        Enable      = RxBuffer[0] & B00000010;
        //rx_ = RxBuffer[0] & B00000100;
        //rx_ = RxBuffer[0] & B00001000;
        //rx_ = RxBuffer[0] & B00010000;
        //rx_ = RxBuffer[0] & B00100000;
        //rx_ = RxBuffer[0] & B01000000;
        //rx_ = RxBuffer[0] & B10000000;

        // byte 1 is used for digital outputs
        OutputState[0] = RxBuffer[1] & B00000001;
        OutputState[1] = RxBuffer[1] & B00000010;
        OutputState[2] = RxBuffer[1] & B00000100;
        OutputState[3] = RxBuffer[1] & B00001000;
        OutputState[4] = RxBuffer[1] & B00010000;
        OutputState[5] = RxBuffer[1] & B00100000;
        //rx_ = RxBuffer[1] & B01000000;
        //rx_ = RxBuffer[1] & B10000000;

        // bytes 2 through 7 will be for pwm
        for(int i=0; i<6; i++){
              PwmState[i] = constrain(RxBuffer[2 + i], 0 , 180); // start at index 2
        }
    }
}

void handleInputs(){

  // read inputs
  for (int i=0;i<6;i++){
      InputState[i] = DIn[i].read();
  }

}

void handleOutputs(){

  // write digital outputs
  digitalWriteFast(StatusLed, Enable);

  if (Enable && CommsOK && !HeartbeatLost){
    for (int i=0;i<6;i++){
      OutputState[i] ? DOut[i].on() : DOut[i].off();
      Pwm[i].attach(PwmPin[i]); 
      Pwm[i].write(PwmState[i]);
    }
  } else {
	// todo flash led when coms fail or heartbeat lost
    for (int i=0;i<6;i++){
      DOut[i].fail();
      //Pwm[i].detach(); // for some reason this throws an "undefined reference" compiler error.
    }
  }

}

void handleSerial(){
  //SerialOutputTimer = millis();
  //if (SerialOutputTimer - SerialOutputTimer_last > 1000) {
    //SerialOutputTimer_last = SerialOutputTimer;

    //Serial.print(Frequency[0].ival);
    //Serial.print(" ");
    //Serial.print(rx_heartbeat);
    //Serial.print(" ");
    //Serial.println();
  //}

}

// https://www.tutorialspoint.com/cyclic-redundancy-check-crc-in-arduino
uint16_t calculateChecksum(uint8_t * data, uint16_t length)
{
   uint16_t curr_crc = 0x0000;
   uint8_t sum1 = (uint8_t) curr_crc;
   uint8_t sum2 = (uint8_t) (curr_crc >> 8);
   int index;
   for(index = 0; index < length; index = index+1)
   {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}