//#include <Wire.h>
#include "PTO.h"
#include "DigitalInput.h"
#include "DigitalOutput.h"
#include "R_TRIG.h"
#include <Encoder.h>
#include <Servo.h>

#include "EasyCAT.h"
#include <SPI.h>

EasyCAT EASYCAT(10);
bool CommsOK;
#define WATCHDOG 0x80 // this value is returned from EasyCAT.MainTask(). for some reason they dont define it there. i wont fox with it.


unsigned long SerialOutputTimer, SerialOutputTimer_last;
uint8_t TxBuffer[32];
uint8_t RxBuffer[32];

// https://www.pjrc.com/teensy/pinout.html
const int J1_PulsePin = 0; // D0 | PWM | RX1
const int J1_DirPin   = 1; // D1 | PWM | TX1
const int J2_PulsePin = 2; // D2 | PWM
const int J2_DirPin   = 3; // D3 | PWM
const int J3_PulsePin = 4; // D4 | PWM
const int J3_DirPin   = 5; // D5 | PWM
const int J4_PulsePin = 6; // D6 | PWM
const int J4_DirPin   = 7; // D7 | PWM | RX2
const int J5_PulsePin = 8; // D8 | PWM | TX2
const int J5_DirPin   = 9; // D9 | PWM
const int J6_PulsePin = 33; // D33 | PWM
const int J6_DirPin   = 34; // D34 | RX8

const int J1_CalPin = 26; // D26 | A12 |
const int J2_CalPin = 27; // D27 | A13 | 
const int J3_CalPin = 28; // D28 | PWM | RX7
const int J4_CalPin = 29; // D29 | PWM | TX7
const int J5_CalPin = 30; // D30 | 
const int J6_CalPin = 31; // D31 | 

const int J1_EncPinA = 14; // D14 | A0 | PWM | TX3
const int J1_EncPinB = 15; // D15 | A1 | PWM | RX3
const int J2_EncPinA = 38; // D38 | A14
const int J2_EncPinB = 39; // D39 | A15
const int J3_EncPinA = 36; // D36 | PWM
const int J3_EncPinB = 37; // D37 | PWM
const int J4_EncPinA = 20; // D20 | A6 | TX5
const int J4_EncPinB = 21; // D21 | A7 | RX5
const int J5_EncPinA = 22; // D22 | A8 | PWM
const int J5_EncPinB = 23; // D23 | A9 | PWM
const int J6_EncPinA = 24; // D24 | A10 | PWM | SCL2 | TX6
const int J6_EncPinB = 25; // D25 | A11 | PWM | SDA2 | RX6

const int HeartbeatLed = 17; // D17 | A3 | SDA1 | TX4

const int DigitalOutputPin1 = 18; // D18 | A4 | PWM | SDA
const int DigitalOutputPin2 = 19; // D19 | A5 | PWM | SCL
const int DigitalInputPin1 = 40; // D40 | A16
const int DigitalInputPin2 = 41; // D41 | A17

// UNUSED / SPARES / RESERVED
//const int  = 10; // D10 | PWM | CS | RESERVED FOR SPI
//const int  = 11; // D11 | PWM | MOSI | RESERVED FOR SPI
//const int  = 12; // D12 | PWM | MISO | RESERVED FOR SPI
//const int  = 13; // D13 | PWM | LED | SCK | RESERVED FOR SPI
//const int  = 16; // D16 | A2 | SCL1 | RX4
//const int  = 32; // D32 | 
//const int  = 35; // D35 | TX8

union u_pto {
  uint8_t b[2];
  int16_t ival;
} Frequency[6];

union u_enc {
  uint8_t b[4];
  long lval;
} EncPosition[6];

union u_crc {
  uint8_t b[2];
  uint16_t u;
} TxCrc, RxCrc;

struct s_ctrl {
	bool Enable;
	bool Minus;
	bool Plus;
	bool Stop;
} DriveControl[6];

byte Control1[6];
byte Control2[6];

// dirPin, pulsePin, enablePin = -1
PTO Drive[] = {	PTO(J1_DirPin, J1_PulsePin),
                PTO(J2_DirPin, J2_PulsePin), 
                PTO(J3_DirPin, J3_PulsePin), 
                PTO(J4_DirPin, J4_PulsePin), 
                PTO(J5_DirPin, J5_PulsePin), 
                PTO(J6_DirPin, J6_PulsePin)};

//set encoder pins
Encoder Position[] = {Encoder(J1_EncPinA, J1_EncPinB),
                      Encoder(J2_EncPinA, J2_EncPinB),
                      Encoder(J3_EncPinA, J3_EncPinB),
                      Encoder(J4_EncPinA, J4_EncPinB),
                      Encoder(J5_EncPinA, J5_EncPinB),
                      Encoder(J6_EncPinA, J6_EncPinB)};

// let limit / home inputs
DigitalInput Limit[] = {DigitalInput(J1_CalPin, INPUT_PULLUP, 1), // TODO make debounce time configurable
                          DigitalInput(J2_CalPin, INPUT_PULLUP, 1), // TODO make debounce time configurable
                          DigitalInput(J3_CalPin, INPUT_PULLUP, 1), // TODO make debounce time configurable
                          DigitalInput(J4_CalPin, INPUT_PULLUP, 1), // TODO make debounce time configurable
                          DigitalInput(J5_CalPin, INPUT_PULLUP, 1), // TODO make debounce time configurable
                          DigitalInput(J6_CalPin, INPUT_PULLUP, 1)}; // TODO make debounce time configurable

// auxillary inputs and outputs
DigitalInput DigitalInput1(DigitalInputPin1, INPUT_PULLUP, 1); // TODO make debounce time configurable
DigitalInput DigitalInput2(DigitalInputPin2, INPUT_PULLUP, 1); // TODO make debounce time configurable
DigitalOutput DigitalOutput1(DigitalOutputPin1, OUTPUT, HOLD_LAST); // TODO make fail mode configurable
DigitalOutput DigitalOutput2(DigitalOutputPin2, OUTPUT, HOLD_LAST); // TODO make fail mode configurable

//
void setup() {

  Serial.begin(115200);                                           // start serial for output

                                                                  //---- initialize the EasyCAT board -----
                                                                  
  if (EASYCAT.Init() == true)                                     // initialization
  {                                                               // succesfully completed
    CommsOK = true;
    Serial.print ("initialized");                                 //
  }                                                               //
  
  else                                                            // initialization failed   
  {                                                               // the EasyCAT board was not recognized
    CommsOK = false;
    Serial.print ("initialization failed");                       //     
                                                                  // The most common reason is that the SPI 
                                                                  // chip select choosen on the board doesn't 
                                                                  // match the one choosen by the firmware
                                                                  
    pinMode(13, OUTPUT);                                          // stay in loop for ever
                                                                  // with the Arduino led blinking
    while(1)                                                      //
    {                                                             //   
      digitalWrite (13, LOW);                                     // 
      delay(300);                                                 //   
      digitalWrite (13, HIGH);                                    //  
      delay(300);                                                 // 
    }                                                             // 
  }

  //
  SerialOutputTimer = millis();
  SerialOutputTimer_last = SerialOutputTimer;

  //
	for(int i=0;i<=5;i++){
		Drive[i].init();
		Limit[i].init();
	}

  //
  DigitalInput1.init();
  DigitalInput2.init();
  DigitalOutput1.init();
  DigitalOutput2.init();

  //
  pinMode(HeartbeatLed, OUTPUT); digitalWriteFast(HeartbeatLed, LOW);
  
}

bool LimitState[6];
bool DigitalInputState1, DigitalInputState2;
bool DigitalOutputState1, DigitalOutputState2;
void loop() {

  // WATCHDOG                   0x80
  // ESM_INIT                   0x01          // state machine control
  // ESM_PREOP                  0x02          // (state request)
  // ESM_BOOT                   0x03          // 
  // ESM_SAFEOP                 0x04          // safe-operational
  // ESM_OP                     0x08          // operational
  unsigned char Status = EASYCAT.MainTask();
  //EASYCAT.MainTask();

  if (Status & WATCHDOG || !(Status & ESM_OP)){
    CommsOK = false;
  } else {
    CommsOK = true;
  }

  handleTx();
  handleRx();
  handleInputs();
  handleOutputs();
  
	for (int i=0;i<=5;i++){
		if (DriveControl[i].Enable) {
			Drive[i].turnON();
		} else {
			Drive[i].turnOFF();
		}
		if (DriveControl[i].Minus) {
		}
		if (DriveControl[i].Plus) {
		}
		if (DriveControl[i].Stop) {
		}
	}

	SerialOutputTimer = millis();
	if (SerialOutputTimer - SerialOutputTimer_last > 1000) {
		SerialOutputTimer_last = SerialOutputTimer;
		handleSerial();
	}
}

void handleTx(){

  // reserved for future control use
  //TxBuffer[0] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00000001 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00000010 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00000100 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00001000 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00010000 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00100000 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B01000000 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B10000000 : B00000000);

  // bytes 1 through 24 are used for the encoders
  for (int i=0;i<=5;i++){
	  EncPosition[i].lval = Position[i].read();
  }
  int k=1;
  for (int i=0;i<=5;i++){
    for (int j=0;j<=3;j++){
      TxBuffer[k++] = EncPosition[i].b[j];
    }
  }
  
  // byte 25 is used mostly for the limit switches
  TxBuffer[25] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  TxBuffer[25] = TxBuffer[25] | (LimitState[0] ? B00000001 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (LimitState[1] ? B00000010 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (LimitState[2] ? B00000100 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (LimitState[3] ? B00001000 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (LimitState[4] ? B00010000 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (LimitState[5] ? B00100000 : B00000000);
  //TxBuffer[25] = TxBuffer[25] | (tx_ ? B01000000 : B00000000);
  //TxBuffer[25] = TxBuffer[25] | (tx_ ? B10000000 : B00000000);

  // byte 26 is used for auxillary digital inputs
  TxBuffer[26] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  TxBuffer[26] = TxBuffer[26] | (DigitalInputState1 ? B00000001 : B00000000);
  TxBuffer[26] = TxBuffer[26] | (DigitalInputState2 ? B00000010 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B00000100 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B00001000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B00010000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B00100000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B01000000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B10000000 : B00000000);

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
	  
	// reserved for future control use
	//rx_ = RxBuffer[0] & B00000001;
	//rx_ = RxBuffer[0] & B00000010;
	//rx_ = RxBuffer[0] & B00000100;
	//rx_ = RxBuffer[0] & B00001000;
	//rx_ = RxBuffer[0] & B00010000;
	//rx_ = RxBuffer[0] & B00100000;
	//rx_ = RxBuffer[0] & B01000000;
	//rx_ = RxBuffer[0] & B10000000;

	// bytes 1 through 12 are used for frequency
	int k=1;
	for (int i=0;i<=5;i++){
		for (int j=0;j<=1;j++){
		  Frequency[i].b[j]=RxBuffer[k++];
		}
	}
	for (int i=0;i<=5;i++){
		Drive[i].run(Frequency[i].ival);
	}
	
	// bytes 13 through 24 are used for control from NC
	k=13;
	for (int i=0;i<=5;i++){
		// get nctrl1
		DriveControl[i].Enable = RxBuffer[k] & B00001000;
		k++;
		// get nctrl2
		DriveControl[i].Minus = RxBuffer[k] & B01000001;
		DriveControl[i].Plus = RxBuffer[k] & B01000010;
		DriveControl[i].Stop = RxBuffer[k] & B10000000;
		k++;
	}

	// byte 26 is used for auxillary digital outputs
	DigitalOutputState1 = RxBuffer[26] & B00000001;
	DigitalOutputState2 = RxBuffer[26] & B00000010;
	//rx_ = RxBuffer[26] & B00000100;
	//rx_ = RxBuffer[26] & B00001000;
	//rx_ = RxBuffer[26] & B00010000;
	//rx_ = RxBuffer[26] & B00100000;
	//rx_ = RxBuffer[26] & B01000000;
	//rx_ = RxBuffer[26] & B10000000;
}

void handleInputs(){

  // read limit inputs
	for (int i=0;i<=5;i++){
		LimitState[i] = !Limit[i].read();
	}

  // read auxillary digital inputs
  DigitalInputState1 = !DigitalInput1.read();
  DigitalInputState2 = !DigitalInput2.read();

}

void handleOutputs(){

  // write auxillary digital outputs
  if (CommsOK){
    DigitalOutputState1 ? DigitalOutput1.on() : DigitalOutput1.off();
    DigitalOutputState2 ? DigitalOutput2.on() : DigitalOutput2.off();

  } else {
    DigitalOutput1.fail();
    DigitalOutput2.fail();

  }

}

void handleSerial(){
  Serial.print(Frequency[0].ival);
  //Serial.print(" ");
  //Serial.print(rx_heartbeat);
  //Serial.print(" ");
  Serial.println();
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