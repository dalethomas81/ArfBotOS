//#include <Wire.h>
#include "PTO.h"
#include "InputDebounced.h"
#include "R_TRIG.h"
#include <Encoder.h>
#include <Servo.h>

#include "EasyCAT.h"
#include <SPI.h>

EasyCAT EASYCAT(10);

unsigned long SerialOutputTimer, SerialOutputTimer_last;
uint8_t TxBuffer[32];
uint8_t RxBuffer[32];

// https://www.pjrc.com/teensy/pinout.html
const int J1_PulsePin = 0;
const int J1_DirPin   = 1;
const int J2_PulsePin = 2;
const int J2_DirPin   = 3;
const int J3_PulsePin = 4;
const int J3_DirPin   = 5;
const int J4_PulsePin = 6;
const int J4_DirPin   = 7;
const int J5_PulsePin = 8;
const int J5_DirPin   = 9;
const int J6_PulsePin = 33;
const int J6_DirPin   = 34;

const int J1_CalPin = 26;
const int J2_CalPin = 27;
const int J3_CalPin = 28;
const int J4_CalPin = 29;
const int J5_CalPin = 30;
const int J6_CalPin = 31;

const int J1_EncPinA = 14;
const int J1_EncPinB = 15;
const int J2_EncPinA = 38;
const int J2_EncPinB = 39;
const int J3_EncPinA = 36;
const int J3_EncPinB = 37;
const int J4_EncPinA = 20;
const int J4_EncPinB = 21;
const int J5_EncPinA = 22;
const int J5_EncPinB = 23;
const int J6_EncPinA = 24;
const int J6_EncPinB = 25;

const int J1_EnablePin = 16;
const int J2_EnablePin = 18;
const int J3_EnablePin = 19;
const int J4_EnablePin = 40;
const int J5_EnablePin = 41;
const int J6_EnablePin = 32;

const int HeartbeatLed = 17;

// NOTE:
// pins 10, 11, 12, and 13 are reserved for SPI

union u_pto {
  uint8_t b[2];
  int ival;
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

// dirPin, pulsePin, enablePin
PTO Drive[] = {	PTO(J1_DirPin, J1_PulsePin, J1_EnablePin),
				PTO(J2_DirPin, J2_PulsePin, J2_EnablePin), 
				PTO(J3_DirPin, J3_PulsePin, J3_EnablePin), 
				PTO(J4_DirPin, J4_PulsePin, J4_EnablePin), 
				PTO(J5_DirPin, J5_PulsePin, J5_EnablePin), 
				PTO(J6_DirPin, J6_PulsePin, J6_EnablePin)};

//set encoder pins
Encoder Position[] = {Encoder(J1_EncPinA, J1_EncPinB),
					Encoder(J2_EncPinA, J2_EncPinB),
					Encoder(J3_EncPinA, J3_EncPinB),
					Encoder(J4_EncPinA, J4_EncPinB),
					Encoder(J5_EncPinA, J5_EncPinB),
					Encoder(J6_EncPinA, J6_EncPinB)};

InputDebounced Limit[] = {	InputDebounced(J1_CalPin, INPUT_PULLUP, 1),
							InputDebounced(J2_CalPin, INPUT_PULLUP, 1),
							InputDebounced(J3_CalPin, INPUT_PULLUP, 1),
							InputDebounced(J4_CalPin, INPUT_PULLUP, 1),
							InputDebounced(J5_CalPin, INPUT_PULLUP, 1),
							InputDebounced(J6_CalPin, INPUT_PULLUP, 1)};

void setup() {

  Serial.begin(115200);                                           // start serial for output

                                                                  //---- initialize the EasyCAT board -----
                                                                  
  if (EASYCAT.Init() == true)                                     // initialization
  {                                                               // succesfully completed
    Serial.print ("initialized");                                 //
  }                                                               //
  
  else                                                            // initialization failed   
  {                                                               // the EasyCAT board was not recognized
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

  SerialOutputTimer = millis();
  SerialOutputTimer_last = SerialOutputTimer;

	for(int i=0;i<=5;i++){
		Drive[i].init();
		Limit[i].init();
	}

  pinMode(HeartbeatLed, OUTPUT); digitalWriteFast(HeartbeatLed, LOW);
  
}

bool tx_heartbeat, rx_heartbeat, rx_heartbeat_last, rx_heartbeat_lost;
bool tx_en, rx_en, rx_en_last;
bool tx_alarm;
bool LimitState[6];
void loop() {

  // WATCHDOG                   0x80
  // ESM_INIT                   0x01          // state machine control
  // ESM_PREOP                  0x02          // (state request)
  // ESM_BOOT                   0x03          // 
  // ESM_SAFEOP                 0x04          // safe-operational
  // ESM_OP                     0x08          // operational
  unsigned char Status = EASYCAT.MainTask(); 

  handleTx();
  handleRx();
  handleInputs();
  handleOutputs();
  
	for (int i=0;i<=5;i++){
		if (DriveControl[i].Enable) {
			Drive[i].turnON()
		} else {
			Drive[i].turnOFF()
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
		tx_heartbeat = !tx_heartbeat;
		//handleSerial();
	}
}

void handleTx(){

  //
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
	  
	//
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
		  SetVelocity[i].b[j]=RxBuffer[k++];
		}
	}

	//
	for (int i=0;i<=5;i++){
		Drive[i].run(Frequency[i].ival);
	}
	
	// bytes 13 through 24 are used for control from motion
	int k=13;
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
}

void handleInputs(){

	for (int i=0;i<=5;i++){
		LimitState[i] = !Limit.read();
	}

}

void handleOutputs(){

}

void handleSerial(){
  Serial.print(tx_heartbeat);
  Serial.print(" ");
  Serial.print(rx_heartbeat);
  Serial.print(" ");
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