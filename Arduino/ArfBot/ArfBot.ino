//#include <Wire.h>
#include "PTO.h"
#include "InputDebounced.h"
#include "R_TRIG.h"
#include <Encoder.h>
#include <Servo.h>

#include "EasyCAT.h"
#include <SPI.h>
EasyCAT EASYCAT(10);

unsigned long timer1, timer1_last;
uint8_t TxBuffer[32];
uint8_t RxBuffer[32];

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

//const int input0Pin = 34;
const int input1Pin = 35;
const int input2Pin = 40;
const int input3Pin = 41;

//const int output0Pin = 12;
//const int output1Pin = 17;
const int output2Pin = 32;
//const int output3Pin = 33;

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

const int DriveEnablePin = 16;
//const int DriveAlarmPin = 17;

const int HeartbeatLed = 17;

// NOTE:
// pins 10, 11, 12, and 13 are reserved for SPI

union u_pto {
  uint8_t b[4];
  float fval;
} SetVelocity[6];

union u_enc {
  uint8_t b[4];
  long lval;
} EncPosition[6];

union u_crc {
  uint8_t b[2];
  uint16_t u;
} TxCrc, RxCrc;

PTO J1(J1_DirPin, J1_PulsePin); // dirPin, pulsePin
PTO J2(J2_DirPin, J2_PulsePin);
PTO J3(J3_DirPin, J3_PulsePin);
PTO J4(J4_DirPin, J4_PulsePin);
PTO J5(J5_DirPin, J5_PulsePin);
PTO J6(J6_DirPin, J6_PulsePin);

//set encoder pins
Encoder J1encPos(J1_EncPinA, J1_EncPinB);
Encoder J2encPos(J2_EncPinA, J2_EncPinB); // (17, 16) changed to use i2c
Encoder J3encPos(J3_EncPinA, J3_EncPinB); // (18, 19) changed to use i2c1
Encoder J4encPos(J4_EncPinA, J4_EncPinB);
Encoder J5encPos(J5_EncPinA, J5_EncPinB);
Encoder J6encPos(J6_EncPinA, J6_EncPinB);

InputDebounced j1_limit(J1_CalPin, INPUT_PULLUP, 1);
InputDebounced j2_limit(J2_CalPin, INPUT_PULLUP, 1);
InputDebounced j3_limit(J3_CalPin, INPUT_PULLUP, 1);
InputDebounced j4_limit(J4_CalPin, INPUT_PULLUP, 1);
InputDebounced j5_limit(J5_CalPin, INPUT_PULLUP, 1);
InputDebounced j6_limit(J6_CalPin, INPUT_PULLUP, 1);

//InputDebounced input0(input0Pin, INPUT_PULLUP, 1);
InputDebounced input1(input1Pin, INPUT_PULLUP, 1);
InputDebounced input2(input2Pin, INPUT_PULLUP, 1);
InputDebounced input3(input3Pin, INPUT_PULLUP, 1);

bool tx_input0, tx_input1, tx_input2, tx_input3;
bool rx_output0, rx_output1, rx_output2, rx_output3;

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
      delay(500);                                                 //   
      digitalWrite (13, HIGH);                                    //  
      delay(500);                                                 // 
    }                                                             // 
  } 

  //Wire.setClock(400000);
  //Wire.begin(8); // join i2c bus with address 8
  //Wire.onReceive(receiveEvent); // register event
  //Wire.onRequest(requestEvent);

  timer1 = millis();
  timer1_last = timer1;

  J1.init();
  J2.init();
  J3.init();
  J4.init();
  J5.init();
  J6.init();

  j1_limit.init();
  j2_limit.init();
  j3_limit.init();
  j4_limit.init();
  j5_limit.init();
  j6_limit.init();
  
  //input0.init();
  input1.init();
  input2.init();
  input3.init();

 //pinMode(output0Pin, OUTPUT); digitalWriteFast(output0Pin, HIGH);
  //pinMode(output1Pin, OUTPUT); digitalWriteFast(output1Pin, HIGH);
  pinMode(output2Pin, OUTPUT); digitalWriteFast(output2Pin, HIGH);
  //pinMode(output3Pin, OUTPUT); digitalWriteFast(output3Pin, HIGH);

  pinMode(DriveEnablePin, OUTPUT); digitalWriteFast(DriveEnablePin, LOW);
  //pinMode(DriveAlarmPin, INPUT_PULLUP);

  pinMode(HeartbeatLed, OUTPUT); digitalWriteFast(HeartbeatLed, LOW);
  
}

bool tx_heartbeat, rx_heartbeat, rx_heartbeat_last, rx_heartbeat_lost;
bool tx_en, rx_en, rx_en_last;
bool tx_alarm;
bool tx_j1_limit, tx_j2_limit, tx_j3_limit;
bool tx_j4_limit, tx_j5_limit, tx_j6_limit;
bool tx_j1_limit_last, tx_j2_limit_last, tx_j3_limit_last;
bool tx_j4_limit_last, tx_j5_limit_last, tx_j6_limit_last;
bool rx_homing, rx_homing_last;
unsigned long rx_heartb_wdog, rx_heartb_wdog_last;
unsigned long j_lim_rd_tm[6], j_lim_rd_tm_last[6];
void loop() {

  EASYCAT.MainTask();

  handleTx();
  handleRx();
  handleInputs();
  handleOutputs();

  if (rx_homing != rx_homing_last) {
    rx_homing_last = rx_homing;
    if (rx_homing){
      J1encPos.write(0);
      J2encPos.write(0);
      J3encPos.write(0);
      J4encPos.write(0);
      J5encPos.write(0);
      J6encPos.write(0);
    }
  }

  rx_heartb_wdog = millis();
  if (rx_heartbeat != rx_heartbeat_last) {
    // reset watchdog timer
    digitalWriteFast(HeartbeatLed, (rx_heartbeat ? HIGH : LOW));
    rx_heartbeat_last = rx_heartbeat;
    rx_heartb_wdog_last = rx_heartb_wdog;
    rx_heartbeat_lost = false;
  }
  if (rx_heartb_wdog - rx_heartb_wdog_last > 2000){
    // heartbeat from master controller was lost
    rx_heartbeat_lost = true;
  }

  if (rx_en != rx_en_last || rx_heartbeat_lost){
    rx_en_last = rx_en;
    if (rx_en && !rx_heartbeat_lost){
      if (J1.turnON() && J2.turnON() && J3.turnON()
          && J4.turnON() && J5.turnON() && J6.turnON()) {
        digitalWriteFast(DriveEnablePin, HIGH);
        tx_en = true; // all drives are enabled
      } else {
        digitalWriteFast(DriveEnablePin, LOW);
        tx_en = false; // not all drives are enabled
      }
    } else {
      if (!J1.turnOFF() && !J2.turnOFF() && !J3.turnOFF()
          && !J4.turnOFF() && !J5.turnOFF() && !J6.turnOFF()) {
        digitalWriteFast(DriveEnablePin, LOW);
        tx_en = false; // all drives are disabled
      } else {
        digitalWriteFast(DriveEnablePin, HIGH);
        tx_en = true; // not all drives are disabled
      }
    }
  }

  timer1 = millis();
  if (timer1 - timer1_last > 1000) {
    timer1_last = timer1;
    tx_heartbeat = !tx_heartbeat;
    //handleSerial();
  }
}

/*
bool rxMutex;
void receiveEvent(int length){
  // check if buffer is already being read
  if (!rxMutex){
    // lock the rx buffer
    rxMutex = true;
    // parse the rx buffer
    int i = 0;
    while(Wire.available()) // loop through all but the last
    {
      RxBuffer[i++] = Wire.read();
    }
    // calculate crc of rx buffer
    RxCrc.u = calculateChecksum(RxBuffer, 30);
    // unlock the rx buffer
    rxMutex = false;
  }
}

void requestEvent() {

  // calculate checksum and append to tx buffer
  TxCrc.u = calculateChecksum(TxBuffer,30);
  TxBuffer[30] = TxCrc.b[0];
  TxBuffer[31] = TxCrc.b[1];

  Wire.write(TxBuffer,32);
}
*/

void handleTx(){

  // byte 0 is used for misc status bits
  TxBuffer[0] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  TxBuffer[0] = TxBuffer[0] | (tx_heartbeat ? B00000001 : B00000000);
  TxBuffer[0] = TxBuffer[0] | (tx_en ? B00000010 : B00000000);
  TxBuffer[0] = TxBuffer[0] | (tx_alarm ? B00000100 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00001000 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00010000 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B00100000 : B00000000);
  //TxBuffer[0] = TxBuffer[0] | (tx_ ? B01000000 : B00000000);
  TxBuffer[0] = TxBuffer[0] | (rx_heartbeat_lost ? B10000000 : B00000000);

  // bytes 1 through 24 are used for the encoders
  EncPosition[0].lval = J1encPos.read();
  EncPosition[1].lval = J2encPos.read();
  EncPosition[2].lval = J3encPos.read();
  EncPosition[3].lval = J4encPos.read();
  EncPosition[4].lval = J5encPos.read();
  EncPosition[5].lval = J6encPos.read();
  int k=1;
  for (int i=0;i<=5;i++){
    for (int j=0;j<=3;j++){
      TxBuffer[k++] = EncPosition[i].b[j];
    }
  }
  
  // byte 25 is used mostly for the limit switched
  TxBuffer[25] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  TxBuffer[25] = TxBuffer[25] | (tx_j1_limit ? B00000001 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (tx_j2_limit ? B00000010 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (tx_j3_limit ? B00000100 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (tx_j4_limit ? B00001000 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (tx_j5_limit ? B00010000 : B00000000);
  TxBuffer[25] = TxBuffer[25] | (tx_j6_limit ? B00100000 : B00000000);
  //TxBuffer[25] = TxBuffer[25] | (tx_ ? B01000000 : B00000000);
  //TxBuffer[25] = TxBuffer[25] | (tx_ ? B10000000 : B00000000);

  // byte 26 is used for gpio 0 through 7
  TxBuffer[26] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  TxBuffer[26] = TxBuffer[26] | (tx_input0 ? B00000001 : B00000000);
  TxBuffer[26] = TxBuffer[26] | (tx_input1 ? B00000010 : B00000000);
  TxBuffer[26] = TxBuffer[26] | (tx_input2 ? B00000100 : B00000000);
  TxBuffer[26] = TxBuffer[26] | (tx_input3 ? B00001000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B00010000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B00100000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B01000000 : B00000000);
  //TxBuffer[26] = TxBuffer[26] | (tx_ ? B10000000 : B00000000);

  //TxBuffer[27]
  //TxBuffer[28]
  //TxBuffer[29]

  // tx buffer index 30 and 31 will be for crc

  // calculate checksum and append to tx buffer
  TxCrc.u = calculateChecksum(TxBuffer,30);
  TxBuffer[30] = TxCrc.b[0];
  TxBuffer[31] = TxCrc.b[1];

  //Serial.write(TxBuffer,32);

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
      
      // byte 0 is used for misc command bits
      rx_heartbeat  = RxBuffer[0] & B00000001;
      rx_en         = RxBuffer[0] & B00000010;
      rx_homing     = RxBuffer[0] & B00000100;
      //rx_ = RxBuffer[0] & B00001000;
      //rx_ = RxBuffer[0] & B00010000;
      //rx_ = RxBuffer[0] & B00100000;
      //rx_ = RxBuffer[0] & B01000000;
      //rx_ = RxBuffer[0] & B10000000;

      // bytes 1 through 24 are used for velocity
      int k=1;
      for (int i=0;i<=5;i++){
        for (int j=0;j<=3;j++){
          SetVelocity[i].b[j]=RxBuffer[k++];
        }
      }

      // https://docs.google.com/spreadsheets/d/1S5TOxwbAx8pTMUdoWMnyC9WPfphhtUME/edit#gid=595805457
      // all drives are 200 pulses per revolution but we microstep so these are higher
      // need to multiply the gear ratio by ppr to get the desired output translation
      J1.run(SetVelocity[0].fval, 10 * (60/15) * 400); // gear * pulley * ppr
      J2.run(SetVelocity[1].fval, 50 * 400);
      J3.run(SetVelocity[2].fval, 50 * 400);
      // spec sheet for ratio is wrong its not 13+212/289 it is 16
      // manual shows 400 but its actually 600 ppr
      J4.run(SetVelocity[3].fval, 16 * (28/10) * 600); // gear * pulley * ppr
      J5.run(SetVelocity[4].fval, 9.81748 * 800); // 25*pi/8
      // spec sheet for ratio is wrong its not 19+38/187 it is 20+38/187
      J6.run(SetVelocity[5].fval, (1293/64) * 400); // gear ratio * ppr

      //RxBuffer[25]
      
      rx_output0 = RxBuffer[26] & B00000001;
      rx_output1 = RxBuffer[26] & B00000010;
      rx_output2 = RxBuffer[26] & B00000100;
      rx_output3 = RxBuffer[26] & B00001000;
      //rx_ = RxBuffer[26] & B00010000;
      //rx_ = RxBuffer[26] & B00100000;
      //rx_ = RxBuffer[26] & B01000000;
      //rx_ = RxBuffer[26] & B10000000;
      
      //RxBuffer[27]
      //RxBuffer[28]
      //RxBuffer[29]

      // 30 and 31 are reserved for crc
    }
}

void handleInputs(){

  tx_j1_limit = !j1_limit.read();
  tx_j2_limit = !j2_limit.read();
  tx_j3_limit = !j3_limit.read();
  tx_j4_limit = !j4_limit.read();
  tx_j5_limit = !j5_limit.read();
  tx_j6_limit = !j6_limit.read();

  //tx_input0 = !input0.read();
  tx_input1 = !input1.read();
  tx_input2 = !input2.read();
  tx_input3 = !input3.read();

  //tx_alarm = digitalReadFast(DriveAlarmPin);

}

void handleOutputs(){

  //digitalWriteFast(output0Pin, !rx_output0);
  //digitalWriteFast(output1Pin, !rx_output1);
  digitalWriteFast(output2Pin, !rx_output2);
  //digitalWriteFast(output3Pin, !rx_output3);

}

void handleSerial(){
  Serial.print(tx_heartbeat);
  Serial.print(" ");
  Serial.print(rx_heartbeat);
  Serial.print(" ");
  Serial.print(TxCrc.u);
  Serial.print(" ");
  Serial.print(TxCrc.b[0]);
  Serial.print(" ");
  Serial.print(TxCrc.b[1]);
  Serial.print(" ");
  Serial.print(RxCrc.u);
  Serial.print(" ");
  Serial.print(RxCrc.b[0]);
  Serial.print(" ");
  Serial.print(RxCrc.b[1]);
  Serial.println();
  /*Serial.print(rx_heartbeat_lost);
  Serial.print(" ");
  Serial.print(rx_en);
  Serial.print(" ");
  Serial.print(tx_en);
  Serial.print(" ");
  Serial.print(EncPosition[5].lval);
  Serial.print(" ");
  Serial.print(SetVelocity[5].fval);
  Serial.print(" ");
  Serial.println(RxBuffer[25]);*/
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