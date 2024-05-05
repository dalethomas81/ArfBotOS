#include <Wire.h>
#include "PTO.h"
#include "InputDebounced.h"
#include "R_TRIG.h"
#include <Encoder.h>
#include <Servo.h>

unsigned long timer1, timer1_last;
uint8_t I2C_TxBuffer[32]; // i2c has max 32 byte
uint8_t I2C_RxBuffer[32]; // i2c has max 32 byte

const int input0Pin = 0;
const int input1Pin = 1;
const int input2Pin = 2;
const int input3Pin = 3;
const int output0Pin = 4;
const int output1Pin = 5;
const int output2Pin = 6;
const int output3Pin = 7;
const int HeartbeatLed = 13;

// NOTE: make sure to use external pullup resistors for i2c pins
// 4.7k or 10k will work
// pin 18 reserved for i2c
// pin 19 reserved for i2c

union u_crc {
  uint8_t b[2];
  uint16_t u;
} TxCrc, RxCrc;

union u_pto {
  uint8_t b[4];
  float fval;
} SetVelocity[6];

union u_enc {
  uint8_t b[4];
  long lval;
} EncPosition[6];

InputDebounced input0(input0Pin, INPUT_PULLUP, 1);
InputDebounced input1(input1Pin, INPUT_PULLUP, 1);
InputDebounced input2(input2Pin, INPUT_PULLUP, 1);
InputDebounced input3(input3Pin, INPUT_PULLUP, 1);

bool tx_input0, tx_input1, tx_input2, tx_input3;
bool rx_output0, rx_output1, rx_output2, rx_output3;

void setup() {

  Wire.begin(8); // join i2c bus with address 8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  
  Serial.begin(115200);           // start serial for output

  timer1 = millis();
  timer1_last = timer1;
  
  input0.init();
  input1.init();
  input2.init();
  input3.init();

  pinMode(output0Pin, OUTPUT); digitalWriteFast(output0Pin, HIGH);
  pinMode(output1Pin, OUTPUT); digitalWriteFast(output1Pin, HIGH);
  pinMode(output2Pin, OUTPUT); digitalWriteFast(output2Pin, HIGH);
  pinMode(output3Pin, OUTPUT); digitalWriteFast(output3Pin, HIGH);

  pinMode(HeartbeatLed, OUTPUT); digitalWriteFast(HeartbeatLed, LOW);
  
}

bool tx_heartbeat, rx_heartbeat, rx_heartbeat_last, rx_heartbeat_lost;
bool tx_en, rx_en, rx_en_last;
unsigned long rx_heartb_wdog, rx_heartb_wdog_last;
void loop() {

  handleTx();
  handleRx();
  handleInputs();
  handleOutputs();

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
      //
    } else {
      //
    }
  }

  timer1 = millis();
  if (timer1 - timer1_last > 1000) {
    timer1_last = timer1;
    tx_heartbeat = !tx_heartbeat;
    //handleSerial();
  }
}

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
      I2C_RxBuffer[i++] = Wire.read();
    }
    // calculate crc of rx buffer
    RxCrc.u = calculateChecksum(I2C_RxBuffer, 30);
    // unlock the rx buffer
    rxMutex = false;
  }
}

void requestEvent() {

  // calculate checksum and append to tx buffer
  TxCrc.u = calculateChecksum(I2C_TxBuffer,30);
  I2C_TxBuffer[30] = TxCrc.b[0];
  I2C_TxBuffer[31] = TxCrc.b[1];

  Wire.write(I2C_TxBuffer,32);
}

void handleTx(){

  // byte 0 is used for misc status bits
  I2C_TxBuffer[0] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  I2C_TxBuffer[0] = I2C_TxBuffer[0] | (tx_heartbeat ? B00000001 : B00000000);
  I2C_TxBuffer[0] = I2C_TxBuffer[0] | (tx_en ? B00000010 : B00000000);
  //I2C_TxBuffer[0] = I2C_TxBuffer[0] | (tx_ ? B00000100 : B00000000);
  //I2C_TxBuffer[0] = I2C_TxBuffer[0] | (tx_ ? B00001000 : B00000000);
  //I2C_TxBuffer[0] = I2C_TxBuffer[0] | (tx_ ? B00010000 : B00000000);
  //I2C_TxBuffer[0] = I2C_TxBuffer[0] | (tx_ ? B00100000 : B00000000);
  //I2C_TxBuffer[0] = I2C_TxBuffer[0] | (tx_ ? B01000000 : B00000000);
  //I2C_TxBuffer[0] = I2C_TxBuffer[0] | (rx_ ? B10000000 : B00000000);

  // byte 26 is used for gpio 0 through 3
  I2C_TxBuffer[26] = 0x00; // clear it out first (maybe there is a better way of setting bools to bits?)
  I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_input0 ? B00000001 : B00000000);
  I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_input1 ? B00000010 : B00000000);
  I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_input2 ? B00000100 : B00000000);
  I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_input3 ? B00001000 : B00000000);
  //I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_ ? B00010000 : B00000000);
  //I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_ ? B00100000 : B00000000);
  //I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_ ? B01000000 : B00000000);
  //I2C_TxBuffer[26] = I2C_TxBuffer[26] | (tx_ ? B10000000 : B00000000);

  //I2C_TxBuffer[27]
  //I2C_TxBuffer[28]
  //I2C_TxBuffer[29]

  // tx buffer index 30 and 31 will be for crc

}

void handleRx(){
  // check if buffer is already being read
  if (!rxMutex){
    // lock the rx buffer
    //rxMutex = true;
    // compare calc crc with rx crc
    if (RxCrc.b[0] == I2C_RxBuffer[30] && RxCrc.b[1] == I2C_RxBuffer[31]) {
      
      // byte 0 is used for misc command bits
      rx_heartbeat  = I2C_RxBuffer[0] & B00000001;
      rx_en         = I2C_RxBuffer[0] & B00000010;
      //rx_ = I2C_RxBuffer[0] & B00000100;
      //rx_ = I2C_RxBuffer[0] & B00001000;
      //rx_ = I2C_RxBuffer[0] & B00010000;
      //rx_ = I2C_RxBuffer[0] & B00100000;
      //rx_ = I2C_RxBuffer[0] & B01000000;
      //rx_ = I2C_RxBuffer[0] & B10000000;
      
      rx_output0 = I2C_RxBuffer[26] & B00000001;
      rx_output1 = I2C_RxBuffer[26] & B00000010;
      rx_output2 = I2C_RxBuffer[26] & B00000100;
      rx_output3 = I2C_RxBuffer[26] & B00001000;
      //rx_ = I2C_RxBuffer[26] & B00010000;
      //rx_ = I2C_RxBuffer[26] & B00100000;
      //rx_ = I2C_RxBuffer[26] & B01000000;
      //rx_ = I2C_RxBuffer[26] & B10000000;
      
      //I2C_RxBuffer[27]
      //I2C_RxBuffer[28]
      //I2C_RxBuffer[29]

      // 30 and 31 are reserved for crc
    }
    
    // unlock the rx buffer
    //rxMutex = false;
  }

}

void handleInputs(){

  tx_input0 = !input0.read();
  tx_input1 = !input1.read();
  tx_input2 = !input2.read();
  tx_input3 = !input3.read();

}

void handleOutputs(){

  digitalWriteFast(output0Pin, !rx_output0);
  digitalWriteFast(output1Pin, !rx_output1);
  digitalWriteFast(output2Pin, !rx_output2);
  digitalWriteFast(output3Pin, !rx_output3);

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
  Serial.println(I2C_RxBuffer[25]);*/
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