#include <Wire.h>
#include "PTO.h"
#include "InputDebounced.h"
#include "R_TRIG.h"
#include <Encoder.h>
#include <Servo.h>

unsigned long timer1, timer1_last;
uint8_t I2C_TxBuffer[32]; // i2c has max 32 byte
uint8_t I2C_RxBuffer[32]; // i2c has max 32 byte

// https://www.pjrc.com/teensy/pinout.html
const int input0Pin = 0; // D0 | PWM | RX1
const int input1Pin = 1; // D1 | PWM | TX1
const int input2Pin = 2; // D2 | PWM
const int input3Pin = 3; // D3 | PWM
const int output0Pin = 4; // D4 | PWM
const int output1Pin = 5; // D5 | PWM
const int output2Pin = 6; // D6 | PWM
const int output3Pin = 7; // D7 | PWM | RX2
const int pwm0Pin = 8; // D8 | PWM | TX2
const int pwm1Pin = 9; // D9 | PWM
const int pwm2Pin = 10; // D10 | PWM
const int pwm3Pin = 11; // D11 | PWM
//const int  = 12; // D12 | PWM
const int HeartbeatLed = 13; // D13 | PWM | LED
//const int  = 14; // D14 | A0 | PWM | TX3
//const int  = 15; // D15 | A1 | PWM | RX3
//const int  = 16; // D16 | A2 | SCL1 | RX4
//const int  = 17; // D17 | A3 | SDA1 | TX4
//const int  = 18; // D18 | A4 | PWM | SDA | RESERVED FOR I2C
//const int  = 19; // D19 | A5 | PWM | SCL | RESERVED FOR I2C
//const int  = 20; // D20 | A6 | TX5
//const int  = 21; // D21 | A7 | RX5
//const int  = 22; // D22 | A8 | PWM
//const int  = 23; // D23 | A9 | PWM
//const int  = 24; // D24 | A10 | PWM | SCL2 | TX6
//const int  = 25; // D25 | A11 | PWM | SDA2 | RX6
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

Servo Pwm0, Pwm1, Pwm2, Pwm3;
uint8_t rx_Pwm0, rx_Pwm1, rx_Pwm2, rx_Pwm3;

bool tx_input0, tx_input1, tx_input2, tx_input3;
bool rx_output0, rx_output1, rx_output2, rx_output3;

void setup() {

  Wire.begin(8); // join i2c bus with address 8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  
  Serial.begin(115200); // start serial for output

  timer1 = millis();
  timer1_last = timer1;
  
  //
  input0.init();
  input1.init();
  input2.init();
  input3.init();

  //
  pinMode(output0Pin, OUTPUT); digitalWriteFast(output0Pin, HIGH);
  pinMode(output1Pin, OUTPUT); digitalWriteFast(output1Pin, HIGH);
  pinMode(output2Pin, OUTPUT); digitalWriteFast(output2Pin, HIGH);
  pinMode(output3Pin, OUTPUT); digitalWriteFast(output3Pin, HIGH);

  //
  pinMode(HeartbeatLed, OUTPUT); digitalWriteFast(HeartbeatLed, LOW);

  //
  Pwm0.attach(pwm0Pin);
  Pwm1.attach(pwm1Pin);
  Pwm2.attach(pwm2Pin);
  Pwm3.attach(pwm3Pin);
  
}

bool tx_heartbeat, rx_heartbeat, rx_heartbeat_last, rx_heartbeat_lost;
bool tx_en, rx_en, rx_en_last;
unsigned long rx_heartb_wdog, rx_heartb_wdog_last;
bool OutputsEnabled;
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
    digitalWriteFast(HeartbeatLed, LOW);
  }

  if (rx_en != rx_en_last || rx_heartbeat_lost){
    rx_en_last = rx_en;
    if (rx_en && !rx_heartbeat_lost){
      OutputsEnabled = true;
    } else {
      OutputsEnabled = false;
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
      
      rx_output0 = I2C_RxBuffer[1] & B00000001;
      rx_output1 = I2C_RxBuffer[1] & B00000010;
      rx_output2 = I2C_RxBuffer[1] & B00000100;
      rx_output3 = I2C_RxBuffer[1] & B00001000;
      //rx_ = I2C_RxBuffer[1] & B00010000;
      //rx_ = I2C_RxBuffer[1] & B00100000;
      //rx_ = I2C_RxBuffer[1] & B01000000;
      //rx_ = I2C_RxBuffer[1] & B10000000;

      rx_Pwm0 = constrain(I2C_RxBuffer[2], 0 , 180);
      rx_Pwm1 = constrain(I2C_RxBuffer[3], 0 , 180);
      rx_Pwm2 = constrain(I2C_RxBuffer[4], 0 , 180);
      rx_Pwm3 = constrain(I2C_RxBuffer[5], 0 , 180);
      
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

  if (OutputsEnabled){
    digitalWriteFast(output0Pin, !rx_output0);
    digitalWriteFast(output1Pin, !rx_output1);
    digitalWriteFast(output2Pin, !rx_output2);
    digitalWriteFast(output3Pin, !rx_output3);

    if (Pwm0.attached()){    
      Pwm0.write(rx_Pwm0);
    } else {
      Pwm0.attach(pwm0Pin);
    }
    if (Pwm1.attached()){    
      Pwm1.write(rx_Pwm1);
    } else {
      Pwm1.attach(pwm1Pin);
    }
    if (Pwm2.attached()){    
      Pwm2.write(rx_Pwm2);
    } else {
      Pwm2.attach(pwm2Pin);
    }
    if (Pwm3.attached()){    
      Pwm3.write(rx_Pwm3);
    } else {
      Pwm3.attach(pwm3Pin);
    }
  } 
  else {
    digitalWriteFast(output0Pin, true);
    digitalWriteFast(output1Pin, true);
    digitalWriteFast(output2Pin, true);
    digitalWriteFast(output3Pin, true);
    Pwm0.detach();
    Pwm1.detach();
    Pwm2.detach();
    Pwm3.detach();
  }

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