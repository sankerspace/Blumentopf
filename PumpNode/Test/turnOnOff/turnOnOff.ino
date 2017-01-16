

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(6, 10);
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL}; // pipe[0] ist answer-channel


const int transistorPin = 3;    // connected to the base of the transistor
 //const int transistorPin = 9;
 void setup() {
   // set  the transistor pin as output:
   pinMode(transistorPin, OUTPUT);
 }
 
 void loop() {
   digitalWrite(transistorPin, HIGH);
   delay(2000);
   digitalWrite(transistorPin, LOW);
   delay(5000);
 }
