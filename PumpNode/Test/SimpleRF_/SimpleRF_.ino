

/**************PUMP NODE SETTINGS****************************/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(9, 10);




/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
int OnOff=-100;
int cnt=0;
const int LEDPin = 7;    // connected to the base of the transistor


/**********************************************************/
//bool  radioNumber = true;                                                                         // Topology
//byte addresses[][6] = {"Ardui","Photo"};
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };   // Radio pipe addresses for the 2 nodes to communicate.



void setup(){

  Serial.begin(115200);
  

 radio.begin();

 radio.setPALevel(RF24_PA_LOW);
 radio.openWritingPipe(pipes[1]);
 radio.openReadingPipe(1,pipes[0]);
/*  if(radioNumber){
    radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
    radio.openReadingPipe(1,addresses[0]);      // Open a reading pipe on address 0, pipe 1
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
 */  
radio.startListening();
  //Print debug infoS
radio.printDetails();


 //  radio.powerUp();                        //Power up the radio
}

void loop(void) {
 
 
 if( radio.available()){
                                                     
      while (radio.available()) {                       // While there is data ready
        radio.read(&OnOff, sizeof(int) );             // Get the payload
      }
      Serial.print(F("Value received: "));
      Serial.println(OnOff,DEC);
      
                                          
      
   }
 
  
}


