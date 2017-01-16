//That code must be included a app created in the online WEB-IDE
//Particle-24 library must be included

// This #include statement was automatically added by the Particle IDE.
#include "particle-rf24/particle-rf24.h"


/**************CONTROLLER SETTINGS****************************/

#include "particle-rf24/particle-rf24.h"

RF24 radio(D6,A2);


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/

int OnOff=-100;

int mode = LOW;
//const int LEDPin = D7;    // connected to the base of the transistor

//bool  radioNumber = false;  
//byte addresses[][6] = {"Ardui","Photo"};
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };   // Radio pipe addresses for the 2 nodes to communicate.


void setup(){
  pinMode(D7, OUTPUT); 
  digitalWrite(D7,LOW);
  Serial.begin(115200);
  
  
  radio.begin();
  
  radio.setPALevel(RF24_PA_LOW);


 /* if(radioNumber){
    radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
    radio.openReadingPipe(1,addresses[0]);      // Open a reading pipe on address 0, pipe 1
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }*/
 radio.openWritingPipe(pipes[0]);
 radio.openReadingPipe(1,pipes[1]);
 //radio.startListening();
 
  
  
  //Print debug info
  radio.printDetails();
  
  //radio.powerUp();                        //Power up the radio
}

void loop(void) {
mode=(mode ^ 0x1) ;
 
 Serial.print(F("Value will be send: "));Serial.println(OnOff,DEC);
  if(radio.write(&OnOff,sizeof(int)))
  {
     Serial.println(F("Value sended.... "));
     digitalWrite(D7,mode);
           //blank
  }
  else{
    Serial.println(F("Sending failed."));           // If no ack response, sending failed
  } 
 // Serial.println(F("Value increased.... "));
  OnOff++;


 delay(1000);

  
  
}//loop()

