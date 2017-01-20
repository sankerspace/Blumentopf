
/*
   Dec 2014 - TMRh20 - Updated
   Derived from examples by J. Coliz <maniacbug@ymail.com>
*/
/**
 * Example for efficient call-response using ack-payloads
 *
 * This example continues to make use of all the normal functionality of the radios including
 * the auto-ack and auto-retry features, but allows ack-payloads to be written optionlly as well.
 * This allows very fast call-response communication, with the responding radio never having to
 * switch out of Primary Receiver mode to send back a payload, but having the option to switch to
 * primary transmitter if wanting to initiate communication instead of respond to a commmunication.
 */



/*
  PINOUTS
  http://docs.spark.io/#/firmware/communication-spi
  http://maniacbug.wordpress.com/2011/11/02/getting-started-rf24/

  SPARK CORE    SHIELD SHIELD    NRF24L01+
  GND           GND              1 (GND)
  3V3 (3.3V)    3.3V             2 (3V3)
  D6 (CSN)      9  (D6)          3 (CE)
  A2 (SS)       10 (SS)          4 (CSN)
  A3 (SCK)      13 (SCK)         5 (SCK)
  A5 (MOSI)     11 (MOSI)        6 (MOSI)
  A4 (MISO)     12 (MISO)        7 (MISO)

  NOTE: Also place a 10-100uF cap across the power inputs of
        the NRF24L01+.  I/O o fthe NRF24 is 5V tolerant, but
        do NOT connect more than 3.3V to pin 1!!!
 */


/**************PUMP NODE SETTINGS****************************/

#include <SPI.h>
#include "printf.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "RF_tool.h"
//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(9, 10);




/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/

unsigned int OnOff=0, answer=0;
unsigned long previousTime=0,interval=0,dif=0;
const int pumpPin = 3;   
int status;
byte addresses[][6] = {"Pump","Contr"};

void setup(){

  Serial.begin(115200);
  while(!Serial){}
  pinMode(pumpPin, OUTPUT);
  

  radio.begin();

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(112);

  radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1,addresses[0]);      // Open a reading pipe on address 0, pipe 1
    
  radio.startListening();                       // Start listening
  

  //Print debug info
  radio.printDetails();

  interval=0;
  status=0;
 
}


void loop(void) {   
 unsigned long currentTime=millis();  
   
 if(radio.available() && status==0)
 {
    Serial.println(F("Available radio "));
    radio.read( &OnOff, sizeof(unsigned int) );             // Get the payload
    Serial.print(F("Received payload "));Serial.println(OnOff,DEC);
    if(OnOff>0){
      status=1;
      digitalWrite(pumpPin, HIGH);
      interval=OnOff*1000L;
      previousTime=millis();
      Serial.print(F("Calculated interval is "));Serial.print(interval);Serial.println(F("ms"));
    }else{
      status=2;  
    }
    Serial.flush();
 
 }else if(status==1){
    dif=(currentTime-previousTime);
    //printCounter(interval,dif);
   if((dif> interval)){
      //int state=0;
      Serial.print(F("ElapseTime["));Serial.print(dif);Serial.print(F("] greater than Interval["));Serial.print(interval);Serial.println(F("]"));
     
      //state=radio.flush_tx();   
      answer=OnOff;
      Serial.println(F("Turn off the pump "));delay(50);
      digitalWrite(pumpPin, LOW);
      Serial.print(F("Send answer to the controller: "));Serial.println(answer);delay(50);
      radio.stopListening();
      radio.write(&answer,sizeof(unsigned int));
      radio.startListening();
      Serial.println(F("Start listening again...: "));
     
      status=0;
      Serial.println(F("Intervall reseted, we are finished one pump cycle..........................."));
  }
  
 }else if(status==2)
 {
   answer=-1;
   radio.stopListening();
   radio.write(&answer,sizeof(unsigned int));
   radio.startListening(); 
   status=0;
 }
  
   //radio.printDetails(); 
}










