
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
#include <printf.h>
#include "nRF24L01.h"
#include "RF24.h"
//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(9, 10);




/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/

int OnOff=0;
int cnt=0;
const int pumpPin = 3;    // connected to the base of the transistor

byte addresses[][6] = {"Pump","Contr"};
byte counter = 1;                                                          // A single byte to keep track of the data being sent back and forth


void setup(){

  Serial.begin(115200);
  
  pinMode(pumpPin, OUTPUT);
  

  radio.begin();

  radio.setPALevel(RF24_PA_LOW);

  
  //radio.enableAckPayload();                     // Allow optional ack payloads
  //radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1,addresses[0]);      // Open a reading pipe on address 0, pipe 1
    
  radio.startListening();                       // Start listening
  //radio.writeAckPayload(1,&counter,1);          // Pre-load an ack-paylod into the FIFO buffer for pipe 1

   // Start the radio listening for data
  radio.startListening();
  
  
  //Print debug info
  radio.printDetails();
}

void loop(void) {

      
 if( radio.available()){
                                                  
      while (radio.available()) {                                   // While there is data ready
        radio.read( &OnOff, sizeof(OnOff) );             // Get the payload
      }
       Serial.print(F("Received payload "));Serial.println(OnOff,DEC);
      digitalWrite(pumpPin, HIGH);
      delay(OnOff*1000);
      digitalWrite(pumpPin, LOW);
       delay(100);
   }


 

  
}


