
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
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "RF_tool.h"
//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(9, 10);




/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
const int interruptPin=2;
unsigned int OnOff=0,previousTime=0,interval=0,dif=0;
const int pumpPin = 3;    // connected to the base of the transistor
unsigned int  answer=0;
byte addresses[][6] = {"Pump","Contr"};
                                                       // A single byte to keep track of the data being sent back and forth

// Sleep declarations
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;

const short sleep_cycles_per_transmission = 4;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;

void setup_watchdog(uint8_t prescalar);
void do_sleep(void);



void setup(){

  Serial.begin(115200);
  while(!Serial){}
  pinMode(pumpPin, OUTPUT);
  

  radio.begin();

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(112);
  
  //radio.enableAckPayload();                     // Allow optional ack payloads
  //radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  radio.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1,addresses[0]);      // Open a reading pipe on address 0, pipe 1
    
  radio.startListening();                       // Start listening
  //radio.writeAckPayload(1,&counter,1);          // Pre-load an ack-paylod into the FIFO buffer for pipe 1

  //Print debug info
  radio.printDetails();

  interval=0;

   setup_watchdog(wdt_8s);
}


void loop(void) {   
 unsigned long currentTime=millis();    
 if(radio.available())
 {
  Serial.println(F("Available radio "));
  while (radio.available()) {                                   // While there is data ready
    radio.read( &OnOff, sizeof(unsigned int) );             // Get the payload
  }
  Serial.print(F("Received payload "));Serial.println(OnOff,DEC);
  digitalWrite(pumpPin, HIGH);
  interval=OnOff*1000L;
  previousTime=millis();
  Serial.print(F("Calculated interval is "));Serial.print(interval);Serial.println(F("ms"));
  //delay(OnOff*1000);
 }else if(interval>0 ){
    dif=(currentTime-previousTime);
    
   if((dif> interval)){
     Serial.print(F("ElapseTime["));Serial.print(dif);Serial.print(F("] greater than Interval["));Serial.print(interval);Serial.println(F("]"));
    radio.stopListening();
    answer=OnOff;
    digitalWrite(pumpPin, LOW);
    delay(1000);
    radio.write(&answer,sizeof(unsigned int));
    radio.startListening();
    
    interval=0;
     Serial.println(F("Intervall reset:"));
  }
  
 }else{
   Serial.println(F("Sleeping"));
   delay(50); 
  do_sleep();
 }

   radio.printDetails(); 
}




void wakeUp(){
  Serial.println(F("Wake up by Interrupt."));
   delay(50);
  sleep_disable();
}


void setup_watchdog(uint8_t prescalar){

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
}

ISR(WDT_vect)
{
  //--sleep_cycles_remaining;
  Serial.println(F("WDT"));
}



void do_sleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(interruptPin),wakeUp,LOW);
  WDTCSR |= _BV(WDIE);
  sleep_mode();                        // System sleeps here
                                       // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out  
  detachInterrupt(0);  
  WDTCSR &= ~_BV(WDIE);  

}

