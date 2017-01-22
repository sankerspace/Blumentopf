

/**************PUMP NODE SETTINGS****************************/
#include "Blumentopf.h"
#include <SPI.h>
#include "printf.h"
#include "nRF24L01.h"
#include "RF24.h"
//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(9, 10);




/****************** User Config ***************************/


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
 String out="";  
 if(radio.available() && status==0)
 {
   
  
    DEBUG_PRINTLNSTR("Available radio ");
    radio.read( &OnOff, sizeof(unsigned int) );             // Get the payload
    out="Received payload " + OnOff;
    DEBUG_PRINTLN(out);
    if(OnOff>0){
      status=1;
      digitalWrite(pumpPin, HIGH);
      interval=OnOff*1000L;
      previousTime=millis();
      out="Calculated interval is " + String(interval,DEC) + "ms";
      DEBUG_PRINT(out);
      //Serial.print(F("Calculated interval is "));Serial.print(interval);Serial.println(F("ms"));
    }else{
      status=2;  
    }
    Serial.flush();
 
 }else if(status==1){
    dif=(currentTime-previousTime);
    //printCounter(interval,dif);
   if((dif> interval)){
      //int state=0;
      out="ElapseTime[" + String(dif,DEC) + "] greater than Interval[" + String(interval,DEC) + "]";
       DEBUG_PRINT(out);
     // Serial.print(F("ElapseTime["));Serial.print(dif);Serial.print(F("] greater than Interval["));Serial.print(interval);Serial.println(F("]"));
     
      //state=radio.flush_tx();   
      answer=OnOff;
      DEBUG_PRINTLNSTR("Turn off the pump ");delay(50);
      //Serial.println(F("Turn off the pump "));delay(50);
      digitalWrite(pumpPin, LOW);
      out = "Send answer to the controller: " + String(answer,out);
      DEBUG_PRINTLN(out);
      radio.stopListening();
      radio.write(&answer,sizeof(unsigned int));
      radio.startListening();
      DEBUG_PRINTLNSTR("Start listening again...: ");
      status=0;
      DEBUG_PRINTLNSTR("Intervall reseted, we are finished one pump cycle...........................");
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










