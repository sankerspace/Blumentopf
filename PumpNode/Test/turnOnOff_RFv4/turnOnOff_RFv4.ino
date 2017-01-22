

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
 /********STATE 0*******/  
 if(status==0){
   if(radio.available()){
      DEBUG_PRINTLNSTR("------------------------------------------------------");
      DEBUG_PRINTLNSTR("Available radio ");
      radio.read( &OnOff, sizeof(unsigned int) );             // Get the payload
      out="[Status 0]Received payload " + String(OnOff,DEC);
      DEBUG_PRINTLN(out);
      if(OnOff>0){
        status=1;
        digitalWrite(pumpPin, HIGH);
        interval=OnOff*1000L;
        previousTime=millis();
        out="[Status 0]Pump will work for " + String(interval,DEC) + "ms";
        DEBUG_PRINTLN(out);
        answer=OnOff;
        DEBUG_PRINTLNSTR("Send acknowledgment to the pump request.");
        /********Sending acknowlegment,which is the same number as OnOff time request**************/
        radio.stopListening();
        radio.write(&answer,sizeof(unsigned int));
        //no listening, because a second confirmation must be send
       /*******************************************************************************************/
      }else{
        status=2;  
      }
      Serial.flush();
   }
  /********STATE 1*******/ 
 }else if(status==1){
    dif=(currentTime-previousTime);
    
   if((dif> interval)){
      out="[Status 1]ElapseTime[" + String(dif,DEC) + "] greater than Interval[" + String(interval,DEC) + "]";
      DEBUG_PRINTLN(out);
      answer=dif;//send him the total time needed
      DEBUG_PRINTLNSTR("Turn off the pump ");delay(50);
      digitalWrite(pumpPin, LOW);
      out = "[Status 1]Send final confirmation to the controller-ElapsedTime: " + String(answer,DEC);
      DEBUG_PRINTLN(out);
      /********Sending final confirmation,which is the total time measured during pump activation**/
      //is already in Transfer Mode
      radio.write(&answer,sizeof(unsigned int));
      radio.startListening();
      /*******************************************************************************************/
      DEBUG_PRINTLNSTR("[Status 1]Start listening again...: ");
      status=0;
      Serial.flush();
  }
   /********STATE 2*******/ 
 }else if(status==2)
 {
   /*There was a bad pump time request*/
   answer=-1;
    DEBUG_PRINTLNSTR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
   out="[Status 2]A bad pump time request was received:  " + String(answer,DEC);
    DEBUG_PRINTLN(out);
   radio.stopListening();
   radio.write(&answer,sizeof(unsigned int));
   radio.startListening(); 
   DEBUG_PRINTLNSTR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
   status=0;
 }
  
   //radio.printDetails(); 
}










