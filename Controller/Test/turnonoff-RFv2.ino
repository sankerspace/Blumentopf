/**************CONTROLLER SETTINGS****************************/

#include "particle-rf24/particle-rf24.h"


RF24 radio(D6,A2);
//
// Hardware configuration
// from https://github.com/mshoemaker/SparkCore-RF24


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
unsigned int OnOff=1,answer=0;
unsigned long started_waiting_at=0;
int status=0;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins D6 & A2 */

/**********************************************************/

byte addresses[][6] = {"Pump","Contr"};



//byte counter = 1;                                                          // A single byte to keep track of the data being sent back and forth


void setup(){

  Serial.begin(115200);
  while(!Serial){}
  
  radio.begin();

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(112);
  
 // radio.enableAckPayload();                     // Allow optional ack payloads
 // radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);

  //radio.startListening();                       // Start listening  
  //radio.writeAckPayload(1,&counter,1);          // Pre-load an ack-paylod into the FIFO buffer for pipe 1
  //radio.startListening(); 

  //Print debug info
  radio.printDetails();
}

void loop(void) {

/****************** Ping Out Role ***************************/

    
    
    if(!status && !radio.write(&OnOff,sizeof(unsigned int)) ){
            Serial.println(F("Sending failed."));           // If no ack response, sending failed
    }else{
        
        radio.startListening(); 
        if(!status){
            Serial.println(F("Start Listening answer..."));
            started_waiting_at = millis();               // Set up a timeout period, get the current microseconds
            status=1;
        }
        
        unsigned long dif=0;
        boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
      
      
        
        while ( !radio.available() && status==1){                             // While nothing is received
            dif=(millis() - started_waiting_at);
            if (dif > (2*OnOff*1000) ){               // If waited longer than 200ms, indicate timeout and exit while loop
                timeout = true;
                status=2;
                break;
            }      
        }
        
        
        if ( timeout && status==2){                                             // Describe the results
            Serial.print(F("Failed, response timed out[wait:"));
            Serial.print(dif);Serial.print(F(",OnOff:"));Serial.print(OnOff);Serial.println(F("]"));
        }else{
        
            radio.read( &answer, sizeof(unsigned int));
            OnOff++;
            Serial.print(F("Answer: "));  
            Serial.println(answer,DEC);
        }
        status=0;
        radio.stopListening(); 
        Serial.print(F("Start Sending next period:  "));Serial.println(OnOff);
    }
    
   

 
}//loop()



