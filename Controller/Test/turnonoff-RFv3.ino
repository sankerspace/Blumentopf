// This #include statement was automatically added by the Particle IDE.
#include "particle-rf24/particle-rf24.h"

/**************CONTROLLER SETTINGS****************************/

#include "particle-rf24/particle-rf24.h"


RF24 radio(D6,A2);
//
// Hardware configuration
// from https://github.com/mshoemaker/SparkCore-RF24


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
const unsigned int criticalTime=60000; //1min maximal freeze of the MCU allowed/
///////////////////////////////////////////////////////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!CHANGE THAT VALUE , ITS TOO SLOW
unsigned int OnOff=5,answer=0;
unsigned long started_waiting_at=0,previousTime=0;
int status=0;
int cnt=0;
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

    
    /* Here starts a communication cycle with a node
    *Controller send data to a node and the  node must answer
    *with a report or with payload of data
    * status [0 - 2] interaction with pumpNode
    */
    
    if(!status && !radio.write(&OnOff,sizeof(unsigned int)) ){
        if(!cnt){
            Serial.println(F("Sending failed."));           // If no ack response, sending failed
            cnt++;
        }
    }else{
        previousTime=millis();//A change of state occured here
        cnt=0;
        radio.startListening(); 
        if(!status){
            Serial.println(F("Start Listening answer..."));
                    
            status=1;
        }
        started_waiting_at = millis();       
        unsigned long dif=0;
        boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
      
      
        
        while ( !radio.available() && status==1){                             // Wait for response
            dif=(millis() - started_waiting_at);
            if (dif > (2*OnOff*1000) ){              
                timeout = true;
                status=2;
                 //previousTime=millis();//A change of state occured here
                break;
            }      
        }
        
        
        if ( timeout && status==2){                                             // Describe the results
            Serial.print(F("Failed, response timed out[wait:"));
            Serial.print(dif);Serial.print(F(",OnOff:"));Serial.print(OnOff);Serial.println(F("]"));
            status=1;// a change of state occured but we are in a critical situation 
        }else{
        
            radio.read( &answer, sizeof(unsigned int));
            OnOff+=5;
            Serial.print(F("Answer: "));  
            Serial.println(answer,DEC);
            status=0;
            radio.stopListening(); 
            Serial.print(F("Start Sending next period:  "));Serial.println(OnOff);
            delay(OnOff*1000);
             previousTime=millis();//A change of state occured here
        }
        
    }
    if((millis()-previousTime)>criticalTime)
    {
        bool tx_ok,tx_fail,rx_ready;
         Serial.println(F("NO ANSWER, WE WILL RESET THE STATE MACHINE!!"));
         delay(50);
        //RESET THE STATE COORDINATION
        radio.whatHappened 	(tx_ok,tx_fail,rx_ready);
        if(tx_ok || tx_fail || rx_ready) {
            Serial.print(F("TX_OK:"));Serial.println(tx_ok);
            Serial.print(F("TX_FAIL:"));Serial.println(tx_fail);
            Serial.print(F("RX_READY:"));Serial.println(rx_ready);
        }
        
	 	
        if (radio.rxFifoFull())
        {
            Serial.println(F("RX Buffer FULL."));
        }
        status=0;// CHANGE TO A COMFORTABLE STATE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //only for testing with PumpNode now
         previousTime=millis();//A change of state occured here
    }
    


 
}//loop()



