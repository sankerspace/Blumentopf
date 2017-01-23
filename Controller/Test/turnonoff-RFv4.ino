

// This #include statement was automatically added by the Particle IDE.
#include "particle-rf24/particle-rf24.h"

/**************CONTROLLER SETTINGS****************************/



// Hardware configuration
// from https://github.com/mshoemaker/SparkCore-RF24
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins D6 & A2 */
RF24 radio(D6,A2);


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
unsigned int criticalTime=60000; 
int timeOFF=5000;//maximal 5s to wait for an confirmation
///////////////////////////////////////////////////////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!CHANGE THAT VALUE , ITS TOO SLOW
unsigned int OnOff=1;
unsigned long previousTime=0;
unsigned long started_waiting_at=0;
int status=0;

/**********************************************************/

byte addresses[][6] = {"Pump","Contr"};



void setup(){

  Serial.begin(115200);
  while(!Serial){}
  
  radio.begin();

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(112);
  

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);

  //Print debug info
  radio.printDetails();
  status=0;
  OnOff=5;
  previousTime=0;
}

void loop(void) {

/****************** Ping Out Role ***************************/
    unsigned long dif=0;
    unsigned int answer=0;
    
  
  
    /* Here starts a communication cycle with a node
    *Controller send data to a node and the  node must answer
    *with a report or with payload of data
    * status [0 - 2] interaction with pumpNode
    *status 0: Sending Mode-Try to send pumping time, until sending operation was successfull,then initialize Receiving Mode
    *status 1: Receiving Mode
    *status 2: Receiving Mode-Error State
 
    */
    /*
    *State 1: Try to send next pump time -> SENDING
    */
    
    if(status==0){
        radio.stopListening();
        radio.write(&OnOff,sizeof(unsigned int));
        radio.startListening(); 
        Serial.println(F("[State 0:]Pump time sended ."));
        status=1;
        previousTime=millis();//A change of state occured here
        started_waiting_at = millis();       
        Serial.println(F("[State 0:]Start Listening first confirmation from the pump..."));
    }   
    /*
         unsigned long dif=0;
        unsigned int answer=0;
    
    *   unsigned int criticalTime=60000; 
        unsigned int OnOff=1;
        unsigned long previousTime=0;
        unsigned long started_waiting_at=0;
        int status=0;
    */
    
    if(status==1){
        /*
            *State 1: Wait for answer from Node in Receiving Mode -> RECEIVING PENDING
        */
        if(!radio.available()){
            dif=(millis() - started_waiting_at);
            if (dif > timeOFF ){              
                
                status=3;

            }
        }else
        {
            String answ = "[State 1:]First Confirmation: ";
            radio.read( &answer, sizeof(unsigned int));
            answ = answ + answer;  
            Serial.println(answ);
            started_waiting_at = millis();   
            previousTime=millis();//A change of state occured here
            status=2;
        }
    }  
    
    
    if(status==2){
        /*
            *State 2: Wait for answer from Node in Receiving Mode -> RECEIVING PENDING
        */
        if(!radio.available()){
            dif=(millis() - started_waiting_at);
            if (dif > (2*OnOff*1000) ){              
                
                status=4;

            }
        }else
        /*
        *   State 2: Node answered  in Receiving Mode-> RECEIVING SUCCESSFULL
        */
        {
            String answ = "[State 2:]Second Confirmation: ";
            radio.read( &answer, sizeof(unsigned int));
            answ = answ + answer;
            if(answer>0)
                OnOff+=5;
               
            
        
            Serial.println(answ);
            status=0;
            previousTime=millis();//A change of state occured here
        
            delay(2000);
            Serial.print(F("[State 0:]Start Sending next period:  "));Serial.println(OnOff);
            //delay(OnOff*1000);           
        }
    }
    
    
    
    /*
    *State 3: Receiving answer unsuccessfull -> RECEIVING UNSUCCESSFULL
    */
    if (status==3){                                           
        Serial.print(F("[State 3:]Failed, response timed out[wait:"));
        Serial.print(dif);Serial.print(F(",OnOff:"));Serial.print(OnOff);Serial.println(F("]"));
        started_waiting_at=millis();
        status=1;// a change of state occured but we are still in a critical situation,watchdog can break that cycle between state 1 and 2
    }
    /*
    *State 4: Receiving answer unsuccessfull -> RECEIVING UNSUCCESSFULL
    */
    if (status==4){                                           
        Serial.print(F("[State 4:]Failed, response timed out[wait:"));
        Serial.print(dif);Serial.print(F(",OnOff:"));Serial.print(OnOff);Serial.println(F("]"));
        started_waiting_at=millis();
        status=2;// a change of state occured but we are still in a critical situation,watchdog can break that cycle between state 1 and 2
    }
    //STATE MACHINE    
    
    
    
    /*
    * WATCHDOG 
    *  in every cycle (loop()) it will be checked if some state is freezing
    *  after every state change(in non critical sections) the previousTime variable is updated with actual time
    *
    */
    
    if((millis()-previousTime)>(criticalTime+dif))
    {
       // bool tx_ok,tx_fail,rx_ready;
         Serial.println(F("NO ANSWER, WE WILL RESET THE STATE MACHINE!!"));
         delay(50);
        //RESET THE STATE COORDINATION
       /* radio.whatHappened 	(tx_ok,tx_fail,rx_ready);
        if(tx_ok || tx_fail || rx_ready) {
            Serial.print(F("TX_OK:"));Serial.println(tx_ok);
            Serial.print(F("TX_FAIL:"));Serial.println(tx_fail);
            Serial.print(F("RX_READY:"));Serial.println(rx_ready);
        }
        
	 	
        if (radio.rxFifoFull())
        {
            Serial.println(F("RX Buffer FULL."));
        }*/
        status=0;// CHANGE TO A COMFORTABLE STATE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //only for testing with PumpNode now
         previousTime=millis();//A change of state occured here
    }
    


 
}//loop()


