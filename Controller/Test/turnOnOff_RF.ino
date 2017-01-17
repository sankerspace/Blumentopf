/**************CONTROLLER SETTINGS****************************/

#include "particle-rf24/particle-rf24.h"


RF24 radio(D6,A2);
//
// Hardware configuration
// from https://github.com/mshoemaker/SparkCore-RF24


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
int OnOff=1;
int counter=0;
const int pumpPin = 3;    // connected to the base of the transistor

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins D6 & A2 */

/**********************************************************/

byte addresses[][6] = {"Pump","Contr"};



//byte counter = 1;                                                          // A single byte to keep track of the data being sent back and forth


void setup(){

  Serial.begin(115200);
  
  
  radio.begin();

  radio.setPALevel(RF24_PA_LOW);

  
 // radio.enableAckPayload();                     // Allow optional ack payloads
 // radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);

  //radio.startListening();                       // Start listening  
  //radio.writeAckPayload(1,&counter,1);          // Pre-load an ack-paylod into the FIFO buffer for pipe 1


  //Print debug info
  radio.printDetails();
}

void loop(void) {

/****************** Ping Out Role ***************************/

    ///radio.stopListening();
                          // Send the counter variable to the other radio
    Serial.print(F("Pump should run "));  
    Serial.print(OnOff,DEC);
     Serial.println(F(" seconds."));  
    if(radio.write(&OnOff,sizeof(int)))
     {
        /*if(!radio.available()){
            Serial.println(F("Received Acknowledgment"));           // If no ack response, sending failed
        }else{
           
        }*/
     }
     else{
        Serial.println(F("Sending failed."));           // If no ack response, sending failed
     }
   

   delay(OnOff*1000);  // Try again later
  
  
  OnOff++;
}//loop()


