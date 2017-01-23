


/**************PUMP NODE SETTINGS****************************/
#include "Blumentopf.h"
#include <SPI.h>
#include "printf.h"
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>

//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(9, 10);




/****************** User Config ***************************/
const int radio_channel = 108;

unsigned int OnOff = 0, answer = 0;
unsigned long started_waiting_at = 0, previousTime = 0, interval = 0, dif = 0, criticalTime = 0;
const int pumpPin = 3;
int status; //normal states are positive numbers , erro states are negative
//byte addresses[][6] = {"Pump", "Contr"};
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL};
/******************************/
struct sensorData myData;
struct responseData myResponse;





void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  pinMode(pumpPin, OUTPUT);

  randomSeed(analogRead(0));//initialize Random generator

  radio.begin();

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(radio_channel);

  radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1, pipes[0]);     // Open a reading pipe on address 0, pipe 1

  radio.startListening();                       // Start listening


  //Print debug info
  radio.printDetails();

  struct EEPROM_Data myEEPROMData;
  interval = 0;
  status = 0;
  criticalTime = 60000; //software watchdog looks for freesing states

  // read EEPROM
  EEPROM.get(EEPROM_ID_ADDRESS, myEEPROMData);  // reading a struct, so it is flexible...
  myData.ID = myEEPROMData.ID;                  // passing the ID to the RF24 message
  myData.state = 0;

  while(registerNode() > 0)                          //register PumpNode at the controller
  {
    delay(2000);
  }

  myData.state |= (1 << NODE_TYPE);       // set node type to pump node

}


/***********************************S T A T E  M A C H I N E**************************************
   PUMPNode                                 Controller
  STATE 0:
  recv() pump Time                <--        send() pumpTime
  send() Acknowledgment           -->        recv() Acknowledgment(PumpNode confirms Pumptime

  STATE 1:
  recv() Response(by protocoll)   <--       send() Response(by Protocol of Blumentopf)
  -------------------------    TURN ON PUMP  -----------------------------------------------

  STATE 2:
        ..................Wait for pump period time ................

  -------------------------    TURN OFF PUMP  -----------------------------------------------
  send() Confirmation(Pump off)   -->        recv() Controller knows that Pump is Off now

  STATE 3:
  recv() Response(by protocoll)   <--       send() Response(by Protocol of Blumentopf)

  STATE -1:

  STATE -2:


  STATE 10: Registration request

  STATE 11: registration fonfirmation


**************************************************************************************************/

void loop(void) {
  unsigned long currentTime = millis();
  String out = "";
  /********STATE 0*******/
  if (status == 0) { //state: get new period time tot turn on the pump
    dif = 0;
    if (radio.available()) {
      DEBUG_PRINTLNSTR("------------------------------------------------------");
      DEBUG_PRINTLNSTR("Available radio ");
      /********Receiving next pumping time period**************/
      OnOff = recvData();

      out = "[Status 0]Received payload " + String(OnOff, DEC);
      DEBUG_PRINTLN(out);

      if (OnOff > 0) {
        answer = OnOff;
        DEBUG_PRINTLNSTR("[Status 0]Send acknowledgment to the pump request.");
        /********Sending acknowlegment,which is the same number as OnOff time request**************/
        sendData(answer);
        /*******************************************************************************************/
        status = 1;
        previousTime = millis();
      } else {
        status = -1;
        previousTime = millis();
      }
      Serial.flush();
    }
    /********STATE 1*******/
  } else if (status == 1) { //In state 0 pumpNode send acknowledgment,now we get confirmation from controller

    if (radio.available()) {
      /********Receiving ACKNOWLEGMENT**************/
      if (recvData() > 0) {
        digitalWrite(pumpPin, HIGH);
        out = "[Status 1]Pump will work for " + String(interval, DEC) + "ms";
        DEBUG_PRINTLN(out);
        interval = OnOff * 1000L;
        started_waiting_at = millis();
        previousTime = millis();
        status = 2;
      }
    }
    /********STATE 2*******/
  } else if (status == 2) {
    dif = (currentTime - started_waiting_at);

    if ((dif > interval)) {

      out = "[Status 2]ElapseTime[" + String(dif, DEC) + "] greater than Interval[" + String(interval, DEC) + "]";
      DEBUG_PRINTLN(out);
      answer = dif; //send him the total time needed
      DEBUG_PRINTLNSTR("Turn off the pump "); delay(50);
      digitalWrite(pumpPin, LOW);
      out = "[Status 2]Send final confirmation that pump is OFF: " + String(answer, DEC);
      DEBUG_PRINTLN(out);

      /********Sending final confirmation,which is the total time measured during pump activation**/
      sendData(answer);
      /*******************************************************************************************/

      status = 3;
      previousTime = millis();
      Serial.flush();
    }
    /********STATE 2*******/
  } else if (status == 3) {
    if (radio.available())
    {
      /********Receiving ACKNOWLEGMENT**************/
      if (recvData() > 0)
      {
        DEBUG_PRINTLNSTR("[State 3]Received confirmation.");
        DEBUG_PRINTLNSTR("-------------------------------------------------------------");
        status = 0;
        dif = 0;
        previousTime = millis();
        Serial.flush();
      }
    }
  } else if (status == -1)
  {
    /*There was a bad pump time request*/
    answer = -1;
    DEBUG_PRINTLNSTR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    out = "[Status -1]A bad pump time request was received:  " + String(answer, DEC);
    DEBUG_PRINTLN(out);

    sendData(answer);

    previousTime = millis();
    status = -2;
  } else if (status == -2) {
    if (radio.available())
    {
      if (recvData() > 0)
      {
        DEBUG_PRINTLNSTR("[State -2]Received confirmation.");
        DEBUG_PRINTLNSTR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        status = 0;
        previousTime = millis();
      }

    }

  } else if (status == 10) {

  } else if (status == 20) {

  }



  /*********************************************************************************/
  /******************S O F T W A R E   W A T C H D O G *****************************/
  /*********************************************************************************/

  //radio.printDetails();
  if ((millis() - previousTime) > (criticalTime + dif))
  {
    DEBUG_PRINTLNSTR("NO ANSWER, WE WILL RESET THE STATE MACHINE!!");
    delay(50);
    status = 0;
    previousTime = millis(); //A change of state occured here
  }


}//LOOP

/*Send data over RF:
  Every send operation concludes to an immediate response from controller
*/
void sendData(unsigned int answer_)
{
  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data
  myData.interval = answer_;

  radio.stopListening();
  DEBUG_PRINTLNSTR("\t\tSending data...........");
  radio.write(&answer_, sizeof(struct sensorData));
  radio.startListening();
}

//struct sensorData myData;
//struct responseData myResponse;

unsigned int recvData(void)
{
  //return -1 if message is not for us to keep state

  radio.read(&myResponse, sizeof(struct responseData) );          // Get the payload
  if (myResponse.ID == myData.ID)
  {
    return myResponse.interval;
  }
  return -1;
}


int registerNode(void)
{

  struct EEPROM_Data myEEPROMData;
  long numb = 0;
  if (myData.ID < 0xffff)                        // this is a known node - 20170110... this is the new check..
  {
    DEBUG_PRINTSTR("[registerNode()]:EEPROM-ID found: ");
    DEBUG_PRINT(myData.ID);                // Persistent ID
    DEBUG_PRINTLNSTR(". Registering at the server with this persistent ID... ");

    myData.state &= ~(1 << NEW_NODE_BIT);    // this is a known node
  }
  else                                      // this is a new node
  {
    DEBUG_PRINTSTR("[registerNode()]:No EEPROM-ID found. Registering at server and get new ID ");
    myData.state |= (1 << NEW_NODE_BIT);    // this is a new node

    //randNumber = random(300);
    numb = random(50000000L, 100000000L);
    myData.temperature = (float)numb;


    DEBUG_PRINTSTR("random session ID: ");
    DEBUG_PRINT(myData.temperature);
    DEBUG_PRINTLNSTR("...");
  }

  myData.state &= ~(1 << MSG_TYPE_BIT);     // set message type to init

  radio.stopListening();
  // Send the measurement results
  DEBUG_PRINTSTR("[registerNode()]:Sending data...");
  DEBUG_PRINT(myData.state);
  DEBUG_PRINTSTR("ID: ");
  DEBUG_PRINTLN(myData.ID);
  radio.write(&myData, sizeof(struct sensorData));
  radio.startListening();

//  while (!radio.available());   // pump node hangs here in case the registration request gets lost. That's why there should be a timeout check
// Wait here until we get a response, or timeout (REGISTRATION_TIMEOUT_INTERVAL == ~500ms)
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( (radio.available() == 0 ) && ! timeout )
  {
    if (millis() - started_waiting_at > REGISTRATION_TIMEOUT_INTERVAL )
    {
      timeout = true;
    }
  }
  // Describe the results
  if ( timeout )              // the controller did not answer. Abort registration and retry later.
  {
    DEBUG_PRINTLNSTR("Failed, response timed out.");
    return 1;
  }

  // There was a response --> read the message
  radio.read(&myResponse , sizeof(myResponse));



  if (myData.state & (1 << NEW_NODE_BIT))       // This is a new node!
  {

    DEBUG_PRINTLNSTR("[registerNode()]:Got response!");
    DEBUG_PRINTSTR("  Received Session ID: ");
    DEBUG_PRINT((int)(myResponse.interval / 100));

    DEBUG_PRINTSTR(",  expected: ");
    DEBUG_PRINTLN(myData.temperature);

    /* is the response for us? (yes, we stored the session ID in the temperature to keep the message small 
     *  and reception easy...it could be changed to a struct in a "struct payload" 
     * which can be casted in the receiver depending on the status flags)
    */
    if (((int)(myResponse.interval / 100)) == (int) (myData.temperature))    
    {
      myData.ID = myResponse.ID;
      myData.interval = (myResponse.interval % 100);
      myEEPROMData.ID = myResponse.ID;
      EEPROM.put(EEPROM_ID_ADDRESS, myEEPROMData);  // writing the data (ID) back to EEPROM...

      DEBUG_PRINTLNSTR("...ID matches");
      DEBUG_PRINTSTR("  Persistent ID: ");
      DEBUG_PRINT(myResponse.ID);
      DEBUG_PRINTSTR(", Interval: ");
      DEBUG_PRINTLN(myData.interval);
      DEBUG_PRINTLNSTR("Stored Persistent ID in EEPROM...");
    }
    else                                                  // not our response
    {
      DEBUG_PRINTLNSTR("ID missmatch! Ignore response...");
      return 11;
    }


  }//if (myData.state & (1 << NEW_NODE_BIT))
  else                                              // this is a known node
  {
    if (myResponse.ID == myData.ID)                     // is the response for us?
    {
      myData.interval = myResponse.interval;

      DEBUG_PRINTSTR("[registerNode()]:Got response for ID: ");
      DEBUG_PRINT(myResponse.ID);
      DEBUG_PRINTSTR("...ID matches, registration successful! Interval: ");
      DEBUG_PRINTLN(myData.interval);
    }
    else                                                  // not our response
    {
      DEBUG_PRINTSTR("[registerNode()]:Got response for ID: ");
      DEBUG_PRINT(myResponse.ID);
      DEBUG_PRINTLNSTR("...ID missmatch! Ignore response...");
      return 12;
    }
  }////////

  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data

  return 0;   // all okay
}











