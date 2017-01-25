


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




/************************************************************************************************/
/*********************************S E T U P*******************************************************/
/***********************************************************************************************/
void setup() {

  Serial.begin(BAUD);
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

  interval = 0;
  status = 0;
  criticalTime = 60000; //software watchdog looks for freesing states

  /*Blumentopf protocol specific**/
  struct EEPROM_Data myEEPROMData;

  // read EEPROM
  EEPROM.get(EEPROM_ID_ADDRESS, myEEPROMData);  // reading a struct, so it is flexible...
  myData.ID = myEEPROMData.ID;                  // passing the ID to the RF24 message
  myData.state = 0;
  myResponse.interval = 100;
    
    
  while (registerNode() > 0)                         //register PumpNode at the controller
  {
    DEBUG_PRINTLNSTR("[Setup()]Registration failed,pause 2 seconds then start again.");
    delay(2000);
  }
  //set standard values
  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data
  myData.humidity = 0.0f;
  myData.moisture = 0;
  myData.brightness = 0;
  myData.voltage = 0;
  myData.VCC = 0;
  myData.realTime = 0;
}


/***********************************S T A T E  M A C H I N E**************************************
   PUMPNode                                 Controller
  ------------------------------------------------------------------------------------------
  (PumpNode and Controller are performing tasks in the same state or divergent states
  according the state plan below)
  (criticalTime should be higher in Controller than in PumpNode)

  STATE 0:
  recv() pump Time                <--       send() pumpTime
  (some delay)
  send() Acknowledgment           -->
                                      |
  STATE 1:                            |     {if wait for response takes too long -> STATE -3}
                                      -->   recv() Acknowledgment(PumpNode confirms Pumptime)
                                      |
                                      |
                                      |
  recv() Response(by protocoll)  <--     __send() Response(by Protocol of Blumentopf)
  (possible error detection necessary)| |
                                      | |
       TURN ON PUMP                   | |
                                      | |
  STATE 2:                            | |
                                      | |
       Wait for pump period time      | |
                                      | |
       TURN OFF PUMP                  | |
                                      |       (maximum wait time = pumptime + timeoff)
  send() Confirmation(Pump off)          -->  recv() Controller knows that Pump is Off now
                                      |       {if wait for response takes too long -> STATE -4}
                                      | |  <--send() Response(by Protocol of Blumentopf)
                                      | |  |
  STATE 3:                            | |  |
  recv() Response(by protocoll) <-- ____| _|
  (possible error detection necessary)| |
                                      | |
                                      | |
                                      | |
  STATE -1:                           | |
  Bad Pump activation Time received   | |
  in State 0,  send error message     | |
  send()  Error Message           -->_| |
                                        |
                                        |
  STATE -2:                             |
                                        |
  recv() confirmation               <--

  STATE -3:(only controller)
                                            We waited timeOFF ms for response in State 1
                                            Restart  State 1

  STATE -4:(only controller)
                                            We waited timeOFF ms for response in State
                                            Restart  State 2

**************************************************************************************************/
/************************************************************************************************/
/*********************************L O O P*******************************************************/
/***********************************************************************************************/
void loop(void) {
  unsigned long currentTime = millis();
  String out = "";
  /********STATE 0*******/
  if (status == PUMPNODE_STATE_0_PUMPREQUEST) { //state: get new period time to turn on the pump
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
        delay(100);//some time to wait
        sendData(answer);
        /*******************************************************************************************/
        status = PUMPNODE_STATE_1_RESPONSE;
        previousTime = millis();
      } else {
        status = PUMPNODE_STATE_ERROR_START;
        previousTime = millis();
      }
      Serial.flush();
    }
    /********STATE 1*******/
  } else if (status == PUMPNODE_STATE_1_RESPONSE) { //In state 0 pumpNode send acknowledgment, now we get confirmation from controller

    if (radio.available()) {
      /********Receiving ACKNOWLEGMENT**************/
      //!!!!!! i COULD BE POSSIBLE THAT IT WAIT TOO LONG IN THAT STATE
      if (recvData() > 0) {//if message was not dedicated to this recvData returns -1
        digitalWrite(pumpPin, HIGH);
        interval = OnOff * 1000L;
        out = "[Status 1]Pump will work for " + String(interval, DEC) + "ms";
        DEBUG_PRINTLN(out);
        started_waiting_at = millis();
        previousTime = millis();
        status = PUMPNODE_STATE_2_PUMPACTIVE;
      }
    }
    /********STATE 2*******/
  } else if (status == PUMPNODE_STATE_2_PUMPACTIVE) {
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

      status = PUMPNODE_STATE_3_RESPONSE;
      previousTime = millis();
      Serial.flush();
    }
    /********STATE 2*******/
  } else if (status == PUMPNODE_STATE_3_RESPONSE) {
    if (radio.available())
    {
      /********Receiving ACKNOWLEGMENT**************/
      //!!!!!! i COULD BE POSSIBLE THAT IT WAIT TOO LONG IN THAT STATE
      if (recvData() > 0)
      { //if message was not dedicated to this pumpNode recvData returns -1
        DEBUG_PRINTLNSTR("[State 3]Received confirmation.");
        DEBUG_PRINTLNSTR("-------------------------------------------------------------");
        status = PUMPNODE_STATE_0_PUMPREQUEST;
        dif = 0;
        previousTime = millis();
        Serial.flush();
      }
    }
  } else if (status == PUMPNODE_STATE_ERROR_START)
  {
    /*There was a bad pump time request*/
    answer = -1;
    DEBUG_PRINTLNSTR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    out = "[Status -1]A bad pump time request was received:  " + String(answer, DEC);
    DEBUG_PRINTLN(out);

    sendData(answer);

    previousTime = millis();
    status = PUMPNODE_STATE_ERROR_END;
  } else if (status == PUMPNODE_STATE_ERROR_END) {
    if (radio.available())
    {
      if (recvData() > 0)
      {
        DEBUG_PRINTLNSTR("[State -2]Received confirmation.");
        DEBUG_PRINTLNSTR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        status = PUMPNODE_STATE_0_PUMPREQUEST;
        previousTime = millis();
      }

    }
  }




  /******************S O F T W A R E   W A T C H D O G *****************************/


  //radio.printDetails();
  if ((millis() - previousTime) > (criticalTime + dif))
  {
    DEBUG_PRINTLNSTR("NO ANSWER, WE WILL RESET THE STATE MACHINE!!");
    delay(50);
    status = PUMPNODE_STATE_0_PUMPREQUEST;
    previousTime = millis(); //A change of state occured here
    digitalWrite(pumpPin, LOW);//for security reasons
  }


}//LOOP




/*********************************************************************************/

/*********************************************************************************/

/*Send data over RF:
  Every send operation concludes to an immediate response from controller
*/
void sendData(unsigned int answer_)
{
  DEBUG_PRINTSTR("state_1:");
  DEBUG_PRINTLN(myData.state);
  myData.state |= (1 << NODE_TYPE);       // set node type to pump node
  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data (to ensure in case it got overwritten)
  DEBUG_PRINTSTR("state_2:");
  DEBUG_PRINTLN(myData.state);

  myData.interval = answer_;

  radio.stopListening();
  DEBUG_PRINTSTR("\tSending data...........");
  DEBUG_PRINTSTR("Data-state:");
  DEBUG_PRINTLN(myData.state);
  //  radio.write(&answer_, sizeof(struct sensorData));
  radio.write(&myData, sizeof(struct sensorData));
  DEBUG_PRINTLNSTR("done");
  radio.startListening();
}

//struct sensorData myData;
//struct responseData myResponse;

unsigned int recvData(void)
{
  //return -1 if message is not for us to keep state

  radio.read(&myResponse, sizeof(struct responseData) );          // Get the payload
  DEBUG_PRINTLN("[recvData]: Resp-interval:"+String(myResponse.interval,DEC)+
  ", Resp-ID:"+String(myResponse.ID,DEC)+", Data-ID:"+String(myData.ID,DEC)+", Resp-state:"+String(myResponse.state,BIN));

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

  myData.state |= (1 << NODE_TYPE);       // set node type to pump node
  // A NODE WITH IDE=0 IS VALID????
  if ((myData.ID < 0xffff) && (myData.ID > 0))                        // this is a known node - 20170110... this is the new check..
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
    myData.temperature = (float)(numb % 100);


    DEBUG_PRINTSTR("random session ID: ");
    DEBUG_PRINT(myData.temperature);
    DEBUG_PRINTLNSTR("...");
  }

  myData.state &= ~(1 << MSG_TYPE_BIT);     // set message type to init
  myData.state |= (1 << NODE_TYPE);       // set node type to pump node

  // Send the measurement results
  DEBUG_PRINTSTR("[registerNode()]:Sending data...");
  DEBUG_PRINT(myData.state);
  DEBUG_PRINTSTR(" ID: ");
  DEBUG_PRINTLN(myData.ID);
  /*********Sending registration request to the Controller***************************/
  radio.stopListening();
  radio.write(&myData, sizeof(struct sensorData));
  radio.startListening();
  /*********************************************************************************/
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
    DEBUG_PRINTLNSTR("[registerNode()]:Failed, response timed out.");
    return 1;
  }

  // There was a response --> read the message
  /*******************Registration Response from Controller*********************/
  radio.read(&myResponse , sizeof(myResponse));
  /****************************************************************************/
  DEBUG_PRINTLNSTR("[registerNode()]received: ID: ");
  DEBUG_PRINT(String(myResponse.ID,DEC) +", Status: " + String(myResponse.state,BIN));
  DEBUG_PRINTLNSTR("");
  if ((myResponse.state & (1 << ID_REGISTRATION_ERROR)) == false)
  {
    if (myData.state & (1 << NEW_NODE_BIT))       // This is a new node!
    {

      DEBUG_PRINTLNSTR("  [registerNode()]:Got response!");
      DEBUG_PRINTSTR("  [registerNode()]Received Session ID: ");
      DEBUG_PRINT((int)(myResponse.interval / 100));

      DEBUG_PRINTSTR(",  expected: ");
      DEBUG_PRINTLN(myData.temperature);

      /* is the response for us? (yes, we stored the session ID in the temperature to keep the message small
          and reception easy...it could be changed to a struct in a "struct payload"
         which can be casted in the receiver depending on the status flags)
      */
      if (((int)(myResponse.interval / 100)) == (int) (myData.temperature))
      {
        myData.ID = myResponse.ID;
        myData.interval = (myResponse.interval % 100);
        myEEPROMData.ID = myResponse.ID;
        EEPROM.put(EEPROM_ID_ADDRESS, myEEPROMData);  // writing the data (ID) back to EEPROM...

        DEBUG_PRINTLNSTR("  [registerNode()]...ID matches");
        DEBUG_PRINTSTR("  [registerNode()]Persistent ID: ");
        DEBUG_PRINT(myResponse.ID);
        DEBUG_PRINTSTR(", Interval: ");
        DEBUG_PRINTLN(myData.interval);
        DEBUG_PRINTLNSTR("  [registerNode()]Stored Persistent ID in EEPROM...");
      }
      else                                                  // not our response
      {
        DEBUG_PRINTLNSTR("  [registerNode()]ID missmatch! Ignore response...");
        return 11;
      }


    }//if (myData.state & (1 << NEW_NODE_BIT))
    else                                              // this is a known node
    {
      if (myResponse.ID == myData.ID)                     // is the response for us?
      {
        myData.interval = myResponse.interval;

        DEBUG_PRINTSTR("  [registerNode()]:Got response for ID: ");
        DEBUG_PRINT(myResponse.ID);
        DEBUG_PRINTSTR("...ID matches, registration successful! Interval: ");
        DEBUG_PRINTLN(myData.interval);
      }
      else                                                  // not our response
      {
        DEBUG_PRINTSTR("  [registerNode()]:Got response for ID: ");
        DEBUG_PRINT(myResponse.ID);
        DEBUG_PRINTLNSTR("  ...ID missmatch! Ignore response...");
        return 12;
      }
    }
  } else
  {
    return 10;
  }////////

  return 0;   // all okay
}











