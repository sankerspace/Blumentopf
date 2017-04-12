/*
   Project: NESE_Blumentopf
   File:    pumpNode.ino
   Authors: Bernhard Fritz  (0828317@student.tuwien.ac.at)
            Marko Stanisic  (0325230@student.tuwien.ac.at)
            Helmut Bergmann (0325535@student.tuwien.ac.at)
   The copyright for the software is by the mentioned authors.

   This program uses a set of sensors to gather information about it's surrounding.
   It registers to a controller and periodically sends the data to the controller.
   During the measurements the setup goes into a low power mode.

*/


/**************PUMP NODE SETTINGS****************************/
//#include <SPI.h>
#include "printf.h"
#include "Blumentopf.h"

//#include "nRF24L01.h"
//#include "RF24.h"
#include <EEPROM.h>

//function to initiate radio device with radio(CE pin,CS pin)
RF24 radio(9, 10);




/****************** User Config ***************************/
bool bounce = false;
uint16_t OnOff = 0, answer = 0;
uint32_t started_waiting_at = 0, previousTime = 0, dif = 0, criticalTime = 0;
uint32_t pump_worktime = 0;
const uint8_t pumpPin = 3;
const uint8_t buttonPin = 4;
int status; //normal states are positive numbers , erro states are negative
int buttonstate = 0;
//byte addresses[][6] = {"Pump", "Contr"};
//const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL};
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};


/******************************/
struct sensorData myData;
struct responseData myResponse;

#if (DEBUG_RF24 == 1)
uint32_t time_;
#endif



/************************************************************************************************/
/*********************************S E T U P*******************************************************/
/***********************************************************************************************/
void setup() {


  DEBUG_SERIAL_INIT_WAIT;
  printf_begin();
  pinMode(pumpPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  randomSeed(analogRead(0));//initialize Random generator

  killID();
  //radio.begin();
  radio.begin(RADIO_AUTO_ACK,RADIO_DELAY, RADIO_RETRIES, RADIO_SPEED, RADIO_CRC, RADIO_CHANNEL, RADIO_PA_LEVEL);
  //radio.setPALevel(RF24_PA_LOW);
  //radio.setChannel(RADIO_CHANNEL);

  radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1, pipes[0]);     // Open a reading pipe on address 0, pipe 1

  radio.startListening();                       // Start listening

#if (DEBUG_==1)
#if (DEBUG_RF24==1)
  radio.printDetails();
#endif
#if (DEBUG_INFO == 1)
  
    DEBUG_PRINTSTR("\n\tSize of myData: "); DEBUG_PRINTLN(sizeof(myData));
    DEBUG_PRINTSTR("\n\tSize of myResponse: "); DEBUG_PRINTLN(sizeof(myResponse));
#endif
#endif



  //Print debug info
  //radio.printDetails();

  //criticalTime = PUMPNODE_CRITICAL_STATE_OCCUPATION/2; //software watchdog looks for freesing states
  criticalTime = (uint32_t)(PUMPNODE_CRITICAL_STATE_OCCUPATION / 2); //software watchdog looks for freesing states
  DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("CRITICALTIME:"); DEBUG_PRINTLN(criticalTime);

  registration(true);


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
   (Data: OnOff)                      |
  STATE 1:                            |     {if wait for response takes too long -> STATE -3}
                                      -->   recv() Acknowledgment(PumpNode confirms Pumptime)
                                      |
                                      |
                                      |
  recv() Response(by protocoll)  <--     __send() Response(by Protocol of Blumentopf)
  (possible error detection necessary)| |   (Data: 2*OnOff)
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
  (Data: dif)                         |       {if wait for response takes too long -> STATE -4}
                                      | |  <--send() Response(by Protocol of Blumentopf)
                                      | |  |   (Data:0xffff))
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
  uint32_t currentTime = millis();

#if (DEBUG_RF24 == 1 && DEBUG_==1)
  if ((millis() - time_) > 20000)
  {
    radio.printDetails();
    time_ = millis();


  }
#endif
  buttonstate = digitalRead(buttonPin);
  if (buttonstate == HIGH)
  {
    dif = 0;
    DEBUG_PRINTLNSTR("[PUMPNODE]BUTTON PRESSED!!!!!!!!!!!!");
    status == PUMPNODE_STATE_0_PUMPREQUEST;
    if (!bounce)
      previousTime = millis();
    bounce = true;

  } else if (bounce == true) {

    dif = millis() - previousTime;
    if (dif > 2000)
    {
      DEBUG_PRINTLNSTR("[PUMPNODE]REGISTRATION RESET WITH REQUESTING A NEW ID");
      registration(false);
      bounce = false;
      previousTime = millis();
    } else if (dif > 500)
    {
      DEBUG_PRINTLNSTR("[PUMPNODE]REGISTRATION RESET A KNOWN  ID");
      registration(false);
      bounce = false;
      previousTime = millis();
    }

  }
  //DEBUG_PRINTSTR("[MEMORY]:Between Heap and Stack still "); DEBUG_PRINT(String(freeRam(), DEC));
  //DEBUG_PRINTLNSTR(" bytes available.");
  /*******************************STATE 0*****************************/
  if (status == PUMPNODE_STATE_0_PUMPREQUEST) { //state: get new period time to turn on the pump

    if (radio.available() > 0) {
      dif = 0;
      DEBUG_PRINTLNSTR("------------------------------------------------------");
      DEBUG_PRINTLNSTR("Available radio ");
      /********Receiving next pumping time period**************/
      OnOff = recvData(); //receive pumptime[ms] from controller
      /*******************************************************/

      DEBUG_PRINTSTR("[PUMPNODE][Status 0]Received payload "); DEBUG_PRINTLN(OnOff);

      if (OnOff > 0) {
        answer = OnOff;
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Status 0]Send acknowledgment to the pump request.");
        /********Sending acknowlegment,which is the same number as OnOff time request**************/
        delay(WAIT_SEND_INTERVAL);//some time to wait
        sendData(answer);//will be received by Controller::Pump_handler in State 1
        /*******************************************************************************************/
        status = PUMPNODE_STATE_1_RESPONSE;
        previousTime = millis();
      } else {
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Status 0]Received message was not dedicated to this Pump-Node");
        previousTime = millis();
      }
      DEBUG_FLUSH;
    }
    /***************************STATE 1************************/
  } else if (status == PUMPNODE_STATE_1_RESPONSE) { //In state 0 pumpNode send acknowledgment, now we get confirmation from controller

    if (radio.available() > 0) {
      /********Receiving ACKNOWLEGMENT**************/
      //!!!!!! i COULD BE POSSIBLE THAT IT WAIT TOO LONG IN THAT STATE
      uint16_t recv = recvData();
      if (recv > 0) {
        if (recv == (2 * OnOff)) { //received from Controller in State 1
          digitalWrite(pumpPin, HIGH);
          DEBUG_PRINTSTR("[PUMPNODE][Status 1]Pump will work for");
          DEBUG_PRINT(OnOff);
          DEBUG_PRINTLNSTR("ms");
          started_waiting_at = millis();
          previousTime = millis();
          status = PUMPNODE_STATE_2_PUMPACTIVE;
        }
      } else {
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Status 1]Received message was not dedicated to this Pump-Node");
        previousTime = millis();
      }
    }
    /**************************STATE 2**************************/
  } else if (status == PUMPNODE_STATE_2_PUMPACTIVE) {
    dif = (currentTime - started_waiting_at);

    if ((dif > OnOff)) {


      DEBUG_PRINTSTR("[PUMPNODE][Status 2]ElapseTime[");
      DEBUG_PRINT(dif);
      DEBUG_PRINTSTR("] greater than Interval[");
      DEBUG_PRINT(OnOff);
      DEBUG_PRINTLNSTR("]");
      answer = dif; //send him the total time needed
      DEBUG_PRINTSTR("[PUMPNODE][Status 2]"); DEBUG_PRINTLNSTR("Turn off the pump "); delay(50);
      digitalWrite(pumpPin, LOW);

      DEBUG_PRINTSTR("[PUMPNODE][Status 2]Send final confirmation that pump is OFF: ");
      DEBUG_PRINTLN(answer);

      /********Sending final confirmation,which is the total time measured during pump activation**/
      //no delay necessary because of pumptime
      if (dif < WAIT_SEND_INTERVAL)
        delay(WAIT_SEND_INTERVAL);//some time to wait
      sendData(answer);
      /*******************************************************************************************/

      status = PUMPNODE_STATE_3_RESPONSE;
      previousTime = millis();
      DEBUG_FLUSH;
    }
    /********STATE 2*******/
  } else if (status == PUMPNODE_STATE_3_RESPONSE) {
    if (radio.available())
    {
      /********Receiving ACKNOWLEGMENT**************/
      uint16_t recv = recvData();
      //!!!!!! i COULD BE POSSIBLE THAT IT WAIT TOO LONG IN THAT STATE
      if (recv > 0)//are data adressed to this node
      {
        if (recv == 0xffff)
        { //if message was not dedicated to this pumpNode recvData returns -1
          DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[State 3]Received confirmation.");
          DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("-------------------------------------------------------------");
          status = PUMPNODE_STATE_0_PUMPREQUEST;
          dif = 0;
          previousTime = millis();
          pump_worktime += OnOff;
          DEBUG_PRINTSTR("[PUMPNODE]");
          DEBUG_PRINTSTR("OVERALL PUMPTIME :");
          DEBUG_PRINT(pump_worktime / 1000);
          DEBUG_PRINTLNSTR(" seconds.");
          DEBUG_FLUSH;
        }
      } else {
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Status 3]Received message was not dedicated to this Pump-Node");
        previousTime = millis();
      }
    }
  }


  /******************S O F T W A R E   W A T C H D O G *****************************/


  //radio.printDetails();
  if ((millis() - previousTime) > (criticalTime + dif))
  {
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("NO ANSWER, WE WILL RESET THE STATE MACHINE!!");
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
void sendData(uint16_t answer_)
{

  myData.state |= (1 << NODE_TYPE);       // set node type to pump node
  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data (to ensure in case it got overwritten)

  myData.interval = answer_;


  DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("\t[SENDING]Sending data "); DEBUG_PRINT(answer_);
  DEBUG_PRINTSTR(" with STATE:");
  DEBUG_PRINTLN(myData.state);
  //  radio.write(&answer_, sizeof(struct sensorData));
  radio.stopListening();
  radio.write(&myData, sizeof(struct sensorData));
  radio.startListening();
  DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("done");

}

//struct sensorData myData;
//struct responseData myResponse;

uint16_t recvData(void)
{
  //return 0 if message is not for us to keep state
  //ID_INEXISTENT soll behandelt werden??????????????ß
  while (radio.available()) {
    radio.read(&myResponse, sizeof(struct responseData) );          // Get the payload
  }
  DEBUG_PRINTSTR("[PUMPNODE][RECEIVING]: Resp-interval:");
  DEBUG_PRINTDIG(myResponse.interval, DEC);
  DEBUG_PRINTSTR(", Resp-ID:");
  DEBUG_PRINTDIG(myResponse.ID, DEC);
  DEBUG_PRINTSTR(", Data-ID:");
  DEBUG_PRINTDIG(myData.ID, DEC);
  DEBUG_PRINTSTR(", Resp-state:");
  DEBUG_PRINTDIG(myResponse.state, BIN);
  DEBUG_PRINTLNSTR("");


  if (myResponse.ID == myData.ID)
  {
    setTime(myResponse.ControllerTime);

    displayTimeFromUNIX(myResponse.ControllerTime);
    return myResponse.interval;
  }
  return 0;
}


int registerNode(void)
{

  struct EEPROM_Data myEEPROMData;
  long numb = 0;

  myData.state |= (1 << NODE_TYPE);       // set node type to pump node
  // A NODE WITH IDE=0 IS VALID????
  if ((myData.ID < 0xffff) && (myData.ID > 0))                        // this is a known node - 20170110... this is the new check..
  {
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("[registerNode()]:EEPROM-ID found: ");
    DEBUG_PRINT(myData.ID);                // Persistent ID
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR(". Registering at the server with this persistent ID... ");

    myData.state &= ~(1 << NEW_NODE_BIT);    // this is a known node

  }
  else                                      // this is a new node
  {
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("[registerNode()]:No EEPROM-ID found. Registering at server and get new ID ");
    myData.state |= (1 << NEW_NODE_BIT);    // this is a new node

    //randNumber = random(300);
    numb = random(50000000L, 100000000L);
    myData.temperature = (float)(numb % 100);


    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("random session ID: ");
    DEBUG_PRINT(myData.temperature);
    DEBUG_PRINTLNSTR("...");
  }

  myData.state &= ~(1 << MSG_TYPE_BIT);     // set message type to init
  myData.state |= (1 << NODE_TYPE);       // set node type to pump node

  // Send the measurement results
  DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("[registerNode()]:Sending data...");
  DEBUG_PRINT(myData.state);
  DEBUG_PRINTSTR(" ID: ");
  DEBUG_PRINTLN(myData.ID);
  /*********Sending registration request to the Controller***************************/
  /*LÖSCHEN*/




  DEBUG_PRINTLNSTR("Initialize struct sensorData with arbitrary values: ");
  myData.humidity = 12000.234f;
  myData.moisture = 13000;
  myData.brightness = 21546;
  myData.voltage = 45258;
  myData.VCC = 55879;
  myData.realTime = millis();

  DEBUG_PRINTSTR(" humidity: ");
  DEBUG_PRINTLN(myData.humidity);
  DEBUG_PRINTSTR(" moisture: ");
  DEBUG_PRINTLN(myData.moisture);
  DEBUG_PRINTSTR(" brightness: ");
  DEBUG_PRINTLN(myData.brightness);
  DEBUG_PRINTSTR(" voltage: ");
  DEBUG_PRINTLN(myData.voltage);
  DEBUG_PRINTSTR(" VCC: ");
  DEBUG_PRINTLN(myData.VCC);
  DEBUG_PRINTSTR(" realTime: ");
  DEBUG_PRINTLN(myData.realTime);
  DEBUG_PRINTSTR("SIZE OF SENDING DATA STRUCTURE : ");
  DEBUG_PRINTLN(sizeof(myData));
  /*LÖSCHEN*/
  radio.stopListening();
  if (!radio.write( &myData, sizeof(myData) )) {
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("[registerNode()]:Sending data...");
    DEBUG_PRINTLNSTR("  FAILED!!!!!!!!!!!");
  }
  radio.startListening();

  /*********************************************************************************/
  //  while (!radio.available());   // pump node hangs here in case the registration request gets lost. That's why there should be a timeout check
  // Wait here until we get a response, or timeout (REGISTRATION_TIMEOUT_INTERVAL == ~500ms)
  uint32_t started_waiting_at = millis();
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
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[registerNode()]:Failed, response timed out.");
    return 1;
  }

  // There was a response --> read the message
  /*******************Registration Response from Controller*********************/
  while (radio.available())
  {
    radio.read(&myResponse , sizeof(myResponse));
  }
  /****************************************************************************/
  displayTimeFromUNIX(myResponse.ControllerTime);
  DEBUG_PRINTSTR("[PUMPNODE][registerNode()]received: ID: ");
  DEBUG_PRINT(myResponse.ID);
  DEBUG_PRINTSTR(", Status: ");
  DEBUG_PRINTDIG(myResponse.state, BIN);
  DEBUG_PRINTLNSTR("");
  if ((myResponse.state & (1 << ID_REGISTRATION_ERROR)) == false)
  {
    if (myData.state & (1 << NEW_NODE_BIT))       // This is a new node!
    {

      DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("  [registerNode()]:Got response!");
      DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("  [registerNode()]Received Session ID: ");
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

        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("  [registerNode()]...ID matches");
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("  [registerNode()]Persistent ID: ");
        DEBUG_PRINT(myResponse.ID);
        DEBUG_PRINTSTR(", Interval: ");
        DEBUG_PRINTLN(myData.interval);
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("  [registerNode()]Stored Persistent ID in EEPROM...");
      }
      else                                                  // not our response
      {
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("  [registerNode()]ID missmatch! Ignore response...");
        return 11;
      }


    }//if (myData.state & (1 << NEW_NODE_BIT))
    else                                              // this is a known node
    {
      if (myResponse.ID == myData.ID)                     // is the response for us?
      {
        myData.interval = myResponse.interval;

        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("  [registerNode()]:Got response for ID: ");
        DEBUG_PRINT(myResponse.ID);
        DEBUG_PRINTSTR("...ID matches, registration successful! Interval: ");
        DEBUG_PRINTLN(myData.interval);
      }
      else                                                  // not our response
      {
        DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTSTR("  [registerNode()]:Got response for ID: ");
        DEBUG_PRINT(myResponse.ID);
        DEBUG_PRINTLNSTR("  ...ID missmatch! Ignore response...");
        return 12;
      }
    }
  } else
  {
    return 10;
  }////////

  setTime(myResponse.ControllerTime);
  return 0;   // all okay
}


void registration(bool refreshID)
{
  struct EEPROM_Data myEEPROMData;
  OnOff = 0;
  status = PUMPNODE_STATE_0_PUMPREQUEST;

  /*Blumentopf protocol specific**/

  if (refreshID == true)
  {
    myEEPROMData.ID = 0xffff;
    EEPROM.put(EEPROM_ID_ADDRESS, myEEPROMData);
  } else
  {
    // read EEPROM
    EEPROM.get(EEPROM_ID_ADDRESS, myEEPROMData);  // reading a struct, so it is flexible...
  }
  myData.ID = myEEPROMData.ID;                  // passing the ID to the RF24 message
  myData.state = 0;
  myResponse.interval = 100;
  while (registerNode() > 0)                         //register PumpNode at the controller
  {
    DEBUG_PRINTLNSTR("[PUMPNODE][Setup()]Registration failed,pause 2 seconds then start again.");
    buttonstate = digitalRead(buttonPin);
    if (buttonstate == HIGH) {
      DEBUG_PRINTLNSTR("[PUMPNODE][Setup()]BUTTON PRESSED IN SETUP MODE!!!!!!!");
      myEEPROMData.ID = 0xffff;
      myData.ID = myEEPROMData.ID;
      EEPROM.put(EEPROM_ID_ADDRESS, myEEPROMData);
    }
    delay(2000);
  }
  DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Setup()]REGISTRATION SUCCEDD!!!");
  //set standard values
  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data
  myData.humidity = 0.0f;
  myData.moisture = 0;
  myData.moisture2 = 0;
  myData.brightness = 0;
  myData.voltage = 0;
  myData.VCC = 0;
  myData.realTime = 0;

  previousTime = millis();
  displayTimeFromUNIX(now());
}











