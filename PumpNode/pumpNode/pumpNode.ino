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
RF24 radio(CE_PIN, CS_PIN);




/****************** User Config ***************************/
bool bounce = false;

uint32_t started_waiting_at = 0, previousTime = 0, dif = 0, criticalTime = 0, waitonPump = 0, OnOff_1 = 0, OnOff_2 = 0, answer_1 = 0, answer_2 = 0;
uint32_t pump_worktime_1 = 0;
uint32_t pump_worktime_2 = 0;
#if (DEBUG_TIMING_LOOP>0)
uint32_t timing_ = 0;
#endif
const uint8_t pumpPin_1 = PUMP1_PIN;//3
const uint8_t pumpPin_2 = PUMP2_PIN;//2
const uint8_t buttonPin = BUTTON_PIN;//4
uint8_t write_cnt = RADIO_RESEND_NUMB;
int status; //normal states are positive numbers , erro states are negative
int buttonstate = 0;
bool pumpOn_1 = false;
bool pumpOn_2 = false;
//byte addresses[][6] = {"Pump", "Contr"};
//const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL};
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};


/******************************/
struct Data myData;
struct Data myResponse;

#if (DEBUG_RF24 == 1)
uint32_t time_;
#endif



/************************************************************************************************/
/*********************************S E T U P*******************************************************/
/***********************************************************************************************/
void setup() {


  DEBUG_SERIAL_INIT_WAIT;
  printf_begin();
  pinMode(pumpPin_1, OUTPUT);
  pinMode(pumpPin_2, OUTPUT);
  pinMode(buttonPin, INPUT);
  randomSeed(analogRead(0));//initialize Random generator

  killID();
  //radio.begin();
  radio.begin(RADIO_AUTO_ACK, RADIO_DELAY, RADIO_RETRIES, RADIO_SPEED, RADIO_CRC, RADIO_CHANNEL, RADIO_PA_LEVEL);
  //radio.setPALevel(RF24_PA_LOW);
  //radio.setChannel(RADIO_CHANNEL);

  radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1, pipes[0]);     // Open a reading pipe on address 0, pipe 1

  radio.startListening();                       // Start listening

#if (DEBUG==1)
#if (DEBUG_RF24==1)
  radio.printDetails();
#endif
#if (DEBUG_INFO == 1)

  DEBUG_PRINTSTR("\n\tSize of myData: "); DEBUG_PRINTLN(sizeof(myData));
  DEBUG_PRINTSTR("\n\tSize of myResponse: "); DEBUG_PRINTLN(sizeof(myResponse));
#endif
#endif


  myData.humidity = 0.0f;
  myData.moisture = 0;
  myData.moisture2 = 0;
  myData.brightness = 0;
  myData.voltage = 0;
  myData.VCC = 0;
  myData.Time = 0;
  myData.Time_2 = 0;
  myData.packetInfo = 0;

  setDATA_PumpPacket(&myData);
  setDATA_NormalDatapacket(&myData);
  setDATA_RegistrationPacket(&myData);
  //myData.dummy16 |= (1 << DATA_NODE_BIT); //packet is identified as pump packet
  //myData.dummy16 |= (1 << DATA_REGISTRATION_BIT); //packet is identified registration packet
  DEBUG_PRINTSTR("[PUMPNODE]");
  DEBUG_PRINTSTR("[MYDATA PACKETINFO before REGISTRATION]:");
  DEBUG_PRINTDIG(myData.packetInfo, BIN);
  DEBUG_PRINTLNSTR("");
  //Print debug info
  //radio.printDetails();

  //criticalTime = PUMPNODE_CRITICAL_STATE_OCCUPATION/2; //software watchdog looks for freesing states
  criticalTime = (uint32_t)(PUMPNODE_CRITICAL_STATE_OCCUPATION / 2); //software watchdog looks for freesing states
  DEBUG_PRINTSTR("[PUMPNODE]");
  DEBUG_PRINTSTR("CRITICALTIME:");
  DEBUG_PRINTLN(criticalTime);

  registration(true);

  myData.state |= (1 << NODE_TYPE);       // set node type to pump node
  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data (to ensure in case it got overwritten)
  setDATA_NO_RegistrationPacket(&myData);
  //myData.dummy16 &= ~(1 << DATA_REGISTRATION_BIT);
  DEBUG_PRINTSTR("[PUMPNODE]");
  DEBUG_PRINTSTR("[MYDATA PACKETINFO]:");
  DEBUG_PRINTDIG(myData.packetInfo, BIN);
  DEBUG_PRINTLNSTR("");
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

#if (DEBUG_RF24 == 1 && DEBUG==1)
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
  /*******************************STATE 0*****************************/
  if (status == PUMPNODE_STATE_0_PUMPREQUEST) { //state: get new period time to turn on the pump
    int ret = 0;
    if (radio.available() > 0) {
#if (DEBUG_TIMING_LOOP>0)
      timing_ = micros();
#endif
      DEBUG_PRINTLNSTR("------------------------------------------------------");
      DEBUG_PRINTLNSTR("Available radio ");
      /********Receiving next pumping time period**************/
      ret = recvData(); //receive pumptime[ms] from controller
      /*******************************************************/

      DEBUG_PRINTSTR("[PUMPNODE][Status 0]RECEIVED PAYLOAD : "); 
      DEBUG_PRINT(OnOff_1);
      DEBUG_PRINTSTR(" AND ");
      DEBUG_PRINTLN(OnOff_2);

      if (ret > 0) { //Is that message for us and not redundant
#if(PUMPNODE_PUMPS_PARALLEL>0)
        if (OnOff_1 >= On_Off_2)
          answer_1 = OnOff_1;//prepare data for sending in the next state
        answer_2 = 0;
        //On_Off_2=0;
        else
          answer_1 = 0;
        //On_Off_1=0;
        answer_2 = OnOff_2;//prepare data for sending in the next state
#else
        answer_1 = OnOff_1;//prepare data for sending in the next state
        answer_2 = OnOff_2;//prepare data for sending in the next state
#endif
        status = PUMPNODE_STATE_1_PUMPACTIVE;
        previousTime = millis();
      } else {
        DEBUG_PRINTSTR("[PUMPNODE]");
        DEBUG_PRINTLNSTR("[Status 0]Received message was not dedicated to this Pump-Node or it was a redundant message");
      }


    }
    /***************************STATE 1************************/
    /***************************STATE 1************************/
  } else if (status == PUMPNODE_STATE_1_PUMPACTIVE) { //In state 0 pumpNode send acknowledgment, now we get confirmation from controller

    /********Sending acknowlegment,which is the same number as OnOff time request**************/
    //NO WAIT_SEND_INTERVAL , it should send as fast as possible, the controller take care of regular timing
#if (DEBUG_TIMING_LOOP>0)
    DEBUG_PRINTSTR("[TIMING][PumpNode ID: ");
    DEBUG_PRINT(myData.ID);
    DEBUG_PRINTSTR(" ][State 0->1 - from recv - before send]: ");
    DEBUG_PRINT(micros() - timing_);
    DEBUG_PRINTLNSTR(" microseconds.");

#endif
    sendData();
#if (DEBUG_TIMING_LOOP>0)
    DEBUG_PRINTSTR("[TIMING][PumpNode ID: ");
    DEBUG_PRINT(myData.ID);
    DEBUG_PRINTSTR(" ][State 0->1 - from recv - until send]: ");
    DEBUG_PRINT(micros() - timing_);
    DEBUG_PRINTLNSTR(" microseconds.");

#endif
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Status 1]Send acknowledgment to the pump request.");
    /*******************************************************************************************/

#if(PUMPNODE_PUMPS_PARALLEL>0) //PUMPS IN PARALLEL

    pumpOn_1 = true;
    pumpOn_2 = true;
    DEBUG_PRINTSTR("[PUMPNODE][Status 1]Pump1 and Pump2 will work for maximal ");
    //pump time plus half of roundTripDelay
    if (OnOff_1 > 0)
    { //which pump time is longer, important for WATCHDOG
      //maximal wait time until both pumps finishes
      waitonPump = OnOff_1 + (WAIT_RESPONSE_INTERVAL >> 1);
      DEBUG_PRINT(OnOff_1);

    }
    else
    {
      //maximal wait time until both pumps finishes
      waitonPump = OnOff_2  + (WAIT_RESPONSE_INTERVAL >> 1);
      DEBUG_PRINT(OnOff_2);
    }
    DEBUG_PRINTLNSTR("ms in PARALLEL!!!!");
    digitalWrite(pumpPin_1, HIGH);  // TURN PUMP ON
    digitalWrite(pumpPin_2, HIGH);  // TURN PUMP ON

#else//------------------------PUMPS IN SERIES

    pumpOn_1 = true;
    pumpOn_2 = false;
    //pump time plus half of roundTripDelay
    waitonPump = OnOff_1 + OnOff_2  + (WAIT_RESPONSE_INTERVAL >> 1);
    DEBUG_PRINTSTR("[PUMPNODE][Status 1]Pump1 will work for ");
    DEBUG_PRINT(OnOff_1);
    DEBUG_PRINTSTR("ms and THEN Pump2 will work for ");
    DEBUG_PRINT(OnOff_2);
    DEBUG_PRINTSTR("ms  in SERIES.");
    digitalWrite(pumpPin_1, HIGH);  // TURN PUMP ON
#endif

    status = PUMPNODE_STATE_2_PUMPOFF;
    started_waiting_at = millis();
    previousTime = millis();//only state change
    /**************************STATE 2**************************/ 
    /**************************STATE 2**************************/
  } else if (status == PUMPNODE_STATE_2_PUMPOFF) {
    dif = (currentTime - started_waiting_at);
    if (pumpOn_1)
    {
      if ((dif > OnOff_1)) {
        digitalWrite(pumpPin_1, LOW);
#if(PUMPNODE_PUMPS_PARALLEL==0) //in case if in SERIES
        pumpOn_2 = true;
        started_waiting_at = millis();
        digitalWrite(pumpPin_2, HIGH);  // TURN PUMP ON
#endif
        pumpOn_1 = false;
        DEBUG_PRINTSTR("[PUMPNODE][Status 2]"); DEBUG_PRINTLNSTR("Turned off the PUMP 1 ");
        DEBUG_PRINTSTR("[PUMPNODE][Status 2]ElapseTime[");
        DEBUG_PRINT(dif);
        DEBUG_PRINTSTR("] greater than Interval[");
        DEBUG_PRINT(OnOff_1);
        DEBUG_PRINTLNSTR("]");

      }
    }
    if (pumpOn_2)
    {
      if (dif > OnOff_2)
      {
        digitalWrite(pumpPin_2, LOW);

        pumpOn_2 = false;
        DEBUG_PRINTSTR("[PUMPNODE][Status 2]"); DEBUG_PRINTLNSTR("Turned off the PUMP 2 ");
        DEBUG_PRINTSTR("[PUMPNODE][Status 2]ElapseTime[");
        DEBUG_PRINT(dif);
        DEBUG_PRINTSTR("] greater than Interval[");
        DEBUG_PRINT(OnOff_2);
        DEBUG_PRINTLNSTR("]");
      }
    }
    if ((pumpOn_1 == false) && (pumpOn_2 == false)) {
      if (radio.available())
      {
#if (DEBUG_TIMING_LOOP>0)
        timing_ = micros();
#endif
        /********Receiving ACKNOWLEGMENT**************/
        int recv = recvData();
        /*******************************************/
        if (recv > 0)//are data adressed to this node
        {
          answer_1 = OnOff_1;
          answer_2 = OnOff_2;
          //if message was not dedicated to this pumpNode recvData returns -1
          DEBUG_PRINTSTR("[PUMPNODE]");
          DEBUG_PRINTLNSTR("[State 2]RECEIVED CONFIRMATION REQUEST FROM CONTROLLER.");
          DEBUG_PRINTSTR("[PUMPNODE]");

          status = PUMPNODE_STATE_3_ACKNOWLEDGMENT;
          previousTime = millis();
          waitonPump = 0;


        } else {
          DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Status 2]Received message was not dedicated to this Pump-Node");
        }
      }

    }
    /*******************STATE 3*********************/
    /*******************STATE 3*********************/
  } else if (status == PUMPNODE_STATE_3_ACKNOWLEDGMENT) {
    /********Sending acknowlegment,which is the same number as OnOff time request**************/

    //NO WAIT_SEND_INTERVAL , it should send as fast as possible, the controller take care of regular timing
#if (DEBUG_TIMING_LOOP>0)
    DEBUG_PRINTSTR("[TIMING][PumpNode ID: ");
    DEBUG_PRINT(myData.ID);
    DEBUG_PRINTSTR(" ][State 2->3 - from recv - BEFORE send]: ");
    DEBUG_PRINT(micros() - timing_);
    DEBUG_PRINTLNSTR(" microseconds.");

#endif
    sendData();
#if (DEBUG_TIMING_LOOP>0)
    DEBUG_PRINTSTR("[TIMING][PumpNode ID: ");
    DEBUG_PRINT(myData.ID);
    DEBUG_PRINTSTR(" ][State 2->3 - from recv - until send]: ");
    DEBUG_PRINT(micros() - timing_);
    DEBUG_PRINTLNSTR(" microseconds.");

#endif
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("[Status 3]Send last acknowledgment to the pump request.");
    /*******************************************************************************************/
    DEBUG_PRINTLNSTR("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    pump_worktime_1 += OnOff_1;
    pump_worktime_2 += OnOff_2;
    DEBUG_PRINTSTR("[PUMPNODE]");
    DEBUG_PRINTSTR("OVERALL PUMPTIME FOR PUMP 1:");
    DEBUG_PRINT(pump_worktime_1 / 1000);
    DEBUG_PRINTLNSTR(" seconds.");
    DEBUG_PRINTSTR("[PUMPNODE]");
    DEBUG_PRINTSTR("OVERALL PUMPTIME FOR PUMP 2:");
    DEBUG_PRINT(pump_worktime_2 / 1000);
    DEBUG_PRINTLNSTR(" seconds.");

    status = PUMPNODE_STATE_0_PUMPREQUEST;
    previousTime = millis();

  }


  /******************S O F T W A R E   W A T C H D O G *****************************/


  //radio.printDetails();
  if ((millis() - previousTime) > (criticalTime + waitonPump))
  {
    DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("NO ANSWER, WE WILL RESET THE STATE MACHINE!!");
    status = PUMPNODE_STATE_0_PUMPREQUEST;
    waitonPump = 0;
    pumpOn_1 = false;
    pumpOn_2 = false;
    write_cnt = RADIO_RESEND_NUMB;
    previousTime = millis(); //A change of state occured here
    digitalWrite(pumpPin_1, LOW);//for security reasons
    digitalWrite(pumpPin_1, LOW);//for security reasons
  }

  DEBUG_FLUSH;
}//LOOP




/*********************************************************************************/

/*********************************************************************************/

/*Send data over RF:
  Every send operation concludes to an immediate response from controller
*/
void sendData()
{
  //switches myData.state in setup()
  myData.Time = answer_1;
  myData.Time_2 = answer_2;
  setDATA_Pumpstate(&myData, status);
  //myData.dummy8 = status; //mark that package with current state
  DEBUG_PRINTSTR("[PUMPNODE]");
  DEBUG_PRINTSTR("\t[SENDING]Sending data1:");
  DEBUG_PRINT(answer_1);
  DEBUG_PRINTSTR(", data2:");
  DEBUG_PRINT(answer_2);
  DEBUG_PRINTSTR("\t myData.state: ");
  DEBUG_PRINTLN_D(myData.state, BIN);
  DEBUG_PRINTSTR("\t pumpState: ");
  DEBUG_PRINTLN(status);
  DEBUG_PRINTSTR("\t myData.packetInfo: ");
  DEBUG_PRINTLN_D(myData.packetInfo, BIN);

  radio.stopListening();
  while (write_cnt > 0) //handleDataMessage and handleMotorMessage could manipulate write_cnt
  {
    radio.write(&myData, sizeof(struct Data));
    write_cnt--;
  }
  write_cnt = RADIO_RESEND_NUMB;
  radio.startListening();
  DEBUG_PRINTSTR("[PUMPNODE]"); DEBUG_PRINTLNSTR("done");

}



int recvData(void)
{
  //return 0 if message is not for us to keep state
  //ID_INEXISTENT soll behandelt werden??????????????
  while (radio.available()) {
    radio.read(&myResponse, sizeof(struct Data) );          // Get the payload
  }
  //incoming message must in correspondence to current state,
  //otherwise it is a redundant message and will be skipped

  DEBUG_PRINTSTR("[PUMPNODE][RECEIVING]: Resp-data1:");
  DEBUG_PRINTDIG(myResponse.Time, DEC);
  DEBUG_PRINTSTR(", Resp-data2:");
  DEBUG_PRINTDIG(myResponse.Time_2, DEC);
  DEBUG_PRINTSTR(", Resp-ID:");
  DEBUG_PRINTDIG(myResponse.ID, DEC);
  DEBUG_PRINTSTR(", Data-ID:");
  DEBUG_PRINTDIG(myData.ID, DEC);
  DEBUG_PRINTSTR(", Resp-state:");
  DEBUG_PRINTDIG(myResponse.state, BIN);
  DEBUG_PRINTSTR(", PacketInfo:");
  DEBUG_PRINTDIG(myResponse.packetInfo, BIN);
  DEBUG_PRINTLNSTR(" .");


  if (myResponse.ID == myData.ID)
  {
    uint8_t packetstate = getData_PumpState(&myResponse);
    //incoming message may be a redundant message
    //in that case skip it
    if (status == packetstate)
    {
      uint32_t currentTime = getCombinedData(myResponse.moisture2, myResponse.moisture);
      setTime(currentTime);
      displayTimeFromUNIX(currentTime);
      OnOff_1 = myResponse.Time;
      OnOff_2 = myResponse.Time_2;
                //return myResponse.pumpTime;
      return 1;
    }
    DEBUG_PRINTLNSTR("[PUMPNODE][RECEIVING]: REDUNDANT MESSAGE");
    DEBUG_PRINTSTR("\t RECEIVED PACKET STATE : ");
    DEBUG_PRINTLN(packetstate);
  }



  return -1;
}
/*******************************************************************************/
/****************R E G I S T R A T I O N***************************************/
/*******************************************************************************/
int registerNode(void)
{

  struct EEPROM_Data myEEPROMData;
  long numb = 0;

  myData.state |= (1 << NODE_TYPE);       // set node type to pump node
  myData.VCC++; //for debug purposes, how many Debug attempts
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
  DEBUG_PRINT(myData.ID);
  DEBUG_PRINTSTR(" Registration-Request: ");
  DEBUG_PRINTLN(myData.VCC);
  /*********Sending registration request to the Controller***************************/

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

  displayTimeFromUNIX(myResponse.Time);
  DEBUG_PRINTSTR("[PUMPNODE][registerNode()]received: ID: ");
  DEBUG_PRINT(myResponse.ID);
  DEBUG_PRINTSTR(", Status: ");
  DEBUG_PRINTDIG(myResponse.state, BIN);
  DEBUG_PRINTLNSTR("");
  if ((myResponse.state & (1 << ID_REGISTRATION_ERROR)) == false)
  {
    if (isControllerPacket(&myResponse) && isRegistrationPacket(&myResponse))
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
    } else {
      return 5;
    }
  } else
  {
    return 10;
  }////////


  setTime(myResponse.Time);
  return 0;   // all okay
}//registerNode()

/*REGISTRATION PROCEDURE**********************************************************/
void registration(bool refreshID)
{
  struct EEPROM_Data myEEPROMData;
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


  previousTime = millis();
  displayTimeFromUNIX(now());
}











