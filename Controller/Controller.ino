/*
   Project: NESE_Blumentopf
   File:    Controller.ino
   Authors: Bernhard Fritz  (0828317@student.tuwien.ac.at)
            Marko Stanisic  (0325230@student.tuwien.ac.at)
            Helmut Bergmann (0325535@student.tuwien.ac.at)
   The copyright for the software is by the mentioned authors.

   The purpose of the module is to coordinate the network of
   wireless nodes and to connect it to the IOT platform.
   The controller module can run on an Arduino or Particle Photon.
   Therefore the corresponding flags (HW, HW_RTC, SD_AVAILABLE, etc.) have to be set in the header file.

*/
//#include <SPI.h>
#include "Blumentopf.h"
#include <LinkedList.h> //for PumpHandler List


#if (SD_AVAILABLE == 1)
  #include <SD.h>
#endif


/**some declarations for the RTC , in case we have a RTC*/


#if (HW_RTC == RTC_1302)
  RTC_DS1302 myRTC(2, 3, 4);  // CE, IO, CLK (RST, DAT, CLK)
#elif (HW_RTC == RTC_3231)
  RTC_DS3231 myRTC;
#elif (HW_RTC == RTC_3232)
  RTC_DS3232 myRTC;
#endif

#if (DEBUG_==1)
  #if(DEBUG_RF24 == 1)
    uint32_t time_;
  #endif
  uint16_t nDummyCount=0;
#endif

//#define INTERVAL (600)


struct responseData myResponse; //9byte
struct sensorData myData; //25byte
class CommandHandler myCommandHandler;

//marko@: wozu brauchen wir diese variable?
//verwende sie jetzt Um Pumphandler zu zählen
uint16_t nPumpHandlerCnt=0;
/*Marko@: introduced for increasing communication reliability by redundance
*        important for Pump Handler,  instead waiting for acknowledgment
*        we resend a message to ensure that at least one message arrives desination
*        maximum resend count defined in RADIO_RESEND_NUMB
*/
uint8_t write_cnt=1;
/*
*
*/
bool bResponseNeeded = true;

#if(HW==HW_PHOTON)
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
//function to initiate radio device with radio(CE pin,CS pin)
  RF24 radio(D6,A2);
#else
  RF24 radio(9, 10);
#endif

//brauchen wir 3 pipes!!!!!!!!!!??
//const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL}; // pipe[0] ist answer-channel
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};//didn't reduce dynamic memory usage
nodeList myNodeList;
LinkedList<PumpNode_Handler*> PumpList = LinkedList<PumpNode_Handler*>();

#if TEST_PUMP
uint16_t nTestWatering = 1000;
uint8_t i_=0;
#endif


/*
   SETUP

   PREREQUISITE:
    EEPROM: every byte in the EEPROM must be initialized with 0xff
    (only,if you use the Blumentopf library the first time)
    (or if you want to delete old nodes and want to start again->nodeList::clearEEPROM_Nodelist())
*/
/*******************************************************************************************/
/*******************************************************************************************/
/********************************S E T U P***************************************************/
void setup(void)
{
  DEBUG_SERIAL_INIT_WAIT;

  //radio.begin();
  //Marko@ : want to ensure that all three node types use the same settings
  radio.begin(RADIO_AUTO_ACK,RADIO_DELAY,RADIO_RETRIES,RADIO_SPEED,RADIO_CRC,RADIO_CHANNEL,RADIO_PA_LEVEL);
  //  radio.setRetries(15,15);
  //  radio.setPALevel(RF24_PA_MIN);//@Marko: Test other configuration, maybe better communication
  //radio.setChannel(RADIO_CHANNEL);  // Above most Wifi Channels

  //  radio.setPayloadSize(8);
  radio.openReadingPipe(1, pipes[1]);
  radio.openWritingPipe(pipes[0]);
  radio.startListening();
  pinMode(LED_BUILTIN, OUTPUT);

DEBUG_PRINTLNSTR("\r\n****************");
/*  DEBUG_PRINTSTR("[CONTROLLER]");
  DEBUG_PRINTSTR("[STRUCT SensorData SIZE]");
  DEBUG_PRINTLN(sizeof(struct sensorData));
  DEBUG_PRINTSTR("[CONTROLLER]");
  DEBUG_PRINTSTR("[STRUCT ResponseData SIZE]");
  DEBUG_PRINTLN(sizeof(struct responseData));
*/

  //Initiate Real Time Clock
#if (HW_RTC > NONE)

#if (HW_RTC == RTC_1302)
  myRTC.init(&myData.state);
#elif (HW_RTC == RTC_3231)
  #if(HW == HW_PHOTON)
    pinMode(D4, OUTPUT);
    digitalWrite(D4, HIGH);
  #endif
  myRTC.init(&myData.state);
#elif (HW_RTC == RTC_3232)
  pinMode(HW_RTC_PIN, OUTPUT);
  digitalWrite(HW_RTC_PIN, HIGH);
  //Bernhard@: anschauen ob myrepsonse.state oder mydata.state
  //displayTime(RTC.get());
  myRTC.init(&(myResponse.state));
  //  displayTime(myRTC.getTime());
#endif

  displayTimeFromUNIX(myRTC.getTime());   // if there is a RTC --> display the time independently of the RTC type
#endif

  myNodeList.clearEEPROM_Nodelist();    // deletes the node list!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  myNodeList.getNodeList();

  //  myResponse.ControllerTime = 1481803260;   // dummy time for testing..since I have only one RTC for testing
  //  myRTC.setTime(1485362865);



#if (SD_AVAILABLE == 1)

  if (initStorage() == false)
  {
    return;
  }

  DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("card initialized.");

  digitalWrite(12, HIGH);
  SPI.transfer(0xAA);
#endif




}//setup

/*
   Initializes the storage for data logging
   The Arduino writes to an SD card for debugging purposes,
   the particle will either write to his internal memory or not log the data at all.

*/
bool initStorage()
{
  if (HW == HW_ARDUINO)             // using arduino
  {
#if (SD_AVAILABLE == 1)
    DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(SD_CHIPSELECT))
    {
      DEBUG_PRINTLNSTR("Card failed, or not present");
      // don't do anything more:
      return false;
    }
#endif
  }
  else if (HW == HW_PHOTON)         // using photon
  {
    // #toimplement
  }
  return true;
}

/*******************************************************************************************/
/********************************L O O P***************************************************/
/*******************************************************************************************/

/*
  LOOP
    1) Check for Received Data pakets

    2)  No message arrived. In this case the controller can check the schedule whether pumps
        have to be activated and it has to check whether there have been IOT requests.
     2.1) Either Test cases are processed
      T1)TEST_PUMP: 1
          It is a test implementation which enables a pump all 30000 times
          the loop gets called. This is not very feasible but fine for testing.
      T2)TEST_PUMP 2
          In this mode the controller waits for N datasets of all sensornodes currently in the nodelist.(N can be set in Blumentopf.h)
          This means the number of sensor nodes in the nodelist * N * INTERVAL /10 is the number of seconds between the end of one watering period and the begin of the next one.
          If other nodes register in the meantime, they get ignored by this algotihm.
    2.2) Or the IOT interaction will be performed here

    2.3)Then

  END LOOP
*/
void loop(void)
{
#if (DEBUG_==1)
      if((millis()-time_)>20000)
      {
        if(DEBUG_RF24 == 1)
        {
          DEBUG_PRINTLNSTR("<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>");
          radio.printDetails();
          time_=millis();
          DEBUG_PRINTLNSTR("RF24-Settings:");
          DEBUG_PRINTSTR("\tRADIO_CHANNEL: ");DEBUG_PRINTLN(RADIO_CHANNEL);
          DEBUG_PRINTSTR("\tRADIO_DELAY: ");DEBUG_PRINTLN(RADIO_DELAY);
          DEBUG_PRINTSTR("\tRADIO_RETRIES: ");DEBUG_PRINTLN(RADIO_RETRIES);
          DEBUG_PRINTSTR("\tRADIO_SPEED: ");DEBUG_PRINTLN(RADIO_SPEED);
          DEBUG_PRINTSTR("\tRADIO_CRC: ");DEBUG_PRINTLN(RADIO_CRC);
          DEBUG_PRINTSTR("\tRADIO_PA_LEVEL: ");DEBUG_PRINTLN(RADIO_PA_LEVEL);
          DEBUG_PRINTLNSTR("<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>");
      }
      if(DEBUG_INFO==1)
      {
            DEBUG_PRINTSTR("\n\tSize of myData: ");DEBUG_PRINTLN(sizeof(myData));
            DEBUG_PRINTSTR("\n\tSize of myResponse: ");DEBUG_PRINTLN(sizeof(myResponse));
      }

    }
#endif


  uint8_t nPipenum;
#if (TEST_PUMP == 0)
  uint8_t nICA;   // Interactive Command Answer
  uint8_t nSCA;   // Scheduled Command Answer
  uint16_t nDuration;
  uint16_t nID;
#endif
  bResponseNeeded = true;    // not always a message needs to be sent
  uint8_t ret;


  //struct interactiveCommand myInteractiveCommand;
  myResponse.state = 0;
  myResponse.interval = INTERVAL;
  //DEBUG_PRINTSTR("Time before taking RTC:");DEBUG_PRINTLN(millis());
  //  myResponse.ControllerTime = getCurrentTime(); //maybe it is not clever to request time from RTC in EVERY loop
  //DEBUG_PRINTSTR("Time after taking RTC:");DEBUG_PRINTLN(millis());
  //DEBUG_PRINTSTR("[MEMORY]:Between Heap and Stack still "); DEBUG_PRINT(String(freeRam(), DEC));
  //DEBUG_PRINTLNSTR(" bytes available.");

  /**********************************************************************************************************************/
  /*
   *  1) CHECK FOR INCOMING MESSAGES
   *
   */
  //if (radio.available(&nPipenum) == true)    // 19.1.2017     checks whether data is available and passes back the pipe ID
    if (radio.available())
  {
    DEBUG_PRINTSTR("[CONTROLLER]RF24::Payloadsize: ");  DEBUG_PRINT(radio.getPayloadSize()); DEBUG_PRINTLNSTR(".");
    DEBUG_PRINTSTR("[TIME] : ");
    displayTimeFromUNIX(getCurrentTime());
//    DEBUG_PRINTSTR("\n[CONTROLLER] Message available at pipe ");
//    DEBUG_PRINTLN(nPipenum);
    while (radio.available()) {
      radio.read(&myData, sizeof(myData));
    }

// output message details only if required
    if (DEBUG_MESSAGE_HEADER > 0)
    {
      DEBUG_PRINTSTR("[CONTROLLER][RECEIVED]State: ");
      DEBUG_PRINTDIG(myData.state, BIN);
      DEBUG_PRINTSTR(" from ID: ");
      DEBUG_PRINT(myData.ID);
      DEBUG_PRINTSTR(" with Interval: ");
      DEBUG_PRINTLN(myData.interval);
      if(DEBUG_MESSAGE_HEADER_2 > 0)
      {
        DEBUG_PRINTSTR("humidity: ");
        DEBUG_PRINTLN(myData.humidity);
        DEBUG_PRINTSTR("moisture: ");
        DEBUG_PRINTLN(myData.moisture);
        DEBUG_PRINTSTR("brightness: ");
        DEBUG_PRINTLN(myData.brightness);
        DEBUG_PRINTSTR("voltage: ");
        DEBUG_PRINTLN(myData.voltage);
        DEBUG_PRINTSTR("VCC: ");
        DEBUG_PRINTLN(myData.VCC);
        DEBUG_PRINTSTR("realTime: ");
        DEBUG_PRINTLN(myData.realTime);
      }
    }

    //      myResponse.ControllerTime = 1481803260;
    //    getUNIXtime(&myResponse.ControllerTime);    // gets current timestamp

    // log the data to the SD card:
    logData();

    if ((myData.state & (1 << MSG_TYPE_BIT)) == false) // this is a registration request. Send ack-message
    {
      DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("Registration request");
      handleRegistration();                 // answer the registration request. There is no difference between sensor nodes and motor nodes.
    }
    else                                    // This is a data message
    {
      myResponse.ControllerTime = getCurrentTime();
      if ((myData.state & (1 << NODE_TYPE)) == false) // it is a sensor node
      {
        DEBUG_PRINTLNSTR("[CONTROLLER] SENSOR MESSAGE");
        handleDataMessage();
      }
      else                                  // it is a motor node message
      {
        DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("MOTOR MESSAGE:ID:");
        DEBUG_PRINT(myData.ID); DEBUG_PRINTSTR(", Data:");
        DEBUG_PRINTLN(myData.interval);
        handleMotorMessage();

        //        bResponseNeeded = false;
      }
    }
    //in every loop bResponseNeeded is redefined to true
    if (bResponseNeeded == true)          // If it is necessary to send an answer, send it.
    {

      //to make state changes and to turn on receiver mode, otherwise the message is lost

      //DEBUG_PRINTLN(myResponse.ControllerTime);

      // Send back response, controller real time and the next sleep interval:
      DEBUG_PRINTSTR("\tSending back response - interval: ");
      DEBUG_PRINT(myResponse.interval);
      DEBUG_PRINTSTR(", ID:");
      DEBUG_PRINT(myResponse.ID);
      DEBUG_PRINTSTR(", STATUS-BYTE:");
      DEBUG_PRINTLN(String(myResponse.state, BIN));
      delay(WAIT_SEND_INTERVAL);//ther is some time to, to ensure that node is prepared to receive messages     // 20170312 - Berhnard: Can we find another way for this as this solution slows down the communication and is suspected to leading to timeouts.
      radio.stopListening();
      while(write_cnt > 0) //handleDataMessage and handleMotorMessage could manipulate write_cnt
      {
        radio.write(&myResponse, sizeof(myResponse));
        write_cnt--;
      }
      radio.startListening();
//      DEBUG_PRINTLNSTR("\tDone");

      DEBUG_PRINTLNSTR("[CONTROLLER] Listening now...");

    }else{
        DEBUG_PRINTSTR("\tResponse sending is skipped: ");
        DEBUG_PRINTLNSTR("[CONTROLLER] Listening now...");
    }
    digitalWrite(LED_BUILTIN, LOW);
    write_cnt=1;//reset
  }

  else  // no message arrived. In this case the controller can check the schedule whether pumps have to be activated and it has to check whether there have been IOT requests.
  /*
   *  2) NO MESSAGE ARRIVED. CHECK PUMP SCHEDULE
   * */
  {

/*    2.1)
* TEST_PUMP: 1
* It is a test implementation which enables a pump all 30000 times the loop gets called. This is not very feasible but fine for testing.
*
* TEST_PUMP: 2
* It is a pump scheduling implementation which can be considered as final, if there are no other requirement requests about the pump scheduling.
* In this mode the controller waits for N datasets of all sensornodes currently in the nodelist. (N can be set in Blumentopf.h)
* This means the number of sensor nodes in the nodelist * N * INTERVAL /10 is the number of seconds between the end of one watering period and the begin of the next one.
* If other nodes register in the meantime, they get ignored by this algotihm.
*
*

*/
#if (TEST_PUMP == 1)
    /* this is only for testing!! */
    // DEBUG_PRINTLN(nTestWatering);

    if (myNodeList.mnNodeCount > 0) {
      if(i_ >= myNodeList.mnNodeCount)
        i_= 0;
      nTestWatering++;
    }

    if ((nTestWatering % 10000) == 0 )
    {

      if (myNodeList.getNodeType(myNodeList.myNodes[i_].ID) == 1)
      {

        DEBUG_PRINTSTR("\t[CONTROLLER][TEST] Node id: ");
        DEBUG_PRINTLN(myNodeList.myNodes[i_].ID);
        if (myNodeList.isActive(myNodeList.myNodes[i_].ID) == 0)//check if the first node (index=0)in the list is active
        {
          ret = doWateringTasks(myNodeList.myNodes[i_].ID, 10000, 0); //here a new order to a pump Node has to be planned
          if (ret > 0) {
            DEBUG_PRINTSTR("\t[CONTROLLER]");
            DEBUG_PRINTLN(handle_ErrorMessages(ret));

          }
        } else
        DEBUG_PRINTLNSTR("\t[CONTROLLER][TEST] ERROR: Node already in use.");
      }
      i_++;

    }
    /* Testing end */
#elif (TEST_PUMP == 2)
  /* second test .. or real mode..we will see*/
  time_t myCurrentTime;
  static time_t myPreviousTime;
  static bool bProcessPumps = false;
  uint16_t connectedSensorNode;

  nTestWatering++;
  if (myNodeList.mnPumpSlotEnable == true)
  {

    if ((nTestWatering % DEBUG_CYCLE) == 0 )    // Delays the program flow..only once a second this will be executed:
    {
//      DEBUG_PRINTSTR("[TIME] : ");
      myCurrentTime = getCurrentTime();
//      displayTimeFromUNIX(myCurrentTime, 1);

if (bProcessPumps == false)
{
        if (myNodeList.mnPumpSlot <= myCurrentTime)      // The sensorNode-slots are over. Now it's time to go through the pumps and activate them if needed.
        {
          DEBUG_PRINTLNSTR("\r\n\tAll data arrived. Activating the pumps...");
          bProcessPumps = true;
          myNodeList.mnActivePump = 0;

        }
        else
        {
          DEBUG_PRINTLNSTR("\tWaiting until the pump timeslot starts..");
        }
}

      if (bProcessPumps == true)    // The pumps have to be processed?
      {
        if (PumpList.size() > 0)    // are there still active pumps?
        {
          DEBUG_PRINTLNSTR("\t\t Waiting for pump to finish pumping...");
        }
        else
        {
          DEBUG_PRINTLNSTR("\t\t No currently active pump.. Next pump can be started.");
// Now the pumps have to be processed, one after the other.
          for(myNodeList.mnActivePump; myNodeList.mnActivePump < myNodeList.mnNodeCount; myNodeList.mnActivePump++)
          {

            if ((myNodeList.myNodes[myNodeList.mnActivePump].state & (1<<NODELIST_NODETYPE)) == 1)       // it is a pump node
            {
              DEBUG_PRINTSTR("\t\t Node ID: ");
              DEBUG_PRINT(myNodeList.mnActivePump);
              DEBUG_PRINTLNSTR(" is a pump node.\r\n\t\tActivating the pump...");


  // if commands are sent to active pumps only, an inactive pump will never become active again, except if it registers itself again through a manual restart.
  // Therefore it seems also inactive pumpnodes should be addressed here.
              connectedSensorNode = myNodeList.findNodeByID(myNodeList.myNodes[myNodeList.mnActivePump].sensorID);   // this is the connected Sensor
              DEBUG_PRINTSTR("\t\t\tMoisture: ");
              DEBUG_PRINT(myNodeList.myNodes[connectedSensorNode].nodeData.moisture);
              DEBUG_PRINTSTR(", ");
              DEBUG_PRINTLN(myNodeList.myNodes[connectedSensorNode].nodeData.moisture2);
              if (myNodeList.myNodes[connectedSensorNode].nodeData.moisture <= WATERING_THRESHOLD)    // watering needed
//if (1)
              {
                DEBUG_PRINTLNSTR("\t\tWatering needed.");

                ret = doWateringTasks(myNodeList.myNodes[myNodeList.mnActivePump].ID, POL_WATERING_DEFAULT_DURATION*1000, 0); //here a new order to a pump Node has to be planned
                if (ret > 0)
                {
                  DEBUG_PRINTSTR("\t[CONTROLLER]");
                  DEBUG_PRINTLN(handle_ErrorMessages(ret));
                }
                myNodeList.mnActivePump++;
                break;
              }
              else
              {
                DEBUG_PRINTLNSTR("\t\tNo watering needed.");
              }
            }
            else
            {
              DEBUG_PRINTSTR("\t\t Node ID: ");
              DEBUG_PRINTLN(myNodeList.mnActivePump);

            }
          }
        }

//          DEBUG_PRINTSTR("\t\tBroken - ");
//          DEBUG_PRINTLN(myNodeList.mnActivePump);
// If the loop finishes, all pumps have been processed
        if (myNodeList.mnActivePump >= myNodeList.mnNodeCount)
        {
          DEBUG_PRINTLNSTR("\t\tAll pumpnodes are processed");
          myNodeList.mnPumpSlotEnable = false;
          bProcessPumps = false;
        }

        nTestWatering = 0;
      }
    }

  }
  else
  {
    bProcessPumps = false;
  }


/*  if (myNodeList.mnCycleCount >= WATERING_CYCLES_TO_WAIT)   // there have been enough measurement cycles. Schedule the pump nodes now
  {

  }
  */
#else
    /*perform normal operation, there is no Testcase*/
    nICA = myCommandHandler.getInteractiveCommands();        // checks whether the user requested watering with its app.
    // The following is the actual scheduling algorithm, but commented out as the above test section tests the pump communication for now. Afterwards the scheduling can be tested, debugged and implementation finished:
    //    nSCA = myCommandHandler.checkSchedule(myNodeList, &nID, &nDuration, currentTime);
    nSCA = NO_SCHEDULED_WATERING;                                                             // for communication tests just pretend there is nothing to schedule
    if (nSCA == SCHEDULED_WATERING)
    {
      //@marko  CHECK RETURN VALUE with
      //ATTENTION: parameter for nDuration must be in ms, but for workaround I multiplied with 1000
      doWateringTasks(nID, nDuration * 1000, 0);               //  the node is added to the "active pumps"-list and the pump is notified
    }
    /*    if (nICA == INTERACTIVE_COMMAND_AVAILABLE )                                     // some IOT watering needs to be done
        {
           //ATTENTION : PUMPTIME IN MILLISECONDS
          //doWateringTasks(1, 10); //here a new order to a pump Node has to be planned
        }
    */

#endif
    digitalWrite(LED_BUILTIN, LOW);
    //DEBUG_PRINTLN("nothing yet..");
    DEBUG_FLUSH;

  }


  /*Most of the time the Controller waits for incoming messages from Pumpnodes
     It is important that the Controller is not stuck in a state, because of waiting
     for a message from a Node who is not able to send a message
     Additionally it is necessary to delete storage of handler which are not required anymore
  */
  if (PumpList.size() > 0)
  {
    // DEBUG_PRINTLNSTR("Checking pump list");
    PumpNode_Handler *handler;
    for (int i = 0; i < PumpList.size(); i++) {
      handler = PumpList.get(i);
#if (TEST_PUMP == 1)
      if ((nTestWatering % DEBUG_CYCLE) == 0) {
        DEBUG_PRINTSTR("[TIME] : ");
        displayTimeFromUNIX(getCurrentTime(), 1);
        DEBUG_PRINTSTR("\r\n\t[CONTROLLER]"); DEBUG_PRINTSTR("Processing PumpHandler for NODE-ID:");
        DEBUG_PRINTLN(handler->getID());
      }
#endif
      handler->processPumpstate(0);//there is no Income Data (0), only process the state machine

      // This commented section is for the real pump scheduling. It is commented for now to not influence the pump protocol testing.
      /*    nDummyCount++;
            DEBUG_PRINTSTR("Dummy: ");
            DEBUG_PRINTLN(nDummyCount );
            if (nDummyCount >= 20) // pumping done
            { if_begin
            DEBUG_PRINTLNSTR("Deleting node form active pump list!");

            //Marko@ : eine active pump list habe ich bereits erstellt -> myNodeList.setPumpInactive(handler->getID());
      */
      if (handler->getState() == PUMPNODE_STATE_3_RESPONSE)
      {

        DEBUG_PRINTSTR("[TIME] : ");
        displayTimeFromUNIX(getCurrentTime(), 1);
        DEBUG_PRINTSTR("\n[CONTROLLER]Number of PumpHandlers: "); DEBUG_PRINTLN(PumpList.size());
        DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("Deleting PumpHandler Class because Watering finished ");
        DEBUG_PRINTLN(handler->getID());

        removePumphandler(i, handler);
        i--;

        printFreeRam();

      } else if (handler->getState() == PUMPNODE_STATE_ERROR)
      {
        DEBUG_PRINTSTR("[TIME] : ");
        displayTimeFromUNIX(getCurrentTime(), 1);
        DEBUG_PRINTSTR("\n[CONTROLLER]Number of PumpHandlers:"); DEBUG_PRINTLN(PumpList.size());
        DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("ERROR:PUMP STATEHANDLER IS IN ERROR STATE");
        DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("ERROR:RESTART PUMP ID:"); DEBUG_PRINT(handler->getID());
        DEBUG_PRINTSTR(" ,PumpTime: "); DEBUG_PRINTLN(handler->getPumpTime());
        //@Marko should PUMP always be restarted, maybe it is OFFLINE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (handler->getStateErrorCount() <= MAX_RETRIES) {
          uint8_t ret = doWateringTasks(handler->getID(), handler->getPumpTime(), handler);//getPumpTime is in ms

          DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLN(handle_ErrorMessages(ret));
          if (ret > 0) {
            DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("ERROR:Deleting PumpHandler Class because ");
            DEBUG_PRINTSTR("of some Error in doWateringTasks()");

            removePumphandler(i, handler);
            i--;

            printFreeRam();
          }
        } else {
          DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("ERROR:PUMP IS NOT RESPONDING(");
          DEBUG_PRINT(MAX_RETRIES);
          DEBUG_PRINTSTR(" RETRIES), ");
          DEBUG_PRINTSTR("DELETE PUMPHANDLER AND SET PUMP NODE "); DEBUG_PRINT(handler->getID());
          DEBUG_PRINTLNSTR(" OFFLINE, THAT PUMP NODE MUST REGISTRATE AGAIN.");
          myNodeList.setNodeOffline(handler->getID());
          removePumphandler(i, handler);
          i--;

          printFreeRam();
        }
      }
    }

  }
#if (TEST_PUMP > 0)
  //only informatve, can be deleted later
  if ((nTestWatering % DEBUG_CYCLE) == 0) {
    DEBUG_PRINTSTR("[TIME] : ");
    displayTimeFromUNIX(getCurrentTime(), 1);
    DEBUG_PRINTLNSTR(" ");
//    DEBUG_PRINTSTR("\r\n\t[CONTROLLER]"); DEBUG_PRINTSTR("nTestWatering="); DEBUG_PRINTLN(nTestWatering);     // It is sufficient and clearer to just print the time.
    DEBUG_PRINTSTR_D("\t", DEBUG_FREE_MEMORY);
    printFreeRam();
    //nTestWatering = 0;  //Bernhard@: darf nicht auf 0 gesetzt werden, da sonst nicht hochgezählt werden kann!!!!!!!!


  }
#endif

}//loop()


inline void removePumphandler(int index, PumpNode_Handler* handler)
{
  PumpList.remove(index);
  myNodeList.setPumpInactive(handler->getID());
  delete handler;
  DEBUG_PRINTLNSTR("-----------------------------------------------------------------------------------------");

}



/**
   This Error Handler summarizes all error messages from different functions
   This should be suitable for a Logging System

   doWateringTasks:
    return 10: Pump not Online
    return 20: Node ID PumpNode_ID, is not a pumpNode
    return 30: Pump already active

*/

String handle_ErrorMessages(uint8_t ret)
{
  switch (ret)
  {
    case 10:
      return F(Error_WateringTask_1);
      break;
    case 20:
      return F(Error_WateringTask_2);
      break;
    case 30:
      return F(Error_WateringTask_3);
      break;
    case 40:
      return F(Error_WateringTask_4);
      break;
    default:
      break;
  }
  return "";

}


/*A pumpNode must perform watering, so we decide to start this task by creating a new
  PumpNode_Handler Class which controls the state changes and the whole interaction betwen
  Controller and PumpHandler
   return 0 : everything allright
   return 10: Pump not Online
   return 20: Node ID PumpNode_ID, is not a pumpNode
   return 30: Pump already active
  NOTIFICATION: (01.02.2017)Changed requirement for pumpTime to be in Milliseconds not in seconds

*/
uint8_t doWateringTasks(uint16_t PumpNode_ID, uint16_t pumpTime, PumpNode_Handler *handler_)
{
  DEBUG_PRINTSTR("[TIME] : ");
  displayTimeFromUNIX(getCurrentTime(), 1);
  DEBUG_PRINTLNSTR(" ");
  if (handler_ > 0) {
    if ((handler_->getID() == PumpNode_ID) && (handler_->getPumpTime() == pumpTime)) {
      handler_->reset();
      handler_->processPumpstate(pumpTime);
      myResponse.ID = PumpNode_ID;
      myResponse.interval = handler_->getResponseData();
      myResponse.state &= ~(1 << ID_INEXISTENT);
      DEBUG_PRINTSTR("[CONTROLLER]");
      DEBUG_PRINTSTR("[doWateringTasks()]Retry pump request to Node-ID: ");
      DEBUG_PRINT(PumpNode_ID);
      DEBUG_PRINTSTR(" with duration of ");
      DEBUG_PRINT(pumpTime);
      DEBUG_PRINTLNSTR("ms");
      //the first communication with the pumpNode must be initiate here
      handlePumpCommunications();
      DEBUG_PRINTSTR("[CONTROLLER]");
      DEBUG_PRINTSTR("[doWateringTasks()]");
      DEBUG_PRINT(PumpList.size());
      DEBUG_PRINTLNSTR(" pumps ACTIVE!!!!");
    }
    else
      return 40;


  } else if (myNodeList.isOnline(PumpNode_ID) == 1) {
    if (myNodeList.getNodeType(PumpNode_ID) == 1)
    {
      DEBUG_PRINTSTR("[CONTROLLER]");
      DEBUG_PRINTSTR("[doWateringTasks()]Pump ID ");
      if (myNodeList.isActive(PumpNode_ID) == 0)
      {
        DEBUG_PRINT(PumpNode_ID);
        DEBUG_PRINTLNSTR(" will be activated now.");
        myNodeList.setPumpActive(PumpNode_ID);
        //  uint16_t pumptime=0;
        PumpNode_Handler *handler = new PumpNode_Handler(PumpNode_ID);
        PumpList.add(handler);          // todo in february: the handler should only add the pump node if it isn't in the list already.
        nPumpHandlerCnt++;
        handler->setPumpHandlerID(nPumpHandlerCnt);
        DEBUG_PRINTLNSTR("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        handler->processPumpstate(pumpTime);
        myResponse.ID = handler->getID();
        myResponse.interval = handler->getResponseData();
        myResponse.state &= ~(1 << ID_INEXISTENT);

        DEBUG_PRINTSTR("[CONTROLLER]");
        DEBUG_PRINTSTR("[doWateringTasks()]Sending pump request to Node-ID: ");
        DEBUG_PRINT(PumpNode_ID);
        DEBUG_PRINTSTR(" with duration of ");
        DEBUG_PRINT(handler->getPumpTime());
        DEBUG_PRINTLNSTR(" ms");
        //the first communication with the pumpNode must be initiate here

        handlePumpCommunications();
        DEBUG_PRINTSTR("[CONTROLLER]");
        DEBUG_PRINTSTR("[doWateringTasks()]");
        DEBUG_PRINT(PumpList.size());
        DEBUG_PRINTLNSTR(" pumps ACTIVE!!!!");

      } else {
        DEBUG_PRINTLNSTR("\t[CONTROLLER]"); DEBUG_PRINTLNSTR("ERROR: PUMP IS ALREADY IN USE!!");
        return 30;
      }
    } else {
      DEBUG_PRINTLNSTR("\t[CONTROLLER]"); DEBUG_PRINTLNSTR("ERROR: THIS IS NOT A PUMP NODE!!");
      return 20;
    }
  } else {
    DEBUG_PRINTLNSTR("\t[CONTROLLER]"); DEBUG_PRINTLNSTR("ERROR: PUMP NODE IS NOT ONLINE!!");
    return 10;

  }

  return 0;
}


void handlePumpCommunications()
{

  radio.stopListening();//!!!!!!!!!!!!!!!!! KEEP ATTENTION OF TIME SLOT, IAM ALLOWED TO SEND here??
  while(write_cnt > 0) //handleDataMessage and handleMotorMessage could manipulate write_cnt
  {
    radio.write(&myResponse, sizeof(myResponse));
    write_cnt--;
  }
  radio.startListening();
  write_cnt=1;
  // delay(100);   // ist das delay notwendig?
}


/*
   This function logs the data to the storage.
   The Arduino writes the data to the SD card, the photon to its flash.
*/
void logData(void)
{
#if (SD_AVAILABLE == 1)
  String currentData = "";

  // parse the data to the string:
  currentData += String(myData.ID);
  currentData += ",";
  currentData += String(myData.realTime);
  currentData += ",";
  currentData += String(myData.temperature);
  currentData += ",";
  currentData += String(myData.humidity);
  currentData += ",";
  currentData += String(myData.moisture);
  currentData += ",";
  currentData += String(myData.moisture2);
  currentData += ",";
  currentData += String(myData.brightness);
  currentData += ",";
  currentData += String(myData.voltage);
  currentData += ",";
  currentData += String(myData.VCC);
  currentData += ",";
  currentData += String(myData.state);
  currentData += ",";
  currentData += String(myData.interval);
#endif
  if (HW == HW_ARDUINO)
  {
#if (SD_AVAILABLE == 1)
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    //int dataFile;

    // if the file is available, write to it:
    if (dataFile)
    {
      dataFile.println(currentData);
      dataFile.close();
      // print to the serial port too:
      DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[logData()]Writing to SD...");
      DEBUG_PRINTLN(currentData);

    }
    // if the file isn't open, pop up an error:
    else
    {
      DEBUG_PRINTSTR("[CONTROLLER]");
      DEBUG_PRINTLNSTR("[logData()]error opening datalog.txt");
      DEBUG_PRINTSTR("[CONTROLLER]");
      DEBUG_PRINTLNSTR("[logData()]data:");
      DEBUG_PRINTLN(currentData);
    }
#endif
  }
  else if (HW == HW_PHOTON)   // in case of a particle, the data might be logged to the flash or not at all.
  {
    // #toimplement
  }
}

/*
   This function registers a node that joins the network.
   It generates a new persistent ID if needed.
   The function still has to be extended.
*/
void handleRegistration(void)
{
  uint8_t nRet;
  bool newNode = true;
  struct nodeListElement currentNode;

  myResponse.ControllerTime = getCurrentTime();
  DEBUG_PRINTLNSTR("[CONTROLLER][handleRegistration()] Registration request!");
  myResponse.state = (1 << REGISTER_ACK_BIT);
  //  if (myData.ID > 0)                      // known node
  if ((myData.state & (1 << NEW_NODE_BIT)) == false)                    // known node
  { //a node with ID = 0x0 is valid????
    myResponse.ID = myData.ID;
    newNode = false;
  }
  else                                    // new node
  {
    myResponse.interval = 100 * myData.temperature + 20; // this is the session ID (we abused the temperature attribute here.)
    myResponse.ID = myResponse.interval * myResponse.ControllerTime / 100;                       // this is the persistent ID.. Todo : it has to be compared to the node-list, to ensure no ID is used twice
    //newNode=true;
  }

  // store the node in the list if it doesn't exist yet.
  //  DEBUG_PRINTSTR("ID:");
  //  DEBUG_PRINTLN(myResponse.ID);
  //  DEBUG_PRINTSTR("state: ");
  //  DEBUG_PRINTLN(myData.state);

  currentNode.ID = myResponse.ID;
  currentNode.sensorID = 0;//Marko@ wann erfolgt eigentlich Zuweisung zum Pumpnode???

  if ((myData.state & (1 << NODE_TYPE)) == 0)
  {
    DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]SensorNode");
    currentNode.state &= ~(1 << NODELIST_NODETYPE);  // SensorNode
    myNodeList.mnLastAddedSensorNode = currentNode.ID;
    DEBUG_PRINTSTR("\t\tNew last SensorNode - ID: ");
    DEBUG_PRINTLN(myNodeList.mnLastAddedSensorNode);
  }
  else
  {
    DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]PumpNode");
    currentNode.state |= (1 << NODELIST_NODETYPE);  // MotorNode
    currentNode.sensorID = myNodeList.mnLastAddedSensorNode;
    DEBUG_PRINTSTR("\t\tUsing current last SensorNode - ID: ");
    DEBUG_PRINTLN(myNodeList.mnLastAddedSensorNode);
  }
  DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]Storing node..");

  if (newNode) {

    //Bit NODELIST_NODEONLINE in currentNode.state should stay 0 in the EEPROM forever
    nRet = myNodeList.addNode(currentNode);
    if (nRet > 0)       // there was a serious problem when adding the node. Node has not been added
    {
      //introduced a new Flag, the registrating Node must be somehow informed about
      //bad registration
      myResponse.state |= (1 << ID_REGISTRATION_ERROR);
      DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]ERROR:Node registration aborted!");
    }
    else
    { //succefull registration of a new node
      DEBUG_PRINTSTR("[CONTROLLER][handleRegistration()] Session-ID: ");
      DEBUG_PRINT(myResponse.interval / 100);
      DEBUG_PRINTSTR(", Interval: ");
      DEBUG_PRINT(myResponse.interval % 100);
      DEBUG_PRINTSTR(", Persistent ID: ");
      DEBUG_PRINT(myResponse.ID);
      DEBUG_PRINTSTR(", Timestamp of controller: ");
      DEBUG_PRINTLN(myResponse.ControllerTime);
      //now the node is online
      myNodeList.setNodeOnline(myResponse.ID);
    }
  } else
  {
    if (myNodeList.findNodeByID(myData.ID) == 0xffff)
    {
      myResponse.state |= (1 << ID_REGISTRATION_ERROR);
      DEBUG_PRINTSTR("[CONTROLLER]");
      DEBUG_PRINTLNSTR("[handleRegistration()]ERROR:ID is not in the NodeList registered!");
      DEBUG_PRINTSTR("[CONTROLLER]");
      DEBUG_PRINTLNSTR("[handleRegistration()]ERROR:Node registration aborted!");
    } else
    { //succefull registration of a known node
      DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]Session-ID and interval: ");
      DEBUG_PRINT(myResponse.interval);
      DEBUG_PRINTSTR(", Persistent ID: ");
      DEBUG_PRINTLN(myResponse.ID);
      DEBUG_PRINTSTR(", Time: ");
      DEBUG_PRINTLN(myResponse.ControllerTime);
      //now the node is online
      myNodeList.setNodeOnline(myResponse.ID);
    }
  }

}


void handleDataMessage(void)
{
  uint16_t nodeIndex;
  nodeIndex = myNodeList.findNodeByID(myData.ID);
  myResponse.state &= ~(1 << ID_INEXISTENT);     // per default the controller knows the node ID
  if (nodeIndex == 0xffff)      // if the node does not exist
  {
    DEBUG_PRINTLNSTR("\t*ERROR*  Node does not exist in the node list - there seems to be a topology problem.\r\n\tID: ");
    DEBUG_PRINT(myData.ID);

    myResponse.state |= (1 << ID_INEXISTENT);     // tell the node, the controller doesn't know him.
  }
  DEBUG_PRINTLNSTR("\tData message.");
  if (DEBUG_DATA_CONTENT > 0)   // show the data contents on the serial interface
  {
    DEBUG_PRINT("\t\tID: ");
    DEBUG_PRINT(myData.ID);
    DEBUG_PRINTSTR(", Message Type: ");
    DEBUG_PRINT(myData.state & (1 << MSG_TYPE_BIT));
    DEBUG_PRINTSTR(", RTC-Status: ");
    DEBUG_PRINTLN(myData.state & (1 << RTC_RUNNING_BIT));
    DEBUG_PRINT("\t\tTemp: ");
    DEBUG_PRINT(myData.temperature);
    DEBUG_PRINTSTR(", Humidity: ");
    DEBUG_PRINT(myData.humidity);
    DEBUG_PRINTSTR(", Moisture: ");
    DEBUG_PRINT(myData.moisture);
    DEBUG_PRINTSTR(", Moisture2: ");
    DEBUG_PRINT(myData.moisture2);
    DEBUG_PRINTSTR(", Brightness: ");
    DEBUG_PRINTLN(myData.brightness);
    DEBUG_PRINTSTR("\t\tInterval: ");
    DEBUG_PRINT(myData.interval);
    DEBUG_PRINTSTR(", Battery Voltage: ");
    DEBUG_PRINTDIG((float)myData.voltage / 100, 2);
    DEBUG_PRINTSTR(", VCC Voltage: ");
    DEBUG_PRINTDIG((float)myData.VCC / 100, 2);
    DEBUG_PRINTSTR(", Time: ");
    DEBUG_PRINTLN(myData.realTime);
  //      DEBUG_PRINTSTR("Time: ");
  //      DEBUG_PRINTLN(myResponse.ControllerTime);
  }

  //  myResponse.state &= ~((1 << FETCH_EEPROM_DATA1) | (1 << FETCH_EEPROM_DATA2));   // We do not want to have EEPROM data now
    myResponse.state |= (1 << FETCH_EEPROM_DATA1);   // We do want to have EEPROM data now
    myResponse.state &= ~(1 << FETCH_EEPROM_DATA2);

    myResponse.ID = myData.ID;

// copying the data to the nodelist
    myNodeList.myNodes[nodeIndex].nodeData = myData;

  //  myResponse.interval = INTERVAL;
  //  time_t nextSlotTime = getNextMeasurementSlot(nodeIndex);

  if ((myData.state & (1 << EEPROM_DATA_PACKED)) == 0)   // only for live data. EEPROM data doesn't get scheduled
  {
    DEBUG_PRINTLNSTR("\tIt was live data...schedule next measurement");
    myResponse.interval = getNextMeasurementSlot(nodeIndex);
  }
  else
  {
    DEBUG_PRINTLNSTR("\tIt was EEPROM data...no scheduling needed.");
  }

}

/*
   This function deals with motor node messages.
   They can either be state messages (for example when watering is done)
   or responses to a instruction message.
   See the state diagram in the repository for details.
*/
void handleMotorMessage(void)
{
  DEBUG_PRINTSTR("[CONTROLLER]");
  DEBUG_PRINTLNSTR("[handleMotorMessage()]A new Motormessage received.....");
  // check if the node ID actually exists in the node table..
  uint16_t nodeIndex;
  nodeIndex = myNodeList.findNodeByID(myData.ID);   // or however the ID is called..

  myResponse.state &= ~(1 << ID_INEXISTENT);     // per default the controller knows the node ID
  if (nodeIndex == 0xffff)      // if the node does not exist
  {
    DEBUG_PRINTSTR("[CONTROLLER]");
    DEBUG_PRINTLNSTR("[handleMotorMessage()]Node does not exist - there seems to be a topology problem.");
    myResponse.state |= (1 << ID_INEXISTENT);     // tell the node, the controller doesn't know him.
  } else if (isActive(nodeIndex))
  {
    DEBUG_PRINTSTR("[CONTROLLER]");
    DEBUG_PRINTLNSTR("[handleMotorMessage()]Iterate pump list and search for Pumphandler");
    PumpNode_Handler *handler;
    for (int i = 0; i < PumpList.size(); i++) {
      handler = PumpList.get(i);
      //DEBUG_PRINTLN("Processing PumpHandler for NODE-ID:" + String(handler->getID(), DEC));
      if (handler->getID() == myData.ID)
      {
        /*
        *dummy8 contains state from pumpnode, if the message state  is not equal
        *pumphandler state, we have a redundant message here, which will be skipped
        */
        if(myData.dummy8 ==  handler->getState())
        {
          handler->processPumpstate(myData.interval);
          DEBUG_PRINTSTR("[CONTROLLER]");
          DEBUG_PRINTLNSTR("[handleMotorMessage()]Iterate pump list and search for Pumphandler");
          if(handler->getState() < PUMPNODE_STATE_3_FINISHED)
          {
            myResponse.ID = myData.ID;
            myResponse.interval = handler->getResponseData();
            myResponse.dummy8=handler->getState();
            write_cnt=RADIO_RESEND_NUMB;
            bResponseNeeded=true;
            DEBUG_PRINTLNSTR("\tPumpHandler processed, we will send a respond.");
          }else{
            write_cnt=1;
            bResponseNeeded=false;
            DEBUG_PRINTLNSTR("\tA Retransmitted Message, will be skipped.");
          }//else
        }else{
          DEBUG_PRINTLNSTR("\tA Retransmitted Message, will be skipped.");
        }
        i = PumpList.size(); //get out of the for lopp, we are finished

      }//if
    }//for
  }else
  {
    DEBUG_PRINTSTR("[CONTROLLER]");
    DEBUG_PRINTLNSTR("[handleMotorMessage()]Error: Got Message from PumpNode which is not active!");
  }
}




/*
   It is an abstraction layer to get the UNIX timestamp from RTC or web, depending on what's available
*/
time_t getCurrentTime(void)
{
#if (HW_RTC > 0)
  return myRTC.getTime();
#endif
  return 0;
}







/*
  TODO:

   - doWateringtask()-retry-show Pumplist.size
   - if controller set pumpnode offline, how to ensure lifesign from pumpnode
      NEED SOMETHING LIKE A PING

  - DELAY herausholen
  Logging System
    Where should the log be stored
    No LOG in case of Serial prints

  Unklar:
  - isOnline noch notwendig????
  - (607) currentNode.sensorID = 0;//Marko@ wann erfolgt eigentlich Zuweisung zum Pumpnode???
    removePumphandler() hier sollte oder kann pumphandler vom sensor gelöscht werden
  -  void handleDataMessage(void):: if ((myData.state & (1 << EEPROM_DATA_PACKED)) == 0)   // only for live data. EEPROM data doesn't get scheduled
    (werden nur eeprom daten versendet vom sensornode, dann erhält der sensornode kein neues zeitfenster?)
*/



/*
   The function calculates how long the next sensor node needs to sleep.
   Therefore it checks whether the start of watering is scheduled now.
   It returns in 100ms the sleep duration of the sensor node.
*/
uint16_t getNextMeasurementSlot(uint16_t nodeIndex)
{
  // todo: check if watering is triggered
  time_t currentTime;
  currentTime = getCurrentTime();

  if (myNodeList.mnPreviouslyScheduledNode >= nodeIndex) // The the nodelist has been parsed once
  {
    myNodeList.mnCycleCount++;    // as the whole list has been run through once more, increase the counter.
  }

  DEBUG_PRINTSTR_D("\t\tNumber of Sensor Nodes: ", DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINTLN_D(myNodeList.getNumberOfSensorNodes(), DEBUG_SENSOR_SCHEDULING);

  time_t tLastScheduledSensorNode = myNodeList.myNodes[myNodeList.getLastScheduledSensorNode()].nextSlot;   // get the last scheduled time slot of all sensor nodes
  DEBUG_PRINTSTR_D("\t\tTime of last scheduled sensor node: ", DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINTLN_D(tLastScheduledSensorNode, DEBUG_SENSOR_SCHEDULING);

  //  if (myNodeList.myNodes[nodeIndex].nextSlot == 0)  // this is the first slot
  //  {
  //    myNodeList.myNodes[nodeIndex].nextSlot =
  //  }

  //  DEBUG_PRINTSTR("Previous slot: ");
  //  DEBUG_PRINTLN(myNodeList.myNodes[nodeIndex].nextSlot);

  // if watering is not triggered, all sensor nodes will have a fixed interval:

  //  myNodeList.myNodes[nodeIndex].nextSlot = myNodeList.myNodes[nodeIndex].nextSlot + INTERVAL * myNodeList.getNumberOfSensorNodes();
  //  myNodeList.myNodes[nodeIndex].nextSlot = tLastScheduledSensorNode + INTERVAL;

  // are there active tasks? (Including the current one)
  if (tLastScheduledSensorNode > currentTime - 7)  // yes
  {
    DEBUG_PRINTLNSTR_D("\t\tThere are tasks active", DEBUG_SENSOR_SCHEDULING);
    myNodeList.myNodes[nodeIndex].nextSlot = tLastScheduledSensorNode + INTERVAL / 10;
  }
  else            // no tasks are scheduled for the future. Start scheduling now
  {
    DEBUG_PRINTLNSTR_D("\t\tNo active tasks", DEBUG_SENSOR_SCHEDULING);
    myNodeList.myNodes[nodeIndex].nextSlot = currentTime + INTERVAL / 10;
  }

  DEBUG_PRINTSTR("\t\tCycleCount: ");
  DEBUG_PRINTLN(myNodeList.mnCycleCount);
  if (myNodeList.mnCycleCount >= WATERING_CYCLES_TO_WAIT) // there have been enough cycles so far...move the next measurements after the pump node runs..
  {
//
    DEBUG_PRINTLNSTR("\t\tIn Pump Epoch");

    DEBUG_PRINTSTR("\t\t\tEpoch Length: ");
    DEBUG_PRINTLN(myNodeList.getPumpEpochLength());


    myNodeList.mnPumpSlotEnable = true;
    myNodeList.mnPumpSlot = myNodeList.myNodes[nodeIndex].nextSlot;
    myNodeList.myNodes[nodeIndex].nextSlot += myNodeList.getPumpEpochLength();


    myNodeList.mnCycleCount = 0;
  }

  DEBUG_PRINTSTR_D("\t\tCurrent Time: ", DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINTLN_D(currentTime, DEBUG_SENSOR_SCHEDULING);

  DEBUG_PRINTSTR_D("\t\tNext slot time: ", DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINT_D(myNodeList.myNodes[nodeIndex].nextSlot, DEBUG_SENSOR_SCHEDULING);



  DEBUG_PRINTSTR_D(" (Interval: ", DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINT_D(myNodeList.myNodes[nodeIndex].nextSlot - currentTime, DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINTSTR_D(",", DEBUG_SENSOR_SCHEDULING);

  DEBUG_PRINTSTR_D(" adjusted by protocol delays: ", DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINT_D(myNodeList.myNodes[nodeIndex].nextSlot - currentTime - (REGISTRATION_TIMEOUT_INTERVAL / 1000), DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINTLNSTR_D(")", DEBUG_SENSOR_SCHEDULING);


// show all scheduled measurements:
    printNodeList(currentTime);

// remember which node has been scheduled
  myNodeList.mnPreviouslyScheduledNode = nodeIndex;

  // for testing it is assumed that the watering is not triggered.
  return (myNodeList.myNodes[nodeIndex].nextSlot - currentTime - (REGISTRATION_TIMEOUT_INTERVAL / 1000));
}


void printNodeList(time_t currentTime)
{
  uint16_t i;
  if (DEBUG_LIST_SENSOR_SCHEDULING == 0)
  {
    return;
  }
  DEBUG_PRINTLNSTR("\tNodelist:");

  for(i = 0; i < myNodeList.mnNodeCount; i++)
  {
    if ((myNodeList.myNodes[i].state & (1<<NODELIST_NODETYPE)) == 0) // it is a sensor node
    {
      DEBUG_PRINTSTR("\t\tID: ");
      DEBUG_PRINT(myNodeList.myNodes[i].ID);
      DEBUG_PRINT(" - ");
      displayTimeFromUNIX(myNodeList.myNodes[i].nextSlot, 1);
      DEBUG_PRINTLN(" ");
    }
  }
}
