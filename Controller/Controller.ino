/*
Project:  NESE_Blumentopf
File:     Controller.ino
Authors:  Bernhard Fritz  (0828317@student.tuwien.ac.at)
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

#if (DEBUG==1)
uint32_t time_;
uint16_t nDummyCount=0;
#if(DEBUG_TIMING_LOOP > 0)
long unsigned duration_loop=0;
long unsigned duration_max=0;
long unsigned duration_tmp=0;
long unsigned duration_sending=0;
#endif

#endif

//#define INTERVAL (600)


struct Data myResponse; //32byte
struct Data myData; //32byte
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

  #if(HW==HW_PHOTON  && DEBUG==1)

  uint16_t max_cnt=30000;
  uint32_t _timer_=millis();
  uint32_t dif=0;
  while((dif=(millis()-_timer_))<max_cnt)
  {
    if((dif>10000) && ((dif % 5000)==0))
    {
      DEBUG_PRINTLNSTR("PARTICLE PHOTON DELAYED STARTUP.................");
      DEBUG_PRINTSTR("SETUP WILL BE CONTINUED IN ");
      DEBUG_PRINT(max_cnt-dif);
      DEBUG_PRINTLNSTR(" ms.");
    }
  }
  #endif
  //Initiate Real Time Clock


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

  DEBUG_PRINTLNSTR("\r\n****************************************************");
  /*The Photon Board is not able to print messages from Setup() from Startup
  * Some time must pass to be able to see Serial prints
  */







  #if (HW_RTC > NONE)

  #if (HW_RTC == RTC_1302)
  myRTC.init(&myData.state);
  #elif (HW_RTC == RTC_3231)
//  #if(HW == HW_PHOTON)
//  pinMode(D4, OUTPUT);
//  digitalWrite(D4, HIGH);
//  #endif
  pinMode(HW_RTC_PIN, OUTPUT);
  digitalWrite(HW_RTC_PIN, HIGH);
  delay(20);
  myRTC.init(&myData.state);
  #elif (HW_RTC == RTC_3232)
  pinMode(HW_RTC_PIN, OUTPUT);
  digitalWrite(HW_RTC_PIN, HIGH);
  delay(20);
  //Bernhard@: anschauen ob myrepsonse.state oder mydata.state
  //displayTime(RTC.get());
  /*
  tmElements_t tm;
  tm.Second=00;
  tm.Minute=25;
  tm.Hour=15;
  tm.Wday=1;   // day of week, sunday is day 1
  tm.Day=30;
  tm.Month=4;
  tm.Year=CalendarYrToTm(2017);//y2kYearToTm(2017) ;   // offset from 1970;
  //setTime(makeTime(tm));
  myRTC.setTime(makeTime(tm));
  */
  myRTC.init(&(myResponse.state));
  //  displayTime(myRTC.getTime());
  #endif

  displayTimeFromUNIX(myRTC.getTime());   // if there is a RTC --> display the time independently of the RTC type
  #endif

  myNodeList.clearEEPROM_Nodelist();    // deletes the node list!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  myNodeList.getNodeList();

  //mark every response as Controller packet
  setDATA_ControllerPacket(&myResponse);//Helmut@: for Logging
  DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("[MYRESPONSE PACKETINFO]:"); DEBUG_PRINTLN(myResponse.packetInfo);

  #if (SD_AVAILABLE == 1)

  if (initStorage() == false)
  {
    return;
  }

  DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("card initialized.");

  digitalWrite(12, HIGH);
  SPI.transfer(0xAA);
  #endif

  DEBUG_FLUSH;


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

//DEBUG_PRINT(":");
//  DEBUG_PRINTLN(myNodeList.mnLastAddedSensorNode);

  #if (DEBUG==1)
  duration_loop=micros();

  #if(DEBUG_INFO>0)
  if((millis()-time_)>20000)
  {
    #if(DEBUG_RF24>0)

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
    #endif
    DEBUG_PRINTSTR("\n\tSize of myData: ");DEBUG_PRINTLN(sizeof(myData));
    DEBUG_PRINTSTR("\n\tSize of myResponse: ");DEBUG_PRINTLN(sizeof(myResponse));
    #if(DEBUG_TIMING_LOOP==1)
    DEBUG_PRINTSTR("[CONTROLLER][TIMING LOOP]Maximal loop time:");
    DEBUG_PRINT_D(duration_max, DEC);
    DEBUG_PRINTLNSTR(" microseconds.");
    #endif
  }//if((millis()-time_)>20000)
  #endif//#if(DEBUG_INFO>0)
  #endif//#if (DEBUG==1)


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
    #if (DEBUG_INFO>0)
    #if(DEBUG_RF24>0)
    DEBUG_PRINTSTR("[CONTROLLER]RF24::Payloadsize: ");
    DEBUG_PRINT(radio.getPayloadSize());
    #endif
    DEBUG_PRINTLNSTR(".");
    DEBUG_PRINTSTR("[TIME] : ");
    displayTimeFromUNIX(getCurrentTime());
    #endif

    while (radio.available()) {

      radio.read(&myData, sizeof(myData));
    }

    // output message details only if required
    #if (DEBUG_MESSAGE_HEADER > 0)

    DEBUG_PRINTSTR("[CONTROLLER][RECEIVED]State: ");
    DEBUG_PRINTDIG(myData.state, BIN);
    DEBUG_PRINTSTR(" from ID: ");
    DEBUG_PRINT(myData.ID);
    DEBUG_PRINTSTR(" PACKETINFO BITS: ");
    DEBUG_PRINTDIG(myData.packetInfo,BIN);
    DEBUG_PRINTLNSTR("  . ");
    #if(DEBUG_MESSAGE_HEADER_2 > 0)

    DEBUG_PRINTSTR("PumpTime: ");
    DEBUG_PRINTLN(myData.pumpTime);
    DEBUG_PRINTSTR("Interval: ");
    DEBUG_PRINTLN(myData.interval);
    DEBUG_PRINTSTR("temperature: ");
    DEBUG_PRINTLN(myData.temperature);
    DEBUG_PRINTSTR("humidity: ");
    DEBUG_PRINTLN(myData.humidity);
    DEBUG_PRINTSTR("moisture: ");
    DEBUG_PRINTLN(myData.moisture);
    DEBUG_PRINTSTR("moisture2: ");
    DEBUG_PRINTLN(myData.moisture2);
    DEBUG_PRINTSTR("brightness: ");
    DEBUG_PRINTLN(myData.brightness);
    DEBUG_PRINTSTR("voltage: ");
    DEBUG_PRINTLN(myData.voltage);
    DEBUG_PRINTSTR("VCC: ");
    DEBUG_PRINTLN(myData.VCC);
    DEBUG_PRINTSTR("realTime: ");
    DEBUG_PRINTLN(myData.Time);
    #endif
    #endif

    //      myResponse.ControllerTime = 1481803260;
    //    getUNIXtime(&myResponse.ControllerTime);    // gets current timestamp

    // log the data to the SD card:
    logData();

    if ((myData.state & (1 << MSG_TYPE_BIT)) == false) // this is a registration request. Send ack-message
    {
      DEBUG_PRINTSTR_D("[CONTROLLER] Registration request", DEBUG_MESSAGE);

      handleRegistration();                 // answer the registration request. There is no difference between sensor nodes and motor nodes.
      setDATA_RegistrationPacket(&myResponse);//Helmut@: for Logging

    }
    else                                    // This is a data message
    {
      setDATA_NO_RegistrationPacket(&myResponse);//Helmut@: for Logging

      myResponse.Time = getCurrentTime();
      if ((myData.state & (1 << NODE_TYPE)) == false) // it is a sensor node
      {
        DEBUG_PRINTLNSTR_D("[CONTROLLER] SENSOR MESSAGE", DEBUG_MESSAGE);

        handleDataMessage();
        setDATA_SensorPacket(&myResponse);//Helmut@: for Logging
      }
      else                                  // it is a motor node message
      {
        #if(DEBUG_MESSAGE>0)
        DEBUG_PRINTSTR("[CONTROLLER] MOTOR MESSAGE:ID:");
        DEBUG_PRINT(myData.ID);
        DEBUG_PRINTSTR(", Data:");
        DEBUG_PRINTLN(myData.pumpTime);
        #endif
        handleMotorMessage();
        setDATA_PumpPacket(&myResponse);//Helmut@: for Logging

        //        bResponseNeeded = false;
      }
    }
    //in every loop bResponseNeeded is redefined to true
    if (bResponseNeeded == true)          // If it is necessary to send an answer, send it.
    {

      //to make state changes and to turn on receiver mode, otherwise the message is lost

      //DEBUG_PRINTLN(myResponse.ControllerTime);

      // Send back response, controller real time and the next sleep interval:
      #if(DEBUG_MESSAGE>0)
      DEBUG_PRINTSTR("[CONTROLLER][SENDING RESPONSE] - interval: ");
      DEBUG_PRINT(myResponse.interval);
      DEBUG_PRINTSTR(", ID:");
      DEBUG_PRINT(myResponse.ID);
      DEBUG_PRINTSTR(", STATUS-BYTE:");
      DEBUG_PRINTLN(String(myResponse.state, BIN));
      //DEBUG_PRINTSTR(", dummy8:");
      //DEBUG_PRINTLN(String(getData_PumpState(&myResponse), DEC));
      DEBUG_PRINTSTR(", PACKET-INFO:");
      DEBUG_PRINTLN(String(myResponse.packetInfo, BIN));
      #endif

      delay(WAIT_SEND_INTERVAL);//ther is some time to, to ensure that node is prepared to receive messages     // 20170312 - Berhnard: Can we find another way for this as this solution slows down the communication and is suspected to leading to timeouts.
      #if (DEBUG_TIMING_LOOP>0)
      duration_sending=micros();
      #endif
      radio.stopListening();
      while(write_cnt > 0) //handleDataMessage and handleMotorMessage could manipulate write_cnt
      {
        radio.write(&myResponse, sizeof(myResponse));
        write_cnt--;
      }
      radio.startListening();
      #if (DEBUG_TIMING_LOOP>0)
      DEBUG_PRINTSTR("[TIMING][to PumpNode ID: ");
      DEBUG_PRINT(myResponse.ID);
      DEBUG_PRINTSTR(" ][Duration - before stopListening - after startList.]: ");
      DEBUG_PRINT(micros() - duration_sending);
      DEBUG_PRINTLNSTR(" microseconds.");
      #endif
    }
      DEBUG_PRINTSTR_D("\tResponse sending is skipped: ", DEBUG_MESSAGE);
      DEBUG_PRINTLNSTR_D("[CONTROLLER] Listening now...", DEBUG_MESSAGE);

    digitalWrite(LED_BUILTIN, LOW);
    write_cnt=1;//reset
  }//  if (radio.available())

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
      {
        i_= 0;
      }
      nTestWatering++;
    }

    if ((nTestWatering % 10000) == 0 )
    {

      if (myNodeList.getNodeType(myNodeList.myNodes[i_].ID) == 1)
      {
        DEBUG_PRINTSTR_D("\t[CONTROLLER][TEST] Node id: ", DEBUG_MESSAGE);
        DEBUG_PRINTLN_D(myNodeList.myNodes[i_].ID, DEBUG_MESSAGE);

        if (myNodeList.isActive(myNodeList.myNodes[i_].ID) == 0)//check if the first node (index=0)in the list is active
        {
          ret = doWateringTasks(myNodeList.myNodes[i_].ID, 10000, 0); //here a new order to a pump Node has to be planned
          #if(DEBUG_MESSAGE>0)
          if (ret > 0) {

            DEBUG_PRINTLNSTR("\t[CONTROLLER]ERROR: DOWATERING FAILED! ");
            DEBUG_PRINTSTR("\t[CONTROLLER]");
            DEBUG_PRINTLN(handle_ErrorMessages(ret));
            /*Marko@: Some more Error handling necessary?*/
          }
          #endif
        }
        #if(DEBUG_MESSAGE>0)
        else
        DEBUG_PRINTLNSTR("\t[CONTROLLER][TEST] WARNING: Node already in use.");
        #endif
      }
      i_++;

    }
    /* Testing end */
    #elif (TEST_PUMP == 2)
    /* second test .. or real mode..we will see*/
    time_t myCurrentTime;
    static time_t myPreviousTime;
    static bool bProcessPumps = false;
    uint16_t SensorNode_Pump1;
    uint16_t SensorNode_Pump2;

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

            DEBUG_PRINTLNSTR_D("\r\n\tAll data arrived. Activating the pumps...", DEBUG_MESSAGE);

            bProcessPumps = true;
            myNodeList.mnActivePump = 0;

          }
          #if(DEBUG_MESSAGE>0)
          else
          {
            DEBUG_PRINTLNSTR("\tWaiting until the pump timeslot starts..");
          }
          #endif

        }

        if (bProcessPumps == true)    // The pumps have to be processed?
        {
          //Bernhard@ only checkl PumpList.size()==0 and else is under Debug wrapping, can be optionally compiled
          if (PumpList.size() > 0)    // are there still active pumps?
          {
            //Bernhard@: Finde das ist Bad practice, ein if für eine Debug message
            //falls ma Debug abdrehen, beleiben solche Code Konstrukte übrig, abhängig wie der Compiler das handhabt
            DEBUG_PRINTLNSTR_D("\t\t Waiting for pump to finish pumping...", DEBUG_MESSAGE);

          }
          else
          {
            DEBUG_PRINTLNSTR_D("\t\t No currently active pump.. Next pump can be started.", DEBUG_MESSAGE);

            // Now the pumps have to be processed, one after the other.
            //Bernhard@:but not all pumpNodes should get commands at once, one pumpnode start, finish, THEN NEXT pumpnode
              //for loop shpudl be deleted and "if ((nTestWatering % DEBUG_CYCLE) == 0 )", so in every loop cycle
              //a check is performed here and ntestwatering is mnActivePump will be increased
            for(myNodeList.mnActivePump; myNodeList.mnActivePump < myNodeList.mnNodeCount; myNodeList.mnActivePump++)
            {

              if ((myNodeList.myNodes[myNodeList.mnActivePump].state & (1<<NODELIST_NODETYPE)) == 1)       // it is a pump node
              {
                #if(DEBUG_MESSAGE>0)
                DEBUG_PRINTSTR("\t\t Node ID: ");
                DEBUG_PRINT(myNodeList.mnActivePump);
                DEBUG_PRINTLNSTR(" is a pump node.\r\n\t\tActivating the pump...");
                #endif

                // if commands are sent to active pumps only, an inactive pump will never become active again, except if it registers itself again through a manual restart.
                // Therefore it seems also inactive pumpnodes should be addressed here.

                SensorNode_Pump1 = myNodeList.findNodeByID(myNodeList.myNodes[myNodeList.mnActivePump].sensorID1);   // this is the connected Sensor
                SensorNode_Pump2 = myNodeList.findNodeByID(myNodeList.myNodes[myNodeList.mnActivePump].sensorID2);   // this is the connected Sensor
                #if(DEBUG_PUMP_SCHEDULING>0)
                //Marko@: maybe a pump has to be restarted(check errorCounter in the Pumphandler), then there is no check necessary
                //or a restart is even not necessary because the moisture didnt increase, maybe that fact is easier
                  if ((myNodeList.myNodes[myNodeList.mnActivePump].state & (1<<SENSOR_PUMP1)) == 0) // pump 1 is attached to moisture sensor 1
                  {
                    DEBUG_PRINTSTR("\t\t\tMoisture 1: ");
                    DEBUG_PRINT(myNodeList.myNodes[SensorNode_Pump1].nodeData.moisture);
                  }
                  else                      // pump 1 is attached to moisture sensor 2
                  {
                    DEBUG_PRINTSTR("\t\t\tMoisture 2: ");
                    DEBUG_PRINTLN(myNodeList.myNodes[SensorNode_Pump1].nodeData.moisture2);
                  }

                  if ((myNodeList.myNodes[myNodeList.mnActivePump].state & (1<<SENSOR_PUMP2)) == 0) // pump 2 is attached to moisture sensor 1
                  {
                    DEBUG_PRINTSTR("\t\t\tMoisture 1: ");
                    DEBUG_PRINT(myNodeList.myNodes[SensorNode_Pump2].nodeData.moisture);
                  }
                  else                      // pump 2 is attached to moisture sensor 2
                  {
                    DEBUG_PRINTSTR("\t\t\tMoisture 2: ");
                    DEBUG_PRINTLN(myNodeList.myNodes[SensorNode_Pump2].nodeData.moisture2);
                  }
                #endif

/* Here is the actual watering logic:
 *  It checks whether watering is needed for any of the two pumps.
 *  If so, it starts the pump handler for the corresponding pumpnode
 *  and transmits the watering instructions.
*/
                if ((myNodeList.myNodes[SensorNode_Pump1].nodeData.moisture <= WATERING_THRESHOLD) || (myNodeList.myNodes[SensorNode_Pump2].nodeData.moisture <= WATERING_THRESHOLD))
                //if (1)
                {
                  DEBUG_PRINTLNSTR_D("\t\tWatering needed.", DEBUG_PUMP_SCHEDULING);
                  ret = doWateringTasks(myNodeList.myNodes[myNodeList.mnActivePump].ID, POL_WATERING_DEFAULT_DURATION*1000, 0, 0); //here a new order to a pump Node has to be planned
                  if (ret > 0)
                  {/*Marko@: Some more Error handling necessary?*/
                    #if(DEBUG_MESSAGE>0)
                      DEBUG_PRINTLNSTR("\t[CONTROLLER]ERROR: DOWATERING FAILED! ");
                      DEBUG_PRINTSTR("\t[CONTROLLER]");
                      DEBUG_PRINTLN(handle_ErrorMessages(ret));
                    #endif
                  }
                  myNodeList.mnActivePump++;
                  break;
                }
                #if(DEBUG_MESSAGE>0)
                else
                {
                  DEBUG_PRINTLNSTR("\t\tNo watering needed.");
                }
                #endif
              }//end check if it is a pump
              #if(DEBUG_MESSAGE>0)
              else
              {
                DEBUG_PRINTSTR("\t\t Node ID: ");
                DEBUG_PRINTLN(myNodeList.mnActivePump);

              }
              #endif
            }//for
          }//else of  if (PumpList.size() > 0)

          // If the loop finishes, all pumps have been processed
          if (myNodeList.mnActivePump >= myNodeList.mnNodeCount)
          {
            DEBUG_PRINTLNSTR_D("\t\tAll pumpnodes are processed", DEBUG_PUMP_SCHEDULING);
            myNodeList.mnPumpSlotEnable = false;
            bProcessPumps = false;
          }

          nTestWatering = 0;
        }//if (bProcessPumps == true)
      }//if ((nTestWatering % DEBUG_CYCLE) == 0 )

    }//if (myNodeList.mnPumpSlotEnable == true)
    else
    {
      bProcessPumps = false;
    }


    /*  if (myNodeList.mnCycleCount >= WATERING_CYCLES_TO_WAIT)   // there have been enough measurement cycles. Schedule the pump nodes now
    {

  }
  */
  #else
  /*IF NOT TEST IS PERFORMED*/
  /*perform normal operation, there is no Testcase*/
  nICA = myCommandHandler.getInteractiveCommands();        // checks whether the user requested watering with its app.
  // The following is the actual scheduling algorithm, but commented out as the above test section tests the pump communication for now. Afterwards the scheduling can be tested, debugged and implementation finished:
  //    nSCA = myCommandHandler.checkSchedule(myNodeList, &nID, &nDuration, currentTime);
  nSCA = NO_SCHEDULED_WATERING;                                                             // for communication tests just pretend there is nothing to schedule
  if (nSCA == SCHEDULED_WATERING)
  {
    //@marko  CHECK RETURN VALUE with
    //ATTENTION: parameter for nDuration must be in ms, but for workaround I multiplied with 1000
    doWateringTasks(nID, nDuration * 1000, 0, 0);               //  the node is added to the "active pumps"-list and the pump is notified
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


}////  if (radio.available()) ... else


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
      #if(DEBUG_MESSAGE>0)
      DEBUG_PRINTSTR("[TIME] : ");
      displayTimeFromUNIX(getCurrentTime(), 1);
      DEBUG_PRINTSTR("\r\n\t[CONTROLLER]"); DEBUG_PRINTSTR("Processing PumpHandler for NODE-ID:");
      DEBUG_PRINTLN(handler->getID());
      #endif
    }
    #endif
    handler->processPumpstate(0);//there is no Income Data (0), only process the state machine
    if(handler->getResponseAvailability())
    {
      DEBUG_PRINTSTR_D("[CONTROLLER]", DEBUG_MESSAGE);
      DEBUG_PRINTLNSTR_D("SENDING IN NORMAL MODE!!!!", DEBUG_MESSAGE);

      handlePumpCommunications(handler);
    }

    // This commented section is for the real pump scheduling. It is commented for now to not influence the pump protocol testing.
    /*    nDummyCount++;
    DEBUG_PRINTSTR("Dummy: ");
    DEBUG_PRINTLN(nDummyCount );
    if (nDummyCount >= 20) // pumping done
    { if_begin
    DEBUG_PRINTLNSTR("Deleting node form active pump list!");

    */
    if (handler->getState() == PUMPNODE_STATE_4_FINISHED)
    {
      #if(DEBUG_MESSAGE>0)
      DEBUG_PRINTSTR("[TIME] : ");
      displayTimeFromUNIX(getCurrentTime(), 1);
      DEBUG_PRINTSTR("\n[CONTROLLER]Number of PumpHandlers: "); DEBUG_PRINTLN(PumpList.size());
      DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("Deleting PumpHandler Class because Watering finished ");
      DEBUG_PRINTLN(handler->getID());
      #endif
      removePumphandler(i, handler);
      i--;

      printFreeRam();

    } else if (handler->getState() == PUMPNODE_STATE_ERROR)
    {
      #if(DEBUG_MESSAGE>0)
      DEBUG_PRINTSTR("[TIME] : ");
      displayTimeFromUNIX(getCurrentTime(), 1);
      DEBUG_PRINTSTR("\n[CONTROLLER]Number of PumpHandlers:"); DEBUG_PRINTLN(PumpList.size());
      DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("ERROR:PUMP STATEHANDLER IS IN ERROR STATE");
      DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("ERROR:RESTART PUMP ID:"); DEBUG_PRINT(handler->getID());
      DEBUG_PRINTSTR(" ,PumpTime: "); DEBUG_PRINTLN(handler->getPumpTime());
      #endif
      //@Marko should PUMP always be restarted, maybe it is OFFLINE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if (handler->getStateErrorCount() < MAX_RETRIES) {
        uint8_t ret = doWateringTasks(handler->getID(), handler->getPumpTime(), 0, handler);//getPumpTime is in ms
        #if(DEBUG_MESSAGE>0)
        DEBUG_PRINTSTR("[CONTROLLER]");
        DEBUG_PRINTSTR("This is the pumpnode_state_error_counter: ");
        DEBUG_PRINT(handler->getStateErrorCount());
        DEBUG_PRINTSTR(" attempt of  ");
        DEBUG_PRINT(MAX_RETRIES);
        DEBUG_PRINTLNSTR(" retries. ");
        DEBUG_PRINTSTR("[CONTROLLER]");
        DEBUG_PRINTLN(handle_ErrorMessages(ret));
        #endif
        if (ret > 0) {
          #if(DEBUG_MESSAGE>0)
          DEBUG_PRINTSTR("[CONTROLLER]");
          DEBUG_PRINTSTR("ERROR:Deleting PumpHandler Class because ");
          DEBUG_PRINTSTR("of some Error in doWateringTasks()");
          #endif
          removePumphandler(i, handler);
          i--;

          printFreeRam();
        }
      } else {
        #if(DEBUG_MESSAGE>0)
        DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTSTR("ERROR:PUMP IS NOT RESPONDING(");
        DEBUG_PRINT(MAX_RETRIES);
        DEBUG_PRINTSTR(" RETRIES), ");
        DEBUG_PRINTSTR("DELETE PUMPHANDLER AND SET PUMP NODE "); DEBUG_PRINT(handler->getID());
        DEBUG_PRINTLNSTR(" OFFLINE, THAT PUMP NODE MUST REGISTRATE AGAIN.");
        #endif
        myNodeList.setNodeOffline(handler->getID());
        removePumphandler(i, handler);
        i--;

        printFreeRam();
      }
    }
  }//for

}//if (PumpList.size() > 0)
#if (TEST_PUMP > 0)
//only informatve, can be deleted later
if ((nTestWatering % DEBUG_CYCLE) == 0) {
  #if(DEBUG_MESSAGE>0)
  DEBUG_PRINTSTR("[TIME] : ");
  displayTimeFromUNIX(getCurrentTime(), 1);
  DEBUG_PRINTLNSTR(" ");
  //    DEBUG_PRINTSTR("\r\n\t[CONTROLLER]"); DEBUG_PRINTSTR("nTestWatering="); DEBUG_PRINTLN(nTestWatering);     // It is sufficient and clearer to just print the time.
  DEBUG_PRINTSTR_D("\t", DEBUG_FREE_MEMORY);
  #endif
  printFreeRam();
}
#endif



#if(DEBUG > 0 && DEBUG_TIMING_LOOP==1)
duration_tmp=micros()-duration_loop;
duration_max=((duration_max>duration_tmp) ? duration_max : duration_tmp);

#endif
DEBUG_FLUSH;
}//loop()


inline void removePumphandler(int index, PumpNode_Handler* handler)
{
  PumpList.remove(index);
  myNodeList.setPumpInactive(handler->getID());
  delete handler;
  DEBUG_PRINTLNSTR("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

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
Precondition: doWateringTasks() triggers a chain of communication events

*/
uint8_t doWateringTasks(uint16_t PumpNode_ID, uint16_t pumpTime_Motor1, uint16_t pumpTime_Motor2, PumpNode_Handler *handler_)
{
  uint16_t pumpTime = pumpTime_Motor1;
  DEBUG_PRINTSTR("[TIME] : ");
  displayTimeFromUNIX(getCurrentTime(), 1);
  DEBUG_PRINTLNSTR(" ");

  if (handler_ > 0) { //a pumphandler already exists

    if ((handler_->getID() == PumpNode_ID) && (handler_->getPumpTime() == pumpTime)) {

      handler_->reset();
      handler_->processPumpstate(pumpTime);

      DEBUG_PRINTSTR("[doWateringTasks()]Retry pump request to Node-ID: ");
      DEBUG_PRINT(PumpNode_ID);
      DEBUG_PRINTSTR(" with duration of ");
      DEBUG_PRINT(pumpTime);
      DEBUG_PRINTLNSTR("ms");
      //the first communication with the pumpNode must be initiate here
      handlePumpCommunications(handler_);

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

        DEBUG_PRINTSTR("[CONTROLLER]");
        DEBUG_PRINTSTR("[doWateringTasks()]Sending pump request to Node-ID: ");
        DEBUG_PRINT(PumpNode_ID);
        DEBUG_PRINTSTR(" with duration of ");
        DEBUG_PRINT(handler->getPumpTime());
        DEBUG_PRINTLNSTR(" ms");
        //the first communication with the pumpNode must be initiate here
        handlePumpCommunications(handler);

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


void handlePumpCommunications(PumpNode_Handler *handler)
{
  myResponse.Time = getCurrentTime();
  setDATA_NO_RegistrationPacket(&myResponse);
  setDATA_PumpPacket(&myResponse);

  myResponse.ID=handler->getID();
  myResponse.pumpTime=handler->getResponseData();
  setDATA_Pumpstate(&myResponse,handler->getPacketState());
  myResponse.state &= ~(1 << ID_INEXISTENT);
  write_cnt=RADIO_RESEND_NUMB;
  delay(WAIT_SEND_INTERVAL);
  DEBUG_PRINTSTR("\t[CONTROLLER][HandlePumpCommunication()] Sending Pump Data to pump Node:[");
  DEBUG_PRINT(write_cnt);
  DEBUG_PRINTLNSTR(" times]");
  DEBUG_PRINTSTR("\t ID:");DEBUG_PRINTLN(myResponse.ID);
  DEBUG_PRINTSTR("\t PumpTime:");DEBUG_PRINTLN(myResponse.pumpTime);
  DEBUG_PRINTSTR("\t PumpState snapshot:");DEBUG_PRINTLN(getData_PumpState(&myResponse));
  DEBUG_PRINTSTR("\t PacketInfo:");DEBUG_PRINTDIG(myResponse.packetInfo, BIN);
  DEBUG_PRINTSTR("\n\t TIME:");DEBUG_PRINTLN(myResponse.Time);

  #if (DEBUG_TIMING_LOOP>0)
  duration_sending=micros();
  #endif
  radio.stopListening();
  while(write_cnt > 0) //handleDataMessage and handleMotorMessage could manipulate write_cnt
  {
    radio.write(&myResponse, sizeof(myResponse));
    write_cnt--;
  }
  radio.startListening();
  #if (DEBUG_TIMING_LOOP>0)
  DEBUG_PRINTSTR("[TIMING][to PumpNode ID: ");
  DEBUG_PRINT(myResponse.ID);
  DEBUG_PRINTSTR(" ][Duration - before stopListening - after startList.]: ");
  DEBUG_PRINT(micros() - duration_sending);
  DEBUG_PRINTLNSTR(" microseconds.");
  #endif
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
  currentData += String(myData.Time);
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
  currentData += ",";
  currentData += String(myData.pumpTime);
  currentData += ",";
  currentData += String(myData.packetInfo);

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

  myResponse.Time = getCurrentTime();
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
    // this is the session ID (we abused the temperature attribute here.)
    myResponse.interval = 100 * myData.temperature + 20;
    // this is the persistent ID.. Todo : it has to be compared to the node-list, to ensure no ID is used twice
    myResponse.ID = myResponse.interval * myResponse.Time / 100;
    //newNode=true;
  }

  // store the node in the list if it doesn't exist yet.
  //  DEBUG_PRINTSTR("ID:");
  //  DEBUG_PRINTLN(myResponse.ID);
  //  DEBUG_PRINTSTR("state: ");
  //  DEBUG_PRINTLN(myData.state);

  currentNode.ID = myResponse.ID;
  currentNode.sensorID1 = 0;
  currentNode.sensorID2 = 1;

  if ((myData.state & (1 << NODE_TYPE)) == 0)
  {
    setDATA_SensorPacket(&myResponse);
    DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]SensorNode");
    currentNode.state &= ~(1 << NODELIST_NODETYPE);  // SensorNode
    myNodeList.mnLastAddedSensorNode = currentNode.ID;
    DEBUG_PRINTSTR("\t\tNew last SensorNode - ID: ");
    DEBUG_PRINTLN(myNodeList.mnLastAddedSensorNode);
  }
  else
  {
    DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]PumpNode");
    DEBUG_PRINTSTR("\t How many registration attempts from that node: ");
    DEBUG_PRINTLN(myData.VCC);
    setDATA_PumpPacket(&myResponse);
    currentNode.state |= (1 << NODELIST_NODETYPE);  // MotorNode
    //Bernhard@(2017.April):Gibts hier eine Prüfung ob ein SensorNode auch da ist?
    currentNode.sensorID1 = myNodeList.mnLastAddedSensorNode;
    currentNode.sensorID2 = myNodeList.mnLastAddedSensorNode;
    DEBUG_PRINTSTR("\t\tUsing current last SensorNode - ID: ");
    DEBUG_PRINTLN(myNodeList.mnLastAddedSensorNode);
  }
  DEBUG_PRINTSTR("[CONTROLLER]"); DEBUG_PRINTLNSTR("[handleRegistration()]Storing node..");

  if (newNode)
  {

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
      DEBUG_PRINTLN(myResponse.Time);
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
      DEBUG_PRINTLN(myResponse.Time);
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
    DEBUG_PRINTSTR("\t*ERROR*  Node does not exist in the node list - there seems to be a topology problem.\r\n\tID: ");
    DEBUG_PRINTLN(myData.ID);

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

    DEBUG_PRINTLN(myData.Time);
    //      DEBUG_PRINTSTR("Time: ");
    //      DEBUG_PRINTLN(myResponse.ControllerTime);
  }

  //  myResponse.state &= ~((1 << FETCH_EEPROM_DATA1) | (1 << FETCH_EEPROM_DATA2));   // We do not want to have EEPROM data now
  myResponse.state |= (1 << FETCH_EEPROM_DATA1);   // We do want to have EEPROM data now
  myResponse.state &= ~(1 << FETCH_EEPROM_DATA2);


  myResponse.ID = myData.ID;

  // copying the data to the nodelist
  myNodeList.myNodes[nodeIndex].nodeData = myData;


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
  } else if (myNodeList.isActive(nodeIndex))
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
        /*LÖSCHEN*/
        DEBUG_PRINTSTR("[CONTROLLER]");
        DEBUG_PRINTSTR("[handleMotorMessage()]NOTIFY:MYDATA->PUMPSTATE :");
        DEBUG_PRINTLN(getData_PumpState(&myData));
        DEBUG_PRINTSTR("\t HANDLER->STATE:");
        DEBUG_PRINTLN(handler->getState());
        /*LÖSCHEN*/
        if(getData_PumpState(&myData) ==  handler->getState())
        {

          handler->processPumpstate(myData.pumpTime);

          DEBUG_PRINTSTR("[CONTROLLER]");
          DEBUG_PRINTLNSTR("[handleMotorMessage()]Iterate pump list and search for Pumphandler");

          myResponse.ID = myData.ID;
          if(handler->getResponseAvailability())
          {  //Marko@: dieser Codeteil könnte gelöscht werden, da es nie ausgeführt wird!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            DEBUG_PRINTSTR("[CONTROLLER]");
            DEBUG_PRINTLNSTR("[handleMotorMessage()]SENDING OVER HANDLEMOTORMESSAGE()!!!!");

            setDATA_Pumpstate(&myResponse,handler->getPacketState());
            myResponse.pumpTime = handler->getResponseData();
            write_cnt=RADIO_RESEND_NUMB;
            bResponseNeeded=true;
            DEBUG_PRINTLNSTR("\tPumpHandler processed, we will send a respond.");
          }else{
            myResponse.pumpTime = 0;
            bResponseNeeded=false;
            DEBUG_PRINTLNSTR("\tPumpHandler processed, but we will not send.");
          }
        }else{
          DEBUG_PRINTSTR("[CONTROLLER]");
          DEBUG_PRINTLNSTR("[handleMotorMessage()]INFO:SKIP REDUNDANT MESSAGE!!!!!!!!!!!!");

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

  if (myNodeList.mnPreviouslyScheduledNode >= nodeIndex) // The nodelist has been parsed once
  {
    myNodeList.mnCycleCount++;    // as the whole list has been run through once more, increase the counter.
  }

  DEBUG_PRINTSTR_D("\t\tNumber of Sensor Nodes: ", DEBUG_SENSOR_SCHEDULING);
  DEBUG_PRINTLN_D(myNodeList.getNumberOfNodesByType(SENSORNODE), DEBUG_SENSOR_SCHEDULING);

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
