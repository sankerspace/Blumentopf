#include "Blumentopf.h"

#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
//#include "nRF24L01.h"
#include "RF24.h"

// For RTC:
#include <Time.h>
#include <TimeLib.h>

//#ifdef SD_CARD_AVAILABLE
#include <SD.h>
//#elif 

#include <LinkedList.h>

#define INTERVAL (100)

const int chipSelect = 4;

struct responseData myResponse;
struct sensorData myData;
class CommandHandler myCommandHandler;

uint16_t nDummyCount;

RF24 radio(9, 10);
//brauchen wir 3 pipes!!!!!!!!!!??
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL}; // pipe[0] ist answer-channel
RTC_DS3231 myRTC;
nodeList myNodeList;
LinkedList<PumpNode_Handler*> PumpList = LinkedList<PumpNode_Handler*>();

uint16_t nTestWatering = 0;
/*
   SETUP

   PREREQUIITE:
    EEPROM: every byte in the EEPROM must be initialized with 0xff
    (only,if you use the Blumentopf library the first time)
    (or if you want to delete old nodes and want to start again->nodeList::clearEEPROM_Nodelist())
*/

void setup(void)
{
  Serial.begin(BAUD);
  radio.begin();

  //  radio.setRetries(15,15);
  //  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(108);  // Above most Wifi Channels

  //  radio.setPayloadSize(8);
  radio.openReadingPipe(1, pipes[1]);
  radio.openWritingPipe(pipes[0]);
  radio.startListening();
  pinMode(LED_BUILTIN, OUTPUT);

  DEBUG_PRINTLN(sizeof(struct sensorData));

  myRTC.init(&myData.state);
  //  myNodeList.clearEEPROM_Nodelist();    // deletes the node list
  myNodeList.getNodeList();

  //  myResponse.ControllerTime = 1481803260;   // dummy time for testing..since I have only one RTC for testing
  //  myRTC.setTime(1485362865);

  displayTimeFromUNIX(myRTC.getTime());

  if (SD_AVAILABLE == 1)
  {
    if (initStorage() == false)
    {
      return;
    }

    DEBUG_PRINTLNSTR("card initialized.");
  }

  digitalWrite(12, HIGH);
  SPI.transfer(0xAA);

}

/*
   Initializes the storage for data logging
   The Arduino writes to an SD card for debugging purposes,
   the particle will either write to his internal memory or not log the data at all.

*/
bool initStorage()
{
  if (HW == HW_ARDUINO)             // using arduino
  {
    DEBUG_PRINTLNSTR("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect))
    {
      DEBUG_PRINTLNSTR("Card failed, or not present");
      // don't do anything more:
      return false;
    }
  }
  else if (HW == HW_PHOTON)         // using photon
  {
    // #toimplement
  }
  return true;
}

void loop(void)
{ 
  uint8_t nPipenum;
  uint8_t nICA;   // Interactive Command Answer
  uint8_t nSCA;   // Scheduled Command Answer
  bool bResponseNeeded = true;    // not always a message needs to be sent
  uint16_t nDuration;
  uint16_t nID;
  time_t currentTime;

  //struct interactiveCommand myInteractiveCommand;
  myResponse.state = 0;
  myResponse.interval = INTERVAL;

  //DEBUG_PRINTSTR("[MEMORY]:Between Heap and Stack still "); DEBUG_PRINT(String(freeRam(), DEC));
  //DEBUG_PRINTLNSTR(" bytes available.");


  if (radio.available(&nPipenum) == true)    // 19.1.2017     checks whether data is available and passes back the pipe ID
  {
    DEBUG_PRINTSTR("\nMessage available at pipe ");
    DEBUG_PRINTLN(nPipenum);
    radio.read(&myData, sizeof(struct sensorData));
    DEBUG_PRINTLN("[RECEIVED:]State:"+String(myData.state,BIN)+" from ID:"+String(myData.ID,DEC)+
    " with Interval:"+String(myData.interval));
   
    //      myResponse.ControllerTime = 1481803260;
    //    getUNIXtime(&myResponse.ControllerTime);    // gets current timestamp

    // log the data to the SD card:
    logData();
   
    if ((myData.state & (1 << MSG_TYPE_BIT)) == false) // this is a registration request. Send ack-message
    {
      DEBUG_PRINTLNSTR("Registration request");
      handleRegistration();                 // answer the registration request. There is no difference between sensor nodes and motor nodes.
    }
    else                                    // This is a data message
    {
      if ((myData.state & (1 << NODE_TYPE)) == false) // it is a sensor node
      {
        DEBUG_PRINTLNSTR("Sensor Data");
        handleDataMessage();
      }
      else                                  // it is a motor node message
      {
        DEBUG_PRINTLNSTR("Motor Message");
        DEBUG_PRINTLN("ID:" + String(myData.ID, DEC) + ", Data:" + String(myData.interval, DEC));
        handleMotorMessage();

        //        bResponseNeeded = false;
      }
    }

    if (bResponseNeeded == true)          // If it is necessary to send an answer, send it.
    {
      radio.stopListening();
      //to make state changes and to turn on receiver mode, otherwise the message is lost

      DEBUG_PRINTLN(myResponse.ControllerTime);

      // Send back response, controller real time and the next sleep interval:
      DEBUG_PRINT("Sending back response: "+String(myResponse.interval,DEC)+
        ", ID:"+String(myResponse.ID,DEC)+", STATE:"+String(myResponse.state,BIN));
      delay(2000);
      radio.write(&myResponse, sizeof(myResponse));

      DEBUG_PRINTLNSTR("............Done");
      //delay(100);   // ist das delay notwendig?
      radio.startListening();
      delay(100);   // ist das delay notwendig?
      DEBUG_PRINTLNSTR("Listening...");
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
  }
  else                                      // no message arrived. Checking the schedule
  {
    /* this is only for testing!! */
    //DEBUG_PRINTLN(nTestWatering);
    if(myNodeList.isOnline(myNodeList.myNodes[0].ID)){
        if((nTestWatering % 20000)==0)
          DEBUG_PRINTLN(nTestWatering);
        nTestWatering++;
    }
    if ((nTestWatering % 20000)==0 )
    {

      if (myNodeList.getNodeType(myNodeList.myNodes[0].ID) == 1)
      {
        DEBUG_PRINT("Node list id: ");
        DEBUG_PRINTLN(myNodeList.myNodes[0].ID);
        if (myNodeList.isActive(myNodeList.myNodes[0].ID) == 0)//check if the first node (index=0)in the list is active
          doWateringTasks(myNodeList.myNodes[0].ID, 10); //here a new order to a pump Node has to be planned
      }
    }
    /* Testing end */

    currentTime = getCurrentTime();
    nICA = myCommandHandler.getInteractiveCommands();        // checks whether the user requested watering with its app.
    // The following is the actual scheduling algorithm, but commented out as the above test section tests the pump communication for now. Afterwards the scheduling can be tested, debugged and implementation finished:
    //    nSCA = myCommandHandler.checkSchedule(myNodeList, &nID, &nDuration, currentTime);
    nSCA = NO_SCHEDULED_WATERING;                                                             // for communication tests just pretend there is nothing to schedule
    if (nSCA == SCHEDULED_WATERING)
    {
      doWateringTasks(nID, nDuration);                    //  the node is added to the "active pumps"-list and the pump is notified
    }
    /*    if (nICA == INTERACTIVE_COMMAND_AVAILABLE )                                     // some IOT watering needs to be done
        {
          //doWateringTasks(1, 10); //here a new order to a pump Node has to be planned
        }
    */
    digitalWrite(LED_BUILTIN, LOW);
    //DEBUG_PRINTLN("nothing yet..");

  }
  /*Most of the time the Controller waits for incoming messages from Pumpnodes
     It is important that the Controller is not stuck in a state, because of waiting
     for a message from a Node who is not able to send a message
     Additionally it is necessary to delete storage of handler which are not required anymore
  */

  //DEBUG_PRINT(PumpList.size());
  //DEBUG_PRINTLNSTR(" pumps available!");


  if (PumpList.size() > 0)
  {
    // DEBUG_PRINTLNSTR("Checking pump list");
    PumpNode_Handler *handler;
    for (int i = 0; i < PumpList.size(); i++) {
      handler = PumpList.get(i);
      DEBUG_PRINTLN("Processing PumpHandler for NODE-ID:" + String(handler->getID(), DEC));
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
        DEBUG_PRINTLN("Deleting PumpHandler Class because Watering finished " + String(handler->getID(), DEC));
        PumpList.remove(i); i--;
        myNodeList.setPumpInactive(handler->getID());
        delete handler;
        DEBUG_PRINTSTR("[MEMORY]:Between Heap and Stack still "); DEBUG_PRINT(String(freeRam(), DEC));
        DEBUG_PRINTLNSTR(" bytes available.");
      }
    }
    //  } //if_end
    //  DEBUG_PRINTLNSTR("Finished loop");
    delay(100);

  }
}//loop()
/*A pumpNode must perform watering, so we decide to start this task by creating a new
  PumpNode_Handler Class which controls the state changes and the whole interaction betwen
  Controller and PumpHandler
*/
uint8_t doWateringTasks(uint16_t PumpNode_ID, uint16_t pumpTime)
{
  nDummyCount = 0;
  DEBUG_PRINTSTR("[doWateringTasks()]");
  if (myNodeList.isOnline(PumpNode_ID) == 1) {
    if (myNodeList.getNodeType(PumpNode_ID) == 1)
    {
      DEBUG_PRINTSTR("Pump ID ");
      if (myNodeList.isActive(PumpNode_ID) == 0)
      {
        DEBUG_PRINT(PumpNode_ID);
        DEBUG_PRINTLNSTR(" will be activated now.");
        myNodeList.setPumpActive(PumpNode_ID);
        //  uint16_t pumptime=0;
        PumpNode_Handler *handler = new PumpNode_Handler(
          PumpNode_ID,
          pumpTime);
        PumpList.add(handler);          // todo in february: the handler should only add the pump node if it isn't in the list already.
        handler->processPumpstate(pumpTime);
        myResponse.ID = handler->getID();
        myResponse.interval = handler->getResponseData();
        myResponse.state &= ~(1 << ID_INEXISTENT);
        DEBUG_PRINTSTR("[doWateringTasks()]Sending pump request to Node-ID: ");
        DEBUG_PRINT(PumpNode_ID);
        DEBUG_PRINTSTR(" with duration of ");
        DEBUG_PRINT(handler->getPumpTime());
        DEBUG_PRINTLNSTR(" ms");
        //the first communication with the pumpNode must be initiate here
        handlePumpCommunications();
        DEBUG_PRINTSTR("[doWateringTasks()]");
        DEBUG_PRINT(PumpList.size());
        DEBUG_PRINTLNSTR(" pumps ACTIVE!!!!");
      }else
        DEBUG_PRINTLNSTR("Pump is already i use!!");
    }else
      DEBUG_PRINTLNSTR("THIS IS NOT A PUMP NODE!!");
  }else
    DEBUG_PRINTLNSTR("");

    DEBUG_PRINTSTR("[MEMORY]:Between Heap and Stack still "); DEBUG_PRINT(String(freeRam(), DEC));
    DEBUG_PRINTLNSTR(" bytes available.");
  return 0;
}


void handlePumpCommunications()
{

  radio.stopListening();//!!!!!!!!!!!!!!!!! KEEP ATTENTION OF TIME SLOT, IAM ALLOWED TO SEND here??
  radio.write(&myResponse, sizeof(myResponse));
  radio.startListening();
  delay(100);   // ist das delay notwendig?
}


/*
   This function logs the data to the storage.
   The Arduino writes the data to the SD card, the photon to its flash.
*/
void logData(void)
{
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
  currentData += String(myData.brightness);
  currentData += ",";
  currentData += String(myData.voltage);
  currentData += ",";
  currentData += String(myData.VCC);
  currentData += ",";
  currentData += String(myData.state);
  currentData += ",";
  currentData += String(myData.interval);

  if (HW == HW_ARDUINO)
  {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    //int dataFile;

    // if the file is available, write to it:
    if (dataFile)
    {
      dataFile.println(currentData);
      dataFile.close();
      // print to the serial port too:
      DEBUG_PRINTLNSTR("Writing to SD...");
      DEBUG_PRINTLN(currentData);
    }
    // if the file isn't open, pop up an error:
    else
    {
      DEBUG_PRINTLNSTR("error opening datalog.txt");
      DEBUG_PRINTLNSTR("data:");
      DEBUG_PRINTLN(currentData);
    }
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
void handleRegistration()
{
  uint8_t nRet;
  bool newNode = true;;
  struct nodeListElement currentNode;
  DEBUG_PRINTLNSTR("Registration request!");
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
  currentNode.sensorID = 0;

  if ((myData.state & (1 << NODE_TYPE)) == 0)
  {
    DEBUG_PRINTLNSTR("SensorNode");
    currentNode.state &= ~(1 << NODELIST_NODETYPE);  // SensorNode
  }
  else
  {
    DEBUG_PRINTLNSTR("PumpNode");
    currentNode.state |= (1 << NODELIST_NODETYPE);  // MotorNode
  }
  DEBUG_PRINTLNSTR("Storing node..");

  if (newNode) {

    //Bit NODELIST_NODEONLINE in currentNode.state should stay 0 in the EEPROM forever
    nRet = myNodeList.addNode(currentNode);
    if (nRet > 0)       // there was a serious problem when adding the node. Node has not been added
    {
      //introduced a new Flag, the registrating Node must be somehow informed about
      //bad registration
      myResponse.state |= (1 << ID_REGISTRATION_ERROR);
      DEBUG_PRINTLNSTR("Node registration aborted!");
    }
    else
    { //succefull registration of a new node
      DEBUG_PRINTSTR("Session-ID and interval: ");
      DEBUG_PRINT(myResponse.interval);
      DEBUG_PRINTSTR(", Persistent ID: ");
      DEBUG_PRINTLN(myResponse.ID);
      DEBUG_PRINTSTR(", Time: ");
      DEBUG_PRINTLN(myResponse.ControllerTime);
      //now the node is online
      myNodeList.setNodeOnline(myResponse.ID);
    }
  } else
  {
    if (myNodeList.findNodeByID(myData.ID) == 0xff)
    {
      myResponse.state |= (1 << ID_REGISTRATION_ERROR);
      DEBUG_PRINTLNSTR("ID is not in the NodeList registered!");
      DEBUG_PRINTLNSTR("Node registration aborted!");
    } else
    { //succefull registration of a known node
      DEBUG_PRINTSTR("Session-ID and interval: ");
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


void handleDataMessage()
{
  uint8_t nodeIndex;
  nodeIndex = myNodeList.findNodeByID(myData.ID);
  myResponse.state &= ~(1 << ID_INEXISTENT);     // per default the controller knows the node ID
  if (nodeIndex == 0xff)      // if the node does not exist
  {
    DEBUG_PRINTLNSTR("Node does not exist - there seems to be a topology problem.");
    myResponse.state |= (1 << ID_INEXISTENT);     // tell the node, the controller doesn't know him.
  }
  DEBUG_PRINTLNSTR("Data message");
  DEBUG_PRINT("ID: ");
  DEBUG_PRINT(myData.ID);
  DEBUG_PRINTSTR(", Message Type: ");
  DEBUG_PRINT(myData.state & (1 << MSG_TYPE_BIT));
  DEBUG_PRINTSTR(", RTC-Status: ");
  DEBUG_PRINTLN(myData.state & (1 << RTC_RUNNING_BIT));
  DEBUG_PRINT("Temp: ");
  DEBUG_PRINT(myData.temperature);
  DEBUG_PRINTSTR(", Humidity: ");
  DEBUG_PRINT(myData.humidity);
  DEBUG_PRINTSTR(", Moisture: ");
  DEBUG_PRINT(myData.moisture);
  DEBUG_PRINTSTR(", Brightness: ");
  DEBUG_PRINTLN(myData.brightness);
  DEBUG_PRINTSTR("Interval: ");
  DEBUG_PRINT(myData.interval);
  DEBUG_PRINTSTR(", Battery Voltage: ");
  DEBUG_PRINTDIG((float)myData.voltage / 100, 2);
  DEBUG_PRINTSTR(", VCC Voltage: ");
  DEBUG_PRINTDIG((float)myData.VCC / 100, 2);
  DEBUG_PRINTSTR(", Time: ");
  DEBUG_PRINTLN(myData.realTime);
  //      DEBUG_PRINTSTR("Time: ");
  //      DEBUG_PRINTLN(myResponse.ControllerTime);

  //  myResponse.state &= ~((1 << FETCH_EEPROM_DATA1) | (1 << FETCH_EEPROM_DATA2));   // We do not want to have EEPROM data now
  myResponse.state |= (1 << FETCH_EEPROM_DATA1);   // We do want to have EEPROM data now
  myResponse.state &= ~(1 << FETCH_EEPROM_DATA2);

  myResponse.ID = myData.ID;
  myResponse.interval = INTERVAL;
}

/*
   This function deals with motor node messages.
   They can either be state messages (for example when watering is done)
   or responses to a instruction message.
   See the state diagram in the repository for details.
*/
void handleMotorMessage()
{
  DEBUG_PRINTLNSTR("[handleMotorMessage()]A new Motormessage received.....");
  // check if the node ID actually exists in the node table..
  uint8_t nodeIndex;
  nodeIndex = myNodeList.findNodeByID(myData.ID);   // or however the ID is called..

  myResponse.state &= ~(1 << ID_INEXISTENT);     // per default the controller knows the node ID
  if (nodeIndex == 0xff)      // if the node does not exist
  {
    DEBUG_PRINTLNSTR("[handleMotorMessage()]Node does not exist - there seems to be a topology problem.");
    myResponse.state |= (1 << ID_INEXISTENT);     // tell the node, the controller doesn't know him.
  } else if (PumpList.size() > 0)
  {
    DEBUG_PRINTLNSTR("[handleMotorMessage()]Checking pump list");
    PumpNode_Handler *handler;
    for (int i = 0; i < PumpList.size(); i++) {
      handler = PumpList.get(i);
      //DEBUG_PRINTLN("Processing PumpHandler for NODE-ID:" + String(handler->getID(), DEC));
      if (handler->getID() == myData.ID)
      {
        handler->processPumpstate(myData.interval);
        myResponse.ID = myData.ID;
        myResponse.interval = handler->getResponseData();
        i = PumpList.size(); //get out of the for lopp, we are finished
      }
    }
  }

}




/*
   It is an abstraction layer to get the UNIX timestamp from RTC or web, depending on what's available
*/
time_t getCurrentTime()
{
  if (HW_RTC == 1)
  {
    return myRTC.getTime();
  }
  else    // from the web....
  {

  }
}
