#include "Blumentopf.h"

#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
//#include "nRF24L01.h"
#include "RF24.h"

// For RTC:
#include <Time.h>
#include <TimeLib.h>
#include <SD.h>

#include <LinkedList.h>

#define INTERVAL (100)

const int chipSelect = 4;

struct responseData myResponse;
struct sensorData myData;

RF24 radio(9, 10);
//brauchen wir 3 pipes??
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL}; // pipe[0] ist answer-channel
RTC_DS3231 myRTC;
nodeList myNodeList;
LinkedList<PumpNode_Handler*> PumpList = LinkedList<PumpNode_Handler*>();

uint8_t nTestWatering = 0;

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

  myResponse.ControllerTime = 1481803260;   // dummy time for testing..since I have only one RTC for testing


  if (SD_AVAILABLE == 1)
  {
    if (initStorage() == false)
    {
      return;
    }

    DEBUG_PRINTLN("card initialized.");
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
    DEBUG_PRINTLN("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect))
    {
      DEBUG_PRINTLN("Card failed, or not present");
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
  class CommandHandler myCommandHandler;
  uint8_t nICA;   // Interactive Command Answer
  uint8_t nSCA;   // Scheduled Command Answer
  bool bResponseNeeded = true;    // not always a message needs to be sent
  //  struct interactiveCommand myInteractiveCommand;
  myResponse.state=0;
  myResponse.interval = INTERVAL;

  //  if (radio.available() == true)
  //  if (radio.available(pipes[1]) == true)    // 15.12.2016   checks only the receive pipe..otherwise it will react to also other pipes and that can lead to problems
  if (radio.available(&nPipenum) == true)    // 19.1.2017     checks whether data is available and passes back the pipe ID
  {
    DEBUG_PRINTSTR("\nMessage available at pipe ");
    DEBUG_PRINTLN(nPipenum);
    radio.read(&myData, sizeof(struct sensorData));
    DEBUG_PRINT("State: ");
    DEBUG_PRINTLN(myData.state);
    DEBUG_PRINT("ID: ");
    DEBUG_PRINTLN(myData.ID);
    //      myResponse.ControllerTime = 1481803260;
    //    getUNIXtime(&myResponse.ControllerTime);    // gets current timestamp

    // log the data to the SD card:
    logData();
    DEBUG_PRINT("State: ");
    DEBUG_PRINTLN(myData.state);

  
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
        handleMotorMessage();

        //        bResponseNeeded = false;
      }
    }

    if (bResponseNeeded == true)          // If it is necessary to send an answer, send it.
    {
      radio.stopListening();
      delay(200);   // A delay ensure that a node on receiver side, has enough time 
                    //to make state changes and to turn on receiver mode, otherwise the message is lost

      DEBUG_PRINTLN(myResponse.ControllerTime);

      // Send back response, controller real time and the next sleep interval:
      DEBUG_PRINTSTR("Sending back response...");
      radio.write(&myResponse, sizeof(myResponse));

      DEBUG_PRINTLN("Done");
      //delay(100);   // ist das delay notwendig?
      radio.startListening();
      delay(100);   // ist das delay notwendig?
      DEBUG_PRINTLN("Listening...");
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
  }
  else                                      // no message arrived. Checking the schedule
  {
    /* this is only for testing!! */
    //DEBUG_PRINTLN(nTestWatering);
    nTestWatering++;
    if (nTestWatering == 100)
    {
      if (myNodeList.myNodes[0].nodeType == 1)    // it is a motor node
      {
        DEBUG_PRINT("Node list id: ");
        DEBUG_PRINTLN(myNodeList.myNodes[0].ID);
        doWateringTasks(myNodeList.myNodes[0].ID, 10); //here a new order to a pump Node has to be planned
      }
    }
    /* Testing end */

    nICA = myCommandHandler.getInteractiveCommands();        // checks whether the user requested watering with its app.
    nSCA = myCommandHandler.checkSchedule();                        // checks whether there is watering scheduled now.
    if ((nICA == INTERACTIVE_COMMAND_AVAILABLE || (nSCA == SCHEDULED_WATERING)))                                     // some watering needs to be done
    {
      //doWateringTasks(1, 10); //here a new order to a pump Node has to be planned
    }
    digitalWrite(LED_BUILTIN, LOW);
    //DEBUG_PRINTLN("nothing yet..");

  }
  /*Most of the time the Controller waits for incoming messages from Pumpnodes
     It is important that the Controller is not stuck in a state, because of waiting
     for a message from a Node who is not able to send a message
     Additionally it is necessary to delete storage of handler which are not required anymore
  */
  
  if (PumpList.size() > 0)
  {
    DEBUG_PRINTLNSTR("Checking pump list");
    PumpNode_Handler *handler;
    for (int i = 0; i < PumpList.size(); i++) {
      handler = PumpList.get(i);
      DEBUG_PRINTLN("Processing PumpHandler for NODE-ID:" + String(handler->getID(), DEC));
      handler->processPumpstate(0);//there is no Income Data (0), only process the state machine
      if (handler->getState() == PUMPNODE_STATE_3_RESPONSE)
      {
        DEBUG_PRINTLN("Deleting PumpHandler Class because Watering finished " + String(handler->getID(), DEC));
        PumpList.remove(i); i--;
        delete handler;
      }

    }
  }
  //  DEBUG_PRINTLNSTR("Finished loop");
  delay(100);

}
/*A pumpNode must perform watering, so we decide to start this task by creating a new 
* PumpNode_Handler Class which controls the state changes and the whole interaction betwen
* Controller and PumpHandler
*/
void doWateringTasks(uint16_t PumpNode_ID, uint16_t pumpTime)
{
  //  uint16_t pumptime=0;
  PumpNode_Handler *handler = new PumpNode_Handler(
    PumpNode_ID,
    pumpTime);
  PumpList.add(handler);          // todo in february: the handler should only add the pump node if it isn't in the list already.
  handler->processPumpstate(0);
  myResponse.ID = PumpNode_ID;
  myResponse.interval = pumpTime;
  DEBUG_PRINTSTR("[doWateringTasks()]Sending pump request to Node-ID: ");
  DEBUG_PRINT(PumpNode_ID);
  DEBUG_PRINTSTR(" with duration of "); DEBUG_PRINTLN(handler->getPumpTime());
  DEBUG_PRINTLNSTR("ms");
  //the first communication with the pumpNode must be initiate here
  handlePumpCommunications();
  DEBUG_PRINT(PumpList.size());
  DEBUG_PRINTLNSTR(" pumps vailable which are pumping currently!");
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
void logData()
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
      DEBUG_PRINTLN("Writing to SD...");
      DEBUG_PRINTLN(currentData);
    }
    // if the file isn't open, pop up an error:
    else
    {
      DEBUG_PRINTLN("error opening datalog.txt");
      DEBUG_PRINTLN("data:");
      DEBUG_PRINTLN(currentData);
    }
  }
  else if (HW == HW_PHOTON)   // on case of a particle, the data might be logged to the flash or not at all.
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
  struct nodeListElement currentNode;
  DEBUG_PRINTLNSTR("Registration request!");
  myResponse.state = (1 << REGISTER_ACK_BIT);
  //  if (myData.ID > 0)                      // known node
  if ((myData.state & (1 << NEW_NODE_BIT))==false)                      // known node
  {   //a node with ID = 0x0 is valid????
    myResponse.ID = myData.ID;
  }
  else                                    // new node
  {
    myResponse.interval = 100 * myData.temperature + 20; // this is the session ID (we abused the temperature attribute here.)
    myResponse.ID = myResponse.interval * myResponse.ControllerTime / 100;                       // this is the persistent ID.. Todo : it has to be compared to the node-list, to ensure no ID is used twice
  }

  // store the node in the list if it doesn't exist yet.
  DEBUG_PRINTSTR("ID:");
  DEBUG_PRINTLN(myResponse.ID);
  DEBUG_PRINTSTR("state: ");
  DEBUG_PRINTLN(myData.state);


  currentNode.ID = myResponse.ID;
  currentNode.sensorID = 0;
  if ((myData.state & (1 << NODE_TYPE)) == 0)
  {
    DEBUG_PRINTLNSTR("SensorNode");
    currentNode.nodeType = 0;    // SensorNode
  }
  else
  {
    DEBUG_PRINTLNSTR("PumpNode");
    currentNode.nodeType = 1;    // MotorNode
  }
  DEBUG_PRINTLNSTR("Storing node..");

  nRet = myNodeList.addNode(currentNode);
  if (nRet > 0)       // there was a serious problem when adding the node. Node has not been added
  {
    //introduced a new Flag, the registrating Node must be somehow informed about
    //bad registration
    myResponse.state |= (1 << ID_REGISTRATION_ERROR);
    DEBUG_PRINTLNSTR("Node registration aborted!");
  }
  else
  {
    DEBUG_PRINTSTR("Session-ID and interval: ");
    DEBUG_PRINT(myResponse.interval);
    DEBUG_PRINTSTR(", Persistent ID: ");
    DEBUG_PRINTLN(myResponse.ID);
    DEBUG_PRINTSTR(", Time: ");
    DEBUG_PRINTLN(myResponse.ControllerTime);
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
  }

  if (PumpList.size() > 0)
  {
    DEBUG_PRINTLNSTR("[handleMotorMessage()]Checking pump list");
    PumpNode_Handler *handler;
    for (int i = 0; i < PumpList.size(); i++) {
      handler = PumpList.get(i);
      //DEBUG_PRINTLN("Processing PumpHandler for NODE-ID:" + String(handler->getID(), DEC));
      if(handler->getID()==myData.ID)
      {
        handler->processPumpstate(0);//there is no Income Data (0), only process the state machine 
        myResponse.ID = myData.ID;
        myResponse.interval = handler->getResponseData();
        i=PumpList.size();//get out of the for lopp, we are finished   
      }
    }
  }

}



