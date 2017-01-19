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

#define INTERVAL (100)

const int chipSelect = 4;

struct responseData myResponse;
struct sensorData myData;



RF24 radio(9, 10);
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL}; // pipe[0] ist answer-channel
RTC_DS3231 myRTC;
nodeList myNodeList;

void setup(void)
{
  Serial.begin(9600);
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

  myNodeList.getNodeList();

  myResponse.ControllerTime = 1481803260;   // dummy time for testing..since I have only one RTC for testing


  if (initStorage() == false)
  {
    return;
  }
  
  digitalWrite(12,HIGH);
  SPI.transfer(0xAA);
  Serial.println("card initialized.");
}

/*
 * Initializes the storage for data logging
 * The Arduino writes to an SD card for debugging purposes, 
 * the particle will either write to his internal memory or not log the data at all.
 * 
 */
bool initStorage()
{
  if (HW == HW_ARDUINO)             // using arduino
  {
    Serial.print("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect))
    {
      Serial.println("Card failed, or not present");
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
//  struct interactiveCommand myInteractiveCommand;
  myResponse.interval = INTERVAL;
  
//  if (radio.available() == true)
//  if (radio.available(pipes[1]) == true)    // 15.12.2016   checks only the receive pipe..otherwise it will react to also other pipes and that can lead to problems
  if (radio.available(&nPipenum) == true)    // 19.1.2017     checks whether data is available and passes back the pipe ID
  {
    DEBUG_PRINTSTR("\nMessage available at pipe ");
    DEBUG_PRINTLN(nPipenum);
    radio.read(&myData, sizeof(struct sensorData));

//      myResponse.ControllerTime = 1481803260;    
//    getUNIXtime(&myResponse.ControllerTime);    // gets current timestamp

// log the data to the SD card:
    logData();
      
    myResponse.state  = 0;
    if ((myData.state&(1<<MSG_TYPE_BIT)) == false)  // this is a registration request. Send ack-message
    {
      handleRegistration();                 // answer the registration request. There is no difference between sensor nodes and motor nodes.
    }
    else                                    // This is a data message
    {
      if (myData.state & (1 << NODE_TYPE) == false) // it is a sensor node
      {
        handleDataMessage();
      }
      else                                  // it is a motor node message
      {
        handleMotorMessage(0);
      }
    }
    
    radio.stopListening();
    delay(100);   // ist das delay notwendig?
    
    DEBUG_PRINTLN(myResponse.ControllerTime);

// Send back response, controller real time and the next sleep interval:
    DEBUG_PRINTSTR("Sending back response...");
    radio.write(&myResponse, sizeof(myResponse));

    DEBUG_PRINTLN("Done");
    delay(100);   // ist das delay notwendig?
    radio.startListening();
    delay(100);   // ist das delay notwendig?


    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
  }
  else                                      // no message arrived. Checking the schedule
  {
    nICA = myCommandHandler.getInteractiveCommands();        // checks whether the user requested watering with its app.
    nSCA = myCommandHandler.checkSchedule();                        // checks whether there is watering scheduled now.
    if ((nICA == INTERACTIVE_COMMAND_AVAILABLE || (nSCA == SCHEDULED_WATERING)))                                     // some watering needs to be done
    {
        doWateringTasks();
    }
    digitalWrite(LED_BUILTIN, LOW);
    //DEBUG_PRINTLN("nothing yet..");  

  }
}

void doWateringTasks()
{
}



/*
 * This function logs the data to the storage.
 * The Arduino writes the data to the SD card, the photon to its flash.
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
      Serial.println("error opening datalog.txt");
      DEBUG_PRINTLN("data:");
      DEBUG_PRINTLN(currentData);
    }
  }
  else if(HW == HW_PHOTON)    // on case of a particle, the data might be logged to the flash or not at all.
  {
    // #toimplement
  }
}

/*
 * This function registers a node that joins the network.
 * It generates a new persistent ID if needed.
 * The function still has to be extended.
 */
void handleRegistration()
{
  uint8_t nRet;
  struct nodeListElement currentNode;
  DEBUG_PRINTLNSTR("Registration request!");
  myResponse.state = (1 << REGISTER_ACK_BIT);
//  if (myData.ID > 0)                      // known node
  if (myData.ID < 0xff)                      // known node
  {
    myResponse.ID = myData.ID;
  }
  else                                    // new node
  {
    myResponse.interval = 100*myData.temperature+20;    // this is the session ID (we abused the temperature attribute here.)
    myResponse.ID = myResponse.interval*myResponse.ControllerTime/100;                           // this is the persistent ID.. Todo : it has to be compared to the node-list, to ensure no ID is used twice
  }

// store the node in the list if it doesn't exist yet.
  currentNode.ID = myResponse.ID;
  currentNode.sensorID = 0;
  if (myData.state&(1<<NODE_TYPE) == 0)
  {
    currentNode.nodeType = 0;    // SensorNode
  }
  else
  {
    currentNode.nodeType = 1;    // MotorNode
  }
  nRet = myNodeList.addNode(currentNode);
  if (nRet > 0)       // there was a serious problem when adding the node. Node has not been added
  {
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
  DEBUG_PRINT(myData.state&(1<<MSG_TYPE_BIT));
  DEBUG_PRINTSTR(", RTC-Status: ");
  DEBUG_PRINTLN(myData.state&(1<<RTC_RUNNING_BIT));
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
  DEBUG_PRINTDIG((float)myData.voltage/100, 2);
  DEBUG_PRINTSTR(", VCC Voltage: ");
  DEBUG_PRINTDIG((float)myData.VCC/100, 2);
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
 * This function deals with motor node messages.
 * They can either be state messages (for example when watering is done)
 * or responses to a instruction message.
 * See the state diagram in the repository for details.
 */
void handleMotorMessage(uint16_t ID)
{
  // check if the node ID actually exists in the node table..
  uint8_t nodeIndex;
  nodeIndex = myNodeList.findNodeByID(ID);   // or however the ID is called..
  myResponse.state &= ~(1 << ID_INEXISTENT);     // per default the controller knows the node ID
  if (nodeIndex == 0xff)      // if the node does not exist
  {
    DEBUG_PRINTLNSTR("Node does not exist - there seems to be a topology problem.");
    myResponse.state |= (1 << ID_INEXISTENT);     // tell the node, the controller doesn't know him.
  }



  // handle the motor message stuff:
  // Tha actual controller logic should be inserted here.
  /*
  * #toimplement
  */
  
}



