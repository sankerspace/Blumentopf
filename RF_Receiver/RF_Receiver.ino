#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// For RTC:
#include <Time.h>
#include <TimeLib.h>
#include <DS1302RTC.h>

// Comment this line for the release version:
#define DEBUG 1

// For getting rid of serial communication in the release version:
#ifdef DEBUG
  #define DEBUG_PRINT(x)        Serial.print(x)
  #define DEBUG_PRINTSTR(x)     Serial.print(F(x))
  #define DEBUG_PRINTDIG(x, c)  Serial.print (x, c)
  #define DEBUG_PRINTLN(x)      Serial.println (x)
  #define DEBUG_PRINTLNSTR(x)   Serial.println(F(x))
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTSTR(x)
  #define DEBUG_PRINT(x, c)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLNSTR(x)
#endif 


// RTC_RUNNING_BIT: 0...no RTC, 1...RTC okay
#define RTC_RUNNING_BIT (0)
// MSG_TYPE_BIT: 0...init, 1...data
#define MSG_TYPE_BIT (1)


// Set if this is the response to a registration
#define REGISTER_ACK_BIT (0)
#define FETCH_EEPROM_DATA1 (1)
#define FETCH_EEPROM_DATA2 (2)

struct sensorData
{
  uint16_t ID;
  float temperature;
  float humidity;
  int moisture;
  int brightness;
  float voltage;
  uint8_t state;
  time_t realTime;
  uint16_t interval = 2;
};

struct responseData
{
  uint16_t ID;
  time_t ControllerTime;
  uint16_t interval;
  uint8_t state;
};
struct responseData myResponse;

RF24 radio(9, 10);
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL}; // pipe[0] ist answer-channel


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
}

void loop(void)
{
//  uint8_t i;
  struct sensorData myData;

  myResponse.interval = 10;

  
  if (radio.available() == true)
  {
    DEBUG_PRINTLNSTR("Message available");



    radio.read(&myData, sizeof(struct sensorData));

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
    DEBUG_PRINTSTR(", Voltage: ");
    DEBUG_PRINTLN(myData.voltage);




    
    myResponse.ControllerTime = 1480607987;
    myResponse.state  = 0;
    DEBUG_PRINTLN(myData.state&(1<<MSG_TYPE_BIT));
    if ((myData.state&(1<<MSG_TYPE_BIT)) == false)  // this is a registration response. Send ack-bit
    {
      DEBUG_PRINTLNSTR("Registration request!");
      myResponse.state = (1 << REGISTER_ACK_BIT);
      if (myData.ID > 0)                      // known node
      {
        myResponse.ID = myData.ID;
      }
      else                                    // new node
      { 
        myResponse.interval = 100*myData.temperature+20;    // this is the session ID (we abused the temperature attribute here.)
        myResponse.ID = myResponse.interval*myResponse.ControllerTime/100;                           // this is the persistent ID..has to be compared to the node-list, to ensure no ID is used twice
      }
DEBUG_PRINTSTR("Session-ID and interval: ");
DEBUG_PRINT(myResponse.interval);
DEBUG_PRINTSTR(", Persistent ID: ");
DEBUG_PRINTLN(myResponse.ID);
    }
    else                                    // This is a data message
    {
      DEBUG_PRINTLNSTR("Data message");
      myResponse.state &= ~((1 << FETCH_EEPROM_DATA1) | (1 << FETCH_EEPROM_DATA2));   // We do not want to have EEPROM data now
    }
    
    radio.stopListening();
    delay(100);   // ist das delay notwendig?
    
    myResponse.ControllerTime = 1480607987;
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
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    //DEBUG_PRINTLN("nothing yet..");  

  }
  //delay(1000);
}
