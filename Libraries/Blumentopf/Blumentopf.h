#include <Time.h>
#include "Wire.h"
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

// DS3231 TWI communication address:
#define DS3231_I2C_ADDRESS 0x68


#define DHT11PIN 5                // Pin number for temperature/humidity sensor
#define MOISTURE_THRESHOLD (1000) // wet/dry threshold
#define LIGHT_THRESHOLD (512)     // day/nigth threshold
#define BAUD (9600)               // serial BAUD rate
#define PRE_SLEEP_DELAY (100)     // time to finish serial communication before sleep
#define IREF  (1.1)               // 1V1 voltage of the ADC
#define EEPROM_ID_ADDRESS (0)     // ADDRESS of init data within the EEPROM
#define RTC_SYNC_THRESHOLD (10)    // How many seconds the Controller and Node clocks can drift apart before resynchronization


/*
 * The following defines are for node status bit operations
 */
#define RTC_RUNNING_BIT (0)       // RTC_RUNNING_BIT:       0...no RTC, 1...RTC okay
#define MSG_TYPE_BIT (1)          // MSG_TYPE_BIT:          0...init, 1...data
#define NEW_NODE_BIT (2)          // NEW_NODE_BIT:          0...known node, 1...new node
#define EEPROM_DATA_AVAILABLE (3) // EEPROM_DATA_AVAILABLE: 0...no data, 1...data available
#define EEPROM_DATA_PACKED (4)    // EEPROM_DATA_PACKED:    0...live data, 1...EEPROM data


/*
 * The following defines are for controller status bit operations
 */
#define REGISTER_ACK_BIT (0)
#define FETCH_EEPROM_DATA1 (1)
#define FETCH_EEPROM_DATA2 (2)


/*
 * stores all measurment data in 12 bytes. 32 bytes are available in a message.
*/
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


struct EEPROM_Data
{
  uint16_t ID;
};


/*
* to be able to deal with multiple kinds of RTCs, we implement an abstraction layer.
*/

class RTCLayer
{
public:
	RTCLayer() {}

	virtual int init(uint8_t* state);
	virtual	uint8_t setTime(time_t);
	virtual time_t getTime();
	virtual int setAlarm(time_t);
	virtual int adjustRTC(int, uint8_t*, time_t);
};


class RTC_DS3231 : public RTCLayer
{
public:
	RTC_DS3231() {};	//: RTCLayer() {};

	int init(uint8_t* state);
	uint8_t setTime(time_t);
	time_t getTime();
	int setAlarm(time_t);
	int adjustRTC(int, uint8_t*, time_t);
};


class RTC_DS1302 : public RTCLayer
{
public:
	RTC_DS1302(uint8_t, uint8_t, uint8_t);

	int init(uint8_t* state);
	uint8_t setTime(time_t);
	time_t getTime();
	int setAlarm(time_t);
	int adjustRTC(int, uint8_t*, time_t);
private:
	DS1302RTC RTC;
};



/* manipulates a specified control register */
void writeControlByte(byte, bool);

/* sets the time to the RTC */
void setDS3231time(byte, byte, byte, byte, byte, byte, byte);

/* reads the time from the RTC */
void readDS3231time(byte *, byte *, byte *, byte *, byte *, byte *, byte *);

/* returns the time as UNIX timestamp */
//void getUNIXtime(time_t *);
void getUNIXtime(time_t *, tmElements_t*);

/* displays the time on the Serial interface*/
void displayTime();

/* converts from decimal to binary decimal */
byte decToBcd(byte);

/* converts from binary decimal to decimal */
byte bcdToDec(byte);