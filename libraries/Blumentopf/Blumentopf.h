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

// sets Particle or photon:
#define HW_ARDUINO (1)
#define HW_PHOTON (2)
#define HW HW_ARDUINO   // tells whether to compile for Arduino or Photon

#define SD_AVAILABLE 0


// DS3231 TWI communication address:
#define DS3231_I2C_ADDRESS 0x68

#define randomPIN A6
#define BATTERY_SENSE_PIN A3      // Pin for Battery voltage
#define DHT11PIN 5                // Pin number for temperature/humidity sensor
#define MOISTURE_THRESHOLD (1000) // wet/dry threshold
#define LIGHT_THRESHOLD (512)     // day/nigth threshold
#define BAUD (9600)               // serial BAUD rate
#define PRE_SLEEP_DELAY (100)     // time to finish serial communication before sleep
#define IREF  (1.1)               // 1V1 voltage of the ADC
#define RTC_SYNC_THRESHOLD (10)    // How many seconds the Controller and Node clocks can drift apart before resynchronization
#define REF_VAL	(1.1)				// actual voltage of the 1.1 regulator
#define V_max (6.0)
#define R1 (109300.0)
#define R2 (497000.0)
#define V_ADC_max (R1*V_max / ((R1+R2)))
#define VOLTAGE_GAP (IREF - V_ADC_max)
#define MAX_OFFSET (VOLTAGE_GAP / (IREF - V_ADC_max))
#define VOLTS_PER_SCALE (IREF / 1024.0)
#define SCALE_OFFSET (VOLTAGE_GAP / VOLTS_PER_SCALE)
#define VOLTAGE_DIVIDER_FACTOR ((R1+R2)/R1)	// division factor by the resistors
#define REMAINING_FLAG_SPACE (5)    // number of flags left in the EEPROM settings (6 bit max, because we want to merge some addresses in future)

#define NODELIST_FILENAME "nodelist.txt"
#if (HW == HW_ARDUINO)
  #if (SD_AVAILABLE == 1)
    #define NODELISTSIZE (20)            // this defines how many nodes can be stored in the node list. So this limits the maximum number of nodes that can be controlled.
                                      // in case it is not big enough, it can be increased, but keep in mind that it will use up a lot of space, even if just few nodes are connected!
                                      // If there is time, a list can be implemented...
  #else
    #define NODELISTSIZE (4)            // Only 4 nodes! Stored in EEPROM
  #endif
#endif
#if (HW == HW_PHOTON)
  #define NODELISTSIZE (255)            // The photon has a bigger memory
#endif

// SCALE_OFFSET = ((IREF - (R1*V_max / ((R1+R2)))) / (IREF / 1024.0))

// (((R1+R2)/R1) * R1*V_max / ((R1+R2))) / ((IREF - (R1*V_max / ((R1+R2)))) / (IREF / 1024.0))
// (R1*R1*V_max) / ((IREF - (R1*V_max / ((R1+R2)))) / (IREF / 1024.0))
// (R1*R1*V_max) / ((IREF - (R1*V_max / (R1+R2))) / (IREF / 1024.0))
// (R1*R1*V_max) / ((IREF - (R1*V_max / (R1+R2))) / 1 // (IREF / 1024.0))
// (R1*R1*V_max) / (1024.0*(IREF - (R1*V_max / (R1+R2))) / IREF)
// (R1*R1*V_max) / (1024.0*(1 - (R1*V_max / ((R1+R2)*IREF)))


/*
 * The following defines are for node status bit operations
 */
#define RTC_RUNNING_BIT (0)       // RTC_RUNNING_BIT:       0...no RTC,						1...RTC okay
#define MSG_TYPE_BIT (1)          // MSG_TYPE_BIT:          0...init,						1...data
#define NEW_NODE_BIT (2)          // NEW_NODE_BIT:          0...known node,					1...new node
#define EEPROM_DATA_AVAILABLE (3) // EEPROM_DATA_AVAILABLE: 0...no data,					1...data available
#define EEPROM_DATA_PACKED (4)    // EEPROM_DATA_PACKED:    0...live data,					1...EEPROM data
#define EEPROM_DATA_LAST (5)	  // EEPROM_DATA_LAST:		0...this is not the last data,	1...this is the last data
#define NODE_TYPE (6)             // NODE_TYPE:             0...this is a SensorNode,       1...this is a MotorNode

/*
 * The following defines are for controller status bit operations
 */
#define REGISTER_ACK_BIT (0)
#define FETCH_EEPROM_DATA1 (1)
#define FETCH_EEPROM_DATA2 (2)
#define ID_INEXISTENT (3)       // there is no such ID
//#define EEPROM_OVERFLOW_OFFSET_BIT (1<<5*6-5)
#define EEPROM_OVERFLOW_OFFSET_BIT (1L<<5*REMAINING_FLAG_SPACE)
#define FETCH_EEPROM_REG_MASK (6)

#define FETCH_EEPROM_REG_SKIP 	(0)
#define FETCH_EEPROM_REG_SEND 	(2)
#define FETCH_EEPROM_REG_DELETE (4)
//#define FETCH_EEPROM_REG_UNUSED (6)

/*
*  EEPROM defines
*/
#define EEPROM_ID_ADDRESS (0)     // ADDRESS of init data within the EEPROM
#define INDEXBEGIN (10)
//#define INDEX_ELEMENT_SIZE (4)	// has to be the
#define INDEXELEMENTS (10)
//#define INDEXENDE  (INDEXBEGIN + INDEXELEMENTS * INDEX_ELEMENT_SIZE)
#define DATARANGE_END (900)
// EEPROM address for the node list
#define NODELIST_ADDRESS (950)

#define EEPROM_HEADER_STATUS_VALID 15	// indicates this data is valid. Unset when the EEPROM header is initialized
//#define EEPROM_DATA_STATUS_LAST 14		// indicates this is the last data element in the list.


#define INTERACTIVE_COMMAND_AVAILABLE 1
#define NO_INTERACTIVE_COMMAND_AVAILABLE 2
#define SCHEDULED_WATERING 1
#define NO_SCHEDULED_WATERING 2

/*
 * stores all measurment data in 12 bytes. 32 bytes are available in a message.
*/
struct sensorData
{
  uint16_t ID;
  float temperature;		// should be shortended to uint16_t
  float humidity;			// should be shortended to uint16_t
  uint16_t moisture;
  uint16_t brightness;
//  float voltage;
  uint16_t voltage;
  uint16_t VCC;
  uint8_t state;			// could be moved to unused data bits...
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

struct EEPROM_Header
{
	uint16_t DataStartPosition;
};


class DataStorage
{
///	DataStorage() : mbOverflow = 0; {}
public:
	DataStorage() {}
	
	uint8_t init();
//	uint8_t findIndex();
	uint8_t add(struct sensorData);
	void getNext(struct sensorData *);
	uint8_t remove(uint16_t);
	uint8_t findQueueEnd();
	void delIndex();
	void printElements();
	void stashData();
  void readNextItem(struct sensorData*);
  void freeNextItem();
  void unsetHeaders();
  bool getEmpty();
private:
  void incrementHeaderAddress(uint16_t*);
  void incrementDataAddress(uint16_t*);
  void decrementDataAddress(uint16_t*);
  void setDataAsLast(uint16_t);
  void setDataAsNotLast(uint16_t);
	uint16_t mnIndexBegin;
	uint16_t mnDataBlockBegin;
	uint16_t mnNextDataBlock;		// Block where next data is stored to. If equal mnDataBlockBegin then it's the first data element.
	uint16_t mnLastData;		// Block where previous data has been stored to.
//	uint16_t mnNextData;
	bool mbOverflow;		// signals there were more failed transmissions than memory available
	bool empty;
};

/*
 * This handler manages everything related to the IOT and watering schedule.
 * It checks whether there are realtime requests available or if watering is scheduled somewhere.
 * Also it cares about connecting the SensorNodes to MotorNodes.
 */

class CommandHandler
{
  public:
//    CommandHandler();
    uint8_t getInteractiveCommands();
    uint8_t checkSchedule();
  private:
};


/*
 * In this struct all information about the nodes is stored.
 * It lists the node IDs, whether it is a sensor or motor node, 
 * which sensorNode controls which motorNode and it can be extended by
 * the node specific settings for the control algorithm.
 */
struct nodeListElement
{
  uint16_t ID;
  uint8_t nodeType;     // NODE_TYPE:             0...this is a SensorNode,       1...this is a MotorNode
  uint16_t sensorID;    // in case it is a motor node, the corresponding SensorNode is stored here.
};

class nodeList
{
public:
  struct nodeListElement myNodes[NODELISTSIZE];
  uint16_t mnNodeCount;
  void getNodeList();
  uint8_t addNode(struct nodeListElement);
  uint8_t findNodeByID(uint16_t);         // checks if the node exists
  void clearEEPROM_Nodelist();
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

/* displays the time in the UNIX timestamp */
void displayTimeFromUNIX(time_t);

/* converts from decimal to binary decimal */
byte decToBcd(byte);

/* converts from binary decimal to decimal */
byte bcdToDec(byte);