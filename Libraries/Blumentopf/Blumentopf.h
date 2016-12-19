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

/*
 * The following defines are for controller status bit operations
 */
#define REGISTER_ACK_BIT (0)
#define FETCH_EEPROM_DATA1 (1)
#define FETCH_EEPROM_DATA2 (2)
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

#define EEPROM_HEADER_STATUS_VALID 15	// indicates this data is valid. Unset when the EEPROM header is initialized
//#define EEPROM_DATA_STATUS_LAST 14		// indicates this is the last data element in the list.
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
private:
	uint16_t mnIndexBegin;
	uint16_t mnDataBlockBegin;
	uint16_t mnNextDataBlock;		// Block where next data is stored to. If equal mnDataBlockBegin then it's the first data element.
	uint16_t mnLastData;		// Block where next data is stored to. If equal mnDataBlockBegin then it's the first data element.
//	uint16_t mnNextData;
	byte mbOverflow;		// Block where next data is stored to. If equal mnDataBlockBegin then it's the first data element.
	bool empty;
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