/*
 * Project: NESE_Blumentopf
 * File:    Blumentopf.h
 * Authors: Bernhard Fritz  (0828317@student.tuwien.ac.at)
 *          Marko Stanisic  (0325230@student.tuwien.ac.at)
 *          Helmut Bergmann (0325535@student.tuwien.ac.at)
 * The copyright for the software is by the mentioned authors.
 *
 * This header file provides all kind of constants used to
 * adjust the program's specifics.
*/


#include "Time.h"

/***************************************************************************************/
/****************************** PARTICLE/ARDUINO **************************************/
/**************************************************************************************/
// sets Particle or photon:
#define HW_ARDUINO  1
#define HW_PHOTON   2
#define USEParticleCloud 0 //sitch only used in case of particle photon


#if defined(SPARK) || defined(PLATFORM_ID)
  #define HW  HW_PHOTON  // tells whether to compile for Arduino or Photon
#else
  #define HW HW_ARDUINO
#endif



#if (HW == HW_ARDUINO)
  #include <Wire.h>
  #include <Arduino.h>
  #include <SPI.h>
  #include "RF24.h"
#endif

#if (HW == HW_PHOTON)
  #if (USEParticleCloud==1)
    #define PARTICLE_CLOUD
  #endif
  #include <application.h>
  #ifdef F
    #undef F  //because its defined in application.h and that cannot handle const string
    #define F(x) ((const char*)(x)) //ther is no
  #else
    #define F(x) ((const char*)(x)) //ther is no
  #endif
   #include "particle-rf24.h"
#endif


/***************************************************************************************/
/****************************** D E B U G **********************************************/
/**************************************************************************************/
// General debug messages:
//Marko@:dont change DEBUG_ i have a warning about redundant definition
#define DEBUG_ 1//1						// DEBUG messages master switch				(0: no debug messages at all		1: the settings below apply)
#define DEBUG_NODE_LIST 0				// 0: disabled		1: show messages about what is going on when a node ID is stored, etc. (for debugging storage)
#define DEBUG_MESSAGE_HEADER 0			// 0: disabled		1: show the protocol details of incoming messages (for debugging the protocol)
#define DEBUG_MESSAGE_HEADER_2 0		// 0: disabled		1: show the protocol details of incoming messages (for debugging the protocol)
#define DEBUG_MESSAGE 1
#define DEBUG_PUMP_SCHEDULING 1			// 0: disabled		1: show details about the pump scheduler
#define DEBUG_DATA_CONTENT 1			// 0: disabled		1: show the content of the data messages (for debugging data handling)
#define DEBUG_SENSOR_SCHEDULING 1		// 0 : disabled		1: show details about the sensor node scheduling (for debugging the scheduling)
#define DEBUG_LIST_SENSOR_SCHEDULING 1	// 0: disabled		1: lists all scheduled sensor nodes (for debugging the scheduling and communication)
#define DEBUG_FREE_MEMORY 0				// 0: disabled		1: show the amount of memory still available (for debugging memory issues)
#define DEBUG_RTC 1						// 0: disabled		1: show RTC infos
#define DEBUG_INFO 1         			// 0: disabled		1: show infos
#define DEBUG_PUMP 1					//DEBUG_INFO=1 must be enabled , PUMPHANDLER infos
#define DEBUG_PUMP_ROUNDTRIPTIME 1
#define DEBUG_RF24 1				//DEBUG_INFO=1 must be enabled, 0: disabled		1: show nRF24L01 infos
#define DEBUG_TIMING_LOOP 1				//DEBUG_INFO=1 must be enabled, 1: show how much it takes tp process one loop
#define DEBUG_CYCLE 10000				// Debug information after all X ms in the loop() function
#define DEBUG_TIMESTAMP 0
#ifdef PARTICLE_CLOUD
  #define DEBUG_PARTICLE_CLOUD 1
#endif
// For debugging the sensor node
#define DEBUG_DATA_STORAGE 0			// 0: disabled		1: for analysing the EEPROM Data class internals
#define DEBUG_SENSOR_MESSAGE_HEADER 0	// 0: disabled		1:  //only in SensorNode.ino

// For debugging the pump node

//#define TEST_PUMP 1 //Testcase every 10 seconds turn on first pump in the list
//#define TEST_PUMPSCHEDULE 1
#define TEST_PUMP 1 //Testcase every every 2nd sensor round the pumpnode-condition gets checked


// For getting rid of serial communication in the release version:
#if (DEBUG_== 1)
  #define DEBUG_PRINT(x)        		Serial.print(x)
  #define DEBUG_PRINT_D(x, d)   		if(d>0){ Serial.print(x);}
  #define DEBUG_PRINTSTR(x)     		Serial.print(F(x))
  #define DEBUG_PRINTSTR_D(x, d)    if(d>0){ Serial.print(x);} //output depending of a define
  #define DEBUG_PRINTDIG(x, c)  		Serial.print (x, c)
  #define DEBUG_PRINTLNDIG(x, c)  	Serial.println (x, c)
  #define DEBUG_PRINTLN(x)      		Serial.println(x)
  #define DEBUG_PRINTLN_D(x, d)    	if(d>0){ Serial.println(x);}
  #define DEBUG_PRINTLNSTR(x)   		Serial.println(F(x))
  #define DEBUG_PRINTLNSTR_D(x, d)	if(d>0){ Serial.println(F(x));}
  #define DEBUG_PRINT_T(x ,l,c)     if((c%l)== 0){Serial.print(x);}//output depending of point in time of a runnung variable
  #define DEBUG_PRINTSTR_T(x, l, c)	if((c%l)== 0){Serial.print(F(x));}
  #define DEBUG_PRINTLN_T(x, l, c)	if((c%l)== 0){Serial.println(x);}
  #define DEBUG_PRINTLNSTR_T(x, l, c)	if((c%l)== 0){Serial.println(F(x));}


  #define DEBUG_FLUSH           		Serial.flush()
  #if (HW_ARDUINO>0)
  #define DEBUG_SERIAL_INIT_WAIT 		Serial.begin(BAUD);while (!Serial) {}
  #elif(HW_PHOTON>0)
  #define DEBUG_SERIAL_INIT_WAIT 		Serial.begin(BAUD);while (!Serial) {Particle.process();}
  #endif
  #define DEBUG_PRINT_MEM

#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINT_D(x, d)
  #define DEBUG_PRINTSTR(x)
  #define DEBUG_PRINTSTR_D(x, d)
  #define DEBUG_PRINTDIG(x, c)
  #define DEBUG_PRINTLNDIG(x, c)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLN_D(x, d)
  #define DEBUG_PRINTLNSTR(x)
  #define DEBUG_PRINTLNSTR_D(x, d)
  #define DEBUG_PRINT_T(x ,l,c)
  #define DEBUG_PRINTSTR_T(x, l, c)
  #define DEBUG_PRINTLN_T(x, l, c)
  #define DEBUG_PRINTLNSTR_T(x, l, c)
  #define DEBUG_FLUSH
  #define DEBUG_SERIAL_INIT_WAIT
#endif




/***************************************************************************************/
/****************************** R T C **********************************************/
/**************************************************************************************/
//#define HW_RTC (0)    // there is no RTC
#define NONE		0
#define RTC_1302	1
#define RTC_3231	2
#define RTC_3232	3

// SET THE RTC TYPE HERE:


#define HW_RTC RTC_3231      // kind of RTC. NONE for disable it.

//#define HW_RTC RTC_3232      // kind of RTC. NONE for disable it.

#if (HW_RTC > NONE)
//  #include <Time.h>		// if it's included here, some functions will miss it
//  #include "Wire.h"
  //defines if we want to set the HW RTC in the setup
  #define _YEAR   2017
  #define _MONTH  02
  #define _DAY    01
  #define _WEEKDAY 04   //sunday is 1
  #define _HOUR   02
  #define _MINUTE 50
  #define _SECOND 30

//  #define HW_RTC_DS1302 (1) //normaly used  (DS1302 OR DS3231)
//  #define HW_RTC_DS3232 (0) //alternative RTC (DS3232)
//  #if (HW_RTC_DS1302==1) //
  #if (HW_RTC == RTC_1302 && HW != HW_PHOTON) //
    #include <TimeLib.h>
    #include <DS1302RTC.h>
//  #elif(HW_RTC_DS3232==1)
  #elif(HW_RTC == RTC_3231)
//    #include <TimeLib.h>
  #elif(HW_RTC == RTC_3232 && HW != HW_PHOTON)
    #include "DS3232RTC.h"
  #endif
#endif

// DS3231 TWI communication address:
#define DS3231_I2C_ADDRESS 0x68


/*
* to be able to deal with multiple kinds of RTCs, we implement an abstraction layer.
*/
#if (HW_RTC > NONE)
class RTCLayer
{
public:
  virtual ~RTCLayer() {}
	virtual int init(uint8_t* state)=0;
	virtual	uint8_t setTime(time_t)=0;
	virtual time_t getTime()=0;
	virtual int setAlarm(time_t)=0;
	virtual int adjustRTC(int, uint8_t*, time_t)=0;
};

#if (HW_RTC == RTC_1302)

class RTC_DS1302 : public RTCLayer
{
public:
	RTC_DS1302(uint8_t, uint8_t, uint8_t);
    ~RTC_DS1302() {};
	int init(uint8_t* state);
	uint8_t setTime(time_t);
	time_t getTime();
	int setAlarm(time_t);
	int adjustRTC(int, uint8_t*, time_t);
private:
	DS1302RTC RTC;
};

#elif (HW_RTC == RTC_3231)

class RTC_DS3231 : public RTCLayer
{
public:
	   RTC_DS3231() {};	//: RTCLayer() {};
    ~RTC_DS3231() {};
    int init(uint8_t* state);
    uint8_t setTime(time_t);
    time_t getTime();
    int setAlarm(time_t);
    int adjustRTC(int, uint8_t*, time_t);
};


//#elif (HW_RTC_DS3232==1)
#elif (HW_RTC == RTC_3232)

class RTC_DS3232 : public RTCLayer
{
public:
	RTC_DS3232();	//: RTCLayer() {};
   ~RTC_DS3232();
	int init(uint8_t* state);
	uint8_t setTime(time_t);
	time_t getTime();
	int setAlarm(time_t);
	int adjustRTC(int, uint8_t*, time_t);
private:
	DS3232RTC* RTC;
};

#endif
#endif



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

String displayTime(time_t time_);

/* displays the time in the UNIX timestamp */
void displayTimeFromUNIX(time_t, uint8_t nDepth = 4);

/* converts from decimal to binary decimal */
byte decToBcd(byte);

/* converts from binary decimal to decimal */
byte bcdToDec(byte);


/***************************************************************************************/
/******************************  R F 24 COMMUNICATION **********************************/
/**************************************************************************************/

//Radio communication defines
#define RADIO_CHANNEL               152   //[0-125]
#define RADIO_AUTO_ACK              false  //ture,false
#define RADIO_DELAY                  5    //[]
#define RADIO_RETRIES                15   //[]
#define RADIO_SPEED                  RF24_1MBPS //RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
#define RADIO_CRC                    false //CRC8 true,CRC16 false
#define RADIO_PA_LEVEL               RF24_PA_LOW //RF24_PA_HIGH , RF24_PA_MAX, RF24_PA_ERROR


#define RADIO_RESEND_NUMB           3 //rendundant resending of a message to ensure reliability of communication


#if (DEBUG_RF24==1)
  #define SERIAL_DEBUG 1
#endif

/***************************************************************************************/
/****************************** W A T E R I N G **********************************************/
/**************************************************************************************/
// watering policy:
#define POL_WATERING_DEFAULT_DURATION 10      // per default it should give water for 10 seconds every day after 19:00
#define WATERING_START_HOUR    23    // Start of the watering (hour)
#define WATERING_START_MINUTE  24    // Start of the watering (minute)

#define WATERING_CYCLES_TO_WAIT 2
#define WATERING_THRESHOLD		950	// if no sensor is connected: 300, with photoresistor: 950

/* Table 5 - watering policy flags
DO NOT CHANGE:
*/
#define POL_ACTIVE 0            // POL_ACTIVE:            0...not active,                   1...active
#define POL_USE_MOISTURE 1      // POL_USE_MOISTURE:      0...don't user moisture data,     1...use moisture sensor data


/***************************************************************************************/
/******************************  T I M I N G ******************************************/
/**************************************************************************************/
//PUMPSCHDULE TIMINGS
/*
* if progess in a state machine fails, what is the maximal attempt to restart
* that state machine (afterwards node is offline)
*/
#define MAX_RETRIES                     3

/*
* Controller define
*If CONTROLLER sends  request to a node, it should wait at least WAIT_SEND_INTERVAL
* to receive a acknowledgmentfrom that node.
* Send and receive should be managable in that time, because there is NO
* WAIT_SEND_INTERVAL on the node side(send a response as fast as possible)
* Attention: Also depends on the number of retries, because if some
* requests didnt passed to the node, waiting period should ba factor longer
*/

#define WAIT_SEND_INTERVAL           500//300 * MAX_RETRIES  // depend also on retries
/*
* PUMP ,SENSOR  and Controller define
* How long we have to wait to receive an acknowledgment on a registration request
* REGISTRATION_TIMEOUT_INTERVAL=Protocol_Usage + Round trip delay
*/
#define REGISTRATION_TIMEOUT_INTERVAL	5000  // in Milliseconds    // wäre cool, wenn wir das noch kürzer gestalten könnten.. 6s ist lange
/*
* PUMP DEFINE
* WAIT_RESPONSE_INTERVAL=Protocol_Usage + Round trip delay
*     Its the time from sending a message to another node
*     and receiving an acknowledgment with little protocol overhead which could
*     be disregarded
*/
#define WAIT_RESPONSE_INTERVAL       1000//4000// in Milliseconds

//maximal duration for one slot of a pump  [INTERVAL/10] seconds
#define INTERVAL 600	// Duration between two protocol - timeslots [0.1s]!

// measurement policy
#define TIMESLOT_DURATION  (300)      // distance between two timeslots in [0.1s]

/*
* PUMP DEFINE
* What is the maximum time that a pump state machine is allowed to remain
* Controller PumpHandler max state time  = PUMPNODE_CRITICAL_STATE_OCCUPATION
* Pump Node max state time  = PUMPNODE_CRITICAL_STATE_OCCUPATION / 2
*/
#define PUMPNODE_CRITICAL_STATE_OCCUPATION 5000//60000 // in Milliseconds



/***************************************************************************************/
/******************************  P R O T O C O L ***************************************/
/**************************************************************************************/


//PIN definition
//RF24 PIN
#define CE_PIN 9
#define CS_PIN 10
#define PHOTON_CS_PIN D6
#define PHOTON_CE_PIN A2

//Controller
#define PHOTON_LED_BUILTIN D7

// Values for Sensor Node
#if (HW == HW_PHOTON)
  #define HW_RTC_PIN  D4			// for turning on/off the RTC at the Particle
#else
  #define SENSOR_POWER  8
  #define HW_RTC_PIN  SENSOR_POWER				// for turning on/off the RTC at the Arduino
#endif
//SensorNode
#define randomPIN         A6
#define BATTERY_SENSE_PIN A3		// Pin for Battery voltage
#define DHT11PIN 5					// Pin number for temperature/humidity sensor
#define MOISTURE_PIN      A0
#define MOISTURE_PIN_2    A2
#define LIGHT_PIN         A1

#define PUMP1_PIN         3
#define PUMP2_PIN         2
#define BUTTON_PIN        4
// SD Availability check
#define SD_AVAILABLE  (0)
#if (SD_AVAILABLE ==1)
  #define SD_CHIPSELECT (4)
#endif

#define NODELIST_FILENAME "nodelist.txt"
#if (HW == HW_ARDUINO)
  #if (SD_AVAILABLE == 1)
    #define NODELISTSIZE (20)            // this defines how many nodes can be stored in the node list. So this limits the maximum number of nodes that can be controlled.
                                      // in case it is not big enough, it can be increased, but keep in mind that it will use up a lot of space, even if just few nodes are connected!
                                      // If there is time, a list can be implemented...
  #else
    #define NODELISTSIZE (8)            // Only 8 nodes! Stored in EEPROM
  #endif
#endif
#if (HW == HW_PHOTON)
  #define NODELISTSIZE (255)            // The photon has a bigger memory
#endif





#define MOISTURE_THRESHOLD (1000)	// wet/dry threshold
#define LIGHT_THRESHOLD (512)		// day/nigth threshold
#define BAUD (57600)				// serial BAUD rate
#define PRE_SLEEP_DELAY (100)		// time to finish serial communication before sleep
#define IREF  (1.1)					// 1V1 voltage of the ADC
#define RTC_SYNC_THRESHOLD (10)		// How many seconds the Controller and Node clocks can drift apart before resynchronization
#define REF_VAL	(1.1)				// actual voltage of the 1.1 regulator
#define V_max (6.6) //Marko@: to reach V_ADC_max=1,1V : Failure between measured and read value now only ~ 1%
#define R1 (109300.0)
#define R2 (497000.0)
#define V_ADC_max (R1*V_max / ((R1+R2))) //marko@: should be 1,1 V because of analog internal reference, which is the max
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


#define SENSORNODE 0
#define PUMPNODE   1

/*
 * The following defines are for node status bit operations [MYDATA]  [NODE --> CONTROLLER]
 */
#define RTC_RUNNING_BIT       (0)   // RTC_RUNNING_BIT:       0...no RTC,                    1...RTC okay
#define MSG_TYPE_BIT          (1)   // MSG_TYPE_BIT:          0...init,                      1...data
#define NEW_NODE_BIT          (2)   // NEW_NODE_BIT:          0...known node,                1...new node
#define EEPROM_DATA_AVAILABLE (3)   // EEPROM_DATA_AVAILABLE: 0...no data,                   1...data available
#define EEPROM_DATA_PACKED    (4)   // EEPROM_DATA_PACKED:    0...live data,                 1...EEPROM data
#define EEPROM_DATA_LAST      (5)   // EEPROM_DATA_LAST:      0...this is not the last data, 1...this is the last data
#define NODE_TYPE             (6)   // NODE_TYPE:             0...this is a SensorNode,      1...this is a MotorNode


/*
 * The following defines are for controller status bit operations [MYRESPONSE]  [CONTROLLER -> NODE]
 */
#define REGISTER_ACK_BIT      (0)
#define FETCH_EEPROM_DATA1    (1)
#define FETCH_EEPROM_DATA2    (2)
#define ID_INEXISTENT         (3)       // there is no such ID
#define ID_REGISTRATION_ERROR (4)
#define PUMP1_USED            (5)
#define PUMP2_USED            (6)

#define FETCH_EEPROM_REG_SKIP     (0)
#define FETCH_EEPROM_REG_SEND     (2)
#define FETCH_EEPROM_REG_DELETE   (4)
//#define FETCH_EEPROM_REG_UNUSED (6)
#define FETCH_EEPROM_REG_MASK     (6)



/*
*  EEPROM defines
*/
#define EEPROM_ID_ADDRESS (0)     // ADDRESS of init data within the EEPROM
#define INDEXBEGIN        (10)
//#define INDEX_ELEMENT_SIZE (4)	// has to be the
#define INDEXELEMENTS     (10)
//#define INDEXENDE       (INDEXBEGIN + INDEXELEMENTS * INDEX_ELEMENT_SIZE)
#define DATARANGE_END     (900)
// EEPROM address for the node list
#define NODELIST_ADDRESS  (950)

#define EEPROM_HEADER_STATUS_VALID 15	// indicates this data is valid. Unset when the EEPROM header is initialized
//#define EEPROM_DATA_STATUS_LAST 14		// indicates this is the last data element in the list.

/*
* Bit values packetInfo in struct Data
*/

#define DATA_PUMP_STATE_BIT_0  (0) //used with pumpNode
#define DATA_PUMP_STATE_BIT_1  (1) //used with pumpNode
/*identifiy as Sensor Node or Pump Node Packet
*in case if send from Controller we instantly know if that packet
*is send to a sensor node or a pumpnode
  Sensor Node [0] or its a Pump Node [1]
*/
#define DATA_NODE_BIT          (2)
//identifiy as Packet from Controller Node
#define DATA_CONTROLLER_BIT    (3) //if packet send by Controller
//identifiy as packet for registration purpose or its a normal data packet
#define DATA_REGISTRATION_BIT  (4)


                                  // used by Controller or a Node
int setDATA_Pumpstate(struct Data *packet,int pumpState);
void setDATA_SensorPacket(struct Data *packet);
void setDATA_PumpPacket(struct Data *packet);

void setDATA_ControllerPacket(struct Data *packet);
void setDATA_NormalDatapacket(struct Data *packet);
void setDATA_RegistrationPacket(struct Data *packet);
void setDATA_NO_RegistrationPacket(struct Data *packet);

uint8_t getData_PumpState(struct Data *packet);

bool    isPumpPacket(struct Data *packet);
bool    isSensorPacket(struct Data *packet);
bool    isPumpPacket(struct Data *packet);
bool    isControllerPacket(struct Data *packet);
bool    isRegistrationPacket(struct Data *packet);
void killID();



/*sensor data and response data in one data structure*/

struct Data
{


  float temperature;//4byte		// should be shortended to uint16_t
  float humidity;//4byte			// should be shortended to uint16_t
   //used as pumptime on second pump of a pumpnode
  uint32_t Time;//4byte
  uint32_t Time_2;// 4byte  //used as pumptime on first pump of a pumpnode
  uint16_t ID; //2 Byte

  uint16_t interval; // 2byte
  uint16_t voltage; //2 Byte
  uint16_t VCC; //2 Byte
  uint16_t moisture; //2 Byte
  uint16_t moisture2;//2 Byte
  uint16_t brightness; //2 Byte
  uint8_t state;		//1Byte	// could be moved to unused data bits..
  //Helmut@: damit kannst messages bequem ausfiltern
  /*
  * früher dummy16
  *
           -  Bit 1 and Bit 2
              controller/pumpnode state snapshot before sending [0-3]
  *           sender and receiver must be in the same state
  *           redundant messages have sanpshots of older states and can be
              easily recognized as a redundant message
  *        -  Bit 3 its a Sensor Node [0] or its a Pump Node [1]
  *        -  Bit 4 its a Controller Node [1] or some other Node(Sensor/Pump) [0]
  *        -  Bit 5 Registration Packet [1] or Normal Data Packet [0]
  *
  */

  uint8_t  packetInfo;//dummy8;  //1 Byte  - used also in Pumphandler for state inidication


};//30byte
/*
 * stores all measurment data in 12 bytes. 32 bytes are available in a message.
*/
/*
struct sensorData
{


  float temperature;//4byte		// should be shortended to uint16_t
  float humidity;//4byte			// should be shortended to uint16_t

  time_t realTime;//4byte
  uint16_t ID; //2 Byte
  uint16_t interval = 2; //2 Byte
  uint16_t voltage; //2 Byte
  uint16_t VCC; //2 Byte
  uint16_t moisture; //2 Byte
  uint16_t moisture2;//2 Byte
  uint16_t brightness; //2 Byte
  uint8_t state;		//1Byte	// could be moved to unused data bits...
  uint8_t  dummy8;  //1 Byte  - used also in Pumphandler for state inidication


};//28byte


struct responseData
{
  time_t ControllerTime; //4byte
  uint16_t ID;    //2 Byte
  uint16_t interval; //2 Byte
  uint16_t dummy16; //2 Byte
  uint8_t  dummy8;  //1 Byte - used also in Pumphandler for state inidication
  uint8_t state; //1 Byte
};// 12byte
*/

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
  void add(struct Data);
  void getNext(struct Data *);
  uint8_t remove(uint16_t);
  void findQueueEnd();
  void delIndex();
  void printElements();
  void stashData();
  void readNextItem(struct Data*);
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



/*The following are defines for the nodeListElement state variable*/

#define NODELIST_NODETYPE       (0)
#define NODELIST_PUMPACTIVE     (1)
#define NODELIST_NODEONLINE     (2)
#define SENSOR_PUMP1	        (3)		// 0: sensor1		1: sensor2
#define SENSOR_PUMP2    	    (4)		// 0: sensor1		1: sensor2




/*
 * In this struct all information about the nodes is stored.
 * It lists the node IDs, whether it is a sensor or motor node,
 * which sensorNode controls which motorNode and it can be extended by
 * the node specific settings for the control algorithm.
 */
struct nodeListElement
{
  Data nodeData;
  time_t   nextSlot;
  uint16_t ID;
  uint16_t sensorID1;    // in case it is a motor node, the SensorNode corresponding to the pump1 is stored here.
  uint16_t sensorID2;    // in case it is a motor node, the SensorNode corresponding to the pump2 is stored here.
  uint8_t state;
  String name;  //Moisture 1
  String name2; //Moisture 2
  String location;//SensorNode
  // Bit 0: NODELIST_NODETYPE:   0...this is a SensorNode, 1...this is a MotorNode
  // Bit 1: NODELIST_PUMPACTIVE: 0...inactive, 1...active (is currently pumping[1] or not[0])
  // Bit 2: NODELIST_NODEONLINE: 0...OFFLINE,  1...ONLINE (the node has performed a registration)
  // Bit 3: SENSOR_PUMP1:    0...Sensor 1, 1...Sensor 2 (Moisture sensor for Pump 1. Every SensorNode has two moisture sensors..)
  // Bit 4: SENSOR_PUMP2:    0...Sensor 1, 1...Sensor 2 (Moisture sensor for Pump 2. Every SensorNode has two moisture sensors..)
  byte     watering_policy;
  /*instead in pumphandler I put it here to count no reachability of a node
  * increases when a connection to a node failed
  * reset to zero when a connection is successfull
  */
  uint8_t  pumpnode_state_error_counter;

};//4+2+221

// 7258155010
// 1490033280
class nodeList
{
public:
  nodeList(){mnNodeCount=0; mnCycleCount = 0; mnPreviouslyScheduledNode = NODELISTSIZE;
    mnPumpSlot = 0; mnPumpSlotEnable = false; mnCurrentInterval = 0;}
  struct nodeListElement myNodes[NODELISTSIZE];
  time_t   mnPumpSlot;
  uint16_t mnNodeCount;
  uint16_t mnCycleCount;
  uint16_t mnPreviouslyScheduledNode;
  uint16_t mnActivePump;
  uint16_t mnLastAddedSensorNode;
  uint16_t mnCurrentInterval;
  bool     mnPumpSlotEnable;


  void     getNodeList();
  uint8_t  addNode(struct nodeListElement);
  uint16_t findNodeByID(uint16_t);         // checks if the node ID exists,returns his index in nodelist or 0xffff
  void     clearEEPROM_Nodelist();
  uint16_t getNumberOfNodesByType(uint8_t);
//  uint16_t getNumberOfSensorNodes();
//  uint16_t getNumberOfPumpNodes();
  uint16_t getLastScheduledSensorNode();
  uint16_t getPumpEpochLength();

  /*
  *0xff .. Node doesnt exist
  *0x00.. SensorNode
  *0x01 .. PumpNode
  */
  uint8_t getNodeType(uint16_t ID); //Sensor or Pump Node

  /*
  *0xff .. Node doesnt exist
  *0x00 .. PumpNode not active , no WateringTask Send currently
  *0x01 .. PumpNode active
  */
  uint8_t isActive(uint16_t ID); // PumpNode currently active or not
  void setPumpActive(uint16_t ID);
  void setPumpInactive(uint16_t ID);
  //only call that if the node is already stored in the EEPROM
  void setNodeOnline(uint16_t ID);
  //only call that if the node is already stored in the EEPROM
  void setNodeOffline(uint16_t ID); //maybe Node was not responding a long time
  /*
  *0xff .. Node doesnt exist
  *0x00 .. Sensor or Pump Node did'nt registrate (or no lifesign)!!!!!!!!!
  *0x01 .. Sensor or Pump Node has performed a registration(and responds always)
  */
  uint8_t isOnline(uint16_t ID); // PumpNode currently active or not

  void setNodeName(uint16_t ID,String name);
  String getNodeName(uint16_t ID);
  void setNodeName2(uint16_t ID,String name2);
  String getNodeName2(uint16_t ID);
  void setNodeLocation(uint16_t ID,String location);
  String getNodeLocation(uint16_t ID);
};




/***************************************************************************************/
/******************************  SCHEDULE/COMMAND **************************************/
/**************************************************************************************/

#define INTERACTIVE_COMMAND_AVAILABLE 1
#define NO_INTERACTIVE_COMMAND_AVAILABLE 2
#define SCHEDULED_WATERING 1
#define NO_SCHEDULED_WATERING 2



/*
 * This handler manages everything related to the IOT and watering schedule.
 * It checks whether there are realtime requests available or if watering is scheduled somewhere.
 * Also it cares about connecting the SensorNodes to MotorNodes.
 */




class CommandHandler
{
  public:
    CommandHandler();
    void getInteractiveCommands();
    uint8_t checkSchedule(struct nodeList, uint16_t*, uint16_t*, time_t);
  private:
    uint16_t mnCurrentIndex;
    uint8_t mnPreviousHour;
    uint8_t mnPreviousMinute;
    bool bWateringNow = false;

};


/***************************************************************************************/
/******************************  PUMPNODE HANDLE **************************************/
/**************************************************************************************/
//Strateg if two Pumps on ONE PumpNode should work in parallel
#define PUMPNODE_PUMPS_PARALLEL           0 //[1]-yes [0]-no(work in series)

/*
*  PumpNode defines
*/

                                   /*who uses these states*/
#define PUMPNODE_STATE_0_PUMPREQUEST      0 //(pumpNode and Controller)
#define PUMPNODE_STATE_1_PUMPACTIVE       1 //(pumpNode and Controller)
#define PUMPNODE_STATE_2_PUMPOFF          2 //(pumpNode and Controller)
#define PUMPNODE_STATE_3_ACKNOWLEDGMENT   3 //(pumpNode and Controller)
#define PUMPNODE_STATE_4_FINISHED         4 //Controller-SUCCESFULL

#define PUMPNODE_STATE_ERROR             -1 //(Controller)            -ERROR
#define PUMPNODE_STATE_1_RESP_FAILED     -3  //(Controller)
#define PUMPNODE_STATE_3_RESP_FAILED     -4 //(Controller)



/*
*Class PumpNode_Handler
*   Every PumpNode is controlled by its own PumpNode_Handler
*   Processing of a pumpNode and the whole state machine ,
*   including its necessary time monitoring, is handled
*   inside that class
*/

class PumpNode_Handler
{
public:
    /*pumpTime in seconds*/
    PumpNode_Handler(uint16_t pumpNodeID)
    {
        pumpnode_ID=pumpNodeID;
        OnOff_1=0;
        OnOff_2=0;

        pumpnode_status=PUMPNODE_STATE_0_PUMPREQUEST;
        pumpnode_response_1=0;
        pumpnode_response_2=0;
        pumpnode_started_waiting_at=millis();
        pumpnode_previousTime=millis();
        pumpnode_waitforPump=0;
        pumpnode_dif=0;
        pumpnode_debugCounter=DEBUG_CYCLE;
        pumphandler_ID=0;
        //pumpnode_state_error_counter=1;
        pumpnode_reponse_available=false;
        pumpnode_status_packet=pumpnode_status;
        #if (DEBUG_PUMP>0)
        #if (DEBUG_PUMP_ROUNDTRIPTIME)
        pumpNode_RoundTripTime=0;
        #endif
        pumpnode_HandlerGenerationTime=millis();
        #endif
    }

    ~PumpNode_Handler()
    {
      #if (DEBUG_PUMP>0)
      DEBUG_PRINTSTR("\t\t[PUMPHANDLER ");
      DEBUG_PRINT(pumpnode_ID);
      DEBUG_PRINTSTR("]NOTIFY:HANDLER DESCTRUCTION AFTER ");
      DEBUG_PRINT(millis()-pumpnode_HandlerGenerationTime);
      DEBUG_PRINTLNSTR(" ms OF LIFETIME.");
      #endif
    }


    uint32_t getFirstPumpTime(void);
    uint32_t getSecondPumpTime(void);
    int      getState(void);
    int      getPacketState(void);
    uint16_t getID(void);
    uint32_t getResponseData_1(void);
    uint32_t getResponseData_2(void);
    void     processPumpstate(uint32_t IncomeData_1,uint32_t IncomeData_2);
    void     reset(void);
    void     setPumpHandlerID(uint16_t ID_);
    uint16_t getPumpHandlerID(void);
    /*How many times I reached the state error*/
    //uint8_t  getStateErrorCount(void);
    bool     getResponseAvailability(void);
    //bool     isPump1_active(void);
    //bool     isPump2_active(void);
private:
    #if (DEBUG_PUMP>0)
    uint32_t pumpnode_HandlerGenerationTime;
    #endif
    #if (DEBUG_PUMP_ROUNDTRIPTIME)
    uint32_t pumpNode_RoundTripTime;
    #endif
    uint32_t pumpnode_started_waiting_at;//
    uint32_t pumpnode_previousTime;      //needed by the software watchdog
    uint32_t pumpnode_dif;               //how many time passed is stored here,
                                         //only STATE 1 and STATE 2 (controller)
    uint32_t pumpnode_waitforPump;
   // static uint8_t counter;
    /*state variable*/
    uint16_t pumpnode_ID;
    uint32_t OnOff_1;                     //duration of pumping[sec]
    uint32_t OnOff_2;
    uint32_t pumpnode_response_1;         //response Data (Controller send to PumpNode)
    uint32_t pumpnode_response_2;         //response Data (Controller send to PumpNode)
    /*some timers for state observations*/

    uint16_t pumpnode_debugCounter;
    uint16_t pumphandler_ID;
    //uint8_t  pumpnode_state_error_counter;
    int8_t pumpnode_status;                //in which status is the PUMP Node
    /*packet which is send contains a state footprint
    * must not be equal to pumpnode_status, pumpnode_status variable changes
    * to create a control flow and pumpnode_status_packet indicates that state
    * to which it is associated
    */
    int8_t pumpnode_status_packet;
    bool pumpnode_reponse_available;
    //bool activate_pump_1;
    //bool activate_pump_2;
};//5*2byte,4*1byte,5*4byte
//34byte






/***************************************************************************************/
/******************************  E E P R O M ********************************************/
/**************************************************************************************/


/***************************************************************************************/
/******************************  LOG ***************************************************/
/**************************************************************************************/

#define Error_WateringTask_1  "[CONTROLLER][doWateringTasks]ERROR:PUMP NODE IS NOT ONLINE!!"
#define Error_WateringTask_2  "[CONTROLLER][doWateringTasks]ERROR:THIS IS NOT A PUMP NODE!!"
#define Error_WateringTask_3  "[CONTROLLER][doWateringTasks]ERROR:PUMP IS ALREADY IN USE!!"
#define Error_WateringTask_4  "[CONTROLLER][doWateringTasks]ERROR:Parameter not correct!!"

/***************************************************************************************/
/******************************  M E M O R Y ******************************************/
/**************************************************************************************/

/*show free space between heap and the stack*/
/*https://cdn-learn.adafruit.com/downloads/pdf/memories-of-an-arduino.pdf*/
int freeRam(void);

void printFreeRam();

/***************************************************************************************/
/************* H E L P E R F U N T I O N / T O O L S ***********************************/
/**************************************************************************************/

uint32_t getCombinedData(uint16_t HighByte,uint16_t LowByte);
void   setCombinedData(uint32_t Data_,uint16_t& HighByte,uint16_t& LowByte);



/***************************************************************************************/
/******************************  P A R T I C L E ******************************************/
/**************************************************************************************/

#ifdef PARTICLE_CLOUD


#define S_ID_TXT      "Sensor NodeID:"
#define PL_TXT        "PlantName:"
#define LOC_TXT       "Location:"
#define TEMP_TXT      "Temperature:"
#define MOI_TXT       "Moisture:"
#define HU_TEXT       "Humidity:"
#define BR_TXT        "Brightness:"
#define BAT_TXT       "Battery:"
#define LWAT_TXT      "Last Watering:"
#define P_TXT         "PumpID:"

#define MAX_TRACKED_SENSORS 3  //mAXIMUM IS 20
#define SENSOR_TRACKNAME_PREFIX "SensorData"
/*
struct SensorD {
  float temperature;//4byte		// should be shortended to uint16_t
  float humidity;//4byte			// should be shortended to uint16_t
  time_t Time;//4byte
  uint32_t pumpTime;// 4byte    ---> interval
  uint16_t ID; //2 Byte

  uint16_t interval; // 2byte
  uint16_t voltage; //2 Byte
  uint16_t VCC; //2 Byte
  uint16_t moisture; //2 Byte
  uint16_t moisture2;//2 Byte
  uint16_t brightness; //2 Byte
};
*/
/*
*Particle
* All interaction with pArticle Cloud is concentrated in this class
*/
struct Particle_Node
{
  String SensorTXT;
  uint16_t SensorID;
};

class HomeWatering {
  public:
    HomeWatering() {



      bool ret=true;
      for(int i=0;i<MAX_TRACKED_SENSORS;i++)
      {

          String name=(String::format("%s_%d",SENSOR_TRACKNAME_PREFIX,i));

          int len=name.length()+1;
          char buf[len];
          name.toCharArray(buf,len);
          buf[len-1]='\0';
          const char* buf_=buf;
          DEBUG_PRINTSTR_D("[PARTICLE][Variable]:",DEBUG_PARTICLE_CLOUD);
          DEBUG_PRINTLN_D(buf,DEBUG_PARTICLE_CLOUD);


          ret &= Particle.variable(buf_, &(Particle_SensorData[i].SensorTXT),STRING);
          Particle_SensorData[i].SensorID=0;
          DEBUG_PRINTLN_D(ret,DEBUG_PARTICLE_CLOUD);
      }
      if(ret==false)
      {
        DEBUG_PRINTSTR_D("[PARTICLE]",DEBUG_PARTICLE_CLOUD); DEBUG_PRINTLNSTR_D("VARIABLE NOT REGISTERED.",DEBUG_PARTICLE_CLOUD);
      }
      //Particle.function("brew", &CoffeeMaker::brew, this);
    }

    //int brew(String command) {
      // do stuff
  //    return 1;
  //  }
    int8_t isTrackedSensor(uint16_t ID);//if tracked return number of
    void setParticleVariableString(nodeList* list,uint16_t index);

    struct Particle_Node Particle_SensorData[MAX_TRACKED_SENSORS];



};

#endif //#ifdef PARTICLE_CLOUD
