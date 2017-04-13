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


// General debug messages:
#define DEBUG_ 1						// DEBUG messages master switch				(0: no debug messages at all		1: the settings below apply)
#define DEBUG_NODE_LIST 0				// 0: disabled		1: show messages about what is going on when a node ID is stored, etc. (for debugging storage)
#define DEBUG_MESSAGE_HEADER 1			// 0: disabled		1: show the protocol details of incoming messages (for debugging the protocol)
#define DEBUG_MESSAGE_HEADER_2 1			// 0: disabled		1: show the protocol details of incoming messages (for debugging the protocol)
#define DEBUG_DATA_CONTENT 1			// 0: disabled		1: show the content of the data messages (for debugging data handling)
#define DEBUG_SENSOR_SCHEDULING 0		// 0: disabled		1: show details about the sensor node scheduling (for debugging the scheduling)
#define DEBUG_LIST_SENSOR_SCHEDULING 1	// 0: disabled		1: lists all scheduled sensor nodes (for debugging the scheduling and communication)
#define DEBUG_FREE_MEMORY 0			// 0: disabled		1: show the amount of memory still available (for debugging memory issues)
#define DEBUG_RTC 1						// 0: disabled		1: show RTC infos
#define DEBUG_RF24 1           // 0: disabled		1: show nRF24L01 infos
#define DEBUG_INFO 1           // 0: disabled		1: show infos

// For debugging the sensor node
#define DEBUG_DATA_STORAGE 0				// 0: disabled		1: for analysing the EEPROM Data class internals
#define DEBUG_SENSOR_MESSAGE_HEADER 0		// 0: disabled		1:

// For debugging the pump node

#define TEST_PUMP 1 //Testcase every 30 seconds turn on first pump in the list
//#define TEST_PUMP 2 //Testcase every every 2nd sensor round the pumpnode-condition gets checked

#define INTERVAL 600	// Duration between two protocol - timeslots [0.1s]

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

// sets Particle or photon:
#define HW_ARDUINO  1
#define HW_PHOTON   2

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
  #include <application.h>
  #ifdef F
    #undef F  //because its defined in application.h and that cannot handle const string
    #define F(x) ((const char*)(x)) //ther is no
  #else
    #define F(x) ((const char*)(x)) //ther is no
  #endif
   #include "particle-rf24.h"
#endif

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
    #define HW_RTC_PIN (8)
  #endif
#endif




#define SD_AVAILABLE  (0)
#if (SD_AVAILABLE ==1)
  #define SD_CHIPSELECT (4)
#endif
//Radio communication defines
#define RADIO_CHANNEL               152   //[0-125]
#define RADIO_AUTO_ACK              false  //ture,false
#define RADIO_DELAY                  5    //[]
#define RADIO_RETRIES                15   //[]
#define RADIO_SPEED                  RF24_1MBPS //RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
#define RADIO_CRC                    false //CRC8 true,CRC16 false
#define RADIO_PA_LEVEL               RF24_PA_LOW //RF24_PA_HIGH , RF24_PA_MAX, RF24_PA_ERROR
#define WAIT_SEND_INTERVAL           500
#define REGISTRATION_TIMEOUT_INTERVAL	WAIT_SEND_INTERVAL*5  // in Milliseconds    // wäre cool, wenn wir das noch kürzer gestalten könnten.. 6s ist lange
#define WAIT_RESPONSE_INTERVAL        WAIT_SEND_INTERVAL*2 // in Milliseconds
#if (DEBUG_RF24==1)
  #define SERIAL_DEBUG 1
#endif

// DS3231 TWI communication address:
#define DS3231_I2C_ADDRESS 0x68


#define randomPIN A6
#define BATTERY_SENSE_PIN A3      // Pin for Battery voltage
#define DHT11PIN 5                // Pin number for temperature/humidity sensor
#define MOISTURE_THRESHOLD (1000) // wet/dry threshold
#define LIGHT_THRESHOLD (512)     // day/nigth threshold
#define BAUD (57600)               // serial BAUD rate
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

// measurement policy
#define TIMESLOT_DURATION  (300)      // distance between two timeslots in [0.1s]

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
 * The following defines are for node status bit operations
 */
#define RTC_RUNNING_BIT       (0)   // RTC_RUNNING_BIT:       0...no RTC,                    1...RTC okay
#define MSG_TYPE_BIT          (1)   // MSG_TYPE_BIT:          0...init,                      1...data
#define NEW_NODE_BIT          (2)   // NEW_NODE_BIT:          0...known node,                1...new node
#define EEPROM_DATA_AVAILABLE (3)   // EEPROM_DATA_AVAILABLE: 0...no data,                   1...data available
#define EEPROM_DATA_PACKED    (4)   // EEPROM_DATA_PACKED:    0...live data,                 1...EEPROM data
#define EEPROM_DATA_LAST      (5)   // EEPROM_DATA_LAST:      0...this is not the last data, 1...this is the last data
#define NODE_TYPE             (6)   // NODE_TYPE:             0...this is a SensorNode,      1...this is a MotorNode


// For getting rid of serial communication in the release version:
#if (DEBUG_ == 1)
  #define DEBUG_PRINT(x)        		Serial.print(x)
  #define DEBUG_PRINT_D(x, d)   		if(d>0){ Serial.print(x);}
  #define DEBUG_PRINTSTR(x)     		Serial.print(F(x))
  #define DEBUG_PRINTSTR_D(x, d)     	if(d>0){ Serial.print(x);}
  #define DEBUG_PRINTDIG(x, c)  		Serial.print (x, c)
  #define DEBUG_PRINTLN(x)      		Serial.println(x)
  #define DEBUG_PRINTLN_D(x, d)    		if(d>0){ Serial.println(x);}
  #define DEBUG_PRINTLNSTR(x)   		Serial.println(F(x))
  #define DEBUG_PRINTLNSTR_D(x, d)		if(d>0){ Serial.println(F(x));}
  #define DEBUG_FLUSH           		Serial.flush()
  #define DEBUG_SERIAL_INIT_WAIT 		Serial.begin(BAUD);while (!Serial) {}
  #define DEBUG_PRINT_MEM
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINT_D(x, d)
  #define DEBUG_PRINTSTR(x)
  #define DEBUG_PRINTSTR_D(x, d)
  #define DEBUG_PRINTDIG(x, c)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLN_D(x, d)
  #define DEBUG_PRINTLNSTR(x)
  #define DEBUG_PRINTLNSTR_D(x, d)
  #define DEBUG_FLUSH
  #define DEBUG_SERIAL_INIT_WAIT
#endif
#define DEBUG_CYCLE 10000


/*
 * The following defines are for controller status bit operations
 */
#define REGISTER_ACK_BIT      (0)
#define FETCH_EEPROM_DATA1    (1)
#define FETCH_EEPROM_DATA2    (2)
#define ID_INEXISTENT         (3)       // there is no such ID
//#define EEPROM_OVERFLOW_OFFSET_BIT (1<<5*6-5)
#define EEPROM_OVERFLOW_OFFSET_BIT (1L<<5*REMAINING_FLAG_SPACE)
#define NODE_TYPE_MISMATCH    (6)
#define ID_REGISTRATION_ERROR (7)

#define FETCH_EEPROM_REG_SKIP     (0)
#define FETCH_EEPROM_REG_SEND     (2)
#define FETCH_EEPROM_REG_DELETE   (4)
//#define FETCH_EEPROM_REG_UNUSED (6)
#define FETCH_EEPROM_REG_MASK     (6)

/*The following are defines for the nodeListElement state variable*/

#define NODELIST_NODETYPE       (0)
#define NODELIST_PUMPACTIVE     (1)
#define NODELIST_NODEONLINE     (2)


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


#define INTERACTIVE_COMMAND_AVAILABLE 1
#define NO_INTERACTIVE_COMMAND_AVAILABLE 2
#define SCHEDULED_WATERING 1
#define NO_SCHEDULED_WATERING 2
/*
*  PumpNode defines
*/
#define MAX_RETRIES                     2
                                   /*who uses these states*/
#define PUMPNODE_STATE_0_PUMPREQUEST    0 //(pumpNode and Controller)
#define PUMPNODE_STATE_1_RESPONSE       1 //(pumpNode and Controller)
#define PUMPNODE_STATE_2_PUMPACTIVE     2 //(pumpNode and Controller)
#define PUMPNODE_STATE_3_RESPONSE       3 //(pumpNode and Controller)-SUCCESFULL

#define PUMPNODE_STATE_ERROR            -1 //(Controller)            -ERROR
#define PUMPNODE_STATE_3_RESP_FAILED    -2  //(Controller)
#define PUMPNODE_STATE_4_RESP_FAILED    -3 //(Controller)

#define PUMPNODE_CRITICAL_STATE_OCCUPATION 60000 // in Milliseconds

#define Error_WateringTask_1  "[CONTROLLER][doWateringTasks]ERROR:PUMP NODE IS NOT ONLINE!!"
#define Error_WateringTask_2  "[CONTROLLER][doWateringTasks]ERROR:THIS IS NOT A PUMP NODE!!"
#define Error_WateringTask_3  "[CONTROLLER][doWateringTasks]ERROR:PUMP IS ALREADY IN USE!!"
#define Error_WateringTask_4  "[CONTROLLER][doWateringTasks]ERROR:Parameter not correct!!"

/*
 * stores all measurment data in 12 bytes. 32 bytes are available in a message.
*/
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
  uint8_t  dummy8;  //1 Byte


};//25byte   //on particle 28byte
//has to fill a padding  of 3 byte

struct responseData
{
  time_t ControllerTime; //4byte
  uint16_t ID;    //2 Byte
  uint16_t interval; //2 Byte
  uint16_t dummy16; //2 Byte
  uint8_t  dummy8;  //1 Byte
  uint8_t state; //1 Byte
};//9byte      //on particle 12byte


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
  void add(struct sensorData);//Bernhard@:Rückgabewert war uint8_t
  void getNext(struct sensorData *);
  uint8_t remove(uint16_t);
  void findQueueEnd();
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
    CommandHandler();
    void getInteractiveCommands();//Bernhard@::rückgabetyp war uint8_t
    uint8_t checkSchedule(struct nodeList, uint16_t*, uint16_t*, time_t);
  private:
    uint16_t mnCurrentIndex;
    uint8_t mnPreviousHour;
    uint8_t mnPreviousMinute;
    bool bWateringNow = false;

};


/*
 * In this struct all information about the nodes is stored.
 * It lists the node IDs, whether it is a sensor or motor node,
 * which sensorNode controls which motorNode and it can be extended by
 * the node specific settings for the control algorithm.
 */
struct nodeListElement
{
  sensorData nodeData;
  time_t   nextSlot;
  uint16_t ID;
  uint16_t sensorID;    // in case it is a motor node, the corresponding SensorNode is stored here.
  uint8_t state;
  // Bit 0: NODELIST_NODETYPE:   0...this is a SensorNode, 1...this is a MotorNode
  // Bit 1: NODELIST_PUMPACTIVE: 0...inactive, 1...active (is currently pumping[1] or not[0])
  // Bit 2: NODELIST_NODEONLINE: 0...OFFLINE, 1...ONLINE (the node has performed a registration)
  byte     watering_policy;



};

// 7258155010
// 1490033280
class nodeList
{
public:
  nodeList(){mnNodeCount=0; mnCycleCount = 0; mnPreviouslyScheduledNode = NODELISTSIZE; mnPumpSlot = 0; mnPumpSlotEnable = false; mnCurrentInterval = 0;}
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
  uint16_t findNodeByID(uint16_t);         // checks if the node exists
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
};

/*
* to be able to deal with multiple kinds of RTCs, we implement an abstraction layer.
*/
#if (HW_RTC > NONE)
class RTCLayer
{
public:
  //Bernhard@: Ich frage mich cht was das für ein Compiler  in der AArduino IDE
  //da der
	//virtual RTCLayer() {}
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
        OnOff=0;
        pumpnode_status=PUMPNODE_STATE_0_PUMPREQUEST;
        pumpnode_response=0;
        pumpnode_started_waiting_at=millis();
        pumpnode_previousTime=millis();
        pumpnode_dif=0;
        pumpnode_debugCounter=DEBUG_CYCLE;
        pumphandler_ID=0;
        pumpnode_state_error_counter=0;
    }

    ~PumpNode_Handler(){}

    void     setPumpTime(uint16_t pumptime);
    uint16_t getPumpTime(void);
    int      getState(void);
    uint16_t getID(void);
    uint16_t getResponseData(void);
    void     processPumpstate(uint16_t IncomeData);
    void     reset(void);
    void     setPumpHandlerID(uint16_t ID_);
    uint16_t getPumpHandlerID(void);
    /*How many times I reached the state error*/
    uint8_t  getStateErrorCount(void);

private:
   // static uint8_t counter;
    /*state variable*/
    uint16_t pumpnode_ID;
    uint16_t OnOff;                     //duration of pumping[sec]
    int8_t pumpnode_status;                //in which status is the PUMP Node
    uint16_t pumpnode_response;         //response Data (Controller send to PumpNode)
    /*some timers for state observations*/
    uint32_t pumpnode_started_waiting_at;//
    uint32_t pumpnode_previousTime;      //needed by the software watchdog
    uint32_t pumpnode_dif;               //how many time passed is stored here,
                                         //only STATE 1 and STATE 2 (controller)
    uint16_t pumpnode_debugCounter;
    uint16_t pumphandler_ID;
    uint8_t  pumpnode_state_error_counter;
};//3*2byte,1*1byte,3*4byte
//19byte


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

/*show free space between heap and the stack*/
/*https://cdn-learn.adafruit.com/downloads/pdf/memories-of-an-arduino.pdf*/
int freeRam(void);

void printFreeRam();
void killID();
