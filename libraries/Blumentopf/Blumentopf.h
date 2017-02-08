#include <Time.h>
#include "Wire.h"



// Comment this line for the release version:
#define DEBUG 1
#define TEST_PUMP 2 //Testcase every 30 seconds turn on first pump in the list


// watering policy:
#define POL_WATERING_DEFAULT_DURATION (10)      // per default it should give water for 10 seconds every day after 19:00
#define WATERING_START_HOUR    (23)    // Start of the watering (hour)
#define WATERING_START_MINUTE  (24)    // Start of the watering (minute)


/* Table 5 - watering policy flags 
DO NOT CHANGE:
*/
#define POL_ACTIVE (0)            // POL_ACTIVE:            0...not active,                   1...active
#define POL_USE_MOISTURE (1)      // POL_USE_MOISTURE:      0...don't user moisture data,     1...use moisture sensor data


// sets Particle or photon:
#define HW_ARDUINO (1)
#define HW_PHOTON (2)
#define HW HW_ARDUINO   // tells whether to compile for Arduino or Photon

//#define HW_RTC (0)    // there is no RTC
#define HW_RTC (1)      // use the RTC

#if (HW_RTC==1)
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
  
  #define HW_RTC_DS1302 (1) //normaly used  (DS1302 OR DS3231)
  #define HW_RTC_DS3232 (0) //alternative RTC (DS3232)

  #if (HW_RTC_DS1302==1) //
    #include <TimeLib.h>
    #include <DS1302RTC.h>
  #elif(HW_RTC_DS3232==1)
    #include "DS3232RTC.h"
    #define HW_RTC_PIN (4)
  #endif
#endif




#define SD_AVAILABLE  (0)
#if (SD_AVAILABLE ==1)
  #define SD_CHIPSELECT (4)
#endif
//Radio communication defines
#define RADIO_CHANNEL               152//108
#define WAIT_SEND_INTERVAL            2000
#define REGISTRATION_TIMEOUT_INTERVAL	WAIT_SEND_INTERVAL*3  // in Milliseconds    // wäre cool, wenn wir das noch kürzer gestalten könnten.. 6s ist lange
#define WAIT_RESPONSE_INTERVAL        WAIT_SEND_INTERVAL*2 // in Milliseconds   
          

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
#if (DEBUG == 1)
  #define DEBUG_PRINT(x)        Serial.print(x)
  #define DEBUG_PRINTSTR(x)     Serial.print(F(x))
  #define DEBUG_PRINTDIG(x, c)  Serial.print (x, c)
  #define DEBUG_PRINTLN(x)      Serial.println (x)
  #define DEBUG_PRINTLNSTR(x)   Serial.println(F(x))
  #define DEBUG_FLUSH           Serial.flush()
  #define DEBUG_SERIAL_INIT_WAIT while (!Serial) {}
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTSTR(x)
  #define DEBUG_PRINTDIG(x, c)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLNSTR(x)
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
  uint16_t ID;
  float temperature;//4byte		// should be shortended to uint16_t
  float humidity;//4byte			// should be shortended to uint16_t
  uint16_t moisture;
  uint16_t brightness;
//  float voltage;
  uint16_t voltage;
  uint16_t VCC;
  uint8_t state;			// could be moved to unused data bits...
  time_t realTime;//4byte
  uint16_t interval = 2;
};//25byte


struct responseData
{
  uint16_t ID;
  time_t ControllerTime;
  uint16_t interval;
  uint8_t state;
};//9byte


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
    CommandHandler();
    uint8_t getInteractiveCommands();
    uint8_t checkSchedule(struct nodeList, uint16_t*, uint16_t*, time_t);
  private:
    bool bWateringNow = false;
    uint8_t mnPreviousHour;
    uint8_t mnPreviousMinute;
    uint16_t mnCurrentIndex;
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
  uint8_t state;        
  // Bit 0: NODELIST_NODETYPE:   0...this is a SensorNode, 1...this is a MotorNode
  // Bit 1: NODELIST_PUMPACTIVE: 0...inactive, 1...active (is currently pumping[1] or not[0])
  // Bit 2: NODELIST_NODEONLINE: 0...OFFLINE, 1...ONLINE (the node has performed a registration)
  uint16_t sensorID;    // in case it is a motor node, the corresponding SensorNode is stored here.
  byte     watering_policy;
  time_t   nextSlot;
};


class nodeList
{
public:
  struct nodeListElement myNodes[NODELISTSIZE];
  uint16_t mnNodeCount;
  void     getNodeList();
  uint8_t  addNode(struct nodeListElement);
  uint16_t findNodeByID(uint16_t);         // checks if the node exists
  void     clearEEPROM_Nodelist();
  uint16_t getNumberOfSensorNodes();
  uint16_t getLastScheduledSensorNode();
  
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
#if (HW_RTC==1)
class RTCLayer
{
public:
	RTCLayer() {}
  ~RTCLayer() {}
	virtual int init(uint8_t* state);
	virtual	uint8_t setTime(time_t);
	virtual time_t getTime();
	virtual int setAlarm(time_t);
	virtual int adjustRTC(int, uint8_t*, time_t);
};

#if (HW_RTC_DS1302==1)
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

#elif (HW_RTC_DS3232==1)

class RTC_DS3232 : public RTCLayer
{
public:
	RTC_DS3232() {RTC=0;};	//: RTCLayer() {};
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
    }
    
    ~PumpNode_Handler(){}
   
    void     setPumpTime(uint16_t pumptime);
    uint16_t getPumpTime(void);
    int      getState(void);
    uint16_t getID(void);
    uint16_t getResponseData(void);
    void     processPumpstate(uint16_t IncomeData);
    void     reset(void);
     
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
void displayTimeFromUNIX(time_t);

/* converts from decimal to binary decimal */
byte decToBcd(byte);

/* converts from binary decimal to decimal */
byte bcdToDec(byte);

/*show free space between heap and the stack*/
/*https://cdn-learn.adafruit.com/downloads/pdf/memories-of-an-arduino.pdf*/
int freeRam(void);

