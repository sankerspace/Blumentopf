// RF_Test

/*
  This program tests a three different sensors on the arduino.
  -) Brightness sensor implemented by a photodiode via ADC.
  -) Moisture sensor implemented by a YL-69 and YL-38 element via ADC.
  -) Temperature and Humidity sensor implemented by a DHT11 via a serial protocol (dht11.h).
  
  During the measurements the program goes into a low power mode
  
  created 18 Nov 2016
  by Bernhard Fritz
*/
#include "Blumentopf.h"

#include <JeeLib.h>   // For sleeping
#include <dht11.h>    // Termperature and humidity sensor
#include <SPI.h>
#include "nRF24L01.h" // radio
#include "RF24.h"     // radio
#include <EEPROM.h>

// For RTC:
#include <Time.h>
#include <TimeLib.h>


struct sensorData myData;
struct responseData myResponse;

/*
// Hardware configuration for RF
//
* signal  Arduino-Pin
* ** RF **
* CE      9
* CSN     10
* SCK     13
* MOSI    11
* MISO    12
* 
* ** Sensors **
* VCC         8
* Brightness  A1
* Moisture    A0
* Temp&Hum    5
*/

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 

RF24 radio(9,10);
DataStorage myEEPROM;

// Init the DS1302
// Set pins:  CE, IO,CLK

//RTC_DS1302 myRTC(2, 3, 4);    // change as needed. The Idea is that only the constructors need to be changed as another RTC is connected. So the RTC calls below should stay as they are.
RTC_DS3231 myRTC;

// Setting the Sensor Pins
int moisturePin = A0;
int lightPin = A1;
int sensorPower = 8;

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL};



/* 
 *  Hardware configuration for DHT11
 */
 
dht11 DHT11;

    


void setup_RF();
int readDHT11();
void printValues(int DHT11_State);

ISR(WDT_vect)
{
  Sleepy::watchdogEvent();
}

// the setup function runs once when you press reset or power the board
void setup()
{

   int nRet;
   int nDelay;    // transmission duration in ms
   struct EEPROM_Data myEEPROMData;

   myData.state = 0;
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(moisturePin, INPUT);
  pinMode(lightPin, INPUT);
  pinMode(sensorPower, OUTPUT);
  pinMode(BATTERY_SENSE_PIN, INPUT);

  myResponse.interval = 20;  // at default repeat measurement every 2 seconds

//  digitalWrite(sensorPower, LOW);   // turn off the sensor power
  digitalWrite(sensorPower, HIGH);   // turn off the sensor power
  delay(100);
  delay(400);     // RTC needs 500ms startup time in total


  setup_RF();       // initialize the RF24L01+ module
  
// In debug mode, output some stuff
  #if DEBUG
    Serial.begin(BAUD);
  #endif


  DEBUG_PRINTSTR("SensorNode 0.1, ");
  DEBUG_PRINTSTR("DHT11 LIBRARY VERSION: ");
  DEBUG_PRINTLN(DHT11LIB_VERSION);
  DEBUG_PRINTLNSTR("----------------");

  myEEPROM.init();
//  myEEPROM.init();
//  findIndex();
//  findIndex();
//  delIndex();
//    myEEPROM.delIndex();
    
  if (sizeof(struct sensorData) > 32)
  {
    DEBUG_PRINTLNSTR("Fatal error - too much sensor data. Restructure message protocol\n");
  }
  
  // setup the RTC
//  setupRTC();
  myRTC.init(&myData.state);

  // read EEPROM

  EEPROM.get(EEPROM_ID_ADDRESS,myEEPROMData);   // reading a struct, so it is flexible...
  myData.ID = myEEPROMData.ID;                  // passing the ID to the RF24 message



  nRet = registerNode(&nDelay);
//  while (nRet > 0)      // if the registration was not successful, retry the until it is. Sleep inbetween
  {
    #ifdef DEBUG
      Serial.flush();
    #endif
//    Sleepy::loseSomeTime(10000);
//    nRet = registerNode(&nDelay);
  }

  myRTC.adjustRTC(nDelay, &myData.state, myResponse.ControllerTime);
 
}


int registerNode(int *pnDelay)
{
  struct EEPROM_Data myEEPROMData;
  
  int nRet;
   
  if (myData.ID > 0)                        // this is a known node
  {
    DEBUG_PRINTSTR("EEPROM-ID found: ");
    DEBUG_PRINT(myData.ID);                // Persistent ID
    DEBUG_PRINTLNSTR(". Registering at the server with this persistent ID... ");

    myData.state &= ~(1 << NEW_NODE_BIT);    // this is a known node
  }
  else                                      // this is a new node
  {
    DEBUG_PRINTSTR("No EEPROM-ID found. Registering at server with this ");
    myData.state |= (1 << NEW_NODE_BIT);    // this is a new node
    
// works well as a random pattern generator:
    myData.temperature = (analogRead(17) % 100);

    DEBUG_PRINTSTR("random session ID: ");
    DEBUG_PRINT(myData.temperature);
    DEBUG_PRINTLNSTR("...");
  }

  myData.state &= ~(1 << MSG_TYPE_BIT);     // set message type to init


  
  myData.humidity = 0;
  myData.moisture = 0;
  myData.brightness = 0;
  myData.voltage = 0;



// send the request:
  nRet = RF_action(pnDelay);
  
  if (nRet == 0)          // There was a response
  {

      if (myData.state & (1 << NEW_NODE_BIT))       // This is a new node!
      {

        DEBUG_PRINTLNSTR("Got response!");
        DEBUG_PRINTSTR("  Received Session ID: ");
        DEBUG_PRINT((int)(myResponse.interval/100));
        
        if (((int)(myResponse.interval/100)) == (int) (myData.temperature))      // is the response for us? (yes, we stored the session ID in the temperature to keep the message small and reception easy...it could be changed to a struct in a "struct payload" which can be casted in the receiver depending on the status flags)
        {
          myData.ID = myResponse.ID;
          myData.interval = (myResponse.interval % 100);
          myEEPROMData.ID = myResponse.ID;
          EEPROM.put(EEPROM_ID_ADDRESS,myEEPROMData);   // writing the data (ID) back to EEPROM...

          DEBUG_PRINTLNSTR("...ID matches");
          DEBUG_PRINTSTR("  Persistent ID: ");
          DEBUG_PRINT(myResponse.ID);
          DEBUG_PRINTSTR(", Interval: ");
          DEBUG_PRINTLN(myData.interval);
          DEBUG_PRINTSTR("Stored Persistent ID in EEPROM...");
        }
        else                                                  // not our response
        {
          DEBUG_PRINTLNSTR("ID missmatch! Ignore response...");
          return 11;
        }


      }
      else                                              // this is a known node
      {
        if (myResponse.ID == myData.ID)                     // is the response for us?
        {
          myData.interval = myResponse.interval;

          DEBUG_PRINTSTR("Got response for ID: ");
          DEBUG_PRINT(myResponse.ID);
          DEBUG_PRINTSTR("...ID matches, registration successful! Interval: ");
          DEBUG_PRINTLN(myData.interval);
        }
        else                                                  // not our response
        {
          DEBUG_PRINTSTR("Got response for ID: ");
          DEBUG_PRINT(myResponse.ID);
          DEBUG_PRINTLNSTR("...ID missmatch! Ignore response...");
          return 12;
        }
      }
      
  }
  else      // there was no response
  {
    return nRet;
  }

      

  myData.state |= (1 << MSG_TYPE_BIT);    // set message to data
  
  return 0;   // all okay
}



// the loop function runs over and over again forever
void loop()
{
  int nDHT_Status;
  
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on to indicate action
  
  digitalWrite(sensorPower, HIGH);   // turn on the sensor power and wait some time to stabilize



  delay(1000);    // DHT needs 1s to settle


// The accuracy of the ADCs should be improved as in https://www.youtube.com/watch?v=E8GqHvOK4DI&feature=youtu.be
// read the input on analog pin 0 (moisture):
  myData.moisture = analogRead(moisturePin);
    
// read the input on analog pin 1 (light):
  myData.brightness = analogRead(lightPin);

// read the temperature and humidity sensors:
//  nDHT_Status = readDHT11();
  nDHT_Status = DHT11.read(DHT11PIN);

  myData.temperature  = DHT11.temperature;
  myData.humidity = DHT11.humidity;

// getting the battery state:
  myData.voltage = getBatteryVoltage();   // sollten wir diese Methode verwenden, muss sie adaptiert werden. Sie verändert die ADC-Einstellungen


  #if DEBUG
    printValues(nDHT_Status);
  #endif

  myData.state &= ~(1 << EEPROM_DATA_PACKED); // this is live data

  
// reads the current real time value
  myData.realTime = myRTC.getTime();

  digitalWrite(sensorPower, LOW);   // when we finished measuring, turn the sensor power off again



  sendData();



  delay(PRE_SLEEP_DELAY);                       // finish the serial communication and RF communication
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off for sleeping
#ifdef DEBUG
  Serial.flush();
#endif
  Sleepy::loseSomeTime(myResponse.interval * 100);
//delay(myResponse.interval * 100);

}

void sendData()
{
  bool sending = true;
  int nRet;
  int nDelay;
  
  while (sending == true)
  {
    nRet = RF_action(&nDelay);
    
    if (nRet == 0)  // got a response
    {
      if (myResponse.ID == myData.ID)     // response is for us
      {
        if ((myData.state & (1 << EEPROM_DATA_PACKED)) == false)     // transmitted live data
        {
          DEBUG_PRINTSTR("Response is valid, next measurement in ");
          DEBUG_PRINT(myResponse.interval / 10);
          DEBUG_PRINTLNSTR(" seconds.");
  
          myRTC.adjustRTC(nDelay, &myData.state, myResponse.ControllerTime);
        }
        // in case EEPROM data has been transmitted, we skip the clock synchronisation. The synchronisation has been checked some seconds ago.
        
        if (EEPROM_data_available() == true)
        {
          switch (myResponse.state &  FETCH_EEPROM_REG_MASK)
          {
            case FETCH_EEPROM_REG_SKIP :     // controller doesn't want EEPROM data..go to sleep
              DEBUG_PRINTLNSTR("\tThe controller doesn't want EEPROM data now");
              sending = false;
              break;
            case FETCH_EEPROM_REG_SEND :     // controller wants EEPROM data now
              getEEPROMdata();
              break;
            case FETCH_EEPROM_REG_DELETE :     // controller doesn't want EEPROM data..delete them
//              deleteEEPROM();
              myEEPROM.stashData();
              sending = false;
              break;
            default:
              break;
          }
        }
        else
        {
          sending = false;
        }
  
      }
      else      // the response is for another node
      {
        if ((myData.state & (1 << EEPROM_DATA_PACKED)) == false)     // transmitted live data
        {
          store_DATA_to_EEPROM();
        }
        sending = false;
        // otherwise we do not care..start sleeping
      }
  
      // Todo : Check whether data is stored in EEPROM and send it if requested
      
    }
    else            // no response or transmission failed. Store data in EEPROM and try again later.
    {
      if ((myData.state & (1 << EEPROM_DATA_PACKED)) == false)     // transmitted live data
      {
        store_DATA_to_EEPROM();
      }
      // otherwise we do not care..start sleeping anyway
      sending = false;
    }
  }
}

/*
 * Stores data which could not be sent to the EEPROM.
 * It would be good to have an algorithm which evenly uses the EEPROM storage. (Maybe there are simple arduino solutions online...)
 * For example:
 *  Possible addresses for data information headers: 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000
 *  The headers are browsed, always just one header is valid. After the EEPROM data is requested from the controller, the next header is used.
 *  Address blocks of the actual data is stored in the header.
 */
void store_DATA_to_EEPROM()
{   // todo : find a replacement strategy and write the data to EEPROM.
//  uint8_t nIndexBegin;
//  findIndex();
//  myEEPROM.init();
  myEEPROM.add(myData);
  myEEPROM.printElements();
}

//  uint16_t nIndexBegin;
//  uint16_t nDataBlockBegin;
/*
 * This function looks for the EEPROM header. The header is located on a different address after each time the EEPROM data gets fully read by the controller.
 * The purpose is to achieve euqal usage of the EEPROM and avoid defects.
 * 
*
* It runs thorugh a predefined address space in the EEPROM and looks for the header information:
*   INDEXBEGIN      is the start of the address space
*   INDEXELEMENTS   is the number fo elements (the element size is defined by the struct. It is adapted automatically)
*   EEPROM_data_start is the first address of the data address space. It is calculated automatically.
*   DATARANGE_END   is the end of the data address space.
*   The number of data elements is calculated automatically, although some more testing is adviced.
*   
*   Once found it stores this address. Also the current data address stored in the header is retrieved.
*   If no header is found, one header address and one data address is randomly selected.
*   The data address is stored in the EEPROM header at the selected address, so next time it will be found.
*   
*
 */
/*
void findIndex()
{
  uint16_t i;

  uint16_t currentAddress;
  struct EEPROM_Header myEEPROMHeader;

  for (i = 0; i < INDEXELEMENTS; i++)
  {
    currentAddress = INDEXBEGIN + sizeof(myEEPROMHeader) * i;
    EEPROM.get(currentAddress, myEEPROMHeader);   // reading a struct, so it is flexible...
    DEBUG_PRINTSTR("Header ");
    DEBUG_PRINT(i);
    DEBUG_PRINTSTR(" - Address ");
    DEBUG_PRINT(currentAddress);
    DEBUG_PRINTSTR(": ");
    DEBUG_PRINTLN(myEEPROMHeader.DataStartPosition);
    
    if ((myEEPROMHeader.DataStartPosition & (1<<EEPROM_HEADER_STATUS_VALID)) > 0)   // this header is valid.
    {
      nIndexBegin = currentAddress;     // This is the Index Position
      nDataBlockBegin = myEEPROMHeader.DataStartPosition & ~(1<<EEPROM_HEADER_STATUS_VALID);    // This is the position of the first EEPROM data
      DEBUG_PRINTSTR("Found Header at ");
      DEBUG_PRINT(nIndexBegin);
      DEBUG_PRINTSTR(", first data block is at ");
      DEBUG_PRINT(nDataBlockBegin);
      return;
    }
  }

  // if it reaches this point, no index was found. Set random header index:
  nIndexBegin = INDEXBEGIN + sizeof(myEEPROMHeader) * (analogRead(17) % INDEXELEMENTS);

  // if it reaches this point, no index was found. Set random index:
  uint16_t EEPROM_data_start = INDEXBEGIN + sizeof(myEEPROMHeader) * INDEXELEMENTS;
  uint16_t modul = (DATARANGE_END - EEPROM_data_start) / sizeof(struct sensorData);



  nDataBlockBegin = EEPROM_data_start + sizeof(struct sensorData) * (analogRead(17) % modul);     // This is the new index Position;
  myEEPROMHeader.DataStartPosition =  nDataBlockBegin;
  myEEPROMHeader.DataStartPosition |= (1<<EEPROM_HEADER_STATUS_VALID);    // Set this as the position of the first EEPROM data


  EEPROM.put(nIndexBegin, myEEPROMHeader);   // writing the data (ID) back to EEPROM...
  DEBUG_PRINTSTR("No header found.\nHeader adress range:  ");
  DEBUG_PRINT(INDEXBEGIN);
  DEBUG_PRINTSTR(" until ");
  DEBUG_PRINTLN(EEPROM_data_start - sizeof(myEEPROMHeader));
  DEBUG_PRINTSTR("Data adress range:    ");
  DEBUG_PRINT(EEPROM_data_start);
  DEBUG_PRINTSTR(" until ");
  DEBUG_PRINTLN(DATARANGE_END);

  
  DEBUG_PRINTSTR("Randomly set header at ");
  DEBUG_PRINT(nIndexBegin);
  DEBUG_PRINTSTR(" and first data block at ");
  DEBUG_PRINTLN(nDataBlockBegin);

  struct sensorData dummy;
  EEPROM.get(nDataBlockBegin, dummy);   // reading a struct, so it is flexible...
  DEBUG_PRINTSTR("Initializing data block at address ");
  DEBUG_PRINT(nDataBlockBegin);
  DEBUG_PRINTSTR(" - last_data bit is ");
  DEBUG_PRINTLN((dummy.state & (1 << EEPROM_DATA_LAST)) > 0);
  

  dummy.state = (1<< EEPROM_DATA_LAST);    // the EEPROM_DATA_AVAILABLE bit has to be zero

  EEPROM.put(nDataBlockBegin, dummy);   // writing the data (ID) back to EEPROM...
  DEBUG_PRINTSTR("Done");

  EEPROM.get(nDataBlockBegin, dummy);   // reading a struct, so it is flexible...
  DEBUG_PRINTSTR(" - data_available bit is ");
  DEBUG_PRINTLN((dummy.state & (1 << EEPROM_DATA_AVAILABLE)) > 0);
  
}
*/


/*
 * deletes only the index
 */
/*void delIndex()
{
  struct EEPROM_Header myEEPROMHeader;
  
  EEPROM.get(nIndexBegin, myEEPROMHeader);   // reading a struct, so it is flexible...
  myEEPROMHeader.DataStartPosition &= ~(1<<EEPROM_HEADER_STATUS_VALID);
  EEPROM.put(nIndexBegin, myEEPROMHeader);   // writing the new header with removed index
}
*/

/*
 * Deletes data stored in EEPROM
 */
/*void deleteEEPROM()
{
  
}*/

void getEEPROMdata()
{
  
}

/*
 * setup_RF initializes the RF24L01+ for the application.
*/

//void setup_RF(RF24* radio)
void setup_RF()
{
 
 //
 // Setup and configure rf radio
 //

 radio.begin();

 // optionally, increase the delay between retries & # of retries
 // radio.setRetries(15,15);

 // optionally, reduce the payload size. seems to
 // improve reliability
//  radio.setPayloadSize(4);

  // transmission settings:
//  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(108);  // Above most Wifi Channels

 //
 // Open pipes to other nodes for communication
 //

 // This simple sketch opens two pipes for these two nodes to communicate
 // back and forth.
 // Open 'our' pipe for writing
 // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

 
 radio.openWritingPipe(pipes[1]);
 radio.openReadingPipe(1,pipes[0]);
 
 //
 // Start listening
 //

// radio.startListening();

 //
 // Dump the configuration of the rf unit for debugging
 //

 radio.printDetails();

}



uint16_t averageADC(uint8_t ADCpin, uint8_t cycles)
{
  uint8_t i;
  uint32_t nValue;
  for (i = 0; i < 8; i++)  // burn some readings
  {
    analogRead(ADCpin);
  }

  for (i = 0; i < cycles; i++)
  {
    nValue += analogRead(ADCpin);
  }

  return (nValue / cycles);
}


uint16_t getBatteryVoltage()
{

  analogReference(INTERNAL); //set the ADC reference to 1.1V
  burn8Readings(BATTERY_SENSE_PIN); //make 8 readings but don't use them to ensure good reading after ADC reference change
  delay(20);
//  uint16_t nResult = analogRead(BATTERY_SENSE_PIN);
  uint16_t nResult = averageADC(BATTERY_SENSE_PIN, 5);
  
  
  analogReference(DEFAULT); //set the ADC reference back to internal
  burn8Readings(BATTERY_SENSE_PIN); //make 8 readings but don't use them to ensure good reading after ADC reference change
  delay(20);


  DEBUG_PRINTSTR("\n nResult: ");
  DEBUG_PRINTLN(nResult);
  DEBUG_PRINTDIG(V_ADC_max, 2);
  DEBUG_PRINTLNSTR("");

  float fVoltage = 100.0 * nResult * VOLTAGE_DIVIDER_FACTOR * V_ADC_max / (1024);

  
  return (uint16_t) fVoltage;
}

// From https://github.com/ForceTronics/nRF24L01_Wireless_Sensor_Dev_Board/blob/master/WSNode.cpp :
//This function uses the known internal reference value of the 328p (~1.1V) to calculate the VCC value which comes from a battery
//This was leveraged from a great tutorial found at https://code.google.com/p/tinkerit/wiki/SecretVoltmeter?pageId=110412607001051797704
uint16_t getVCCVoltage()
{
  analogReference(EXTERNAL); //set the ADC reference to AVCC
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change
  int buffer = ADMUX;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  unsigned long start = millis(); //get timer value
  while ( (start + 3) > millis()); //delay for 3 milliseconds
  ADCSRA |= _BV(ADSC); // Start ADC conversion
  while (bit_is_set(ADCSRA, ADSC)); //wait until conversion is complete
  int result = ADCL; //get first half of result
  result |= ADCH << 8; //get rest of the result
  uint16_t batVolt = (IREF / result) * 102400; //Use the known iRef to calculate battery voltage [in 0.01V]
  ADMUX = buffer;
  analogReference(DEFAULT); //set the ADC reference back to internal
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change
  //  burn8Readings(A1); //make 8 readings but don't use them to ensure good reading after ADC reference change
  return batVolt;
}



//This function makes 8 ADC measurements but does nothing with them
//Since after a reference change the ADC can return bad readings. This function is used to get rid of the first 
//8 readings to ensure next reading is accurate
void burn8Readings(int pin)
{
  for(int i=0; i<8; i++) {
    analogRead(pin);
  }
}


int readDHT11()
{

  int chk = DHT11.read(DHT11PIN);

  switch (chk)
  {
    case DHTLIB_OK: 
      return 0;
    case DHTLIB_ERROR_CHECKSUM: 
      DEBUG_PRINTLNSTR("DHT11 - Checksum error"); 
      return 1;
    case DHTLIB_ERROR_TIMEOUT: 
      DEBUG_PRINTLNSTR("DHT11 - Time out error");
      return 2;
    default: 
      DEBUG_PRINTLNSTR("DHT11 - Unknown error"); 
      return 3;
  }

  return 4;
}

void printValues(int DHT11_State)
{
  char cDegreeSymbol = 176;
  DEBUG_PRINTLN("\n");

// Print the Temperature and Humidity values:
  if (DHT11_State == 0)
  {
    DEBUG_PRINTSTR("Humidity:    ");
    DEBUG_PRINTDIG((float) myData.humidity , 2);
    DEBUG_PRINTLN("%");
  
    DEBUG_PRINTSTR("Temperature: ");
    DEBUG_PRINTDIG((float) myData.temperature , 2);
    DEBUG_PRINT(cDegreeSymbol);
    DEBUG_PRINTLN("C");
    
  }
  else
  {
    switch (DHT11_State)
    {
    case DHTLIB_ERROR_CHECKSUM: 
      DEBUG_PRINTLNSTR("No Humidity and Temperature data - Checksum error"); 
    case DHTLIB_ERROR_TIMEOUT: 
      DEBUG_PRINTLNSTR("No Humidity and Temperature data - Time out error"); 
    default: 
      DEBUG_PRINTLNSTR("No Humidity and Temperature data - Unknown error"); 
    }
  }

// Print the light value
  if(myData.brightness < LIGHT_THRESHOLD)
  {
    DEBUG_PRINTSTR("It is night (");
    DEBUG_PRINT(myData.brightness);
    DEBUG_PRINT(")\n");
  }
  else
  {
    DEBUG_PRINTSTR("It is day   (");
    DEBUG_PRINT(myData.brightness);
    DEBUG_PRINT(")\n");
  }


  // Print the moisture value
  if(myData.moisture < MOISTURE_THRESHOLD)
  {
    DEBUG_PRINTSTR("The plant doesn't need watering (");
    DEBUG_PRINT(myData.moisture);
    DEBUG_PRINTLN(")");
  }
  else {
    DEBUG_PRINTSTR("It is time to water the plant   (");
    DEBUG_PRINT(myData.moisture);
    DEBUG_PRINTLN(")");
  }

  DEBUG_PRINTSTR("Voltage:     ");
  DEBUG_PRINT(myData.voltage);
  DEBUG_PRINTLN("V");
  return;
}

/*
 * Checks whether there is unsent data waiting in the EEPROM
*/
bool EEPROM_data_available()
{
  return false;
}


//void RF_action(RF24* radio)
int RF_action(int* pnDelay)
{
  
  // First, stop listening so we can talk.
    radio.stopListening();

    if (EEPROM_data_available() == true)      // there is EEPROM data
    {
      myData.state |= (1 << EEPROM_DATA_AVAILABLE);
    }
    else                                    // no EEPROM data
    {
      myData.state &= ~(1 << EEPROM_DATA_AVAILABLE);
    }
    
    
    myData.realTime = myRTC.getTime();
    
    // Send the measurement results
    DEBUG_PRINTSTR("Sending data...");
    radio.write(&myData, sizeof(struct sensorData));
    
  delay(10);   
    // Now, continue listening
    radio.startListening();


    
    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( (radio.available() == 0 ) && ! timeout )
    {
      if (millis() - started_waiting_at > 500 )
      {
        timeout = true;
      }
    }
  
    // Describe the results
    if ( timeout )
    {
      DEBUG_PRINTLNSTR("Failed, response timed out.");
      return 1;
    }
    else      // There was a response - read the new interval
    {
      // Grab the response, compare, and send to debugging spew
      unsigned long got_time;
      got_time = millis();
      radio.read( &myResponse, sizeof(myResponse) );
      
      // Show response:
      //printf("Got response %lu, round-trip delay: %lu\n\r",got_time,millis()-got_time);

      DEBUG_PRINTSTR("Got response (Round-trip delay: ");
      *pnDelay = millis()-started_waiting_at;
      DEBUG_PRINT(*pnDelay);
      DEBUG_PRINTLN(" ms)");

      
    }
    
  radio.powerDown();
    
  return 0;
}

