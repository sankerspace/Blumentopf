#include "Wire.h"
#include <Time.h>
#include <TimeLib.h>
#include <DS1302RTC.h>
#include <EEPROM.h>

#include <SD.h>

#include <dht11.h>
#include "Blumentopf.h"


/*
 * ---------------- Blumentopf - Library ---------------
 * 
 * This library provides all functions to the network protocol,
 * real time clock and memory management.
 * It is seperated into two parts which are not split 
 * into different files, because Arduino and Git don't match well.
 * Further the header provides all kind of constants to adjust the program.
 * 
 *    Part I
 * The first part deals with the real time clocks and time conversions.
 * It is an abstraction layer over the RTC libraries, so when 
 * changing the RTC, only the object class has to be changed 
 * in the sketch. The rest of the code can stay untouched.
 * The relevant RTC functions are implemented in the abstraction 
 * layer within this file. These functions can, of course, be 
 * based on other libraries.
 * 
 *    Part II
 * The second part manages the EEPROM accesses.
 * It's main task is to evenly distribute accesses to
 * the EEPROM to ensure even usage. It implements the 
 * whole storage logic, so it is very easy to use in the main sketch.
 * 
 */



/*
* This scetion deals with the RTC and time conversions
*/

 
/*
*  DS1302
*/

RTC_DS1302::RTC_DS1302(uint8_t CE_pin, uint8_t IO_pin, uint8_t SCLK_pin)
	: RTC(CE_pin, IO_pin, SCLK_pin) {}

int RTC_DS1302::init(uint8_t* state)
{
  if (RTC.haltRTC())
  {
    DEBUG_PRINTSTR("Real time clock stopped.");
  }
  else
  {
    DEBUG_PRINTSTR("Real time clock running.");
  }

  if (RTC.writeEN())
  {
    DEBUG_PRINTLNSTR("   -   Write allowed.");
  }
  else
  {
    DEBUG_PRINTLNSTR("   -   Write protected.");
  }


  { // Muss ich das überhaupt synchronisieren?? Schließlich les ich die RTC ja immer neu aus...sollte ich probieren, wenn ich die RTC hab!
    
    setSyncProvider(RTC.get); // the function to get the time from the RTC
  
    if(timeStatus() == timeSet)
    {
      DEBUG_PRINTLNSTR("RTC sync...okay!");
      *state |= (1 << RTC_RUNNING_BIT); // sets the RTC bit indicating the RTC is running.
    }
    else
    {
      DEBUG_PRINTLNSTR("RTC sync...FAIL!");
      *state &= ~(1 << RTC_RUNNING_BIT); // clears the RTC bit indicating a problem with the RTC.
    }
  }
}
uint8_t RTC_DS1302::setTime(time_t newTime)
{
	return RTC.set(newTime);
}

time_t RTC_DS1302::getTime()
{
	return RTC.get();
}
int RTC_DS1302::setAlarm(time_t)
{
}

/*
 * The aim of this function is to adjust the real time clock that it is synchronized with the Controller clock, 
 * but only if a certain deviation has been reached.
 * The Transmission duration and the controller real time at previous transmission is known.
 * So one can calculate the current time of the controller within some hundred milliseconds.
 * Adjusting the delay with a manual measurement (Synchronized LED blinking) also would be possible. But not stable if router nodes are used.
 * The RC1302 doesn't support direct subsecond synchronisation. So I just the real time second and also do not transmit milliseconds between the nodes.
 * If the difference between the real time clocks is bigger than a certain threshold, the SensorNode clock gets updated.
*/
int RTC_DS1302::adjustRTC(int roundTripDelay, uint8_t* state, time_t controllerTime)
{
// (nDelay - 110)     / eigentliche Laufzeit. Server und Node machen in Summe 110ms Pause
// (nDelay - 110) /2  / Laufzeit pro Richtung
//  (controllerTime + nDelay/2 - 55) // ungefähre aktuelle Zeit. Die Verarbeitungszeit am Node hat das etwas verzögert, aber der Algorithmus kann angepasst werden, wenn wir die RTCs haben.
// since the RC1302  doesn't have easy ways to set it to milliseconds and I don't want spent too much time with synchronizing, we are happy with a resolution of a second and don't adjust the time...


  time_t tLocalTime;

  if ((*state & (1 << RTC_RUNNING_BIT))  == true)     // only sync if the clock is working.
  {
    tLocalTime = RTC.get();
    if (abs(tLocalTime - controllerTime) > RTC_SYNC_THRESHOLD)
    {
      RTC.set(controllerTime);
    }
  }
  
}


/*
*  DS3231
*/
	
/* 
*	Initialises the TWI communication and turns off unnecessary stuff at the RTC.
*	init gets the myData state passed to set the RTC synched-flag. Is it needed?!
*
*	Todo: Maybe it is power consuming to initialise the clock so early.
*		Maybe it makes more sense to init it only when we need it. But for now it is fine.
*		Also the synchronising of the ATMega clock and RTC has to be thought through..
*		Do we need it? I didn't implement it here, so the state is not touched.
 */
int RTC_DS3231::init(uint8_t* state)
{
	DEBUG_PRINTSTR("Initializing the RTC DS3231...");
	Wire.begin();			// start TWI communication
	
	//turn off 32kHz pin:
	  byte temp_buffer = temp_buffer & 0b11110111;
	  writeControlByte(temp_buffer, 1);

    DEBUG_PRINT("...");
    
	  //turn of the SQW signal
	  temp_buffer =   0b01111111;
	  writeControlByte(temp_buffer, 0);
	  DEBUG_PRINTLNSTR("done");
	  
//	  *state |= (1 << RTC_RUNNING_BIT);			// set it to valid, otherwise adjust will not react..
}

/* 
*	convert from UNIX timestamp to second, minute, hour, etc. and set this time.
*/
uint8_t RTC_DS3231::setTime(time_t newTime)
{
	tmElements_t tm;

	DEBUG_PRINTSTR("Setting time to the RTC DS3231...old time: ");
	DEBUG_PRINT(getTime());
	DEBUG_PRINTSTR("...new time: ");
	DEBUG_PRINT(newTime);
	
	breakTime(newTime, tm);

	setDS3231time(tm.Second, tm.Minute, tm.Hour, tm.Wday, tm.Day, tm.Month, tm.Year);
	DEBUG_PRINTLN("done");
}

/* 
*	convert from second, minute, hour, etc. to UNIX timestamp.
*/
time_t RTC_DS3231::getTime()
{
  tmElements_t tm;
  time_t currentTime;
  getUNIXtime(&currentTime, &tm);
  return currentTime;
}
int RTC_DS3231::setAlarm(time_t)
{
}

int RTC_DS3231::adjustRTC(int roundTripDelay, uint8_t* state, time_t controllerTime)
{
	time_t tLocalTime;
	tmElements_t tm;

//  if ((*state & (1 << RTC_RUNNING_BIT))  == true)     // only sync if the clock is working.
//  {
	tLocalTime = getTime();
	
	DEBUG_PRINTSTR("\nController time: ");
	DEBUG_PRINT(controllerTime);
	
	DEBUG_PRINTSTR(" Our time: ");
	DEBUG_PRINTLN(tLocalTime);
    if (abs(tLocalTime - controllerTime) > RTC_SYNC_THRESHOLD)
    {
		DEBUG_PRINTSTR("\tRTC deviation too big. Adjusting RTC...");
		breakTime(controllerTime, tm);
		setDS3231time(tm.Second, tm.Minute, tm.Hour, tm.Wday, tm.Day, tm.Month, tm.Year);
		DEBUG_PRINTLNSTR("done");
    }
//  }
}




/*
* DS3231 helper functions from Eric Ayars:
*/

void writeControlByte(byte control, bool which)
{
// Set DS3121 RTC control bytes
  // Write the selected control byte.
  // which=false -> 0x0e, true->0x0f.
  Wire.beginTransmission(0x68);
  if (which)
  {
    Wire.write(0x0f);
  }
  else 
  {
    Wire.write(0x0e);
  }
  Wire.write(control);
  Wire.endTransmission();
}

void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}

void readDS3231time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

void getUNIXtime(time_t * currentUNIXtime, tmElements_t* tm)
{
//  tmElements_t tm;
//  Serial.println(tm.Second);
//  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  readDS3231time(&tm->Second, &tm->Minute, &tm->Hour, &tm->Wday, &tm->Day, &tm->Month, &tm->Year);
  *currentUNIXtime = makeTime(*tm);
}
  
void displayTime(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
{
//  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
//  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  // send it to the serial monitor
//  Serial.println(".");
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(" Day of week: ");
  switch(dayOfWeek)
  {
  case 1:
    Serial.println("Sunday");
    break;
  case 2:
    Serial.println("Monday");
    break;
  case 3:
    Serial.println("Tuesday");
    break;
  case 4:
    Serial.println("Wednesday");
    break;
  case 5:
    Serial.println("Thursday");
    break;
  case 6:
    Serial.println("Friday");
    break;
  case 7:
    Serial.println("Saturday");
    break;
  default:
    Serial.println("Error");
  }

}

void displayTimeFromUNIX(time_t showTime)
{
	tmElements_t tm;
	breakTime(showTime, tm);
	displayTime(tm.Second, tm.Minute, tm.Hour, tm.Wday, tm.Day, tm.Month, tm.Year);

}
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

/*
*	End of DS3231 helper functions
*/







/*
 *          Part II       --  EEPROM management
 */





/*
*	EEPROM storage class
*/

/*
* This function looks for the EEPROM header. The header is located on a different address after each time the EEPROM data gets fully read by the controller.
* The purpose is to achieve euqal usage of the EEPROM and avoid defects.
* 
*
* It runs thorugh a predefined address space in the EEPROM and looks for the header information:
*   INDEXBEGIN      	is the start of the address space
*   INDEXELEMENTS   	is the number of elements (the element size is defined by the struct. It is adapted automatically)
*   EEPROM_data_start	is the first address of the data address space. It is calculated automatically.
*   DATARANGE_END   	is the end of the data address space.
*   The number of data elements is calculated automatically, although some more testing is advised.
*   
*   Once found it stores the address of the active index. Also the current data address stored.
*   If no header is found, one header address and one data address is randomly selected.
*   The data address is stored in the EEPROM header at the selected address, so next time it will be found.
*   
*
 */
uint8_t DataStorage::init()
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
      mnIndexBegin = currentAddress;     // This is the Index Position
      mnDataBlockBegin = myEEPROMHeader.DataStartPosition & ~(1<<EEPROM_HEADER_STATUS_VALID);    // Calculate the position of the first EEPROM data
      DEBUG_PRINTSTR("Found Header at ");
      DEBUG_PRINT(mnIndexBegin);
      DEBUG_PRINTSTR(", first data block is at ");
      DEBUG_PRINTLN(mnDataBlockBegin);
	  
// find next usable data block
		findQueueEnd();
      return 0;
    }
  }

  // if it reaches this point, no index was found. Set random header index:
  mnIndexBegin = INDEXBEGIN + sizeof(myEEPROMHeader) * (analogRead(17) % INDEXELEMENTS);

  // if it reaches this point, no index was found. Set random index:
  uint16_t EEPROM_data_start = INDEXBEGIN + sizeof(myEEPROMHeader) * INDEXELEMENTS;
  uint16_t modul = (DATARANGE_END - EEPROM_data_start) / sizeof(struct sensorData);



  mnDataBlockBegin = EEPROM_data_start + sizeof(struct sensorData) * (analogRead(17) % modul);     // This is the new index Position;
  myEEPROMHeader.DataStartPosition =  mnDataBlockBegin;
  myEEPROMHeader.DataStartPosition |= (1<<EEPROM_HEADER_STATUS_VALID);    // Set this as the position of the first EEPROM data


  EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the data (ID) back to EEPROM...
  DEBUG_PRINTSTR("No header found.\nHeader adress range:  ");
  DEBUG_PRINT(INDEXBEGIN);
  DEBUG_PRINTSTR(" until ");
  DEBUG_PRINTLN(EEPROM_data_start - sizeof(myEEPROMHeader));
  DEBUG_PRINTSTR("Data adress range:    ");
  DEBUG_PRINT(EEPROM_data_start);
  DEBUG_PRINTSTR(" until ");
  DEBUG_PRINTLN(DATARANGE_END);

  
  DEBUG_PRINTSTR("Randomly set header at ");
  DEBUG_PRINT(mnIndexBegin);
  DEBUG_PRINTSTR(" and first data block at ");
  DEBUG_PRINTLN(mnDataBlockBegin);

  struct sensorData dummy;
  EEPROM.get(mnDataBlockBegin, dummy);   // reading a struct, so it is flexible...
  DEBUG_PRINTSTR("Initializing data block at address ");
  DEBUG_PRINT(mnDataBlockBegin);
  DEBUG_PRINTSTR(" - last_data bit is ");
  DEBUG_PRINTLN((dummy.state & (1 << EEPROM_DATA_LAST)) > 0);
  

  dummy.state = (1 << EEPROM_DATA_LAST);    // the EEPROM_DATA_AVAILABLE bit has to be zero

  EEPROM.put(mnDataBlockBegin, dummy);   // writing the data (ID) back to EEPROM...
  DEBUG_PRINTSTR("Done");

  EEPROM.get(mnDataBlockBegin, dummy);   // reading a struct, so it is flexible...
  DEBUG_PRINTSTR(" - data_available bit is ");
  DEBUG_PRINTLN((dummy.state & (1 << EEPROM_DATA_AVAILABLE)) > 0);
  mnLastData = mnDataBlockBegin;		// the address of the last item is the address we have been looking for
//  mnNextData = mnDataBlockBegin;
  empty = true;
		
  return 0;
}

void DataStorage::unsetHeaders()
{
  uint8_t i;
  uint16_t nAddress;
  struct EEPROM_Header dummyHeader;
  dummyHeader.DataStartPosition = 0;
  
  for (i = 0; i < INDEXELEMENTS; i++)
  {
    nAddress = INDEXBEGIN + i*sizeof(dummyHeader);
    Serial.print("X: ");
    Serial.println(nAddress);
    EEPROM.put(nAddress, dummyHeader);
  }
}

/*
* go through the queue and find the last item
*/
uint8_t DataStorage::findQueueEnd()
{
	uint16_t nCurrentAddress;
	uint16_t nPreviousAddress;
	time_t firstItemTimestamp;
	nCurrentAddress = mnDataBlockBegin;
	uint16_t nAddressOfOldestElement;
	uint16_t nAddressBeforeOldestElement;
	struct sensorData currentElement;
 uint16_t nPreviousOldestElement;

  
  
	firstItemTimestamp = 0;
	firstItemTimestamp--;
	do
	{
		nPreviousAddress = nCurrentAddress;
		DEBUG_PRINTLN(nCurrentAddress);
		EEPROM.get(nCurrentAddress, currentElement);   // reading the current data item

		nCurrentAddress += sizeof(currentElement);
		if (nCurrentAddress >= DATARANGE_END)		// exceeded the data range...go to the beginning of the range
		{
			nCurrentAddress = INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS;
		}
		if (firstItemTimestamp > currentElement.realTime)		// remember the timestamp and address of the oldest element
		{
			firstItemTimestamp = currentElement.realTime;
      nPreviousOldestElement = firstItemTimestamp;
			nAddressOfOldestElement = nCurrentAddress;
			nAddressBeforeOldestElement = nPreviousAddress;
		}
	}
	while (((currentElement.state & (1 << EEPROM_DATA_LAST)) == 0) && (nCurrentAddress != mnDataBlockBegin));   // until we find the last stored element or we recognize it's an overflow
	
	if (nCurrentAddress == mnDataBlockBegin)		// flag is not set in any element...there was an overflow!
	{
		DEBUG_PRINTSTR("No 'last-item'-flag found! --> Overflow. All elements have to be transmitted. Oldest element is: ");
		DEBUG_PRINTLN(nAddressOfOldestElement);
		nCurrentAddress = nAddressOfOldestElement;
		mnLastData = nAddressBeforeOldestElement;
		mbOverflow = true;
	}
	else										// no overflow
	{
	//	mnLastData = nCurrentAddress - sizeof(currentElement);		// the address of the last item is the address we have been looking for
		mnLastData = nPreviousAddress;

    if (nPreviousOldestElement & EEPROM_OVERFLOW_OFFSET_BIT)    // offset reached, reset it!
    {
      struct EEPROM_Data myEEPROMData;    // reset the ID, since the offset is reached
      myEEPROMData.ID = 1;
      EEPROM.put(EEPROM_ID_ADDRESS,myEEPROMData);   // writing the data (ID) back to EEPROM...
    }
	}
	
	DEBUG_PRINTSTR("Next: ");
	DEBUG_PRINTLN(nCurrentAddress);
	empty = true;	// per default we assume there was no data
	if (nCurrentAddress != (mnDataBlockBegin + sizeof(currentElement)))	// there was at least one data block
	{
		empty = false;
	}
		
	DEBUG_PRINTSTR("Last Address: ");
	DEBUG_PRINTLN(mnLastData);
}

/*
*	storing new data:
*/
uint8_t DataStorage::add(struct sensorData currentData)
{
	struct sensorData lastElement;
	uint16_t nNextData;   // at this address the data will be inserted

// calculate next data address, once we arrive at the end of the memory, wrap around..
// for an empty queue mnLastData equals the begin of the list.
	if (empty == false)
	{
    DEBUG_PRINTSTR("Not empty. Previous address: ");
    DEBUG_PRINT(mnLastData);
    DEBUG_PRINTSTR(", next usuable address: ");
	// getting the next usable address:
		nNextData = mnLastData;
   
		incrementDataAddress(&nNextData);             // go to the next data address
//		+ sizeof(currentData);
   DEBUG_PRINTLN(nNextData);
   DEBUG_PRINTSTR("Begin of the data: ");
   DEBUG_PRINT(mnDataBlockBegin);

//		uint16_t EEPROM_data_start = INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS;
//		if (nNextData >= DATARANGE_END)	// if the address is outside of the address range, start from the beginning of the data block
//		{
//			nNextData = EEPROM_data_start;
//		}
		
		if (nNextData == mnDataBlockBegin)		// if we arrive again at the start address...
		{
			mbOverflow = true;		// set overflow flag
		}
	}
	else				// the queue is empty
	{
    DEBUG_PRINTSTR("Empty. ");
		nNextData = mnLastData;
	}

	DEBUG_PRINTSTR("Adding ");
	DEBUG_PRINT(currentData.realTime);
	DEBUG_PRINTSTR(" at address ");
	DEBUG_PRINTLN(nNextData);

	// storing the new data and mark it as last data. In case of an overflow, skip this flag.
	// In this case there will be no "last" element, but the timestamps will be checked.
	if (mbOverflow == false)
	{
		currentData.state |= (1<<EEPROM_DATA_LAST);
	}
	EEPROM.put(nNextData, currentData);   // writing the data (ID) back to EEPROM...	
		
		
	// If the list was not empty: mark the previous element as "not last".
	// If the list was empty, we do not need to do anything
	if (empty == false)
	{
	// adjusting the flags of the previous data element	
		EEPROM.get(mnLastData, lastElement);   			// reading the previous element
		lastElement.state &= ~(1<<EEPROM_DATA_LAST);	// previous data is not the last.
		EEPROM.put(mnLastData, lastElement);   			// writing the state back to the previous element.
		
		DEBUG_PRINTSTR("Deleting flag for ");
		DEBUG_PRINTLN(mnLastData);
	}


	empty = false;
	mnLastData = nNextData;
}

void DataStorage::delIndex()
{
  struct EEPROM_Header myEEPROMHeader;
  
  EEPROM.get(mnIndexBegin, myEEPROMHeader);   // reading a struct, so it is flexible...
  myEEPROMHeader.DataStartPosition &= ~(1<<EEPROM_HEADER_STATUS_VALID);
  EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header with removed index
}

void DataStorage::getNext(struct sensorData * currentData)
{
  
}

/*
 * This function reads one element from the storage and removes it.
 * If it was the last element in the storage, it will unset the valid flag 
 * of the current EEPROM header and set the next EEPROM header valid.
 * 
 * The next element to read always is the element previously put into the storage.
 * 
 * empty
 * mnLastData
 * mbOverflow
 * 
 */
void DataStorage::readNextItem(struct sensorData* dataElement)
{
  // kann man das einfach anhand der Pointer zurückdrehen? :-O

  if (mbOverflow == false)   				// non-overflow situation
  {
//    DEBUG_PRINTSTR("\tNo overflow - retrieving data...");
    DEBUG_PRINTSTR("\tNo overflow - retrieving data from address ");
    DEBUG_PRINT(mnLastData);
    DEBUG_PRINTSTR("...");
    EEPROM.get(mnLastData, *dataElement);   // reading the data we want to transmit
    DEBUG_PRINTLNSTR("done");
  }
  else                      				// overflow situation - read
  {

    // easy way:
    //  transmit the data before the mnLastData address. This way the normal no-overflow-algorithm can handle the rest
    
    // probably better but more complicated:
    // check all elements for the newest and the second newest timestamp.
    // Return the element with the newest timestamp and mark the second newest as the newest.

  // delete one element and:
  //    -) set its previous element as last element
  //    -) unset overflow

    uint16_t currentDataAddress = mnDataBlockBegin;
    DEBUG_PRINTSTR("\tOverflow - current address: ");
    DEBUG_PRINT(currentDataAddress);
    decrementDataAddress(&currentDataAddress);       // get the address of the data to send
    DEBUG_PRINTSTR(", increased address: ");
    DEBUG_PRINT(currentDataAddress);
    DEBUG_PRINT(" - retrieving data...");
    EEPROM.get(currentDataAddress, *dataElement);    // reading out the data
    DEBUG_PRINTLNSTR("done");
  }
}

/*
 * After the data was transmitted successfully, the data element has to be removed.
 */
void DataStorage::freeNextItem()
{
  struct EEPROM_Header myEEPROMHeader;

  if (mbOverflow == false)           // non-overflow situation
  {
    if (mnLastData == mnDataBlockBegin)   // is this the only element?  set the header index to the next element. The EEPROM_DATA_LAST bit doesn't have to be deleted, since the block will be ignored afterwards anyway
    {
      DEBUG_PRINTSTR("\tNo overflow..only element - unsetting validity bit of current header (address: ");
      DEBUG_PRINT(mnIndexBegin);
  // unsetting the header valid bit:
      EEPROM.get(mnIndexBegin, myEEPROMHeader);   // reading the currently valid header
      DEBUG_PRINT(" - data address: ");
      DEBUG_PRINT(myEEPROMHeader.DataStartPosition);
      DEBUG_PRINT(" --> ");
      myEEPROMHeader.DataStartPosition &= ~(1<<EEPROM_HEADER_STATUS_VALID);     // unsetting its validity bit
      EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing back the header to invalidate it.
      DEBUG_PRINT(myEEPROMHeader.DataStartPosition);
      DEBUG_PRINTLN(")");
      
      
  // moving the header index forward:
      incrementHeaderAddress(&mnIndexBegin);


    
  // set the next data address into the new header field:
      incrementDataAddress(&myEEPROMHeader.DataStartPosition);
      mnDataBlockBegin = myEEPROMHeader.DataStartPosition;                    // Store the new data start address
      myEEPROMHeader.DataStartPosition |= (1<<EEPROM_HEADER_STATUS_VALID);    // Set this as the valid position of the first EEPROM data. It is important to also set the empty flag, so it is clear this data is not valid.
      EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header


    DEBUG_PRINTSTR("\tNew header address: ");
    DEBUG_PRINTLN(mnIndexBegin);
    DEBUG_PRINTSTR("\tNew data address: ");
    DEBUG_PRINTLN(mnDataBlockBegin);
    DEBUG_PRINTSTR("\tAdding validity bit: ");
    DEBUG_PRINTLN(myEEPROMHeader.DataStartPosition);
    DEBUG_PRINTLNSTR("\tDone");
    
    mnLastData = mnDataBlockBegin;    // wird evtl nicht richtig gesetzt!?
  // resetting the storage:
      mbOverflow = false;   // no overflow happened so far
      empty = true;         // Queue is empty now
    }
    else              // it is not the only element
    {

    // mark the previous element as the last one...
    DEBUG_PRINTSTR("\tNo overflow... multiple elements left - current data address: ");
    DEBUG_PRINTLN(mnLastData);
      decrementDataAddress(&mnLastData);
    DEBUG_PRINTSTR("\tsetting data element at address ");
    DEBUG_PRINT(mnLastData);
    DEBUG_PRINTLNSTR("\tas last element.");
      
      setDataAsLast(mnLastData);           // mark it as last
//      EEPROM.get(mnLastData, currentData);  // reading the previous data
//      setDataAsLast(currentData);           // mark it as last
//      EEPROM.put(mnLastData, currentData);  // write back the previous data
    DEBUG_PRINTLNSTR("\tDone");
    }
  }
  else              // overflow situation - read
  {
    uint16_t currentData = mnDataBlockBegin;

  DEBUG_PRINTSTR("\tOverflow... next data address: ");
  DEBUG_PRINTLN(currentData);


    decrementDataAddress(&currentData);       // get the address of the sent data
  DEBUG_PRINTSTR("\tPrevious data address: ");
  DEBUG_PRINTLN(currentData);
    decrementDataAddress(&currentData);       // get the address of the now last element in the storage
    setDataAsLast(currentData);               // mark it as last
  DEBUG_PRINTSTR("\tAddress of the now last element: ");
  DEBUG_PRINTLN(currentData);
    
    mbOverflow = false;   // reset overflow
  }
}


/*
 * Find the next Data Element, even if there was a wrap around.
 * 
 * The size of the header data element also is considered by the INDEXELEMENTS '- 1' term.
 */
void DataStorage::incrementHeaderAddress(uint16_t* headerAddress)
{
  *headerAddress += sizeof(struct EEPROM_Header);  // go one element forward
  if (*headerAddress > INDEXBEGIN + sizeof(struct EEPROM_Header) * (INDEXELEMENTS - 1) )          // is it out of the header address range?
  {
    *headerAddress = INDEXBEGIN;   // go to the beginning of the header address range
  }
}


/*
 * Find the next Data Element, even if there was a wrap around.
 * 
 * The DATARANGE_END check has to be adapted by the size of the data element
 */
void DataStorage::incrementDataAddress(uint16_t* dataAddress)
{
  *dataAddress += sizeof(struct sensorData);  // go one element forward
  if (*dataAddress >= DATARANGE_END)          // is it out of the address range?
  {
    *dataAddress = INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS;   // go to the beginning of the address range
  }
}

/*
 * Find the previous Data Element, even if there was a wrap around.
 * 
 * The DATARANGE_END check has to be adapted by the size of the data element
 */
void DataStorage::decrementDataAddress(uint16_t* dataAddress)
{
  *dataAddress -= sizeof(struct sensorData);  // go one element back
  if (*dataAddress < INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS)          // is it out of the address range?
  {
    // find the last possible address in the range:
    do
    {
      *dataAddress += sizeof(struct sensorData);
    }
    while(*dataAddress < DATARANGE_END);
    *dataAddress -= sizeof(struct sensorData);
  }
}

void DataStorage::setDataAsLast(uint16_t dataAddress)   // wie sind die flags gesetzt, wenn kein element valid ist??
{
  struct sensorData data;
  EEPROM.get(dataAddress, data);   // reading the data
  data.state |= (1 << EEPROM_DATA_LAST);
  EEPROM.put(dataAddress, data);   // writing the data back
}

void DataStorage::setDataAsNotLast(uint16_t dataAddress)
{
  struct sensorData data;
  EEPROM.get(dataAddress, data);   // reading the data
  data.state &= ~(1 << EEPROM_DATA_LAST);
  EEPROM.put(dataAddress, data);   // writing the data back
}

bool DataStorage::getEmpty()
{
  return empty;
}


void DataStorage::stashData()
{
  struct EEPROM_Header myEEPROMHeader;
  
  EEPROM.get(mnIndexBegin, myEEPROMHeader);   // reading a struct, so it is flexible...
  DEBUG_PRINTSTR("Resetting index ");
  DEBUG_PRINT(myEEPROMHeader.DataStartPosition);
  DEBUG_PRINTSTR(" - at address ");
  DEBUG_PRINT(mnIndexBegin);
  myEEPROMHeader.DataStartPosition &= ~(1<<EEPROM_HEADER_STATUS_VALID);
  EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header with removed index
  DEBUG_PRINTSTR(" to ");
  DEBUG_PRINTLN(myEEPROMHeader.DataStartPosition);
  
  
  myEEPROMHeader.DataStartPosition = mnNextDataBlock;	// the next data block is the new start address
  myEEPROMHeader.DataStartPosition |= (1<<EEPROM_HEADER_STATUS_VALID);	// set the index valid
  mnIndexBegin += sizeof(myEEPROMHeader);
  if (mnIndexBegin >= INDEXBEGIN + INDEXELEMENTS * sizeof(myEEPROMHeader))
  {
	mnIndexBegin = INDEXBEGIN;
  }
  
  DEBUG_PRINTSTR("Setting next index at address ");
  DEBUG_PRINT(mnIndexBegin);
  DEBUG_PRINTSTR(" to the next data address ");
  DEBUG_PRINTLN(myEEPROMHeader.DataStartPosition);
  
  EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header with removed index
  mbOverflow = false;	// no overflow happened so far
  empty = true;			// Queue is empty now
}


void DataStorage::printElements()
{
	uint16_t currentDataAddress;
	struct sensorData currentElement;
	
	currentDataAddress = mnDataBlockBegin;
	
	do
	{
		EEPROM.get(currentDataAddress, currentElement);   // reading a struct, so it is flexible...
		DEBUG_PRINTSTR("Address: ");
		DEBUG_PRINT(currentDataAddress);
		DEBUG_PRINTSTR(" - time: ");
		DEBUG_PRINT(currentElement.realTime);
		DEBUG_PRINTSTR(" - overflow: ");
		DEBUG_PRINTLN(mbOverflow);
		currentDataAddress += sizeof(currentElement);
		if (currentDataAddress >= DATARANGE_END)		// reached the end -- start from the beginning of the data block.
		{
			currentDataAddress = INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS;
		}

	}
	// As long as the current element is not the last or in case of an overflow not all data has been transmitted
//	while((((currentElement.state & (1 << EEPROM_DATA_LAST)) == 0) && (mbOverflow == false)) || ((mbOverflow == true) && (currentDataAddress == mnDataBlockBegin)));
	while((((currentElement.state & (1 << EEPROM_DATA_LAST)) == 0) && (mbOverflow == false)) || ((mbOverflow == true) && (currentDataAddress != mnDataBlockBegin)));      // as long as it's no overflow and more data to come or if it's an overflow until all data has been written
	
	
}







/*
 * checks whether there was an interactive command from the APP
 */
uint8_t CommandHandler::getInteractiveCommands()
{
  
}

/*
 * Checks whether a watering was scheduled for this time
 */
uint8_t CommandHandler::checkSchedule()
{
  
}



/*
 * This function reads the list of all known nodes from a memory.
 * In case of a particle it should write/read to/from flash.
 * 
 * The arduino should use an external SD card (limited to 20 nodes) or flash (4)
 */
void nodeList::getNodeList()
{
  int nRet;
  uint16_t nCurrentAddress;
  
  mnNodeCount = 0;
/*
 * For now we read it from an SD card...
 */
  if (HW == HW_ARDUINO)
  {
    if (SD_AVAILABLE == 1)
    {
      File nodeListFile;
      if (SD.exists(NODELIST_FILENAME))    // node list file exists. good!
      {
        DEBUG_PRINTSTR(NODELIST_FILENAME);
        DEBUG_PRINTLNSTR(" exists."); 
      }
      else          // file doesn't exist..creating file
      {
        DEBUG_PRINTSTR(NODELIST_FILENAME);
        DEBUG_PRINTLNSTR(" doesn't exist..creating file."); 
        nodeListFile = SD.open(NODELIST_FILENAME, FILE_WRITE);
        nodeListFile.close();
      }
    
    
      nodeListFile = SD.open(NODELIST_FILENAME, FILE_READ);
      if (nodeListFile)         // if the file is available, read it:
      {
        DEBUG_PRINTLN("Reading from file...");
        while (nodeListFile.available())    // there is data in the file
        {
          DEBUG_PRINTLN("Reading one node");
    //      myNodeList.myNodes[myNodeList.mnNodeCount] = nodeListFile.read((uint8_t *)&nodeListElement, sizeof(nodeListElement)/sizeof(uint8_t));      // read one node
           nRet = nodeListFile.read((uint8_t *)&myNodes[mnNodeCount], sizeof(myNodes[mnNodeCount])/sizeof(uint8_t));      // read one node
           if (nRet == 0)
           {
              DEBUG_PRINTLN("Reading error..");
           }
          mnNodeCount++;      // keep track of the number of nodes read so far
        }
    // now all nodes should be read
      }
      // if the file isn't open, pop up an error:
      else
      {
        DEBUG_PRINTSTR("error opening ");
        DEBUG_PRINTLNSTR(NODELIST_FILENAME);
      }
    }
    else      // read nodes from EEPROM
    {
      nCurrentAddress = NODELIST_ADDRESS;
      
      DEBUG_PRINTSTR("Maximum nodelist length: ");
      DEBUG_PRINT(NODELISTSIZE);
      DEBUG_PRINTSTR("Starting search at ");
      DEBUG_PRINTLN(nCurrentAddress);
      DEBUG_PRINTLNSTR("Nodes found: ");
      
      for(mnNodeCount = 0; mnNodeCount < NODELISTSIZE; mnNodeCount++)
      {
        EEPROM.get(nCurrentAddress, myNodes[mnNodeCount]);   // reading a struct, so it is flexible...
        if (myNodes[mnNodeCount].ID == 0xffff) // this is an empty node --> reached end of the list
        {
          break;
        }

        DEBUG_PRINT(myNodes[mnNodeCount].ID);
        DEBUG_PRINT(" - ");
        if (myNodes[mnNodeCount].nodeType == 0)
        {
          DEBUG_PRINTLNSTR("SensorNode");
        }
        else
        {
          DEBUG_PRINTLNSTR("PumpNode");
        }
        nCurrentAddress += sizeof(struct nodeListElement);
      }
      DEBUG_PRINT(mnNodeCount);
      DEBUG_PRINTLNSTR(" nodes found in total.");
    }
  }

 

}

/*
 * Clears the NodeList-EEPROM
 */
void nodeList::clearEEPROM_Nodelist()
{
  uint16_t nCurrentAddress;
  
  nCurrentAddress = NODELIST_ADDRESS;
  myNodes[0].ID = 0xffff;
  myNodes[0].nodeType = 0;
  myNodes[0].sensorID = 0;
  
  for(mnNodeCount = 0; mnNodeCount < NODELISTSIZE; mnNodeCount++)
  {
    EEPROM.put(nCurrentAddress, myNodes[0]);   // reading a struct, so it is flexible...
    nCurrentAddress += sizeof(struct nodeListElement);
  }
}


/* checks whether a node with a specific ID exists
 *  
 */
uint8_t nodeList::findNodeByID(uint16_t ID)
{
  uint8_t i;
  for(i = 0; i < NODELISTSIZE; i++)
  {
    if (myNodes[i].ID == ID)    // element exists already
    {
      return i;
    }
  }

  return 0xff;    // node does not exist
}


/*
 * Adds a node to the node list.
 * If it a node with this ID already exists, it will not be able to connect.
 */
uint8_t nodeList::addNode(struct nodeListElement newElement)
{
  uint8_t nodeIndex = 0xff;
  uint16_t nCurrentAddress;
  
// check if node exists already:
  DEBUG_PRINTLN("\tBrowsing list of existing nodes");
  nodeIndex = findNodeByID(newElement.ID);
  if (nodeIndex != 0xff)        // // element exists already
  {
    DEBUG_PRINTLN("\tNode exists already! Node cannot be added!");
    return 1;
  }
  
  DEBUG_PRINT("\tNode does not exist within the list yet..");
  
// otherwise add the node to the list and to the memory:
  if (mnNodeCount < NODELISTSIZE)
  {
    DEBUG_PRINTLN("list isn't full yet.");
    myNodes[mnNodeCount] = newElement;      // copy new element
    mnNodeCount++;                          // keep track of the number of elements

    if (HW == HW_ARDUINO)
    {
      DEBUG_PRINT("\tArduino ");
      if (SD_AVAILABLE == 1)          // write it to SD
      {
        DEBUG_PRINTLN("(SD version)");
        // todo
      }
      else                            // write it to EEPROM
      {
        DEBUG_PRINTLN("(non SD version)");
        nCurrentAddress = NODELIST_ADDRESS + (mnNodeCount-1) * sizeof(struct nodeListElement);    // calculating the next EEPROM node list address
        EEPROM.put(nCurrentAddress, newElement);                                               // writing the node to EEPROM
        DEBUG_PRINT("\tStored node to EEPROM at address ");
        DEBUG_PRINTLN(nCurrentAddress);
        
      }
    }
    else            // on a particle: write it to the flash memory
    {
      DEBUG_PRINTLN("Particle");
      // todo
    }
    
  }
  else
  {
    DEBUG_PRINTLN("Node list is full! Node cannot be added!");
    return 2;
  }
  return 0;
}
