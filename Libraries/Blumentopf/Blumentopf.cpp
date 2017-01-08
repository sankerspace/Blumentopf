#include "Wire.h"
#include <Time.h>
#include <TimeLib.h>
#include <DS1302RTC.h>
#include <EEPROM.h>

#include <dht11.h>
#include "Blumentopf.h"

/*
* This scetion deals with the RTC and time conversions
*
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
      mnDataBlockBegin = myEEPROMHeader.DataStartPosition & ~(1<<EEPROM_HEADER_STATUS_VALID);    // This is the position of the first EEPROM data
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
  

  dummy.state = (1<< EEPROM_DATA_LAST);    // the EEPROM_DATA_AVAILABLE bit has to be zero

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
		if (firstItemTimestamp > currentElement.realTime)		// store current realtime
		{
			firstItemTimestamp = currentElement.realTime;
			nAddressOfOldestElement = nCurrentAddress;
			nAddressBeforeOldestElement = nPreviousAddress;
		}
	}
	while (((currentElement.state & (1 << EEPROM_DATA_LAST)) == 0) && (nCurrentAddress != mnDataBlockBegin));
	
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
	uint16_t nNextData;

// calculate next data address, once we arrive at the end of the memory, wrap around..
// for an empty queue mnLastData equals the begin of the list.
	if (empty == false)
	{
	// getting the next usable address:
		nNextData = mnLastData + sizeof(currentData);

		uint16_t EEPROM_data_start = INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS;
		if (nNextData >= DATARANGE_END)	// if the address is outside of the address range, start from the beginning of the data block
		{
			nNextData = EEPROM_data_start;
		}
		
		if (nNextData == mnDataBlockBegin)		// if we arrive again at the start address...
		{
			mbOverflow = 1;		// set overflow flag
		}
	}
	else				// the queue is empty
	{
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

void DataStorage::readNextItem(struct sensorData* dataElement)
{
  struct sensorData lastElement;
  if (mbOverflow == false)   				// non-overflow situation
  {
    EEPROM.get(mnLastData, lastElement);   // reading the last data
	*dataElement = lastElement;
  }
  else                      				// overflow situation - read
  {
    
  }
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
	while((((currentElement.state & (1 << EEPROM_DATA_LAST)) == 0) && (mbOverflow == false)) || ((mbOverflow == true) && (currentDataAddress != mnDataBlockBegin)));
	
	
}

