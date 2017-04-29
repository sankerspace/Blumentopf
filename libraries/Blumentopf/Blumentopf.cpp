/*
 * Project: NESE_Blumentopf
 * File:    Blumentopf.cpp
 * Authors: Bernhard Fritz  (0828317@student.tuwien.ac.at)
 *          Marko Stanisic  (0325230@student.tuwien.ac.at)
 *          Helmut Bergmann (0325535@student.tuwien.ac.at)
 * The copyright for the software is by the mentioned authors.
 *
 * This library provides all functions to the network protocol,
 * real time clock and memory management.
 * It is seperated into three main parts which are not split
 * into different files, because Arduino and Git don't match well.
 *
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
 *  Part III
 * PumpNode_Handler is a list of all nodes which have to pump at the moment.
*/
#include "Blumentopf.h"

//for compilation on spark processor problematic
#if (HW == 1)
	#include <DS1302RTC.h>
	#include <EEPROM.h>
	#include <dht11.h> //Bernhard@: brauch ma das im Controller?
#endif




#if (SD_AVAILABLE == 1)
	#include <SD.h>
#endif




/*
* This scetion deals with the RTC and time conversions
*/

#if (HW_RTC > NONE)
/*
*  DS1302
*/
#if (HW_RTC == RTC_1302 && HW==HW_ARDUINO)

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
int RTC_DS1302::adjustRTC(int roundTripDelay, uint8_t* state, time_t Time)
{
// (nDelay - 110)     / eigentliche Laufzeit. Server und Node machen in Summe 110ms Pause
// (nDelay - 110) /2  / Laufzeit pro Richtung
//  (controllerTime + nDelay/2 - 55) // ungefähre aktuelle Zeit. Die Verarbeitungszeit am Node hat das etwas verzögert, aber der Algorithmus kann angepasst werden, wenn wir die RTCs haben.
// since the RC1302  doesn't have easy ways to set it to milliseconds and I don't want spent too much time with synchronizing, we are happy with a resolution of a second and don't adjust the time...


  time_t tLocalTime;

  if ((*state & (1 << RTC_RUNNING_BIT))  == true)     // only sync if the clock is working.
  {
    tLocalTime = RTC.get();
    if (abs(tLocalTime - Time) > RTC_SYNC_THRESHOLD)
    {
      RTC.set(Time);
    }
  }

}

#elif (HW_RTC == RTC_3231)

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
	  //byte temp_buffer = temp_buffer & 0b11110111;
		//Bernhard@: temp_buffer uinitialised
		byte temp_buffer = 0b11110111;
	  writeControlByte(temp_buffer, 1);

    DEBUG_PRINT("...");

	  //turn of the SQW signal
	  temp_buffer =   0b01111111;
	  writeControlByte(temp_buffer, 0);
	  DEBUG_PRINTLNSTR("done");

//	  *state |= (1 << RTC_RUNNING_BIT);			// set it to valid, otherwise adjust will not react..
	return 0;
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
	DEBUG_PRINTLNSTR("done");
	return 0;
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
	//Bernhard@: Funktion leer???
	return 0;
}

int RTC_DS3231::adjustRTC(int roundTripDelay, uint8_t* state, time_t Time)
{
	time_t tLocalTime;
	time_t tDifference;
	tmElements_t tm;

//  if ((*state & (1 << RTC_RUNNING_BIT))  == true)     // only sync if the clock is working.
//  {
	tLocalTime = getTime();


	if (tLocalTime > Time)
	{
	  tDifference = tLocalTime - Time;
	}
	else
	{
	  tDifference = Time - tLocalTime;
	}


	DEBUG_PRINTSTR("\tController time: ");
	DEBUG_PRINT(Time);
	DEBUG_PRINTSTR(" Our time: ");
	DEBUG_PRINT(tLocalTime);
	DEBUG_PRINTSTR(", Deviation: ");

	DEBUG_PRINTLN(tDifference);

    if (tDifference > RTC_SYNC_THRESHOLD)
    {
		DEBUG_PRINTSTR("\tRTC deviation too big. Adjusting RTC...");
		breakTime(Time, tm);
		setDS3231time(tm.Second, tm.Minute, tm.Hour, tm.Wday, tm.Day, tm.Month, tm.Year);
		DEBUG_PRINTLNSTR("done");
    }
//  }

	return 0;
}

#elif (HW_RTC == RTC_3232)


/*
*  DS3232
*/
 RTC_DS3232::RTC_DS3232()
 {
	 RTC=0;
	 RTC=new DS3232RTC;
 }



 RTC_DS3232::~RTC_DS3232()
 {//DS3232RTC* RTC
    if(RTC>0)
      delete RTC;
 }


int RTC_DS3232::init(uint8_t* state)
{
  if (DEBUG_RTC > 0)
  {
	DEBUG_PRINTSTR("Initializing the RTC DS3232...");
	DEBUG_PRINTLNSTR("done");
  }

	//*state |= (1 << RTC_RUNNING_BIT);			// set it to valid, otherwise adjust will not react..
}

/*
*	convert from UNIX timestamp to second, minute, hour, etc. and set this time.
*/
uint8_t RTC_DS3232::setTime(time_t newTime)
{
	tmElements_t tm;

	DEBUG_PRINTSTR("Setting time to the RTC DS3231...old time: ");
	DEBUG_PRINT(getTime());
	DEBUG_PRINTSTR("...new time: ");
	DEBUG_PRINT(newTime);

	breakTime(newTime, tm);

	RTC->write(tm);
	DEBUG_PRINTLNSTR("done");
}

/*
*	convert from second, minute, hour, etc. to UNIX timestamp.
*/
time_t RTC_DS3232::getTime()
{
  time_t currentTime = RTC->get();

  return currentTime;
}
int RTC_DS3232::setAlarm(time_t)
{
  DEBUG_PRINTLNSTR("Still not implemented!!!! ");
}

int RTC_DS3232::adjustRTC(int roundTripDelay, uint8_t* state, time_t Time)
{
/*
	time_t tLocalTime;
	tmElements_t tm;

  if ((*state & (1 << RTC_RUNNING_BIT))  == true)     // only sync if the clock is working.
  {
	tLocalTime = getTime();

	DEBUG_PRINTSTR("\nController time: ");
	DEBUG_PRINT(controllerTime);

	DEBUG_PRINTSTR(" Our time: ");
	DEBUG_PRINTLN(tLocalTime);
    if (abs(tLocalTime - controllerTime) > RTC_SYNC_THRESHOLD)
    {
		DEBUG_PRINTSTR("\tRTC deviation too big. Adjusting RTC...");
		breakTime(controllerTime, tm);
	  this->RTC.write(tm);
		DEBUG_PRINTLNSTR("done");
    }
  }
  */
}







#endif
#endif

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

String displayTime(time_t time_)
{

    String str = "Date: "; //debug
    //str = str + day(time_) + "." + month(time_) + "." + year(time_) + "--" + hour(time_) + ":" + minute(time_) + ":" + second(time_)+"-"; //debug
   str = str + String(day(time_)) + "." + String(month(time_)) + "." + String(year(time_)) + "--" + String(hour(time_)) + ":" + String(minute(time_)) + ":" + String(second(time_))+"-"; //debug

   switch(weekday(time_))
  {
  case 1:
    str+="Sunday";
    break;
  case 2:
    str+="Monday";
    break;
  case 3:
    str+="Tuesday";
    break;
  case 4:
    str+="Wednesday";
    break;
  case 5:
    str+="Thursday";
    break;
  case 6:
    str+="Friday";
    break;
  case 7:
    str+="Saturday";
    break;
  default:
    str="Error";
  }

  DEBUG_PRINTLN(str);//debug
  return str;

}

void displayTime(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year, uint8_t nDepth)
{
//  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
//  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  // send it to the serial monitor
//  Serial.println(".");
  if (nDepth == 0)
  {
    return;
  }
  DEBUG_PRINTDIG(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  DEBUG_PRINTSTR(":");
  if (minute<10)
  {
    DEBUG_PRINTSTR("0");
  }
  DEBUG_PRINTDIG(minute, DEC);
  DEBUG_PRINTSTR(":");
  if (second<10)
  {
    DEBUG_PRINTSTR("0");
  }
  DEBUG_PRINTDIG(second, DEC);

  if (nDepth == 1)
  {
    return;
  }

  DEBUG_PRINTSTR(" ");
  DEBUG_PRINTDIG(dayOfMonth, DEC);
  DEBUG_PRINTSTR("/");
  DEBUG_PRINTDIG(month, DEC);
  DEBUG_PRINTSTR("/");
  DEBUG_PRINTDIG(year, DEC);

  if (nDepth == 2)
  {
    return;
  }

  DEBUG_PRINTSTR(" Day of week: ");
  switch(dayOfWeek)
  {
  case 1:
    DEBUG_PRINTLNSTR("Sunday");
    break;
  case 2:
    DEBUG_PRINTLNSTR("Monday");
    break;
  case 3:
    DEBUG_PRINTLNSTR("Tuesday");
    break;
  case 4:
    DEBUG_PRINTLNSTR("Wednesday");
    break;
  case 5:
    DEBUG_PRINTLNSTR("Thursday");
    break;
  case 6:
    DEBUG_PRINTLNSTR("Friday");
    break;
  case 7:
    DEBUG_PRINTLNSTR("Saturday");
    break;
  default:
    DEBUG_PRINTLNSTR("Error");
  }

}

void displayTimeFromUNIX(time_t showTime, uint8_t nDepth)
{
//Bernhard@:Funktionsaufrufe verschwenden Speicher(und wir haben echt wenig),
//überdenke das nochmal, wennst meinst das unbedingt notwenidg dann ok
#if (DEBUG_ == 1)
	tmElements_t tm;
	breakTime(showTime, tm);
	//marko@: -1970?????
	displayTime(tm.Second, tm.Minute, tm.Hour, tm.Wday, tm.Day, tm.Month, (tm.Year - 30), nDepth);	// -30 to convert von 00 to 1970 format
    //displayTime(tm.Second, tm.Minute, tm.Hour, tm.Wday, tm.Day, tm.Month, tmYearToY2k(tm.Year), nDepth);
#endif
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
	if (DEBUG_DATA_STORAGE > 0)
	{
      DEBUG_PRINTSTR("\tHeader ");
      DEBUG_PRINT(i);
      DEBUG_PRINTSTR(" - Address ");
      DEBUG_PRINT(currentAddress);
      DEBUG_PRINTSTR(": ");
      DEBUG_PRINTLN(myEEPROMHeader.DataStartPosition);
	}

    if ((myEEPROMHeader.DataStartPosition & (1<<EEPROM_HEADER_STATUS_VALID)) > 0)   // this header is valid.
    {
      mnIndexBegin = currentAddress;     // This is the Index Position
      mnDataBlockBegin = myEEPROMHeader.DataStartPosition & ~(1<<EEPROM_HEADER_STATUS_VALID);    // Calculate the position of the first EEPROM data
	  if (DEBUG_DATA_STORAGE > 0)
	  {
	    DEBUG_PRINTSTR("\tFound Header at ");
	    DEBUG_PRINT(mnIndexBegin);
	    DEBUG_PRINTSTR(", first data block is at ");
	    DEBUG_PRINTLN(mnDataBlockBegin);
	  }

// find next usable data block
//Bernhard@:Wieso erstellts du funktionen die im Endeffekt nix zurückgeben?

		findQueueEnd();
      return 0;
    }
  }

  // if it reaches this point, no index was found. Set random header index:
  mnIndexBegin = INDEXBEGIN + sizeof(myEEPROMHeader) * (analogRead(17) % INDEXELEMENTS);

  // if it reaches this point, no index was found. Set random index:
  uint16_t EEPROM_data_start = INDEXBEGIN + sizeof(myEEPROMHeader) * INDEXELEMENTS;
  uint16_t modul = (DATARANGE_END - EEPROM_data_start) / sizeof(struct Data);



  mnDataBlockBegin = EEPROM_data_start + sizeof(struct Data) * (analogRead(17) % modul);     // This is the new index Position;
  myEEPROMHeader.DataStartPosition =  mnDataBlockBegin;
  myEEPROMHeader.DataStartPosition |= (1<<EEPROM_HEADER_STATUS_VALID);    // Set this as the position of the first EEPROM data


  EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the data (ID) back to EEPROM...
  struct Data dummy;
  EEPROM.get(mnDataBlockBegin, dummy);   // reading a struct, so it is flexible...

  if (DEBUG_DATA_STORAGE > 0)
  {
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

  DEBUG_PRINTSTR("Initializing data block at address ");
  DEBUG_PRINT(mnDataBlockBegin);
  DEBUG_PRINTSTR(" - last_data bit is ");
  DEBUG_PRINTLN((dummy.state & (1 << EEPROM_DATA_LAST)) > 0);
  }


  dummy.state = (1 << EEPROM_DATA_LAST);    // the EEPROM_DATA_AVAILABLE bit has to be zero

  EEPROM.put(mnDataBlockBegin, dummy);   // writing the data (ID) back to EEPROM...
  DEBUG_PRINTSTR_D("Done", DEBUG_DATA_STORAGE);

  EEPROM.get(mnDataBlockBegin, dummy);   // reading a struct, so it is flexible...
  DEBUG_PRINTSTR_D(" - data_available bit is ", DEBUG_DATA_STORAGE);
  DEBUG_PRINTLN_D((dummy.state & (1 << EEPROM_DATA_AVAILABLE)) > 0, DEBUG_DATA_STORAGE);
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
    DEBUG_PRINTSTR("X: ");
    DEBUG_PRINTLN(nAddress);
    EEPROM.put(nAddress, dummyHeader);
  }
}

/*
* go through the queue and find the last item
*/
//Bernhard@:: Sollte da nicht etwas zurückgegeben werden???
//Marko: Hab das jetzt so geändert damit ich es für den Spark(Photon)
//compilieren kann, warum hat die Arduino IDE da nicht regiert?
void DataStorage::findQueueEnd()
{
	uint16_t nCurrentAddress;
	uint16_t nPreviousAddress;
	time_t firstItemTimestamp;
	nCurrentAddress = mnDataBlockBegin;
	uint16_t nAddressOfOldestElement=0;
	//Bernhard@: Variablen initialisieren bitte
	uint16_t nAddressBeforeOldestElement=0;
	struct Data currentElement;
  uint16_t nPreviousOldestElement=0;



	firstItemTimestamp = 0;
	firstItemTimestamp--;
	do
	{
		nPreviousAddress = nCurrentAddress;
		DEBUG_PRINTLN_D(nCurrentAddress, DEBUG_DATA_STORAGE);
		EEPROM.get(nCurrentAddress, currentElement);   // reading the current data item

		nCurrentAddress += sizeof(currentElement);
		if (nCurrentAddress >= DATARANGE_END)		// exceeded the data range...go to the beginning of the range
		{
			nCurrentAddress = INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS;
		}
		if (firstItemTimestamp > currentElement.Time)		// remember the timestamp and address of the oldest element
		{
			firstItemTimestamp = currentElement.Time;
      nPreviousOldestElement = firstItemTimestamp;
			nAddressOfOldestElement = nCurrentAddress;
			nAddressBeforeOldestElement = nPreviousAddress;
		}
	}
	while (((currentElement.state & (1 << EEPROM_DATA_LAST)) == 0) && (nCurrentAddress != mnDataBlockBegin));   // until we find the last stored element or we recognize it's an overflow

	if (nCurrentAddress == mnDataBlockBegin)		// flag is not set in any element...there was an overflow!
	{
		DEBUG_PRINTSTR_D("No 'last-item'-flag found! --> Overflow. All elements have to be transmitted. Oldest element is: ", DEBUG_DATA_STORAGE);
		DEBUG_PRINTLN_D(nAddressOfOldestElement, DEBUG_DATA_STORAGE);
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

	DEBUG_PRINTSTR_D("Next: ", DEBUG_DATA_STORAGE);
	DEBUG_PRINTLN_D(nCurrentAddress, DEBUG_DATA_STORAGE);
	empty = true;	// per default we assume there was no data
	if (nCurrentAddress != (mnDataBlockBegin + sizeof(currentElement)))	// there was at least one data block
	{
		empty = false;
	}

	DEBUG_PRINTSTR_D("Last Address: ", DEBUG_DATA_STORAGE);
	DEBUG_PRINTLN_D(mnLastData, DEBUG_DATA_STORAGE);
}

/*
*	storing new data:
*/
void DataStorage::add(struct Data currentData)
{
	struct Data lastElement;
	uint16_t nNextData;   // at this address the data will be inserted

// calculate next data address, once we arrive at the end of the memory, wrap around..
// for an empty queue mnLastData equals the begin of the list.
	if (empty == false)
	{
      if (DEBUG_DATA_STORAGE > 0)
      {
        DEBUG_PRINTSTR("\tNot empty. Previous address: ");
        DEBUG_PRINT(mnLastData);
        DEBUG_PRINTSTR(", next usuable address: ");
      }
	// getting the next usable address:
      nNextData = mnLastData;

      incrementDataAddress(&nNextData);             // go to the next data address
//		+ sizeof(currentData);
      if (DEBUG_DATA_STORAGE > 0)
      {
        DEBUG_PRINTLN(nNextData);
        DEBUG_PRINTSTR("\tBegin of the data: ");
        DEBUG_PRINT(mnDataBlockBegin);
      }
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
      DEBUG_PRINTSTR_D("\tEmpty. ", DEBUG_DATA_STORAGE);
      nNextData = mnLastData;
	}

	if (DEBUG_DATA_STORAGE > 0)
	{
	  DEBUG_PRINTSTR("\tAdding ");
	  DEBUG_PRINT(currentData.Time);
	  DEBUG_PRINTSTR(" at address ");
	  DEBUG_PRINTLN(nNextData);
	}

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

		DEBUG_PRINTSTR_D("Deleting flag for ", DEBUG_DATA_STORAGE);
		DEBUG_PRINTLN_D(mnLastData, DEBUG_DATA_STORAGE);
	}


	empty = false;
	mnLastData = nNextData;
}


/*
 * deletes only the index
 */
void DataStorage::delIndex()
{
  struct EEPROM_Header myEEPROMHeader;

  EEPROM.get(mnIndexBegin, myEEPROMHeader);   // reading a struct, so it is flexible...
  myEEPROMHeader.DataStartPosition &= ~(1<<EEPROM_HEADER_STATUS_VALID);
  EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header with removed index
}

void DataStorage::getNext(struct Data * currentData)
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
void DataStorage::readNextItem(struct Data* dataElement)
{
  // kann man das einfach anhand der Pointer zurückdrehen? :-O

  if (mbOverflow == false)   				// non-overflow situation
  {
//    DEBUG_PRINTSTR("\tNo overflow - retrieving data...");
    if (DEBUG_DATA_STORAGE > 0)
    {
      DEBUG_PRINTSTR("\t\tNo overflow - retrieving data from address ");
      DEBUG_PRINT(mnLastData);
      DEBUG_PRINTSTR("...");
    }
    EEPROM.get(mnLastData, *dataElement);   // reading the data we want to transmit
    DEBUG_PRINTLNSTR_D("done", DEBUG_DATA_STORAGE);
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
    DEBUG_PRINTSTR_D("\t\tOverflow - current address: ", DEBUG_DATA_STORAGE);
    DEBUG_PRINT_D(currentDataAddress, DEBUG_DATA_STORAGE);
    decrementDataAddress(&currentDataAddress);       // get the address of the data to send
    DEBUG_PRINTSTR_D(", increased address: ", DEBUG_DATA_STORAGE);
    DEBUG_PRINT_D(currentDataAddress, DEBUG_DATA_STORAGE);
    DEBUG_PRINT_D(" - retrieving data...", DEBUG_DATA_STORAGE);
    EEPROM.get(currentDataAddress, *dataElement);    // reading out the data
    DEBUG_PRINTLNSTR_D("done", DEBUG_DATA_STORAGE);
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

  // unsetting the header valid bit:
      EEPROM.get(mnIndexBegin, myEEPROMHeader);   // reading the currently valid header
      if (DEBUG_DATA_STORAGE > 0)
	  {
        DEBUG_PRINTSTR("\tNo overflow..only element - unsetting validity bit of current header (address: ");
        DEBUG_PRINT(mnIndexBegin);
        DEBUG_PRINT(" - data address: ");
        DEBUG_PRINT(myEEPROMHeader.DataStartPosition);
        DEBUG_PRINT(" --> ");
	  }
      myEEPROMHeader.DataStartPosition &= ~(1<<EEPROM_HEADER_STATUS_VALID);     // unsetting its validity bit
      EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing back the header to invalidate it.
      DEBUG_PRINT_D(myEEPROMHeader.DataStartPosition, DEBUG_DATA_STORAGE);
      DEBUG_PRINTLN_D(")", DEBUG_DATA_STORAGE);


  // moving the header index forward:
      incrementHeaderAddress(&mnIndexBegin);



  // set the next data address into the new header field:
      incrementDataAddress(&myEEPROMHeader.DataStartPosition);
      mnDataBlockBegin = myEEPROMHeader.DataStartPosition;                    // Store the new data start address
      myEEPROMHeader.DataStartPosition |= (1<<EEPROM_HEADER_STATUS_VALID);    // Set this as the valid position of the first EEPROM data. It is important to also set the empty flag, so it is clear this data is not valid.
      EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header

    if (DEBUG_DATA_STORAGE > 0)
	{
      DEBUG_PRINTSTR("\tNew header address: ");
      DEBUG_PRINTLN(mnIndexBegin);
      DEBUG_PRINTSTR("\tNew data address: ");
      DEBUG_PRINTLN(mnDataBlockBegin);
      DEBUG_PRINTSTR("\tAdding validity bit: ");
      DEBUG_PRINTLN(myEEPROMHeader.DataStartPosition);
      DEBUG_PRINTLNSTR("\tDone");
	}

    mnLastData = mnDataBlockBegin;    // wird evtl nicht richtig gesetzt!?
  // resetting the storage:
      mbOverflow = false;   // no overflow happened so far
      empty = true;         // Queue is empty now
    }
    else              // it is not the only element
    {

    // mark the previous element as the last one...
      DEBUG_PRINTSTR_D("\tNo overflow... multiple elements left - current data address: ", DEBUG_DATA_STORAGE);
      DEBUG_PRINTLN_D(mnLastData, DEBUG_DATA_STORAGE);
      decrementDataAddress(&mnLastData);
	  if (DEBUG_DATA_STORAGE > 0)
	  {
        DEBUG_PRINTSTR("\tsetting data element at address ");
        DEBUG_PRINT(mnLastData);
        DEBUG_PRINTLNSTR("\tas last element.");
	  }

      setDataAsLast(mnLastData);           // mark it as last
//      EEPROM.get(mnLastData, currentData);  // reading the previous data
//      setDataAsLast(currentData);           // mark it as last
//      EEPROM.put(mnLastData, currentData);  // write back the previous data
      DEBUG_PRINTLNSTR_D("\tDone", DEBUG_DATA_STORAGE);
    }
  }
  else              // overflow situation - read
  {
    uint16_t currentData = mnDataBlockBegin;

    if (DEBUG_DATA_STORAGE > 0)
    {
      DEBUG_PRINTSTR("\tOverflow... next data address: ");
      DEBUG_PRINTLN(currentData);
    }


    decrementDataAddress(&currentData);       // get the address of the sent data
    DEBUG_PRINTSTR_D("\tPrevious data address: ", DEBUG_DATA_STORAGE);
    DEBUG_PRINTLN_D(currentData, DEBUG_DATA_STORAGE);
    decrementDataAddress(&currentData);       // get the address of the now last element in the storage
    setDataAsLast(currentData);               // mark it as last
    DEBUG_PRINTSTR_D("\tAddress of the now last element: ", DEBUG_DATA_STORAGE);
    DEBUG_PRINTLN_D(currentData, DEBUG_DATA_STORAGE);

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
  *dataAddress += sizeof(struct Data);  // go one element forward
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
  *dataAddress -= sizeof(struct Data);  // go one element back
  if (*dataAddress < INDEXBEGIN + sizeof(struct EEPROM_Header) * INDEXELEMENTS)          // is it out of the address range?
  {
    // find the last possible address in the range:
    do
    {
      *dataAddress += sizeof(struct Data);
    }
    while(*dataAddress < DATARANGE_END);
    *dataAddress -= sizeof(struct Data);
  }
}

void DataStorage::setDataAsLast(uint16_t dataAddress)   // wie sind die flags gesetzt, wenn kein element valid ist??
{
  struct Data data;
  EEPROM.get(dataAddress, data);   // reading the data
  data.state |= (1 << EEPROM_DATA_LAST);
  EEPROM.put(dataAddress, data);   // writing the data back
}

void DataStorage::setDataAsNotLast(uint16_t dataAddress)
{
  struct Data data;
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
  if (DEBUG_DATA_STORAGE > 0)
  {
    DEBUG_PRINTSTR("Resetting index ");
    DEBUG_PRINT(myEEPROMHeader.DataStartPosition);
    DEBUG_PRINTSTR(" - at address ");
    DEBUG_PRINT(mnIndexBegin);
    myEEPROMHeader.DataStartPosition &= ~(1<<EEPROM_HEADER_STATUS_VALID);
    EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header with removed index
    DEBUG_PRINTSTR_D(" to ", DEBUG_DATA_STORAGE);
    DEBUG_PRINTLN_D(myEEPROMHeader.DataStartPosition, DEBUG_DATA_STORAGE);
  }

  myEEPROMHeader.DataStartPosition = mnNextDataBlock;	// the next data block is the new start address
  myEEPROMHeader.DataStartPosition |= (1<<EEPROM_HEADER_STATUS_VALID);	// set the index valid
  mnIndexBegin += sizeof(myEEPROMHeader);
  if (mnIndexBegin >= INDEXBEGIN + INDEXELEMENTS * sizeof(myEEPROMHeader))
  {
	mnIndexBegin = INDEXBEGIN;
  }

  if (DEBUG_DATA_STORAGE > 0)
  {
    DEBUG_PRINTSTR("Setting next index at address ");
    DEBUG_PRINT(mnIndexBegin);
    DEBUG_PRINTSTR(" to the next data address ");
    DEBUG_PRINTLN(myEEPROMHeader.DataStartPosition);
  }

  EEPROM.put(mnIndexBegin, myEEPROMHeader);   // writing the new header with removed index
  mbOverflow = false;	// no overflow happened so far
  empty = true;			// Queue is empty now
}


void DataStorage::printElements()
{
	uint16_t currentDataAddress;
	struct Data currentElement;

	currentDataAddress = mnDataBlockBegin;

	do
	{
		EEPROM.get(currentDataAddress, currentElement);   // reading a struct, so it is flexible...
		DEBUG_PRINTSTR("Address: ");
		DEBUG_PRINT(currentDataAddress);
		DEBUG_PRINTSTR(" - time: ");
		DEBUG_PRINT(currentElement.Time);
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
 * Constructor
 */
CommandHandler::CommandHandler()
{
  mnPreviousHour = 23;
  mnPreviousMinute = 59;
}

/*
 * checks whether there was an interactive command from the APP
 */
void CommandHandler::getInteractiveCommands()
{

}

/*
 * * Checks whether a watering was scheduled for this time
 * It takes the node list and the current time.
 * From those values it figures out whether it is watering-time and
 * whether a pump has to get active. Once a pump has finished pumping,
 * the next pump gets started. The node ID of the respective pump and
 * the pump duration is returned by the pointers to the calling function.
 *
 */
uint8_t CommandHandler::checkSchedule(struct nodeList myNodeList, uint16_t* nID, uint16_t* nDuration, time_t currentTime)
{
  tmElements_t tm;
  uint16_t nWateringStartTime;
  uint16_t i;
  breakTime(currentTime, tm);

// checks if it's time to start pumping
//  if (bPumpTime == false) // so far we are not pumping


  nWateringStartTime = WATERING_START_HOUR*60+WATERING_START_MINUTE;
/*  DEBUG_PRINT("H: ");
  DEBUG_PRINT(tm.Hour);
  DEBUG_PRINT(" / H_p: ");
  DEBUG_PRINT(mnPreviousHour);
  DEBUG_PRINT(", M: ");
  DEBUG_PRINT(tm.Minute);
  DEBUG_PRINT(" / M_p: ");
  DEBUG_PRINTLN(mnPreviousMinute);

  DEBUG_PRINT(nWateringStartTime);
  DEBUG_PRINT(" : ");
  DEBUG_PRINTLN(tm.Hour*60 + tm.Minute);
*/
  if (nWateringStartTime <= (tm.Hour*60 + tm.Minute))    // we are after the watering start point
  {
//    DEBUG_PRINT(nWateringStartTime);
//    DEBUG_PRINT(" : ");
//    DEBUG_PRINTLN(mnPreviousHour*60 + mnPreviousMinute);
    if (nWateringStartTime > (mnPreviousHour*60 + mnPreviousMinute))    // in the previous round we have been before
    {   //Marko@:(23*60+24)>(23*60+59)???? was soll previoushour bedeuten?
      DEBUG_PRINTLNSTR("\tPassed watering start time");
      // so the watering starts now
      bWateringNow = true;
      mnCurrentIndex = 0;
    }
  }
  mnPreviousHour = tm.Hour;
  mnPreviousMinute = tm.Minute;


  if (bWateringNow == true)       // controller is in watering mode
  {
//    DEBUG_PRINTLNSTR("\t\tWatering mode");
    for(i = mnCurrentIndex; i < NODELISTSIZE; i++)
    {
      DEBUG_PRINTSTR("\t\tElement");
      DEBUG_PRINTLN(i);
      //if (myNodeList.myNodes[i].nodeType == 1)    // it is a motor node
      if ((myNodeList.myNodes[i].state & (1 << NODELIST_NODETYPE)) == 1)    // it is a motor node
      {
//        DEBUG_PRINTSTR("\t\t\tMotorNode");
        if ((myNodeList.myNodes[i].watering_policy & (1<<POL_ACTIVE)) == 1)    // the watering policy is enabled
        {
          DEBUG_PRINTLNSTR("\t\t\tPolicy enabled");
          if ((myNodeList.myNodes[i].watering_policy & (1<<POL_USE_MOISTURE)) == 1)    // use the data of the moisture sensor
          {
            // still has to be implemented. It requires sensor data management within the controller
            continue; // der Node wird daweil ignoriert.
          }
          else      // otherwise just pump for a certain time. This is the most simple way and we will use for the demo
          {
             mnCurrentIndex = i+1;  // store the node so next round the following node is checked.
            *nID = myNodeList.myNodes[i].ID;
            *nDuration = POL_WATERING_DEFAULT_DURATION;
            return SCHEDULED_WATERING;
          }
        }
      }
    }

  }


  // when the function arrives here, there are no pumping actions to do.


  bWateringNow = false;
  return NO_SCHEDULED_WATERING;
}



/*
 * This function reads the list of all known nodes from a memory.
 * In case of a particle it should write/read to/from flash.
 *
 * The arduino should use an external SD card (limited to 20 nodes) or flash (4)
 */
void nodeList::getNodeList()
{

  uint16_t nCurrentAddress;

  mnNodeCount = 0;
/*
 * For now we read it from an SD card...
 */
  if (HW == HW_ARDUINO)
  {

#if (SD_AVAILABLE == 1)
			int nRet;
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
        DEBUG_PRINTLNSTR("Reading from file...");
        while (nodeListFile.available())    // there is data in the file
        {
          DEBUG_PRINTLNSTR("Reading one node");
    //      myNodeList.myNodes[myNodeList.mnNodeCount] = nodeListFile.read((uint8_t *)&nodeListElement, sizeof(nodeListElement)/sizeof(uint8_t));      // read one node
           nRet = nodeListFile.read((uint8_t *)&myNodes[mnNodeCount], sizeof(myNodes[mnNodeCount])/sizeof(uint8_t));      // read one node
           if (nRet == 0)
           {
              DEBUG_PRINTLNSTR("Reading error..");
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


 #else

      nCurrentAddress = NODELIST_ADDRESS;

	  if (DEBUG_NODE_LIST > 0)
	  {
		  DEBUG_PRINTSTR("\tMaximum nodelist length: ");
		  DEBUG_PRINT(NODELISTSIZE);
		  DEBUG_PRINTSTR(" - Starting search at ");
		  DEBUG_PRINTLN(nCurrentAddress);
		  DEBUG_PRINTSTR("\tNodes found: ");
      }

      for(mnNodeCount = 0; mnNodeCount < NODELISTSIZE; mnNodeCount++)
      {
        EEPROM.get(nCurrentAddress, myNodes[mnNodeCount]);   // reading a struct, so it is flexible...
        if (myNodes[mnNodeCount].ID == 0xffff) // this is an empty node --> reached end of the list
        {
          break;
        }
		if (DEBUG_NODE_LIST > 0)
		{
			DEBUG_PRINTLN(" ");
			DEBUG_PRINT(myNodes[mnNodeCount].ID);
			DEBUG_PRINT(" - ");
		  //  if (myNodes[mnNodeCount].nodeType == 0)
			if((myNodes[mnNodeCount].state & (1<<NODELIST_NODETYPE))==0)
			{
			  DEBUG_PRINTLNSTR("SensorNode");
			}
			else
			{
			  DEBUG_PRINTLNSTR("PumpNode");
			}
		}
        nCurrentAddress += sizeof(struct nodeListElement);
      }
	  if (DEBUG_NODE_LIST > 0)			// are debug messages enabled?
	  {
		  if (mnNodeCount > 0)		// there have been nodes
		  {
			  DEBUG_PRINTSTR("\r\n\t");
			  DEBUG_PRINT(mnNodeCount);
			  DEBUG_PRINTLNSTR(" nodes found in total.");
		  }
		  else
		  {
			DEBUG_PRINTLNSTR("none!");
		  }
	  }

#endif

  }



}

/*
 * Clears the NodeList-EEPROM
 * marko@: can not use mnNodeCount as index in the for loop (11.02.17)
 */
void nodeList::clearEEPROM_Nodelist()
{
  uint16_t nCurrentAddress;

  nCurrentAddress = NODELIST_ADDRESS;


  for(int i = 0; i < NODELISTSIZE; i++)
  {
    myNodes[i].ID = 0xffff;
    myNodes[i].state = 0;
    myNodes[i].sensorID = 0;
    myNodes[i].watering_policy = 0;
    EEPROM.put(nCurrentAddress, myNodes[i]);   // reading a struct, so it is flexible...
    nCurrentAddress += sizeof(struct nodeListElement);
  }

  mnNodeCount = 0;

}


/* checks whether a node with a specific ID exists
 *
 */
uint16_t nodeList::findNodeByID(uint16_t ID)
{
  uint16_t i;
  for(i = 0; i < NODELISTSIZE; i++)
  {
    if (myNodes[i].ID == ID)    // element exists already
    {
      return i;
    }
  }

  return 0xffff;    // node does not exist
}


/*
 * Adds a node to the node list.
 * If it a node with this ID already exists, it will not be able to connect.
 * return 1:Node exists already! Node cannot be added
 * return 2:Node list is full! Node cannot be added!
 */
uint8_t nodeList::addNode(struct nodeListElement newElement)
{
  uint16_t nodeIndex = 0xffff;
  uint16_t nCurrentAddress;

// check if node exists already:
//	if (DEBUG_NODE_LIST == 1)
//	{
	DEBUG_PRINTLNSTR_D("\tBrowsing list of existing nodes", DEBUG_NODE_LIST);
//	}
	nodeIndex = findNodeByID(newElement.ID);
	if (nodeIndex != 0xffff)        // // element exists already
	{
		DEBUG_PRINTLNSTR_D("\t\tNode exists already! Node cannot be added!", DEBUG_NODE_LIST);
		return 1;
	}

	DEBUG_PRINTLNSTR_D("\t\tNode does not exist within the list yet..", DEBUG_NODE_LIST);

// otherwise add the node to the list and to the memory:
  if (mnNodeCount < NODELISTSIZE)
  {
	DEBUG_PRINTLNSTR_D("\t\tList isn't full yet.", DEBUG_NODE_LIST);
    myNodes[mnNodeCount] = newElement;      // copy new element
	myNodes[mnNodeCount].nextSlot = 0;		// no slot yet
    mnNodeCount++;                          // keep track of the number of elements

    if (HW == HW_ARDUINO)
    {
      DEBUG_PRINT("\tHW = Arduino ");
             // write it to SD
#if (SD_AVAILABLE == 1)
        DEBUG_PRINTLNSTR("(SD version)");
        // todo

#else
        // write it to EEPROM

        DEBUG_PRINTLNSTR("(non SD version)");
        nCurrentAddress = NODELIST_ADDRESS + ((mnNodeCount-1) * sizeof(struct nodeListElement));    // calculating the next EEPROM node list address
        EEPROM.put(nCurrentAddress, newElement);                                               // writing the node to EEPROM
        DEBUG_PRINT("\tStored node to EEPROM at address ");
        DEBUG_PRINTLN(nCurrentAddress);
#endif

    }
    else            // on a particle: write it to the flash memory
    {
      DEBUG_PRINTLNSTR("Particle");
      // todo
    }

  }
  else
  {
    DEBUG_PRINTLNSTR("\tNode list is full! Node cannot be added!");
    return 2;
  }
  return 0;
}

/*
 * returns the number of sensor nodes in the list
 */
uint16_t nodeList::getNumberOfSensorNodes()
{
  uint16_t i, nNumberOfSensorNodes;
  nNumberOfSensorNodes = 0;

  for(i = 0; i < mnNodeCount; i++)
  {
    if ((myNodes[i].state & (1<<NODELIST_NODETYPE)) == 0) // sensor node
    {
      nNumberOfSensorNodes++;     // increase the counter for every sensor node
    }
  }
  return nNumberOfSensorNodes;
}


uint16_t nodeList::getLastScheduledSensorNode()
{
  uint16_t i, nLastScheduledNodeIndex = 0;
  time_t tLastScheduledNodeTime = 0;
  for(i = 0; i < mnNodeCount; i++)
  {
    if ((myNodes[i].state & (1<<NODELIST_NODETYPE)) == 0) // sensor node
    {
//		DEBUG_PRINT("\t");
//      DEBUG_PRINT(i);
//      DEBUG_PRINT(": ");
//      DEBUG_PRINTLN(myNodes[i].nextSlot);
      if (myNodes[i].nextSlot > tLastScheduledNodeTime)   // found new last node
      {
//        DEBUG_PRINTSTR("\t\tNew last Node - ");
//        DEBUG_PRINTLN(i);
        tLastScheduledNodeTime = myNodes[i].nextSlot;
        nLastScheduledNodeIndex = i;
      }
    }
  }
  return nLastScheduledNodeIndex;
}


uint16_t nodeList::getPumpEpochLength()
{
  uint16_t i;
  uint16_t nPumpNodeCount = 0;
  for(i = 0; i < mnNodeCount; i++)
  {
    if ((myNodes[i].state & (1 << NODELIST_NODETYPE)) == 1)	// pump node
	{
	  nPumpNodeCount++;
	}
  }
  return nPumpNodeCount * INTERVAL / 10;
}

 /*
  *0xff .. Node doesnt exist
  *0x00.. SensorNode
  *0x01 .. PumpNode
  */
  uint8_t nodeList::getNodeType(uint16_t ID) //Sensor or Pump Node
  {
    for(int index=0;index<mnNodeCount;index++)
    {
      if(myNodes[index].ID==ID)
      {
        return (myNodes[index].state & (1<<NODELIST_NODETYPE));
      }
    }
    return 0xff;
  }



  /*
  *0xff .. Node doesnt exist
  *0x00 .. PumpNode not active , no WateringTask Send currently
  *0x01 .. PumpNode active
  */
  uint8_t nodeList::isActive(uint16_t ID) // PumpNode currently active or not
  {
    for(int index=0;index<mnNodeCount;index++)
    {
      if(myNodes[index].ID==ID){
        return ( (myNodes[index].state & (1<<NODELIST_PUMPACTIVE))>> NODELIST_PUMPACTIVE);
      }
    }

    return 0xff;
  }
  /*manipulates state bit, set pump in active mode*/
  void nodeList::setPumpActive(uint16_t ID)
  {
    for(int index=0;index<mnNodeCount;index++)
    {
      if(myNodes[index].ID==ID){
        myNodes[index].state |= (1<<NODELIST_PUMPACTIVE);
      }
    }
  }
  /*manipulates state bit, set pump in inactive mode */
  void nodeList::setPumpInactive(uint16_t ID)
  {
    for(int index=0;index<mnNodeCount;index++)
    {
      if(myNodes[index].ID==ID){
        myNodes[index].state &= ~(1<<NODELIST_PUMPACTIVE);
      }
    }
  }
  /*manipulates state bit, set node Online */
  //only call that if the node is already stored in the EEPROM
   //only the NodeList of the Controller is updated
  void nodeList::setNodeOnline(uint16_t ID)
  {
    for(int index=0;index<mnNodeCount;index++)
    {
      if(myNodes[index].ID==ID){
        myNodes[index].state |= (1<<NODELIST_NODEONLINE);
      }
    }
  }
  /*manipulates state bit, set node offline */
  //only call that if the node is already stored in the EEPROM
  //only the NodeList of the Controller is updated
  void nodeList::setNodeOffline(uint16_t ID) //maybe Node was not responding a long time
  {
    for(int index=0;index<mnNodeCount;index++)
    {
      if(myNodes[index].ID==ID){
        myNodes[index].state &= ~(1<<NODELIST_NODEONLINE);
      }
    }
  }
  /*
  *0xff .. Node doesnt exist
  *0x00 .. Sensor or Pump Node did'nt registrate (or no lifesign)!!!!!!!!!
  *0x01 .. Sensor or Pump Node has performed a registration(and responds always)
  */
  uint8_t nodeList::isOnline(uint16_t ID) // PumpNode currently active or not
   {
    for(int index=0;index<mnNodeCount;index++)
    {
      if(myNodes[index].ID==ID){
        return ( (myNodes[index].state & (1<<NODELIST_NODEONLINE))>> NODELIST_NODEONLINE);
      }
    }

    return 0xff;
  }



/*
 * PumpNode_Handler is a list of all nodes which have to pump at the moment
 */





/**
* For a pumpNode get the current Pump duration
*/
 uint16_t PumpNode_Handler::getPumpTime(void){

    return this->OnOff;
 }
 /**
* get current state of Interaction between Controller and PumpNode
*/
  int PumpNode_Handler::getState(void){
    return this->pumpnode_status;
  }
	/**
 * state of a packet must be in the same state in which pumpnode remains
 */
   int PumpNode_Handler::getPacketState(void){
     return this->pumpnode_status_packet;
   }
/**
* what is the ID of the PumpNode the controller is speaking to
*/
uint16_t PumpNode_Handler::getID(void)
{
  return this->pumpnode_ID;
}
/**
* get Response for the pump Node
*/
 uint16_t PumpNode_Handler::getResponseData(void)
 {
  return this->pumpnode_response;
 }


bool PumpNode_Handler::getResponseAvailability(void)
{
	  return this->pumpnode_reponse_available;
}

 /**
* Reset State machine
*/
 void PumpNode_Handler::reset(void){

    this->pumpnode_status=PUMPNODE_STATE_0_PUMPREQUEST;

    this->OnOff=0;
    this->pumpnode_response=0;
    this->pumpnode_started_waiting_at=millis();
    this->pumpnode_previousTime=millis();
		this->pumpnode_waitforPump=0;
    this->pumpnode_dif=0;
    this->pumpnode_debugCounter=DEBUG_CYCLE;
		this->pumpnode_reponse_available=false;
		this->pumpnode_status_packet=this->pumpnode_status;
 }
 /*only for debug purposes*/
 void PumpNode_Handler::setPumpHandlerID(uint16_t ID_)
 {
    pumphandler_ID=ID_;
 }
   /*only for debug purposes*/
 uint16_t PumpNode_Handler::getPumpHandlerID(void)
 {
  return pumphandler_ID;
 }


 uint8_t  PumpNode_Handler::getStateErrorCount(void)
 {
  return pumpnode_state_error_counter;
 }

 /**
* That function controls the state machine of a Pump Node,
* in such a way that it receives Responses from Pump by IncomeData
* and send responses/actions to the PumpNode by OutcomeData
*
* IncomeData is the response Data from PumpNode if there is some data
*
*/
void PumpNode_Handler::processPumpstate(uint16_t IncomeData){

        //[STATE 0]------------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_0_PUMPREQUEST){
    if(IncomeData>0)//pumphandler triggered with new PumpTime=IncomeData
    {
      this->OnOff=IncomeData;//calculated from seconds in milliseconds
      this->pumpnode_response=this->OnOff;
			this->pumpnode_reponse_available=true;
      DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
      DEBUG_PRINTSTR("][State 0:]Pump time of ");
      DEBUG_PRINT(this->pumpnode_response);
      DEBUG_PRINTSTR("ms sent to PumPnode ");
      DEBUG_PRINTLN(this->pumpnode_ID);
			this->pumpnode_status_packet=this->pumpnode_status;
      this->pumpnode_status=PUMPNODE_STATE_1_PUMPACTIVE;

      DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
      DEBUG_PRINTLNSTR("][State 0:]Start Listening for first confirmation from the pump...");

      this->pumpnode_previousTime=millis();//A change of state occured here
      this->pumpnode_started_waiting_at = millis();
      DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
      DEBUG_PRINTSTR("][State 0:]previousTime=");
      DEBUG_PRINTLN(this->pumpnode_previousTime);
      this->pumpnode_debugCounter=DEBUG_CYCLE;
			#if (DEBUG_PUMP_ROUNDTRIPTIME)
			this->pumpNode_RoundTripTime=micros();
			#endif
    }else{
      if((this->pumpnode_debugCounter % DEBUG_CYCLE)==0){
        DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
        DEBUG_PRINTSTR("][State 0]-ERROR INCOME DATA PARAMETER:[IncomeData ");
        DEBUG_PRINT(IncomeData);
        DEBUG_PRINTLNSTR("]\n");
      }
      this->pumpnode_debugCounter++;
    }
  }else //[STATE 1]-------------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_1_PUMPACTIVE){
		this->pumpnode_reponse_available=false;
    if(IncomeData==0)
    {//WAIT FOR INCOME DATA

      this->pumpnode_dif=millis()-this->pumpnode_started_waiting_at;
      if(this->pumpnode_dif > WAIT_RESPONSE_INTERVAL)
       {

          this->pumpnode_status=PUMPNODE_STATE_1_RESP_FAILED;
       }
    }else if(IncomeData==this->OnOff)
    {//RECEIVED INCOME DATA

      DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler ");
			DEBUG_PRINT(pumphandler_ID);
      DEBUG_PRINTSTR("][State 1]-Got Respond from PumpNode ");
      DEBUG_PRINT(this->pumpnode_ID);
      DEBUG_PRINTSTR(",respond:");DEBUG_PRINTLN(IncomeData);
			#if (DEBUG_PUMP_ROUNDTRIPTIME)
			DEBUG_PRINTSTR("\t\t[ROUNDTRIP TIME (State0 - State 1)] ");
			DEBUG_PRINT_D(micros()-this->pumpNode_RoundTripTime,DEC);
			DEBUG_PRINTLNSTR(" us.");
			#endif
      this->pumpnode_status=PUMPNODE_STATE_2_PUMPOFF;
      this->pumpnode_previousTime=millis();//A change of state occured here
      this->pumpnode_started_waiting_at = millis();//in the next state we wait for the pump
      this->pumpnode_debugCounter=DEBUG_CYCLE;
			//pump time plus half of roundTripDelay
			this->pumpnode_waitforPump=this->OnOff+(WAIT_RESPONSE_INTERVAL >> 1); //division durch 2
			  DEBUG_PRINTSTR("\t\t[PumpNode_Handler]Waiting at least ");
				DEBUG_PRINT_D(this->pumpnode_waitforPump,DEC);
				DEBUG_PRINTLNSTR(" ms, then pumping should be finished");
    }else{
       if((this->pumpnode_debugCounter % DEBUG_CYCLE)==0){
        DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
        DEBUG_PRINTSTR("][State 1]-ERROR INCOME DATA (");
        DEBUG_PRINT(IncomeData);
        DEBUG_PRINTSTR(") FROM PUMP NODE with ID:");
        DEBUG_PRINTLN(this->pumpnode_ID);
      }
      this->pumpnode_debugCounter++;
    }

  }else //[STATE 2]------------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_2_PUMPOFF){
    if(IncomeData==0)
    {
      this->pumpnode_dif=millis()-this->pumpnode_started_waiting_at;
      if(this->pumpnode_dif > this->pumpnode_waitforPump)
       {
				 //PUMPTIME PASSED SO NOW SEND CONFIRMATION
				 this->pumpnode_response=this->pumphandler_ID;//some usefull check
				 this->pumpnode_reponse_available=true;
				 this->pumpnode_status_packet=this->pumpnode_status;
				 DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
	       DEBUG_PRINTSTR("][State 2]-SEND CONFIRMATION REQUEST to Node-ID ");
	       DEBUG_PRINT(this->pumpnode_ID);
	       DEBUG_PRINTSTR(" with respond:");DEBUG_PRINTLN(this->pumpnode_response);


				 DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler ");
	 			 DEBUG_PRINT(pumphandler_ID);
	       DEBUG_PRINTLNSTR("][State 2]-Switch to State 3 and wait for Acknowledgment");
				 this->pumpnode_status=PUMPNODE_STATE_3_ACKNOWLEDGMENT;
				 this->pumpnode_previousTime=millis();//A change of state occured here
	 			 this->pumpnode_started_waiting_at = millis();//in the next state we wait for the pump
	       this->pumpnode_debugCounter=DEBUG_CYCLE;
				 #if (DEBUG_PUMP_ROUNDTRIPTIME)
	 			 this->pumpNode_RoundTripTime=micros();
	 			#endif
			 }else if((this->pumpnode_dif % 2000)==0)
			 {
				 DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
				DEBUG_PRINTLNSTR("][State 2]-WAIT FOR PUMP TO FINISH PUMPING.");
			 }
    }else{
      if((this->pumpnode_debugCounter % DEBUG_CYCLE)==0){
        DEBUG_PRINTLNSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
        DEBUG_PRINTLNSTR("][State 2]-ERROR INCOME DATA , FALSE PARAMETER!!!!!!!");

      }
      this->pumpnode_debugCounter++;
    }

  }else //[STATE 3]------------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_3_ACKNOWLEDGMENT){
		this->pumpnode_reponse_available=false;
    if(IncomeData==0)
    {//WAIT FOR INCOME DATA

      this->pumpnode_dif=millis()-this->pumpnode_started_waiting_at;
      if(this->pumpnode_dif > WAIT_RESPONSE_INTERVAL)
       {

          this->pumpnode_status=PUMPNODE_STATE_3_RESP_FAILED;
       }
    }else if(IncomeData==this->pumphandler_ID)
    {//RECEIVED INCOME DATA

      DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
      DEBUG_PRINTSTR("][State 3]-Got Respond from PumpNode ");
      DEBUG_PRINT(this->pumpnode_ID);
      DEBUG_PRINTSTR(",respond:");DEBUG_PRINTLN(IncomeData);
			#if (DEBUG_PUMP_ROUNDTRIPTIME)
			DEBUG_PRINTSTR("\t\t[ROUNDTRIP TIME (State2 - State 3)] ");
			DEBUG_PRINT_D(micros()-this->pumpNode_RoundTripTime,DEC);
			DEBUG_PRINTLNSTR(" us.");
			#endif
			this->pumpnode_waitforPump=0;
      this->pumpnode_status=PUMPNODE_STATE_4_FINISHED;
      this->pumpnode_previousTime=millis();//A change of state occured here
      this->pumpnode_debugCounter=DEBUG_CYCLE;


    }else{
       if((this->pumpnode_debugCounter % DEBUG_CYCLE)==0){
        DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
        DEBUG_PRINTSTR("][State 3]-ERROR INCOME DATA (");
        DEBUG_PRINT(IncomeData);
        DEBUG_PRINTSTR(") FROM PUMP NODE with ID:");
        DEBUG_PRINTLN(this->pumpnode_ID);
      }
      this->pumpnode_debugCounter++;
    }

  }else //[STATE 4]------------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_4_FINISHED){
      //Nothing to do
      //PUMP CYCLE WAS SUCCESSFULL
			if((this->pumpnode_debugCounter % DEBUG_CYCLE)==0){
			 DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler ");
			 DEBUG_PRINT(pumphandler_ID);
			 DEBUG_PRINTSTR("][State 4]-SUCCESS STATE!!!!!");
			 DEBUG_PRINTSTR("\t\tFOR PUMP NODE with ID:");
			 DEBUG_PRINTLN(this->pumpnode_ID);
		 }
		 this->pumpnode_debugCounter++;
  }else //[STATE -1]------------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_ERROR){
      //Nothing to do
      //PUMP CYCLE WAS NOT SUCCESSFULL
      pumpnode_state_error_counter++;
			this->pumpnode_status = PUMPNODE_STATE_0_PUMPREQUEST;
			this->pumpnode_previousTime=millis();
			DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler ");
			DEBUG_PRINT(pumphandler_ID);
      DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler ");
			DEBUG_PRINT(pumphandler_ID);
			DEBUG_PRINTLNSTR("][State -1]-PUMPHANDLER IN ERROR STATE!!!!!!!!");
      DEBUG_PRINTLNSTR("][State -1]-ERROR STATE COUNTER IS ");DEBUG_PRINTLN(pumpnode_state_error_counter);

  }
  else //[STATE -3]---------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_1_RESP_FAILED){
    DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
    DEBUG_PRINTSTR("][State -1:]Failed, response timed out[wait:");
    DEBUG_PRINT(this->pumpnode_dif);
    DEBUG_PRINTSTR(" ms],ID:");DEBUG_PRINT(this->pumpnode_ID);
    DEBUG_PRINTSTR(",duration:");DEBUG_PRINTLN(this->OnOff);
    this->pumpnode_started_waiting_at = millis();
    this->pumpnode_status=PUMPNODE_STATE_1_PUMPACTIVE;

  }else //[STATE -4]----------------------------------------------------------
  if(this->pumpnode_status == PUMPNODE_STATE_3_RESP_FAILED){
    DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
    DEBUG_PRINTSTR("][State -3:]Failed, response timed out[wait:");
    DEBUG_PRINT(this->pumpnode_dif);
    DEBUG_PRINTSTR(" ms],ID:");DEBUG_PRINT(this->pumpnode_ID);
    DEBUG_PRINTSTR(",duration:");DEBUG_PRINTLN(this->OnOff);
    this->pumpnode_started_waiting_at = millis();
    this->pumpnode_status=PUMPNODE_STATE_3_ACKNOWLEDGMENT;

  }

 /*Software Watch Dog*/
  uint32_t dif=(millis()-this->pumpnode_previousTime);
  if(dif>(PUMPNODE_CRITICAL_STATE_OCCUPATION+this->pumpnode_waitforPump)){

    this->pumpnode_status=PUMPNODE_STATE_ERROR;
    this->pumpnode_waitforPump=0;

    DEBUG_PRINTLNSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
    DEBUG_PRINTSTR("][WATCHDOG]NO ANSWER, WE WILL GO TO ERROR STATE!");
    DEBUG_PRINTLNSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
    DEBUG_PRINTSTR("][WATCHDOG]STATEMACHINE FOR THE CAN BE RESTARTED!");
    DEBUG_PRINTSTR("[BLUMENTOPF]\t[PumpNode_Handler "); DEBUG_PRINT(pumphandler_ID);
    DEBUG_PRINTSTR("][WATCHDOG]Dif=");
    DEBUG_PRINT(dif);
    DEBUG_PRINTSTR(" ms, previousTime=");
    DEBUG_PRINT(this->pumpnode_previousTime);
    DEBUG_PRINTLNSTR("ms");
  }

 }//processPumpstate


//https://cdn-learn.adafruit.com/downloads/pdf/memories-of-an-arduino.pdf
int freeRam(void){
#if (HW==HW_ARDUINO)
 extern int  __heap_start,*__brkval;
 int v;
 return (int) &v-(__brkval==0?(int)&__heap_start:(int) __brkval);
#else
 return 0;
#endif
}

void printFreeRam()
{
  if (DEBUG_FREE_MEMORY > 0)
  {

    #if (HW==HW_ARDUINO)
		 DEBUG_PRINTSTR("[CONTROLLER][MEMORY]:Between Heap and Stack still ");
      DEBUG_PRINT(freeRam());
			DEBUG_PRINTLNSTR(" bytes available.");
		#elif(HW==HW_PHOTON)
		 DEBUG_PRINTSTR("[CONTROLLER][MEMORY]:Free Memory is  ");
			uint32_t freemem = System.freeMemory();
			DEBUG_PRINT(freemem);
			DEBUG_PRINTLNSTR(" bytes on the PHOTON.");
    #endif

  }
}

void killID()
{
  struct EEPROM_Data myEEPROMData;
  myEEPROMData.ID = 0xffff;
  EEPROM.put(EEPROM_ID_ADDRESS,myEEPROMData);   // resetting the ID
}



/*
* Functions to handle  packetInfo in struct Data
*
*/

int setDATA_Pumpstate(struct Data *packet,int pumpState){
	if(pumpState<PUMPNODE_STATE_0_PUMPREQUEST || pumpState > PUMPNODE_STATE_3_ACKNOWLEDGMENT)
		return -1;
  uint8_t value=(uint8_t)pumpState;
	uint8_t a = (value & 0x1) << DATA_PUMP_STATE_BIT_0 ;
	uint8_t b = ((value & 0x2) >> 1) << DATA_PUMP_STATE_BIT_1 ;

	packet->packetInfo = a | b;

	return 0;
}
void setDATA_SensorPacket(struct Data *packet)
{
	packet->packetInfo &= ~(1 << DATA_NODE_BIT);
}
void setDATA_PumpPacket(struct Data *packet)
{
	packet->packetInfo |= (1 << DATA_NODE_BIT);
}
void setDATA_ControllerPacket(struct Data *packet)
{
	packet->packetInfo |= (1 << DATA_CONTROLLER_BIT);
}
void setDATA_NormalDatapacket(struct Data *packet)
{
	packet->packetInfo &= ~(1 << DATA_CONTROLLER_BIT);
}
void setDATA_RegistrationPacket(struct Data *packet)
{
	packet->packetInfo |= (1 << DATA_REGISTRATION_BIT);
}
void setDATA_NO_RegistrationPacket(struct Data *packet)
{
	packet->packetInfo &= ~(1 << DATA_REGISTRATION_BIT);
}

uint8_t getData_PumpState(struct Data *packet)
{
	uint8_t value = ((packet->packetInfo & (1 << DATA_PUMP_STATE_BIT_0))>>DATA_PUMP_STATE_BIT_0);
	value |=(((packet->packetInfo & (1 << DATA_PUMP_STATE_BIT_1))>>DATA_PUMP_STATE_BIT_1) << 1);
	return value;
}
bool    isPumpPacket(struct Data *packet)
{
	uint8_t value = packet->packetInfo & (1 << DATA_NODE_BIT);
	if(value>0)
		return true;
	return false;
}
bool    isSensorPacket(struct Data *packet)
{
	uint8_t value = packet->packetInfo & (1 << DATA_NODE_BIT);
	if(value==0)
		return true;
	return false;
}

bool    isControllerPacket(struct Data *packet)
{
	uint8_t value = packet->packetInfo & (1 << DATA_CONTROLLER_BIT);
	if(value>0)
		return true;
	return false;

}

bool    isRegistrationPacket(struct Data *packet)
{
	uint8_t value = packet->packetInfo & (1 << DATA_REGISTRATION_BIT);
	if(value>0)
		return true;
	return false;
}
