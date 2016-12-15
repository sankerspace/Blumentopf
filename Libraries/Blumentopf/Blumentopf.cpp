#include "Wire.h"
#include <Time.h>
#include <TimeLib.h>
#include <DS1302RTC.h>

#include <dht11.h>
#include "Blumentopf.h"

/*
* This scetion deals with the RTC and time conversations
*
*/


 
//RTC_DS3231::RTC_DS3231() {}
	
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
  
void displayTime()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  // send it to the serial monitor
  Serial.println(".");
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