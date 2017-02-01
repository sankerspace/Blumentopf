
//https://forum.arduino.cc/index.php?topic=346940.0

// JChristensen library for DS3231
// RTC - Real Time Clock

#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include "LowPower.h"
#include "DS3232RTC.h"
#include "TimeLib.h"


//const int pinRTC = 10;
const int pinRTC = 4;
const int wakeUpPin = 2;

void setup()
{
  static time_t time_;
  //External Interrupt Pin
  pinMode(wakeUpPin, INPUT);

  Serial.begin(19200);//debug
  Serial.println("Startup...");//debug
  delay(100);//debug

  pinMode(pinRTC, OUTPUT);
  digitalWrite(pinRTC, HIGH);

  tmElements_t tm;
  

/*
    tm.Second=30;
    tm.Minute=22;
    tm.Hour=0;
    tm.Wday=3;   // day of week, sunday is day 1
    tm.Day=31;
    tm.Month=1;
    tm.Year=CalendarYrToTm(2017);//y2kYearToTm(2017) ;   // offset from 1970;
    //setTime(makeTime(tm));
    RTC.write(tm);
*/

  RTC.read(tm);
  time_ = RTC.get();
  
  String str = "Date: "; //debug
  str = str + tm.Day + "." + tm.Month + "." + tmYearToY2k(tm.Year) + "--" + tm.Hour + ":" + tm.Minute + ":" + tm.Second; //debug
  Serial.println(str);//debug
  // Power for RTC from Arduino PIN to RTC VCC
  String str2 = "Date: "; //debug
  str2 = str2 + day(time_) + "." + month(time_) + "." + year(time_) + "--" + hour(time_) + ":" + minute(time_) + ":" + second(time_); //debug
  Serial.println(str2);//debug

  displayTimeFromUNIX(RTC.get());
  /*
    To preserve the battery, the first time V BAT is applied
    to the device, the oscillator will not start up until V CC
    exceeds V PF , or until a valid I 2 C address is written to
    the part. Typical oscillator startup time is less than one
    second. Approximately 2 seconds after V CC is applied,
    or a valid I 2 C address is written, the device makes a
    temperature measurement and applies the calculated
    correction to the oscillator. Once the oscillator is running,
    it continues to run as long as a valid power source is avail-
    able (V CC or V BAT ), and the device continues to measure
    the temperature and correct the oscillator frequency every
    64 seconds. [DS3231.pdf-p10-Power Control]
  */


  // Set Alarm2 every minute
  //RTC.setAlarm(ALM2_EVERY_MINUTE, 00, 00, 00, (dowSunday | dowMonday | dowTuesday | dowWednesday | dowThursday | dowFriday | dowSaturday));
  //RTC.alarmInterrupt(ALARM_2, true);      //assert the INT pin when Alarm2 occurs.
  delay(5000);
}

void wakeUp()
{
  // handling interrupt code
}


void loop()
{
  /*
    Serial.println("Node goes to sleep.");//debug
    delay(500);//debug
     // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(0, wakeUp, LOW);
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    Serial.println("Node woke up.");delay(500);//debug
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0);
    //debug
    if (RTC.alarm(ALARM_2))
    {
      Serial.println("Alarm fired!");
    }
    else
    {
      Serial.println("-");
    }

    delay (1000);

  */

}
