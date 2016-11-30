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

#include <JeeLib.h>
#include <dht11.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define DEBUG 1

#define DHT11PIN 5
#define MOISTURE_THRESHOLD (1000)
#define LIGHT_THRESHOLD (512)
#define SLEEPDURATION (500)
#define BAUD (9600)
#define PRE_SLEEP_DELAY (100)
#define BREAK_DURATION (200)
#define IREF  (1.1)
#define RF_STAQRTUP_TIME (2000)

/*
 * stores all measurment data in 12 bytes. 32 bytes are available in a message.
*/
struct sensorData
{
  float temperature;
  float humidity;
  int moisture;
  int brightness;
  int interval = 2;
  float voltage;
};
struct sensorData myData;

struct EEPROM_Data
{
  int ID;
};

struct Registration_Data
{
  int ID;
  long real_time;
};
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
void printValues(int DHT11_State, struct sensorData);


ISR(WDT_vect)
{
  Sleepy::watchdogEvent();
}

// the setup function runs once when you press reset or power the board
void setup()
{
   struct EEPROM_Data myEEPROMData;
   struct Registration_Data myRegistration;
   int eeAddress;

   eeAddress = 0;
   
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(moisturePin, INPUT);
  pinMode(lightPin, INPUT);
  pinMode(sensorPower, OUTPUT);

  myData.interval = 20;  // at default repeat measurement every 2 seconds

//  digitalWrite(sensorPower, LOW);   // turn off the sensor power
  digitalWrite(sensorPower, HIGH);   // turn off the sensor power
  delay(100);

// In debug mode, output some stuff
#if DEBUG
  Serial.begin(BAUD);
  Serial.println("Sensor Check Application 0.1");
  Serial.print("DHT11 LIBRARY VERSION: ");
  Serial.println(DHT11LIB_VERSION);
  Serial.println("----------------");

  if (sizeof(struct sensorData) > 32)
  {
    Serial.println("Fatal error - too much sensor data. Restructure message protocol\n");
  }
#endif


// read EEPROM

  EEPROM.get(eeAddress,myEEPROMData);
  if (myEEPROMData.ID > 0)
  {
    Serial.print("EEPROM-ID found: ");
    Serial.print(myEEPROMData.ID);
    Serial.print(". Registering at the server... ");
  }
  else
  {
    Serial.print("No EEPROM-ID found. Registering at server...");
    myRegistration.ID = myEEPROMData.ID;
//      myRegistration.real_time = getRealTime();
//    requestID(&myRegistration);
  }

// initialice the NRF24L01
//  setup_RF();
 
}

/* Sendet eine Nachricht zum Server, dass eine neue ID benötigt wird. Das wird nur einmal für jeden neuen Knoten ausgeführt.
* Der Server antwortet mit einer Zufalls-ID und speichert sie in einer Tabelle. Der Client spechert die ID auch in seinemm EPROM und verwendet für zukünftige starts diese ID.
*/
requestID(struct* Registration_Data myRegistration)
{
   long request_Real_time;
  request_Real_time = myRegistration.real_time;
  digitalWrite(sensorPower, HIGH);   // turn on the sensor power and wait some time to stabilize


  RF24 radio(9,10);
  setup_RF(&radio);
  delay(RF_STAQRTUP_TIME);    // RF24 needs time to start up



// has to be revised:
  radio->write(&myRegistration, sizeof(struct Registration_Data));
  
  delay(10);   
    // Now, continue listening
    radio->startListening();


    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( (radio->available() == false) && ! timeout )
    {
      if (millis() - started_waiting_at > 200 )
      {
        timeout = true;
      }
    }
  
    // Describe the results
    if ( timeout )
    {
      Serial.println("Failed, response timed out.");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      unsigned long got_time;
      got_time = millis();
      radio->read(&myRegistration, sizeof(struct Registration_Data));
      
      // Show response
      Serial.print("Got registration response from Server. ID is ");
      Serial.println(myRegistration.ID);
      Serial.print("Current time: ");
      Serial.println(myRegistration.real_time);
      // Hier sollte man sich eine Synchronisation überlegen...zB so:
//      set_real_time(myRegistration.real_time + (myRegistration.real_time - request_Real_time));
    }
    
  }


// the loop function runs over and over again forever
void loop()
{
  int nDHT_Status;
  

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on to indicate action
  
  digitalWrite(sensorPower, HIGH);   // turn on the sensor power and wait some time to stabilize


  RF24 radio(9,10);
  setup_RF(&radio);
  delay(2000);    // DHT needs 1s to settle, RF24 also needs time

// read the input on analog pin 0 (moisture):
  myData.moisture = analogRead(moisturePin);
    
// read the input on analog pin 1 (light):
  myData.brightness = analogRead(lightPin);

// read the temperature and humidity sensors:
//  nDHT_Status = readDHT11();
  nDHT_Status = DHT11.read(DHT11PIN);

  myData.temperature  = DHT11.temperature;
  myData.humidity = DHT11.humidity;

// getting the battery status:
  myData.voltage = getBatteryVoltage();


#if DEBUG
  printValues(nDHT_Status, myData);
#endif

  RF_action(&radio);
  delay(2000);       // transmission needs some time
  digitalWrite(sensorPower, LOW);   // when we finished measuring, turn the sensor power off again
//  digitalWrite(sensorPower, HIGH);   // when we finished measuring, turn the sensor power off again

  delay(PRE_SLEEP_DELAY);                       // finish the serial communication and RF communication
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off for sleeping
  Sleepy::loseSomeTime(myData.interval * 100);
//delay(BREAK_DURATION);

}




void setup_RF(RF24* radio)
{
 
 //
 // Setup and configure rf radio
 //

 radio->begin();

 // optionally, increase the delay between retries & # of retries
 // radio.setRetries(15,15);

 // optionally, reduce the payload size. seems to
 // improve reliability
//  radio.setPayloadSize(4);

  // transmission settings:
//  radio.setPALevel(RF24_PA_MIN);
  radio->setChannel(108);  // Above most Wifi Channels

 //
 // Open pipes to other nodes for communication
 //

 // This simple sketch opens two pipes for these two nodes to communicate
 // back and forth.
 // Open 'our' pipe for writing
 // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

 
 radio->openWritingPipe(pipes[1]);
 radio->openReadingPipe(1,pipes[0]);
 
 //
 // Start listening
 //

// radio.startListening();

 //
 // Dump the configuration of the rf unit for debugging
 //

 radio->printDetails();

}




// From https://github.com/ForceTronics/nRF24L01_Wireless_Sensor_Dev_Board/blob/master/WSNode.cpp :
//This function uses the known internal reference value of the 328p (~1.1V) to calculate the VCC value which comes from a battery
//This was leveraged from a great tutorial found at https://code.google.com/p/tinkerit/wiki/SecretVoltmeter?pageId=110412607001051797704
float getBatteryVoltage()
{
  analogReference(EXTERNAL); //set the ADC reference to AVCC 
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  unsigned long start = millis(); //get timer value
  while ( (start + 3) > millis()); //delay for 3 milliseconds
  ADCSRA |= _BV(ADSC); // Start ADC conversion
  while (bit_is_set(ADCSRA,ADSC)); //wait until conversion is complete
  int result = ADCL; //get first half of result
  result |= ADCH<<8; //get rest of the result
  float batVolt = (IREF / result)*1024; //Use the known iRef to calculate battery voltage
  analogReference(INTERNAL); //set the ADC reference back to internal
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change 
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
//    Serial.println("OK");
    return 0;
    case DHTLIB_ERROR_CHECKSUM: 
    Serial.println("DHT11 - Checksum error"); 
    return 1;
    case DHTLIB_ERROR_TIMEOUT: 
    Serial.println("DHT11 - Time out error"); 
    return 2;
    default: 
    Serial.println("DHT11 - Unknown error"); 
    return 3;
  }

  return 4;
}

void printValues(int DHT11_State, struct sensorData myData)
{
  char cDegreeSymbol = 176;
  Serial.println("\n");

// Print the Temperature and Humidity values:
  if (DHT11_State == 0)
  {
    Serial.print("Humidity:    ");
    Serial.print((float) myData.humidity , 2);
    Serial.println("%");
  
    Serial.print("Temperature: ");
    Serial.print((float) myData.temperature , 2);
    Serial.print(cDegreeSymbol);
    Serial.println("C");
    
  }
  else
  {
    switch (DHT11_State)
    {
    case DHTLIB_ERROR_CHECKSUM: 
      Serial.println("No Humidity and Temperature data - Checksum error"); 
    case DHTLIB_ERROR_TIMEOUT: 
      Serial.println("No Humidity and Temperature data - Time out error"); 
    default: 
      Serial.println("No Humidity and Temperature data - Unknown error"); 
    }
  }

// Print the light value
  if(myData.brightness < LIGHT_THRESHOLD)
  {
    Serial.print("It is night (");
    Serial.print(myData.brightness);
    Serial.print(")\n");
  }
  else
  {
    Serial.print("It is day   (");
    Serial.print(myData.brightness);
    Serial.print(")\n");
  }


  // Print the moisture value
  if(myData.moisture < MOISTURE_THRESHOLD)
  {
    Serial.print("The plant doesn't need watering (");
    Serial.print(myData.moisture);
    Serial.println(")");
  }
  else {
    Serial.print("It is time to water the plant   (");
    Serial.print(myData.moisture);
    Serial.println(")");
  }

  Serial.print("Voltage: ");
  Serial.println(myData.voltage);
  return;
}



void RF_action(RF24* radio)
{
//    unsigned long time[8];
    unsigned long time;
//    int newIntervall;


  //
  // Send the data
  //
time = 134;

  
  // First, stop listening so we can talk.
    radio->stopListening();
  
    // Take the time, and send it. This will block until complete
    Serial.print("Sending measurement data...");
//    radio.write( &time, sizeof(unsigned long) );
    radio->write(&myData, sizeof(struct sensorData));
    
  delay(10);   
    // Now, continue listening
    radio->startListening();


    
    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( (radio->available() == false) && ! timeout )
    {
      if (millis() - started_waiting_at > 200 )
      {
        timeout = true;
      }
    }
  
    // Describe the results
    if ( timeout )
    {
      Serial.println("Failed, response timed out.");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      unsigned long got_time;
      got_time = millis();
      radio->read( &myData.interval, sizeof(int) );
      
      // Spew it
      //printf("Got response %lu, round-trip delay: %lu\n\r",got_time,millis()-got_time);
      Serial.print("Got response ");
      Serial.print(myData.interval);
      Serial.print(", round-trip delay: ");
      Serial.println(millis()-started_waiting_at);
//      Serial.println("Got response %lu, round-trip delay: %lu",myData.interval,millis()-started_waiting_at);
    }
    
  

}

