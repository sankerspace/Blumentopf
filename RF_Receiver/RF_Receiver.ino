#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

struct sensorData
{
  float temperature;
  float humidity;
  int moisture;
  int brightness;
  int interval = 2;
  float voltage;
};


RF24 radio(9, 10);
const uint64_t pipes[3] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xE8E8F0F0E1LL}; // pipe[0] ist answer-channel


void setup(void)
{
  Serial.begin(9600);
  radio.begin();
//  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(108);  // Above most Wifi Channels

//  radio.setPayloadSize(8);
  radio.openReadingPipe(1, pipes[1]);
  radio.openWritingPipe(pipes[0]);
  radio.startListening();
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println(sizeof(struct sensorData));
}

void loop(void)
{
//  uint8_t i;
  struct sensorData myData;
  int intervall;

  intervall = 10;
  
  if (radio.available() == true)
  {
    Serial.println("Message available");



    radio.read(&myData, sizeof(struct sensorData));

    Serial.print("Temp: ");
    Serial.print(myData.temperature);
    Serial.print(", Humidity: ");
    Serial.print(myData.humidity);
    Serial.print(", Moisture: ");
    Serial.print(myData.moisture);
    Serial.print(", Brightness: ");
    Serial.println(myData.brightness);
    Serial.print("Interval: ");
    Serial.print(myData.interval);
    Serial.print(", Voltage: ");
    Serial.println(myData.voltage);
    
    
    radio.stopListening();
    delay(100);   // ist das delay notwendig?
// Send back next sleep intervall:
    radio.write(&intervall, sizeof(int));
    
    delay(100);   // ist das delay notwendig?
    radio.startListening();
    delay(100);   // ist das delay notwendig?


    digitalWrite(LED_BUILTIN, LOW);


    delay(10);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    //Serial.println("nothing yet..");  

  }
  //delay(1000);
}
