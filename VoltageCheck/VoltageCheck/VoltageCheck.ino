void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initialized");

}

void loop() {
  // poll the voltage:
  float voltage = getBatteryVoltage();
  
  Serial.print("Voltage: ");
  Serial.println(voltage);

  delay(5000);
}

// From https://github.com/ForceTronics/nRF24L01_Wireless_Sensor_Dev_Board/blob/master/WSNode.cpp :
//This function uses the known internal reference value of the 328p (~1.1V) to calculate the VCC value which comes from a battery
//This was leveraged from a great tutorial found at https://code.google.com/p/tinkerit/wiki/SecretVoltmeter?pageId=110412607001051797704
float getBatteryVoltage()
{
  float iREF = 1.1;
  analogReference(EXTERNAL); //set the ADC reference to AVCC 
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  unsigned long start = millis(); //get timer value
  while ( (start + 3) > millis()); //delay for 3 milliseconds
  ADCSRA |= _BV(ADSC); // Start ADC conversion
  while (bit_is_set(ADCSRA,ADSC)); //wait until conversion is complete
  int result = ADCL; //get first half of result
  result |= ADCH<<8; //get rest of the result
  float batVolt = (iREF / result)*1024; //Use the known iRef to calculate battery voltage
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
