const int transistorPin = 3;    // connected to the base of the transistor
 //const int transistorPin = 9;
 void setup() {
   // set  the transistor pin as output:
   pinMode(transistorPin, OUTPUT);
 }
 
 void loop() {
   digitalWrite(transistorPin, HIGH);
   delay(2000);
   digitalWrite(transistorPin, LOW);
   delay(5000);
 }
