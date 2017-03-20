#include <SoftwareSerial.h>// import the serial library

SoftwareSerial mySerial(12, 13); // RX, TX
long previousMillis = 0;        // will store last time LED was updated

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 10000;           // interval at which to blink (milliseconds)
long Counter=0; // counter will increase every 1 second
void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600);
  mySerial.println("Bluetooth On");
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;  
     Counter+=1;
     delay(50);
  
    mySerial.println(Counter);
  }
}
