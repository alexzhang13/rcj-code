#define GPIO_PIN1 4
#define GPIO_PIN2 5  //laser number: 3 
#define GPIO_PIN3 7 //laser number: 2
#define GPIO_PIN4 6 //laser number: 4


#include <Wire.h>
#include <VL6180X.h> //tof short range polulu library
#include <VL53L0X.h> //tof long range polulu library

VL6180X laser1; //init laser var1 SHORT
VL6180X laser2; //init laser var2 SHORT
VL53L0X laser3; //init laser var3 LONG
VL53L0X laser4; //init laser var4 LONG

bool type = false; //true for long range, false for short

unsigned long curr_time;
int i=0; //init iteration loop var

void setup() {

    pinMode(GPIO_PIN1, OUTPUT); //set XSHUT as OUTPUT
    pinMode(GPIO_PIN2, OUTPUT); //set XSHUT of second laser as OUTPUT
    pinMode(GPIO_PIN3, OUTPUT); //set XSHUT of third laser as OUTPUT
    pinMode(GPIO_PIN4, OUTPUT); //set XSHUT of fourth laser as OUTPUT
    digitalWrite(GPIO_PIN1, LOW); //reset XSHUT of first laser
    digitalWrite(GPIO_PIN2, LOW); //reset XSHUT of second laser
    digitalWrite(GPIO_PIN3, LOW); //reset XSHUT of third laser
    digitalWrite(GPIO_PIN4, LOW); //reset XSHUT of fourth laser
  
    Serial.begin(115200); //set baud rate to max to maximize rs232 -> sensor speed
    Wire.begin(); //init i2c as master

    Serial.println("Dual Dual Distance Sensors");
  
    digitalWrite(GPIO_PIN1, HIGH); //begin writing to XSHUT of first laser
    delay(50); //delay
    laser1.init(); //init laser object, look for it
    laser1.configureDefault(); //laser config
    laser1.setTimeout(500); //in case you can't find the laser object, timeout for this long
    laser1.setAddress(0x25);
    delay(500); //delay

    digitalWrite(GPIO_PIN2, HIGH); //begin writing to XSHUT of first laser
    delay(50); //delay
    laser2.init(); //init laser object, look for it
    laser2.configureDefault(); //laser config
    laser2.setTimeout(500); //in case you can't find the laser object, timeout for this long
    laser2.setAddress(0x26);
    delay(500); //delay

    digitalWrite(GPIO_PIN3, HIGH); //begin writing to XSHUT of first laser
    delay(50); //delay
    laser3.init(); //init laser object, look for it
    //laser3.configureDefault(); //laser config
    laser3.setTimeout(500); //in case you can't find the laser object, timeout for this long
    laser3.setAddress(0x27);
    delay(500); //delay

   /* digitalWrite(GPIO_PIN4, HIGH); //begin writing to XSHUT of first laser
    delay(50); //delay
    laser4.init(); //init laser object, look for it
    //laser4.configureDefault(); //laser config
    Serial.println("Hi");
    laser4.setTimeout(500); //in case you can't find the laser object, timeout for this long
    laser4.setAddress(0x28);*/
  
}

void loop() {

  if(type == false)
  {
    if(i<10000)
      {
         curr_time = millis(); //get the current time
         Serial.print("l1 "); Serial.print(laser1.readRangeSingleMillimeters()); //label with l1 and get the laser reading (mm)
         if (laser1.timeoutOccurred()) 
         { Serial.print(" TIMEOUT"); } //if the laser isn't found, print TIMEOUT

         Serial.print(" t ");Serial.print(curr_time); //label with t and print out the current time
         Serial.println(); //start new line
        i++;
      }
  }
  else
  {
    if(i<10000)
      {
         curr_time = millis(); //get the current time
         Serial.print("l2 "); Serial.print(laser3.readRangeSingleMillimeters()); //label with l1 and get the laser reading (mm)
         if (laser3.timeoutOccurred()) 
         { Serial.print(" TIMEOUT"); } //if the laser isn't found, print TIMEOUT

         Serial.print(" t ");Serial.print(curr_time); //label with t and print out the current time
         Serial.println(); //start new line
        i++;
      }
  }
}



