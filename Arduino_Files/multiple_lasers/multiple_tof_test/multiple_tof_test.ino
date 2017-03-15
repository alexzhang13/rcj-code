/* The range readings are in units of mm. */
#define GPIO_PIN1 4  //laser number: 1 pin #3
#define GPIO_PIN2 5  //laser number: 3 
#define GPIO_PINL1 6 //laser number: 2
#define GPIO_PINL2 7 //laser number: 4*/

#include <Wire.h>
#include <VL6180X.h> //tof short range polulu library
#include <VL53L0X.h> //tof long range polulu library

VL6180X laser1; //init laser var1 SHORT
VL6180X laser2; //init laser var2 SHORT
VL53L0X laser3; //init laser var3 LONG
VL53L0X laser4; //init laser var4 LONG

unsigned long curr_time;
int i = 0; //init iteration loop var

void setup()
{

  pinMode(GPIO_PIN1, OUTPUT); //set XSHUT as OUTPUT
  pinMode(GPIO_PIN2, OUTPUT); //set XSHUT of second laser as OUTPUT
  pinMode(GPIO_PINL1, OUTPUT); //set XSHUT of third laser as OUTPUT
  pinMode(GPIO_PINL2, OUTPUT); //set XSHUT of fourth laser as OUTPUT
  digitalWrite(GPIO_PIN1, LOW); //reset XSHUT of first laser
  digitalWrite(GPIO_PIN2, LOW); //reset XSHUT of second laser
  digitalWrite(GPIO_PINL1, LOW); //reset XSHUT of third laser
  digitalWrite(GPIO_PINL2, LOW); //reset XSHUT of fourth laser

  Serial.begin(115200); //set baud rate to max to maximize rs232 -> sensor speed
  Wire.begin(); //init i2c as master

  Serial.println("Dual Dual Distance Sensors");

  digitalWrite(GPIO_PIN1, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laser1.init(); //init laser object, look for it
  laser1.configureDefault(); //laser config
  laser1.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laser1.setAddress(0x25);

  digitalWrite(GPIO_PINL1, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laser3.init(); //init laser object, look for it
  //laser3.configureDefault(); //laser config
  laser3.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laser3.setAddress(0x27);

  digitalWrite(GPIO_PIN2, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laser2.init(); //init laser object, look for it
  laser2.configureDefault(); //laser config
  laser2.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laser2.setAddress(0x26);

  digitalWrite(GPIO_PINL2, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laser4.init(); //init laser object, look for it
  //laser4.configureDefault(); //laser config
  laser4.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laser4.setAddress(0x28);
}

void loop()
{
  if (i == 0)
  {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++ )
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("  !");

        nDevices++;
      }
      else if (error == 4)
      {
        Serial.print("Unknown error at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.println(address, HEX);
      }
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");

    delay(5000);           // wait 5 seconds for next scan
  }
  if (i < 10000)
  {
    curr_time = millis(); //get the current time
    Wire.beginTransmission(37); //0x25 address for laser1
    Serial.print("l1 "); Serial.print(laser1.readRangeSingleMillimeters()); //label with l1 and get the laser reading (mm)
    if (laser1.timeoutOccurred())
    {
      Serial.print(" TIMEOUT");  //if the laser isn't found, print TIMEOUT
    }
    Wire.endTransmission(); //end transmission to laser1*/

    Wire.beginTransmission(38); //0x26 address for laser2
    Serial.print(" l2 ");  Serial.print(laser2.readRangeSingleMillimeters()); //label with l2 and get the laser reading (mm)
    if (laser2.timeoutOccurred())
    {
      Serial.print(" TIMEOUT");  //if the laser isn't found, print TIMEOUT
    }
    Wire.endTransmission(); //end transmission to laser2

    Wire.beginTransmission(39); //0x27 address for laser3
    Serial.print(" l3 ");  Serial.print(laser3.readRangeSingleMillimeters()); //label with l3 and get the laser reading (mm)
    if (laser3.timeoutOccurred())
    {
      Serial.print(" TIMEOUT");  //if the laser isn't found, print TIMEOUT
    }
    Wire.endTransmission(); //end transmission to laser2

    Wire.beginTransmission(40); //0x28 address for laser4
    Serial.print(" l4 ");  Serial.print(laser4.readRangeSingleMillimeters()); //label with l4 and get the laser reading (mm)
    if (laser4.timeoutOccurred())
    {
      Serial.print(" TIMEOUT");  //if the laser isn't found, print TIMEOUT
    }
    Wire.endTransmission(); //end transmission to laser4*/

    Serial.print(" t "); Serial.print(curr_time); //label with t and print out the current time
    Serial.println(); //start new line
    i++;
  }
}
