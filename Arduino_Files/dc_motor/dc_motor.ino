#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int encoder_pin = 1;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

void setup() {
  pinMode(GPIO_PIN1, INPUT); //set XSHUT as OUTPUT
  digitalWrite(GPIO_PIN1, LOW); //reset XSHUT of first laser

  Serial.begin(115200); //set baud rate to max to maximize rs232 -> sensor speed
  AFMS.begin();
    
}

void loop() {
  myMotor->setSpeed(100); 
  myMotor->run(FORWARD);
}
