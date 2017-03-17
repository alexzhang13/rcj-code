#define L_ENCODER_A 11
#define L_ENCODER_B 12
#define R_ENCODER_A 9
#define R_ENCODER_B 10

#define GPIO_PIN1 4  //laser number: 1 pin #3
#define GPIO_PIN2 5  //laser number: 3 
#define GPIO_PINL1 6 //laser number: 2
#define GPIO_PINL2 7 //laser number: 4*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //motorshield object

Adafruit_DCMotor *myMotor = AFMS.getMotor(3); //is backwards, oriention that is
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);

volatile unsigned long leftCount = 0; //left encoder
volatile unsigned long rightCount = 0; //right encoder

void setup() {
  pinMode(L_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(L_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(R_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(R_ENCODER_B, INPUT); //set encoderB as INPUT

  // initialize hardware interrupts
  attachInterrupt(3, leftEncoderEvent, CHANGE);
  attachInterrupt(4, rightEncoderEvent, CHANGE);

  Serial.begin(9600);
  Serial.println("DC Motor Test");
  delay(50);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  myMotor->setSpeed(150);
  myMotor2->setSpeed(150);
  
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
}

void loop() {
  uint8_t i;

  //ON this bot the motors are flipped, forwards is backwards
  
  myMotor->run(BACKWARD);
  myMotor2->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);  
    myMotor2->setSpeed(i);
    Serial.print("Right: "); Serial.println(rightCount);
    Serial.print("Left: "); Serial.println(leftCount);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    myMotor2->setSpeed(i);  
    Serial.print("Right: "); Serial.println(rightCount);
    Serial.print("Left: "); Serial.println(leftCount);
    delay(10);
  }

  myMotor->run(FORWARD);
  myMotor2->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    myMotor2->setSpeed(i);  
    Serial.print("Right: "); Serial.println(rightCount);
    Serial.print("Left: "); Serial.println(leftCount);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i); 
    myMotor2->setSpeed(i); 
    Serial.print("Right: "); Serial.println(rightCount);
    Serial.print("Left: "); Serial.println(leftCount);
    delay(10);
  }

  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
  delay(1000);
}

// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(L_ENCODER_A) == HIGH) {
    if (digitalRead(L_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(L_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}
 
// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(R_ENCODER_A) == HIGH) {
    if (digitalRead(R_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(R_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}
