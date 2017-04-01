#define L_ENCODER_A 6
#define L_ENCODER_B 7
#define R_ENCODER_A 12
#define R_ENCODER_B 13

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //motorshield object

Adafruit_DCMotor *myMotor = AFMS.getMotor(1); //left
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); //right

volatile unsigned long leftCount = 0; //left encoder
volatile unsigned long rightCount = 0; //right encoder
unsigned long curr_time;
unsigned long init_time;
int i = 0;

void setup() {
  pinMode(L_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(L_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(R_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(R_ENCODER_B, INPUT); //set encoderB as INPUT

  attachInterrupt(0, leftEncoderEvent, CHANGE);
  //attachInterrupt(1, rightEncoderEvent, CHANGE);
  
  Serial.begin(115200);
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
  if(i = 0)
  {
    init_time = millis();
  }
  curr_time = millis();
  while((curr_time - init_time) >= 5000)
  {
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
  }
  myMotor->run(BACKWARD);
  myMotor2->run(BACKWARD);
  i++;
  
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
