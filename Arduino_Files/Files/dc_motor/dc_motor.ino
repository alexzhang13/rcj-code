#define L_ENCODER_A 2 //INTERRUPT
#define L_ENCODER_B 11 //PWM
#define R_ENCODER_A 3 //PWM && INTERRUPT
#define R_ENCODER_B 8

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //motorshield object

Adafruit_DCMotor *myMotor = AFMS.getMotor(1); //left
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); //right

volatile long int leftCount = 0; //left encoder
volatile long int rightCount = 0; //right encoder

void setup() {
  pinMode(L_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(L_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(R_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(R_ENCODER_B, INPUT); //set encoderB as INPUT

  attachInterrupt(0, leftEncoderEvent, CHANGE);
  attachInterrupt(1, rightEncoderEvent, CHANGE);
  
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
  leftCount = 0;
  rightCount = 0;
}

void loop() {
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
  Serial.print(millis());
  Serial.print("   Left: "); Serial.print(leftCount);
  Serial.print("   Right: "); Serial.println(rightCount);
  
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
