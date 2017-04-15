//Counts per Rev: 1806.96     Radius: 19.5mm
//mm per rev: counts * 2 * pi * r

/*DIGITAL PINS*/
#define R_ENCODER_A 3
#define R_ENCODER_B 8
#define L_ENCODER_A 2
#define L_ENCODER_B 11

#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Initialize Motorshield

Adafruit_DCMotor *motorRight = AFMS.getMotor(2); //Right is M3
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1); //Left is M4

volatile long int leftEncoder = 0; //left encoder
volatile long int rightEncoder = 0; //right encoder
float left_mm = 0;
float right_mm = 0;

void setup() {
  pinMode(L_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(L_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(R_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(R_ENCODER_B, INPUT); //set encoderB as INPUT
  delay(100);
   // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderEvent, CHANGE);    //Pins 2 and 3 are the only Interrupt Pins 2-3
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderEvent, CHANGE);
  delay(100);
  
  /*START UP SERIAL AND I2C WIRE*/
  Serial.begin(115200);
  Wire.begin();
  AFMS.begin();  // create with the default frequency 1.6KHz
  delay(100);
  
  /*INITIALIZE MOTORS*/
  motorRight->setSpeed(55);
  motorLeft->setSpeed(50);

  motorRight->run(FORWARD);
  motorLeft->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);
  delay(100);
  Motor_Forward();
}

void loop() {
  Motor_Encoder();
  left_mm = leftEncoder*3.1415926535*71.5094666677/1806.96;
  right_mm = rightEncoder*3.1415926535*71.5094666677/1806.96;
  if(left_mm >= 300)
  {
    Motor_Stop();
  }  
  if(leftEncoder < rightEncoder){
    motorRight->setSpeed(55);
  } else {
    motorRight->setSpeed(60);
  }
}

void Motor_Encoder()
{
  String reading = "";
  reading += millis(); reading += " m ";
  reading += left_mm; reading += " "; reading += right_mm;
  Serial.println(reading);
}

void Motor_Forward() //Function for moving forward a certain distance
{
  motorRight->run(BACKWARD); //INVERTED
  motorLeft->run(BACKWARD);
}

void Motor_Backward()
{
  motorRight->run(FORWARD);    //Direction is inverted
  motorLeft->run(FORWARD);
}

void Motor_Stop()
{
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);
}

void Motor_TurnRight() //Makes a x degree turn to the right
{
  motorRight->run(BACKWARD);
  motorLeft->run(FORWARD);
}

void Motor_TurnLeft() //Makes a x degree turn to the left
{
  motorRight->run(FORWARD);
  motorLeft->run(BACKWARD);
}
void Motor_setSpeed(int left_speed, int right_speed)
{
  motorRight->setSpeed(right_speed);
  motorLeft->setSpeed(left_speed);
}

void leftEncoderEvent() {
  if (digitalRead(L_ENCODER_A) == HIGH) {
    if (digitalRead(L_ENCODER_B) == LOW) {
      leftEncoder++;
    } else {
      leftEncoder--;
    }
  } else {
    if (digitalRead(L_ENCODER_B) == LOW) {
      leftEncoder--;
    } else {
      leftEncoder++;
    }
  }
}

// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(R_ENCODER_A) == HIGH) {
    if (digitalRead(R_ENCODER_B) == LOW) {
      rightEncoder++;
    } else {
      rightEncoder--;
    }
  } else {
    if (digitalRead(R_ENCODER_B) == LOW) {
      rightEncoder--;
    } else {
      rightEncoder++;
    }
  }
}

void resetEncoder()
{
  rightEncoder = 0;
  leftEncoder = 0;
}

