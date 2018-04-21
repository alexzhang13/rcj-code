/*DIGITAL PINS*/
#define R_ENCODER_A 3
#define R_ENCODER_B 8
#define L_ENCODER_A 2
#define L_ENCODER_B 11

#define GPIO_PIN1 4  //laser number: 1 pin #3
#define GPIO_PIN2 5  //laser number: 3 
#define GPIO_PINL1 6 //laser number: 2
#define GPIO_PINL2 7 //laser number: 4

#define SERVO_DROPPER 10
#define SERVO_MOUNT 9

#define LED_PIN 12

/*ANALOG PINS*/
#define PHOTOCELL 0

/*IMU*/
#define MPU9250_ADDRESS            0x68
#define GYRO_FULL_SCALE_250_DPS    0x00
#define ACC_FULL_SCALE_2_G        0x00

#define SMPLRT_DIV        0x19
#define ACCEL_CONFIG2     0x1D
#define CONFIG            0x1A
#define FIFO_EN           0x23

#include <Adafruit_MotorShield.h>
#include <QueueArray.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Tpa81.h>
#include <VL6180X.h> //tof short range polulu library
#include <VL53L0X.h> //tof long range polulu library
#include <Wire.h>
#include "pt.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Initialize Motorshield

Adafruit_DCMotor *motorRight = AFMS.getMotor(2); //Right is M3
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1); //Left is M4

VL6180X laserA_s; //init laser var1 SHORT
VL6180X laserB_s; //init laser var2 SHORT
VL53L0X laserA_l; //init laser var3 LONG
VL53L0X laserB_l; //init laser var4 LONG
Tpa81 tempA(1); //Temperature Sensor (Left)
Tpa81 tempB(2); //Temperature Sensor (Right)

Servo dropper; //init dropper RC Servo
Servo mount_laser; //init laser mount RC Servo

static struct pt drop_pt, dcmotor_pt, distance_pt, sensor_pt, imu_pt, write_pt; //Protothread Structs

String command = "";
String sensor_queue; //Queue for sensor commands (toggle)
String dropper_queue; //Queue for dropper commands
String motor_queue; //Queue for motor commands
String distance_queue; //Queue for distance commands
String imu_queue; //Queue for IMU commands

unsigned char tempReadingA[8];
unsigned char tempReadingB[8];

int16_t gyroX[20] = {0}; //Gyro Correction "Shifted" Array X
int16_t gyroY[20] = {0}; //Gyro Correction "Shifted" Array Y
int16_t gyroZ[20] = {0}; //Gyro Correction "Shifted" Array Z
int16_t g_avgX = 0; //Gyro Correction Avg of "Shifted" Array X
int16_t g_avgY = 0; //Gyro Correction Avg of "Shifted" Array Y
int16_t g_avgZ = 0; //Gyro Correction Avg of "Shifted" Array Z
int8_t __iter = 0; //Curr Iteration "Faux Shift"
bool isCalib = false; //Fire calibration y/n?

bool motorWait = false; //wait for motor thread
bool distanceWait = false; //wait for distance thread
bool dropperWait = false; //wait for dropper thread
bool sensorWait = false; //wait for sensor thread
bool sensorSwitch = true; //false meaning turn off
bool imuSwitch = true; //false meaning turn off
bool distanceSwitch = true; //false meaning turn off
bool motorSwitch = true; //false meaning turn off
bool isMoving = false; //if the robot is running
bool isTurning = false; //if the robot is turning
int left_spd = 100; //left motor power
int right_spd = 115; //right motor power
int off_left = 10; //for Encoder() function, offset when left encoder is higher than right
int off_right = 15; //for Encoder() function, offset when right encoder is higher than left
float distance_mm = 0;
volatile long int leftEncoder = 0; //left encoder
volatile long int rightEncoder = 0; //right encoder
volatile float left_mm = 0;
volatile float right_mm = 0;

void setup() {  
  /*RESET PINS AND WRITE*/
  pinMode(LED_PIN, OUTPUT);
  pinMode(L_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(L_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(R_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(R_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(GPIO_PIN1, OUTPUT); //set lasers as OUTPUT
  pinMode(GPIO_PIN2, OUTPUT); //set lasers as OUTPUT
  pinMode(GPIO_PINL1, OUTPUT); //set lasers as OUTPUT
  pinMode(GPIO_PINL2, OUTPUT); //set lasers as OUTPUT
  delay(100);

  digitalWrite(LED_PIN, LOW); //turn off LED
  digitalWrite(GPIO_PIN1, LOW); //reset XSHUT of first short laser
  digitalWrite(GPIO_PINL1, LOW); //reset XSHUT of first long laser
  digitalWrite(GPIO_PIN2, LOW); //reset XSHUT of second short laser
  digitalWrite(GPIO_PINL2, LOW); //reset XSHUT of second long laser
  delay(100);

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderEvent, CHANGE);    //Pins 2 and 3 are the only Interrupt Pins 2-3
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderEvent, CHANGE);
  delay(100);

  /*START UP SERIAL AND I2C WIRE*/
  Serial.begin(115200);
  Serial.setTimeout(50);
  Wire.begin();
  AFMS.begin();
  delay(100);

  Serial.println("Setup Complete.");

  pinMode(GPIO_PIN1, INPUT); //begin writing to XSHUT of first laser
  delay(50); //delay
  laserA_s.init(); //init laser object, look for it
  laserA_s.configureDefault(); //laser config
  laserA_s.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 36);
  laserA_s.setTimeout(0); //in case you can't find the laser object, timeout for this long
  laserB_s.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 52);
  laserA_s.setAddress(0x24);
  laserA_s.stopContinuous();
  delay(300);
  laserA_s.startRangeContinuous();
  delay(100); //delay
  Serial.println("Lasers Complete.");
  
  pinMode(GPIO_PIN2, INPUT);
  delay(50); //delay
  laserB_s.init(); //init laser object, look for it
  laserB_s.configureDefault(); //laser config
  laserB_s.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 36);
  laserB_s.setTimeout(0); //in case you can't find the laser object, timeout for this long
  laserB_s.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 52);
  laserB_s.stopContinuous();
  delay(300);
  laserB_s.startRangeContinuous();
  laserB_s.setAddress(0x25);
  delay(100); //delay

  pinMode(GPIO_PINL1, INPUT);
  delay(50); //delay
  laserA_l.init(); //init laser object, look for it
  laserA_l.setTimeout(0); //in case you can't find the laser object, timeout for this long
  laserA_l.stopContinuous();
  delay(300);
  laserA_l.startContinuous();
  laserA_l.setAddress(0x27);
  delay(100); //delay

  pinMode(GPIO_PINL2, INPUT);; //begin writing to XSHUT of first laser
  delay(50); //delay
  laserB_l.init(); //init laser object, look for it
  laserB_l.setTimeout(0); //in case you can't find the laser object, timeout for this long
  laserB_l.stopContinuous();
  delay(300);
  laserB_l.startContinuous();
  laserB_l.setAddress(0x28);
  delay(100); //delay
    
  /*INITIALIZE MPU9250 AND REQUEST BYTES*/
  //I2CwriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x18); //sample rate
  //I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
  delay(50);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  delay(50);
  
  //Configure Low-Pass Filter
  //I2CwriteByte(MPU9250_ADDRESS, 26, 0x04); //gyro
  //I2CwriteByte(MPU9250_ADDRESS, 29, 0x02);
  delay(50);

  Serial.println("I2C Complete.");
  
  /*START SENSORS*/
  mount_laser.attach(SERVO_MOUNT);   //Attach pin 10 to be for the laser mount
  mount_laser.write(0);
  mount_laser.detach();
  delay(500);
  dropper.attach(SERVO_DROPPER);    //Attach pin 9 to be for the dropper
  dropper.write(60);
  dropper.detach();
  delay(500);

  Serial.println("Servos Complete.");
  
  /*INITIALIZE MOTORS*/
  motorRight->setSpeed(right_spd);
  motorLeft->setSpeed(left_spd);

  motorRight->run(FORWARD);
  motorLeft->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);
  delay(50);

  Serial.println("Motors Complete.");

  /*START PROTOTHREADS*/
  PT_INIT(&drop_pt);  // Init Protothread for Dropper and LED
  PT_INIT(&dcmotor_pt);  // Init Protothread DC Motor
  PT_INIT(&distance_pt);  // Init Protothread Laser and Mount 
  PT_INIT(&sensor_pt);  // Init Protothread other Sensors(Temp, Light)
  PT_INIT(&imu_pt); //Init Protothread IMU
  delay(1000);
  
}

void loop() {
   if (Serial.available() > 0) {
      command = Serial.readString(); //Await command
      if(command == "m c") {
        Motor_Stop();
      }
      sortCommands(command);
      command = " ";
   }
        
   drop_pt_func(&drop_pt, 201);
   dcmotor_pt_func(&dcmotor_pt, 24);
   distance_pt_func(&distance_pt, 153);
   sensor_pt_func(&sensor_pt, 251);
   imu_pt_func(&imu_pt, 25);
}

static int drop_pt_func(struct pt *pt, int interval) { //10 hz = 100ms
  static unsigned long timestamp = 0;
  static char func;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!(dropper_queue == " ")) { //check dropper queue
      func = dropper_queue.charAt(0); //func char val
      dropper_queue.remove(0, 2); //leave only parameter left
      if(func == 'a') { 
        drop();
        dropper_queue = " ";
      } else if (func == 'b') {
        lightUp(dropper_queue.toInt());
        dropper_queue = " ";
      } else {
        Serial.println("Dropper: Invalid");
        dropper_queue = " ";
      }
    }
    
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int dcmotor_pt_func(struct pt *pt, int interval) { //50 hz = 20ms
  static unsigned long timestamp = 0;
  static char func;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!(motor_queue == " ")) {
      func = motor_queue.charAt(0); //func char val
      motor_queue.remove(0, 2); //leave only parameter left
      if(func == 'a') { 
         isMoving = true;
         Motor_Forward(motor_queue.toInt());
         motor_queue = " ";
      } else if (func == 'b') {
         isMoving = true;
         Motor_Backward(motor_queue.toInt());
         motor_queue = " ";
      } else if (func == 'c') {
         Motor_Stop();
         isMoving = false;
         isTurning = false;
         motor_queue = " ";
      } else if (func == 'd') {
         isTurning = true;
         Motor_TurnRight();
         motor_queue = " ";
      } else if (func == 'e') {
         isTurning = true;
         Motor_TurnLeft();
         motor_queue = " ";
      } else if (func == 'f'){
         int split = motor_queue.indexOf(' ');
         String split_str = motor_queue;
         motor_queue.remove(split);  
         split_str.remove(0, split+1);
         Motor_setSpeed(motor_queue.toInt(), split_str.toInt());
         motor_queue = " ";
      } else if (func == 'g') {
         resetEncoder();
         motor_queue = " ";
      } else if (func == 'h') {
         Turn_Small(motor_queue.toInt());
         motor_queue = " ";
      } else if (func == 'i') {
         int split = motor_queue.indexOf(' ');
         String split_str = motor_queue;
         motor_queue.remove(split);
         split_str.remove(0, split+1);
         Motor_setOffset(motor_queue.toInt(), split_str.toInt());
         motor_queue = " ";
      } else {
         Serial.println("Motor: Invalid");
         motor_queue = " ";
      }
    }
    if(isMoving == true && isTurning == false) {
         Motor_Encoder();
    } else if (isTurning == true) {
         Motor_Turn();
    }
    
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int distance_pt_func(struct pt *pt, int interval) { //125 hz = 8ms
  static unsigned long timestamp = 0;
  static char func;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!(distance_queue == " ")) {
      func = distance_queue.charAt(0); //func char val
      distance_queue.remove(0, 2); //leave only parameter left
      if(func == 'a') { 
         Mount_Sweep();
         distance_queue = " ";
      } else if (func == 'b') { //toggle on
         distanceSwitch = !distanceSwitch;
         distance_queue = " ";
      } else {
        Serial.println("Distance: Invalid");
        distance_queue = " ";
      }
    } else {
      if(distanceSwitch == true) {
        getDistanceReading();
      }
    }
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int sensor_pt_func(struct pt *pt, int interval) { //5 hz = 200ms
  static unsigned long timestamp = 0;
  static char func;
  PT_BEGIN(pt);
  while(1) {
    if(!(sensor_queue == " ")) {
      func = sensor_queue.charAt(0); //func char val
      sensor_queue.remove(0, 2); //leave only parameter left
      if(func == 'a') { //toggle sending data to the PI
        sensorSwitch = !sensorSwitch;
        sensor_queue = " ";
      } else {
        Serial.println("ERROR: FUNCTION IN SENSOR QUEUE HAS INVALID FUNCTION CALL (LETTER INVALID)");
        sensor_queue = " ";
      }
    }
    if(sensorSwitch == true) {
      getTempReading(); //Get temperature readings for both sensors
      getLightReading(); //get photocell reading
    }
    
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int imu_pt_func(struct pt *pt, int interval) { //125 hz = 8ms
  static unsigned long timestamp = 0;
  static char func;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!(imu_queue == " ")) {
      func = imu_queue.charAt(0); //func char val
      imu_queue.remove(0, 2); //leave only parameter left
      if(func == 'a') { 
         imuSwitch = !imuSwitch;
         imu_queue = " ";
      } else if (func == 'b') {
        isCalib = !isCalib;
        imu_queue = " ";
      } else {
        Serial.println("i e");
        imu_queue = " ";
      }
    } 
    if(imuSwitch) {
       getIMU();
    }
    if(isCalib) {
       calibrateIMU(); 
    }
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void sortCommands(String command)
{
    char type;
    type = command.charAt(0);
    if(type == 'r') { //enqueue laser command
       command.remove(0, 2); //remove label
       distance_queue = command; //enqueue distance command queue
       command = " "; //dequeue central queue 
    } else if (type == 'm') { //enqueue motor command
       command.remove(0, 2); //remove label
       motor_queue = command; //enqueue motor command queue
       command = " "; //dequeue central queue
    } else if (type == 'd') { //enqueue dropper/led command
       command.remove(0, 2); //remove label
       dropper_queue = command;
       command = " "; //dequeue central queue
    } else if (type == 's') {
       command.remove(0, 2); //remove label
       sensor_queue = command;
       command = " "; //dequeue central queue
    } else if (type == 'i') {
       command.remove(0, 2); //remove label
       imu_queue = command;
       command = " "; //dequeue central queue
    } else { //not supposed to happen but if it somehow does
       String reading = ""; reading += millis(); reading += " z Invalid Command -> Sent";
       Serial.println(reading);
       command = " "; //dequeue central queue
    }
}

void Motor_Forward(int distance) //Function for moving forward a certain distance
{
  distance_mm = distance;
  motorRight->setSpeed(right_spd);
  motorLeft->setSpeed(left_spd);
  motorRight->run(BACKWARD); //INVERTED
  motorLeft->run(BACKWARD);
}

void Motor_Backward(int distance)
{
  distance_mm = distance;
  motorRight->setSpeed(right_spd);
  motorLeft->setSpeed(left_spd);
  motorRight->run(FORWARD);    //Direction is inverted
  motorLeft->run(FORWARD);
}

void Motor_Stop()
{
  resetEncoder();
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);
}

void Motor_TurnRight() //Makes a x degree turn to the right
{
  motorRight->setSpeed(right_spd+15);
  motorLeft->setSpeed(left_spd+15);
  motorRight->run(BACKWARD);
  motorLeft->run(FORWARD);
}

void Motor_TurnLeft() //Makes a x degree turn to the left
{
  motorRight->setSpeed(right_spd+15);
  motorLeft->setSpeed(left_spd+15);
  motorRight->run(FORWARD);
  motorLeft->run(BACKWARD);
}
void Motor_setSpeed(int left_speed, int right_speed)
{
  left_spd = left_speed;
  right_spd = right_speed;
  motorLeft->setSpeed(left_spd);
  motorRight->setSpeed(right_spd);
}
void Motor_setOffset(int left_offset, int right_offset) {
  off_left = left_offset;
  off_right = right_offset;
}
void Motor_Encoder()
{
  String reading = "";
  if(abs(left_mm) >= distance_mm || abs(right_mm) >= distance_mm) {
    motor_queue = "c";
    reading += millis(); reading += "m d";
    Serial.println(reading);
    return;
  }
  if(abs(leftEncoder) < abs(rightEncoder)){
    motorRight->setSpeed(right_spd + off_right); //corecting
    motorLeft->setSpeed(left_spd);
  } else {
    motorRight->setSpeed(right_spd); //take 
    motorLeft->setSpeed(left_spd + off_left);
  }
  //reading += millis(); reading += " m ";
  //reading += left_mm; reading += " "; reading += right_mm;
  //Serial.println(reading);
}

void Motor_Turn() //Turning encoders
{
   if(abs(leftEncoder) < abs(rightEncoder)){
    motorRight->setSpeed(left_spd + off_right); //corecting
  } else {
    motorRight->setSpeed(left_spd + off_left); //take 
  }
}

void Turn_Small(int dir) {
  isTurning = true;
  if(dir == 0) { //turn left
    motorRight->setSpeed(120);
    motorLeft->setSpeed(100);
    Motor_TurnLeft();
    delay(1000);
    Motor_Stop();
    motorRight->setSpeed(right_spd);
    motorLeft->setSpeed(left_spd);
  } else { //turn right
    motorRight->setSpeed(120);
    motorLeft->setSpeed(100);
    Motor_TurnRight();
    delay(1000);
    Motor_Stop();
    motorRight->setSpeed(right_spd);
    motorLeft->setSpeed(left_spd);
  }
  isTurning = false;
}

void Mount_Sweep()
{
  String reading = "";
    // scan from 0 to 180 degrees
  mount_laser.attach(SERVO_MOUNT);
  reading += millis(); reading += " z";
  Serial.println(reading); //take a screenshot at this moment (tell pi
  delay(500);
  
  for(int angle = 0; angle <= 180; angle++)  
  {         
    mount_laser.write(angle);  
    delay(1);                       
  } 
  
  reading = "";
  reading += millis(); reading += " z";
  Serial.println(reading); //take a screenshot at this moment (tell pi)
  delay(500);

  // now scan back from 180 to 0 degrees                            
  for(int angle = 180; angle >= 0; angle--)  
  {         
    mount_laser.write(angle);       
    delay(1);                   
  }     
  delay(50);       
  mount_laser.detach();
}

void getDistanceReading()
{
  String reading = "";
  reading += millis(); reading += (" r "); reading += "0"; //add timestamp, laser label, angle = 0
  reading += " "; reading += laserA_l.readRangeContinuousMillimeters(); //laser 2 val (mm)
  reading += " "; reading += laserA_s.readRangeContinuousMillimeters(); //laser 1 val (mm)
  reading += " "; reading += laserB_l.readRangeContinuousMillimeters(); //laser 4 val (mm)m
  reading += " "; reading += laserB_s.readRangeContinuousMillimeters(); //laser 3 val (mm)
  Serial.println(reading);     
}

void calibrateIMU() {
  g_avgX=0; g_avgY=0; g_avgZ=0;
  for(int i=0; i<sizeof(gyroX)/sizeof(gyroX[0]); i++) {
    g_avgX += gyroX[i];
    g_avgY += gyroY[i];
    g_avgZ += gyroZ[i];
  }
  g_avgX /= sizeof(gyroX)/sizeof(gyroX[0]);
  g_avgY /= sizeof(gyroY)/sizeof(gyroY[0]);
  g_avgZ /= sizeof(gyroZ)/sizeof(gyroZ[0]);
  return;
}

void getIMU()
{
  uint8_t Buf[14];
  uint8_t ST1;
  String reading = "";
  
  reading += millis(); reading += " i ";
  
  //Accelerometer and Gyroscope
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Accelerometer
  int16_t ax=Buf[0]<<8 | Buf[1];
  int16_t ay=Buf[2]<<8 | Buf[3];
  int16_t az=Buf[4]<<8 | Buf[5];
 
  // Gyroscope
  int16_t gx=Buf[8]<<8 | Buf[9];
  int16_t gy=Buf[10]<<8 | Buf[11];
  int16_t gz=Buf[12]<<8 | Buf[13];

  //IF NOT MOVING ADJUST GYRO
  if(!isTurning) {
    /*Getting Drift When Not Moving*/
    if(!isMoving) {
      gyroX[__iter] = gx;
      gyroY[__iter] = gy;
      gyroZ[__iter] = gz;
      ++__iter;
      if(__iter>=sizeof(gyroX)/sizeof(gyroX[0])) 
        __iter=0; //reset
    }
    //Apply drift
    gx -= g_avgX;
    gy -= g_avgY;
    gz -= g_avgZ;
  }
  
  // Accelerometer
  reading += ax; reading += " "; reading += ay; reading += " "; reading += az; reading += " ";
  // Gyroscope
  if(isMoving || isTurning) {
    reading += gx; reading += " "; reading += gy; reading += " "; reading += gz; reading += " ";
  } else {
    reading += "0 0 0";
  }
  
  Serial.println(reading);
}

void drop() //drop a kit
{
  int angle = 60;
  dropper.attach(SERVO_DROPPER);    //Attach pin 9 to be for the dropper
  for(angle = 60; angle > 0; angle--)  
  {                                  
    dropper.write(angle);               
    delay(5);                   
  } 

  delay(500);

  for(angle = 0; angle < 100; angle++)    
  {                                
    dropper.write(angle);           
    delay(1);       
  } 
  delay(500);
  for(angle = 100; angle > 60; angle--)    
  {                                
    dropper.write(angle);           
    delay(5);       
  } 
  dropper.detach();    //Attach pin 9 to be for the dropper
  delay(500);
  Serial.println("d d");
}

void lightUp(int time_stamp) //milliseconds
{
  int curr_time = millis();
  time_stamp += curr_time;
  while(curr_time < time_stamp) {
    digitalWrite(LED_PIN, HIGH);   
    delay(100);                       
    digitalWrite(LED_PIN, LOW);
    delay(100);
    curr_time = millis();
  }
  Serial.println("l d");
}

void getLightReading() //photocell sensor/transistor
{
  String reading = "";
  reading += millis(); reading += " l ";
  reading += analogRead(PHOTOCELL); Serial.println(reading); //writeQueue();
}

void getTempReading() //temperature sensor(s)
{
  String reading = ""; 
  reading += millis(); reading += " t ";
  reading += tempA.getData(tempReadingA); reading += " ";
  for(int i = 0; i < sizeof(tempReadingA)/sizeof(char); i++)
  {
    reading += (int)tempReadingA[i];
    reading += " ";
  } 
  reading += tempB.getData(tempReadingB); reading += " ";
  for(int i = 0; i < sizeof(tempReadingB)/sizeof(char); i++)
  {
    reading += (int)tempReadingB[i];
    reading += " ";
  }
  reading.trim();
  Serial.println(reading); //writeQueue();
}
// encoder event for the interrupt call
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
  left_mm = leftEncoder*3.1415926535*71.5094666677/1806.96;
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
  right_mm = rightEncoder*3.1415926535*71.5094666677/1806.96;
}

void resetEncoder()
{
  rightEncoder = 0;
  leftEncoder = 0;
  left_mm = 0;
  right_mm = 0;
}


