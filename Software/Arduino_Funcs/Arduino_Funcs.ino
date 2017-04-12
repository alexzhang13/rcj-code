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
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MotorShield.h>
#include <VL6180X.h> //tof short range polulu library
#include <VL53L0X.h> //tof long range polulu library
#include <Servo.h>
#include <QueueArray.h>
#include "pt.h"
#include "Adafruit_TMP007.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Initialize Motorshield

Adafruit_DCMotor *motorRight = AFMS.getMotor(1); //Right is M3
Adafruit_DCMotor *motorLeft = AFMS.getMotor(2); //Left is M4

VL6180X laserA_s; //init laser var1 SHORT
VL6180X laserB_s; //init laser var2 SHORT
VL53L0X laserA_l; //init laser var3 LONG
VL53L0X laserB_l; //init laser var4 LONG

Adafruit_TMP007 tempA; //init temp_sensor var 1
Adafruit_TMP007 tempB; //init temp_sensor var 1

Servo dropper; //init dropper RC Servo
Servo mount_laser; //init laser mount RC Servo

static struct pt drop_pt, dcmotor_pt, distance_pt, sensor_pt; //Protothread Structs

char received;
String command = "";
QueueArray <String> write_queue; //Queue for writing commands to the PI
QueueArray <String> command_queue; //Queue for receiving all commands
QueueArray <String> sensor_queue; //Queue for sensor commands (toggle)
QueueArray <String> dropper_queue; //Queue for dropper commands
QueueArray <String> motor_queue; //Queue for motor commands
QueueArray <String> distance_queue; //Queue for distance commands

bool motorWait = false; //wait for motor thread
bool distanceWait = false; //wait for distance thread
bool dropperWait = false; //wait for dropper thread
bool sensorWait = false; //wait for sensor thread
bool sensorSwitch = true; //false meaning turn off
bool imuSwitch = true; //false meaning turn off
bool distanceSwitch = true; //false meaning turn off
bool motorSwitch = true; //false meaning turn off
bool isMoving = false; //if the robot is running
int photocellReading;     // the analog reading from the sensor divider
int mount_angle = 0; //Current mount angle (calibrated)
int dropper_angle = 0; //Current dropper angle (Usually between 3 values)
int left_speed = 150;
int right_speed = 150;
volatile long int leftEncoder = 0; //left encoder
volatile long int rightEncoder = 0; //right encoder
int i = 0; //command interation

void setup() {

  
  /*START PROTOTHREADS*/
  PT_INIT(&drop_pt);  // Init Protothread for Dropper and LED (10 hz)
  PT_INIT(&dcmotor_pt);  // Init Protothread DC Motor (50 hz)
  PT_INIT(&distance_pt);  // Init Protothread Laser and Mount (120 hz)
  PT_INIT(&sensor_pt);  // Init Protothread other Sensors(Temp, Light) (5 hz)
  delay(1000);
  
  /*RESET PINS AND WRITE*/
  pinMode(LED_PIN, OUTPUT);
  pinMode(L_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(L_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(R_ENCODER_A, INPUT); //set encoderA as INPUT
  pinMode(R_ENCODER_B, INPUT); //set encoderB as INPUT
  pinMode(GPIO_PIN1, INPUT); //set encoderA as INPUT
  pinMode(GPIO_PIN2, INPUT); //set encoderB as INPUT
  pinMode(GPIO_PINL1, INPUT); //set encoderA as INPUT
  pinMode(GPIO_PINL2, INPUT); //set encoderB as INPUT
  delay(100);

  digitalWrite(GPIO_PIN1, LOW); //reset XSHUT of first short laser
  digitalWrite(GPIO_PINL1, LOW); //reset XSHUT of first long laser
  digitalWrite(GPIO_PIN2, LOW); //reset XSHUT of second short laser
  digitalWrite(GPIO_PINL2, LOW); //reset XSHUT of second long laser
  digitalWrite(SERVO_DROPPER, LOW); //reset XSHUT of second laser
  digitalWrite(SERVO_MOUNT, LOW); //reset XSHUT of third laser
  delay(100);

  /*START SENSORS*/
  mount_laser.attach(SERVO_MOUNT);   //Attach pin 10 to be for the laser mount
  //dropper.attach(SERVO_DROPPER);    //Attach pin 9 to be for the dropper
  mount_laser.write(0);
  //dropper.write(60);
  delay(500);

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderEvent, CHANGE);    //Pins 2 and 3 are the only Interrupt Pins 2-3
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderEvent, CHANGE);
  delay(100);

  /*START UP SERIAL AND I2C WIRE*/
  Serial.begin(115200);
  Wire.begin();
  AFMS.begin();  // create with the default frequency 1.6KHz
  delay(100);

  /*INITIALIZE MPU9250 AND REQUEST BYTES*/
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  delay(500);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  delay(500);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  delay(500);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  delay(500);
  
  /*INITIALIZE MOTORS*/
  motorRight->setSpeed(right_speed);
  motorLeft->setSpeed(left_speed);

  motorRight->run(FORWARD);
  motorLeft->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);
  delay(100);
  
  /*INITIALIZE ALL SENSORS*/
  if (! tempA.begin()) { //look for tmp007 sensor
    Serial.println("No temperature sensor found");
    while (1);
  }
  delay(100);
  if (! tempB.begin()) { //look for tmp007 sensor
    Serial.println("No temperature sensor found");
    while (1);
  }
  delay(100);
  
  digitalWrite(GPIO_PIN1, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laserA_s.init(); //init laser object, look for it
  laserA_s.configureDefault(); //laser config
  laserA_s.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  laserA_s.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  laserA_s.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laserA_s.setAddress(0x25);
  laserA_s.stopContinuous();
  delay(500);
  laserA_s.startInterleavedContinuous(100);

  digitalWrite(GPIO_PINL1, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laserA_l.init(); //init laser object, look for it
  laserA_l.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laserA_l.stopContinuous();
  delay(500);
  laserA_l.startContinuous();
  laserA_l.setAddress(0x27);
  
  digitalWrite(GPIO_PIN2, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laserB_s.init(); //init laser object, look for it
  laserB_s.configureDefault(); //laser config
  laserB_s.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  laserB_s.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  laserB_s.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laserB_s.stopContinuous();
  delay(500);
  laserB_s.startInterleavedContinuous();
  laserB_s.setAddress(0x26);

  digitalWrite(GPIO_PINL2, HIGH); //begin writing to XSHUT of first laser
  delay(50); //delay
  laserB_l.init(); //init laser object, look for it
  laserB_l.setTimeout(500); //in case you can't find the laser object, timeout for this long
  laserB_l.stopContinuous();
  delay(500);
  laserB_l.startContinuous();
  laserB_l.setAddress(0x28);
  Serial.println("Test");
}

void loop() {
   command = ""; //clear receieved buffer
   while (Serial.available() > 0) {
      command = Serial.readString(); //Await command
      Serial.println(command);
      command_queue.push(command);
      command = ""; // Clear recieved buffer
      }
   if(!command_queue.isEmpty()) {
      Serial.println("Queue is sorting...");
      sortCommands(); //sort queues into each respective protothread queue to check each function
   }
   if(!write_queue.isEmpty()) {
      writeQueue(); //write to the PI if there is a writing queue
   }

   drop_pt_func(&drop_pt, 50);
   dcmotor_pt_func(&dcmotor_pt, 20);
   distance_pt_func(&distance_pt, 100);
   sensor_pt_func(&sensor_pt, 150);
}

static int drop_pt_func(struct pt *pt, int interval) { //10 hz = 100ms
  static unsigned long timestamp = 0;
  static char func;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!dropper_queue.isEmpty()) { //check dropper queue
      dropperWait = true;
      func = dropper_queue.peek().charAt(0); //func char val
      dropper_queue.peek().remove(0, 2); //leave only parameter left
      if(func == 'a') { 
        drop();
      } else if (func == 'b') {
        lightUp(5000);
      } else {
        Serial.println("ERROR: FUNCTION IN DROPPER QUEUE HAS INVALID FUNCTION CALL (LETTER INVALID)");
      }
      dropperWait = false;
    }
    
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval && dropperWait == false);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int dcmotor_pt_func(struct pt *pt, int interval) { //50 hz = 20ms
  static unsigned long timestamp = 0;
  static char func;
  static String command;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!motor_queue.isEmpty()) {
      motorWait = true;
      command = motor_queue.peek();
      func = command.charAt(0); //func char val
      Serial.println(func);
      command.remove(0, 2); //leave only parameter left
      if(func == 'a') { 
         Motor_Forward();
         isMoving = true;
         motor_queue.pop();
      } else if (func == 'b') {
         Motor_Backward();
         isMoving = true;
         motor_queue.pop();
      } else if (func == 'c') {
         Motor_Stop();
         isMoving = false;
         motor_queue.pop();
      } else if (func == 'd') {
         Motor_TurnRight();
         isMoving = true;
         motor_queue.pop();
      } else if (func == 'e') {
         Motor_TurnLeft();
         isMoving = true;
         motor_queue.pop();
      } else if (func == 'f'){
         int split = command.indexOf(' ');
         String split_str = command;
         command.remove(split);  
         split_str.remove(0, split+1);
         left_speed = command.toInt();
         right_speed = split_str.toInt();
         Motor_setSpeed(left_speed, right_speed);
         motor_queue.pop();
      } else {
         Serial.println("ERROR: FUNCTION IN MOTOR QUEUE HAS INVALID FUNCTION CALL (LETTER INVALID)");
         motor_queue.pop();
      }
      if(isMoving == true) {
         Motor_Encoder();
      }
      motorWait = false;
    }
    
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval && motorWait == false);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int distance_pt_func(struct pt *pt, int interval) { //125 hz = 8ms
  static unsigned long timestamp = 0;
  static char func;
  static String command;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!distance_queue.isEmpty()) {
      distanceWait = true;
      command = distance_queue.peek();
      func = command.charAt(0); //func char val
      Serial.println(func);
      command.remove(0, 2); //leave only parameter left
      if(func == 'a') { 
         Mount_Sweep();
         distance_queue.pop();
      } else if (func == 'b') { //toggle on
         if(distanceSwitch == true) {
           distanceSwitch = false; Serial.println("LASER TOGGLE OFF: Not sending data to the PI...");
         } else {
           distanceSwitch = true; Serial.println("LASER TOGGLE ON: Sending data to the PI...");
         }
         distance_queue.pop(); 
      } else if (func == 'c') {
         if(imuSwitch == true) {
           imuSwitch = false; Serial.println("IMU TOGGLE OFF: Not sending data to the PI...");
         } else {
           imuSwitch = true; Serial.println("IMU TOGGLE ON: Sending data to the PI...");
         }
      } else {
        Serial.println("ERROR: FUNCTION IN DISTANCE QUEUE HAS INVALID FUNCTION CALL (LETTER INVALID)");
        distance_queue.pop();
      }
      distanceWait = false;
    } else {
      if(distanceSwitch == true) {
        distanceWait = true;
        getDistanceReading();
      }
      if(imuSwitch == true) {
        distanceWait = true;
        getIMU();
      }
      distanceWait = false;
    }
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval && distanceWait == false);
    timestamp = millis(); // take a new timestamp
  }
  PT_END(pt);
}

static int sensor_pt_func(struct pt *pt, int interval) { //5 hz = 200ms
  static unsigned long timestamp = 0;
  static char func;
  PT_BEGIN(pt);
  while(1) { // never stop 
    if(!sensor_queue.isEmpty()) {
      sensorWait = true;
      func = sensor_queue.peek().charAt(0); //func char val
      sensor_queue.peek().remove(0, 2); //leave only parameter left
      if(func == 'a') { //toggle sending data to the PI
        if(sensorSwitch == true) {
           sensorSwitch = false; Serial.println("SENSOR TOGGLE OFF: Not sending data to the PI...");
        } else {
           sensorSwitch = true; Serial.println("SENSOR TOGGLE ON: Sending data to the PI...");
        }
      } else {
        Serial.println("ERROR: FUNCTION IN SENSOR QUEUE HAS INVALID FUNCTION CALL (LETTER INVALID)");
      }
      sensorWait = false;
    }
    if(sensorSwitch == true) {
      sensorWait = true;
      getTempReading(); //Get temperature readings for both sensors
      getLightReading(); //get photocell reading
      sensorWait = false;
    }
    
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval && sensorWait == false);
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

void sortCommands()
{
  char type;
  String command;
  while(!command_queue.isEmpty()) { //until the queue is completely empty
    type = command_queue.peek().charAt(0);
    command = command_queue.peek();
    Serial.println(type);
    if(type == 'r') { //enqueue laser command
       command.remove(0, 2); //remove label
       command_queue.front() = command;
       distance_queue.push(command); //enqueue distance command queue
       Serial.println(distance_queue.peek());
       command_queue.pop(); //dequeue central queue 
    } else if (type == 'm') { //enqueue motor command
       command.remove(0, 2); //remove label
       command_queue.front() = command;
       motor_queue.push(command); //enqueue motor command queue
       command_queue.pop(); //dequeue central queue 
    } else if (type == 'd') { //enqueue dropper/led command
       command.remove(0, 2); //remove label
       command_queue.front() = command;
       dropper_queue.push(command);
       command_queue.pop(); //dequeue central queue 
    } else if (type == 's') {
       command.remove(0, 2); //remove label
       command_queue.front() = command;
       sensor_queue.push(command);
       command_queue.pop(); //dequeue central queue 
    } else { //not supposed to happen but if it somehow does
       Serial.println("ERROR: COMMAND IN SORTCOMMANDS() DOES NOT HAVE A PROPER LABEL");
       command_queue.pop(); //dequeue central queue 
    }
  }
}
void writeQueue()
{
  String towrite;
  while(!write_queue.isEmpty())
  {
    towrite = write_queue.peek();
    for (int i = 0; i < towrite.length(); i++)
    {
      Serial.write(towrite[i]);   // Push individual chars of the String through the loop (write doesn't support String, only string/char*)
    }
    Serial.write("\n");
    write_queue.pop();
  }
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

void Motor_Encoder()
{
  String reading = "";
  reading += millis(); reading += " m ";
  reading += leftEncoder; reading += " "; reading += rightEncoder;
  write_queue.push(reading);
  Serial.println(reading);
}

void Mount_Sweep()
{
  String reading = "";
    // scan from 0 to 180 degrees
  for(int angle = 0; angle < 180; angle++)  
  {         
    mount_laser.write(angle);     
    if(angle % 3 == 0)
    {       
      reading += millis(); reading += (" r "); reading += angle; //add timestamp, laser label, angle
      reading += " "; reading += laserA_l.readRangeContinuousMillimeters(); //laser 2 val (mm)
      reading += " "; reading += laserA_s.readRangeContinuousMillimeters(); //laser 1 val (mm)
      reading += " "; reading += laserB_l.readRangeContinuousMillimeters(); //laser 4 val (mm)
      reading += " "; reading += laserB_s.readRangeContinuousMillimeters(); //laser 3 val (mm)
      //write_queue.push(reading); //add line to write queue
      reading = "";  
    }        
    delay(8);                   
  } 

  delay(100);
  // now scan back from 180 to 0 degrees
  for(int angle = 180; angle > 0; angle--)    
  {                                
    mount_laser.write(angle);  
    if(angle % 3 == 0)
    { 
      reading += millis(); reading += (" r "); reading += angle; //add timestamp, laser label, angle
      reading += " "; reading += laserA_l.readRangeContinuousMillimeters(); //laser 2 val (mm)
      reading += " "; reading += laserA_s.readRangeContinuousMillimeters(); //laser 1 val (mm)
      reading += " "; reading += laserB_l.readRangeContinuousMillimeters(); //laser 4 val (mm)
      reading += " "; reading += laserB_s.readRangeContinuousMillimeters(); //laser 3 val (mm)
      //write_queue.push(reading); //add line to write queue
      reading = "";  
    }               
    delay(8);       
  } 
}

void getDistanceReading()
{
  String reading = "";
  reading += millis(); reading += (" r "); reading += "0"; //add timestamp, laser label, angle = 0
  reading += " "; reading += laserA_l.readRangeContinuousMillimeters(); //laser 2 val (mm)
  reading += " "; reading += laserA_s.readRangeContinuousMillimeters(); //laser 1 val (mm)
  reading += " "; reading += laserB_l.readRangeContinuousMillimeters(); //laser 4 val (mm)
  reading += " "; reading += laserB_s.readRangeContinuousMillimeters(); //laser 3 val (mm)
  Serial.println(reading);
  //write_queue.push(reading); //add line to write queue           
}

void getIMU()
{
  uint8_t Buf[14];
  uint8_t Mag[7];  
  uint8_t ST1;
  String reading = "";
  
  reading += millis(); reading += " i ";
  
  //Magnetometer
  /*do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));*/
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  //Accelerometer and Gyroscope
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];
 
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  // Magnetometer
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);
 
  // Accelerometer
  reading += ax; reading += " "; reading += ay; reading += " "; reading += az; reading += " ";
  // Gyroscope
  reading += gx; reading += " "; reading += gy; reading += " "; reading += gz; reading += " ";
  // Magnetometer
  reading += (mx+200); reading += " "; reading += (my-70); reading += " "; reading += (mz-700);
  write_queue.push(reading);
  Serial.println(reading);  
}

void drop() //drop a kit
{
  int angle = 60;
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
  delay(500);
}

void lightUp(int time_stamp) //milliseconds
{
  static int curr_time = millis();
  time_stamp += curr_time;
  while(curr_time < time_stamp) {
    digitalWrite(LED_PIN, HIGH);   
    delay(100);                       
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void getLightReading() //photocell sensor/transistor
{
  String reading = "";
  reading += millis(); reading += " l ";
  reading += analogRead(PHOTOCELL); write_queue.push(reading);
  Serial.println(reading);
}

void getTempReading() //temperature sensor(s)
{
  String reading = ""; 
  reading += millis(); reading += " t ";
  reading += tempA.readObjTempC(); reading += " "; reading += tempB.readObjTempC(); write_queue.push(reading);
  Serial.println(reading);
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


