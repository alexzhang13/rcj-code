#include <Servo.h> 
 
int servoPin = 10; //10 is the Dropper
// 9 is the laser mount
 
Servo servo;  
 
int angle = 0;   // servo position in degrees 
 
void setup() 
{ 
  servo.attach(servoPin); 
} 
 
 
void loop() 
{ 
  drop();
  delay(2000);
} 

void drop() {

  angle = 60;
  for(angle = 60; angle > 0; angle--)  
  {                                  
    servo.write(angle);               
    delay(5);                   
  } 

  delay(1000);

  for(angle = 0; angle < 100; angle++)    
  {                                
    servo.write(angle);           
    delay(1);       
  } 
  delay(1000);
  for(angle = 100; angle > 60; angle--)    
  {                                
    servo.write(angle);           
    delay(5);       
  } 
  delay(1000);
  
}

void laser() {
  // scan from 0 to 180 degrees
  for(angle = 0; angle < 180; angle++)  
  {                                  
    servo.write(angle);               
    delay(10);                   
  } 

  delay(1000);
  // now scan back from 180 to 0 degrees
  for(angle = 180; angle > 0; angle--)    
  {                                
    servo.write(angle);           
    delay(10);       
  } 
  delay(1000);
}

