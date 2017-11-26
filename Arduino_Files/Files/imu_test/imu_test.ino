#include <Wire.h>
 
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
 
int16_t gyroX[40] = {0};
int16_t gyroY[40] = {0};
int16_t gyroZ[40] = {0};
int16_t g_avgX = 0;
int16_t g_avgY = 0;
int16_t g_avgZ = 0;

int16_t accelZ[40] = {0};
int16_t a_avgZ = 0;
int8_t __iter = 0;
 
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
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
 
 
// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
 
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x04); //low-pass filter 1khz
  //I2CwriteByte(MPU9250_ADDRESS, );
  // Configure accelerometers rangeM
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x02); //low pass filter accelerometer 1 khz
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
 
 
}
 
 
long int cpt=0;
// Main loop, read and display data
void loop()
{
 
  // _______________
  // ::: Counter :::
 
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print (" ");
  Serial.print ("i ");
 
 
 
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 
 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
 
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=(Buf[4]<<8 | Buf[5]);
 
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]); 
  int16_t gz=(Buf[12]<<8 | Buf[13]);

  gyroX[__iter] = gx;
  gyroY[__iter] = gy;
  gyroZ[__iter] = gz;
  accelZ[__iter] = az-16384;
  
  ++__iter;
  if(__iter>=40) __iter=0; //reset
  
  for(int i=0; i<40; i++) {
    g_avgX += gyroX[i];
    g_avgY += gyroY[i];
    g_avgZ += gyroZ[i];
    a_avgZ += accelZ[i];
  }
  gx -= g_avgX/40;
  gy -= g_avgY/40;
  gz -= g_avgZ/40;
  az -= a_avgZ/40;
  g_avgX=0; g_avgY=0; g_avgZ=0; a_avgZ=0;
    // Display values
 
  // Accelerometer
  Serial.print (ax); 
  Serial.print (" ");
  Serial.print (ay);
  Serial.print (" ");
  Serial.print (az);  
  Serial.print (" ");
 
  // Gyroscope
  Serial.print (gx); 
  Serial.print (" ");
  Serial.print (gy);
  Serial.print (" ");
  Serial.print (gz);  
  Serial.print (" ");
 
  // End of line
  Serial.println("");
//  delay(100);    
}
