/*
   Sanjula Nipun
   www.facebook.com/sanjula.nipun
*/
#include "Wire.h"       //For communicate
#include "I2Cdev.h"     //For communicate with MPU6050
#include "MPU6050.h"    //The main library of the MPU6050


//Define the object to access and cotrol the Gyro and Accelerometer
//not use the Gyro data
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

//Define packet for the direction (X axis and Y axis)
int data[3];


void setup(void) {
  Serial.begin(9600);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(redPin, OUTPUT);
  Wire.begin();
  mpu.initialize();              //Initialize the MPU object

}

void loop(void) {

  //With this function, the acceleration and gyro values of the axes are taken.
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  data[0] = map(ax, -17000, 17000, 125, 255 ); //Send X axis data
  data[1] = map(ay, -17000, 17000, 125, 255);  //Send Y axis data
  data[2] = map(az, -17000, 17000, 125, 255);  //Send Z axis data

  delay(50);

  Serial.print("Axis X = ");
  Serial.print(data[0]);
  Serial.print("  ");
  Serial.print("Axis X = ");
  Serial.print(data[1]);
  Serial.print("  ");
  Serial.print("Axis Z = ");
  Serial.println(data[2]);

}
