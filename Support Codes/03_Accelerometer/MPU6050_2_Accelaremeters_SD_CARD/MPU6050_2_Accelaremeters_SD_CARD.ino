#include<Wire.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>

const int MPU2=0x69,MPU1=0x68;
float AcX1,AcY1,AcZ1,AcX_1,AcY_1,AcZ_1;
float AcX2,AcY2,AcZ2,AcX_2,AcY_2,AcZ_2;
float Act_X1,Act_Y1,Act_Z1;

int time=millis;

//////// DATA BUTTON FILES //////////
int button=5;
int button_val=0;
int counter=0;
int addr = 0;
int EEPROM_val;



//divide acceleration by 16,384
//divide gyro by 131
//SCL to A5
//SDA ro A4

//// SD CARD FILES ////////
File myFile;
char filename[16] = {'/0'};
int pinCS = 53; // Pin 10 on Arduino Uno

//-------------------------------------------------\setup loop\------------------------------------------------------------ 
 void setup(){ 
      Wire.begin(); 
      Wire.beginTransmission(MPU1);
      Wire.write(0x6B);// PWR_MGMT_1 register 
      Wire.write(0); // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);Wire.begin(); 
      Wire.beginTransmission(MPU2);
      Wire.write(0x6B);// PWR_MGMT_1 register 
      Wire.write(0); // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      Serial.begin(9600); 



        // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }



     } 
     
//---------------------------------------------------\void loop\------------------------------------------------------------
 void loop(){
   
      //get values for first mpu having address of 0x68   
      GetMpuValue1(MPU1);

      
      //get values for second mpu having address of 0x69
   //   GetMpuValue2(MPU2);

    
  button_val=digitalRead(button);
  sprintf(filename, "log%d.txt", counter);
  myFile = SD.open(filename, FILE_WRITE);

 
 //////////////////// DATA LOG HEADERS ///////////////////////
  if (myFile) {   
    
  myFile.print("Time (ms)");
    myFile.print(",");    
    
    myFile.print("Acc_L_X (G)");
    myFile.print(",");    
    
    myFile.print("Acc_L_Y (G)");
    myFile.print(",");    
    
    myFile.print("Acc_L_Z (G)");
    myFile.print(",");   

    myFile.print("Acc_R_X (G)");
    myFile.print(",");    
    
    myFile.print("Acc_R_Y (G)");
    myFile.print(",");    
    
    myFile.print("Acc_R_Z (G)");
    myFile.print(",");   

 
   myFile.close(); // close the file
  
  }

myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {    
    
    myFile.print(time);
    myFile.print(",");    
    
    myFile.print(AcX_1);
    myFile.print(",");    
    
    myFile.print(AcY_1);
    myFile.print(",");    
    
    myFile.print(AcZ_1);
    myFile.print(",");   

        
    myFile.print(AcX_2);
    myFile.print(",");    
    
    myFile.print(AcY_2);
    myFile.print(",");    
    
    myFile.println(AcZ_2);
 

    myFile.close(); // close the file
  }


    }
 
//----------------------------------------------\user defined functions\-------------------------------------------------- 
      
 
 void GetMpuValue1(const int MPU){ 
   
      Wire.beginTransmission(MPU); 
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) 
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true); // request a total of 14 registers 
      AcX1=Wire.read()<<8| Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
      AcY1=Wire.read()<<8|  Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ1=Wire.read()<<8| Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
     

      AcX_1=AcX1/4096+0.06;
      AcY_1=AcY1/4096-0.02;
      AcZ_1=(AcZ1/4096)-0.18;

      
      Serial.print(AcX_1);
      Serial.print(","); 
      Serial.print(AcY_1);
      Serial.print(",");
      Serial.print(AcZ_1); 

     }
     
  
 void GetMpuValue2(const int MPU){ 
   
      Wire.beginTransmission(MPU); 
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) 
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true); // request a total of 14 registers 
      AcX2=Wire.read()<<8| Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
      AcY2=Wire.read()<<8|  Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ2=Wire.read()<<8| Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
     

      AcX_2=AcX2/4096+0.06;
      AcY_2=AcY2/4096-0.02;
      AcZ_2=(AcZ2/4096)-0.18;

      
      Serial.print(AcX_2);
      Serial.print(","); 
      Serial.print(AcY_2);
      Serial.print(",");
      Serial.println(AcZ_2);  
     }

    
