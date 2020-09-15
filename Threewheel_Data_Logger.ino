///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// THREE WHEELER DATA ACQUISITION CODE ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//  DATE CREATED  : 02 APRIL 2019
//  VERSION       : 09
//  PROGRAMMER    : MIRAN DABARE & RASHAN PERERA
///////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// NOTES ////////////////////////////////////
//TO SPEED UP THE ADC SAMPLE RATE, OPEN THE ADS1015.cpp FILE IN THE LIBRARY AND DO THE FOLLOWING
// On line 154 replace ADS1015_REG_CONFIG_DR_1600SPS by ADS1015_REG_CONFIG_DR_3300SPS.
// On line 104 replace ADS1115_CONVERSIONDELAY by 2.




/////////////////////////////////////////////////////////////////////////////////

/////// ASSIGNED ADC PIN NUMBERS //////
// THROTTLE = A0
// CLUTCH   = A1
// BRAKE    = A2
// GEAR     = A3

/////// ASSIGNED ARDUINO PIN NUMBERS //////
// ADC SCL = A5
// ADC SDA = A4


// LEFT ENCODER = D2
// RIGHT ENCODER= D3
// DATA SWITCH  = D5

//SCL = A5
//SDA = A4
//BOTH ACC. USE THE SAME SCL AND SDA PINS, CONNECTS THE 3.3V TO THE ADO PIN OF THE 2ND ACC.

// LCD DISPLAY
// RS - D12
// EN - D11
// DB4 - D10
// DB5 - D9
// DB6 - D8
// DB7 - D7

// VSS - GND
// VDD - 5V
// V0 - THROUGH POT 5V
// LEDA - 5V
// LEDK - THROUGH 1K RESISTOR GND
// RW - GND

// OPTIONAL -
//GPS PINS FOR SOME REASON THIS IS SWITCHED, KEEP IT LIKE THIS
//RX = D3
//TX = D4




///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

#include<Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C Interface

#include <LiquidCrystal.h>

LiquidCrystal lcd(12,11,10,9,8,7);    
  




//////////////////////
int sensorValue = 0;  //initialization of sensor variable, equivalent to EMA Y
float EMA_a = 0.3;    //initialization of EMA alpha
int EMA_S = 0;

/////////// ADC VARIABLES /////////
Adafruit_ADS1115 ads;
const float multiplier = 0.1875;

///// RANDOM ////////
unsigned long time_1;
int gear;


///////// ENCODER FILES ////////////
float value=0;
float pulse=0;
float rpm;
int oldtime=0;
int time;
float Velocity1=0;
float Velocity2=0;
int Avg_Vel=0;
float wheel_radius=0.15;
int slots = 60;
///////////////////////////
float valueR=0;
float pulseR=0;
float rpmR;
int oldtimeR=0;
float Velocity1R=0;
float Velocity2R=0;
float Avg_VelR=0;

///////// ENCODER INTERUPT //////////////
const byte interruptPin =2;
const byte interruptPinR =3;

/////////// CALIBRATION FILES /////////////
int throttle_low = 15420;
int throttle_high = 3400;
int clutch_low = 23600;
int clutch_high = 21800;
int brake_low = 5320;
int brake_high = 6200;

float time_new=0;
float time_prev=0;


/////////// ACCELEROMETER FILES /////////////

const int MPU2=0x69,MPU1=0x68;
float AcX1,AcY1,AcZ1,AcX_1,AcY_1,AcZ_1;
float AcX2,AcY2,AcZ2,AcX_2,AcY_2,AcZ_2;
//float Act_X1,Act_Y1,Act_Z1;





void isr() //interrupt service routine
{
pulse++;
}

void isrR() //interrupt service routine
{
pulseR++;
}

//////////////////////////////// SETUP ////////////////////////////////////////////////

void setup() {

///////// LCD DISPLAY ////////////
  lcd.begin(16,2);

  lcd.setCursor(0,0);

  lcd.print("3WHEELER DATA");
  lcd.setCursor(0,1);
  lcd.print("ACQUISITION V2");
  delay(2000);

 ClearLCD();

  lcd.setCursor(0,0);   
  lcd.print("CODED BY -");   
  lcd.setCursor(0,1); 
  lcd.print("HMD"); 
  

  delay(2000);

 ClearLCD();

  

/////// ADC ///////
  ads.begin();

//////////////SD CARD WRITING CODE////////////////

 Serial.begin(115200);
  

//////////ENCODER INTERUPT//////////////////

pinMode(interruptPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(2),isr,RISING);  //attaching the interrupt
pinMode(interruptPinR, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(3),isrR,RISING);  //attaching the interrupt

//////////////// HEADER PRINTS //////////////
Serial.print("Time,Throttle,Clutch,Brake,Gear,Right RPM,Right Velocity,Left RPM,Left Velocity,Left Accel X,Left Accel Y,Left Accel Z,");
Serial.print("Right Accel X,Right Accel Y,Right Accel Z,Raw Throttle,Raw Clutch,Raw Brake,Raw Gear,Baro Pressure (hPa), Temperature (C),");
Serial.println(" Altitude (m)");


/////////// ACCELEROMETER FILES /////////////
      Wire.begin(); 
      Wire.beginTransmission(MPU1);
      Wire.write(0x6B);// PWR_MGMT_1 register 
      Wire.write(0); // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);Wire.begin(); 
      Wire.beginTransmission(MPU2);
      Wire.write(0x6B);// PWR_MGMT_1 register 
      Wire.write(0); // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);


/////////// PRESSURE SENSOR FILES /////////////

 if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
               



}



////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// LOOP ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////




void loop() {
 
  lcd.setCursor(0,0);
  lcd.print("Cccc");
  ////////// ADC ////////////
  int16_t adc0, adc1, adc2, adc3;


////////////////////////////// DATA ACQUISITION /////////////////////////////



time_new = millis()- time_prev;


///////////////////// LEFT WHEEL SPEED CALCULATION ///////////////////

time=millis()-oldtime;        //finds the time 
rpm=((pulse/time)*60000)/slots;         //calculates rpm
Velocity1= (rpm*(2*3.14/60))*wheel_radius*3.6;
Avg_Vel = (Velocity1+Velocity2)/2;
oldtime=millis();
Velocity2=Velocity1;
pulse=0;






//////////////// RIGHT WHEEL SPEED CALCULATION //////////////////////////

time=millis()-oldtimeR;        //finds the time 
rpmR=((pulseR/time)*60000)/slots;         //calculates rpm
Velocity1R= (rpmR*(2*3.14/60))*wheel_radius*3.6;
Avg_VelR = (Velocity1R+Velocity2R)/2;
oldtimeR=millis();
Velocity2R=Velocity1R;
pulseR=0;


///////// ADC READINGS /////////

//WITH 5V ADC READS 26667

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);


////////////////// POTENTIOMETER CALCULATION ////////////////////////////

float throttle_pot_read=adc3;

  EMA_S = (EMA_a*adc0) + ((1-EMA_a)*EMA_S);





//Throttle Percentage convertion
int throttle=((throttle_pot_read - throttle_low)/(throttle_high - throttle_low))*100;

float clutch_pot_read=adc0;

//Clutch Percentage convertion
int clutch=((clutch_pot_read - clutch_low)/(clutch_high - clutch_low))*100;

float brake_pot_read=adc1;

//Brake Percentage convertion
int brake=(((brake_pot_read - brake_low))/(brake_high - brake_low))*100;

float gear_pot_read=adc2;





//////////////////////// GEAR PERCENTAGE CONVERTION /////////////////////////

if ((gear_pot_read > 20000))
{
  gear=1;
  }

  else if(( 17000 > gear_pot_read)&&(gear_pot_read >= 15000))
  {
    gear=0;
    }
  else if(( 15000 > gear_pot_read)&&(gear_pot_read >= 13000))
  {
    gear=2;
    }
      else if(( 13000 > gear_pot_read)&&(gear_pot_read >= 10000))
  {
    gear=3;
    }
      else if(9000 > gear_pot_read)
  {
    gear=4;
    }


////////////////////////// ACCELEROMETER TRIGGERS //////////////////////////////

      //get values for first mpu having address of 0x68   
      GetMpuValue1(MPU1);
   //get values for second mpu having address of 0x69
      GetMpuValue2(MPU2);








////////////////////////// SERIAL PRINTING //////////////////////////////


time_1 = millis();
Serial.print(time_new/1000);
Serial.print(",");


Serial.print(throttle);
Serial.print(",");


Serial.print(clutch);
Serial.print(",");


Serial.print(brake);
Serial.print(",");


Serial.print(gear);
Serial.print(",");
//ENCODER PRINT


Serial.print(Avg_Vel);
Serial.print(",");


Serial.print(Avg_VelR);
Serial.print(",");
// ACCEL PRINT

Serial.print(AcX_1);
Serial.print(",");
Serial.print(AcY_1);
Serial.print(",");
Serial.print(AcZ_1); 

Serial.print(",");
Serial.print(AcX_2);
Serial.print(",");
Serial.print(AcY_2);
Serial.print(",");
Serial.print(AcZ_2); 
Serial.print(",");

Serial.print((adc0));
Serial.print(",");
Serial.print((adc1));
Serial.print(",");
Serial.print((adc2));
Serial.print(",");
Serial.print((adc3));
Serial.print(",");

Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
Serial.print(",");
Serial.print(bmp.readTemperature());
Serial.print(",");
Serial.println(bmp.readAltitude(1019)); //The "1019.66" is the pressure(hPa) at sea level in day in your region

  ClearLCD();
  
  lcd.setCursor(0,0);
  lcd.print("T");
  lcd.setCursor(1,0);
  lcd.print(throttle);
  lcd.setCursor(4,0);
  lcd.print(" ");
  
  lcd.setCursor(5,0);
  lcd.print("C");
  lcd.setCursor(6,0);
  lcd.print(clutch);
  lcd.setCursor(9,0);
  lcd.print(" ");
  
  lcd.setCursor(10,0);
  lcd.print(gear);

  lcd.setCursor(12,0);
  lcd.print("B");
  lcd.setCursor(13,0);
  lcd.print(brake);  


  lcd.setCursor(0,1);
  lcd.print(bmp.readAltitude(1019));
  lcd.setCursor(5,1);
  lcd.print("m"); 

  lcd.setCursor(8,1);
  lcd.print(Avg_Vel);
  lcd.setCursor(12,1);
  lcd.print("kmph"); 
 delay(100);
}


//////////////////////////////// ACCELEROMETER TRIGGER///////////////////////////////////////////

 void GetMpuValue1(const int MPU){ 
   
      Wire.beginTransmission(MPU); 
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) 
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true); // request a total of 14 registers 
      AcX1=Wire.read()<<8| Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
      AcY1=Wire.read()<<8|  Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ1=Wire.read()<<8| Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
     

      AcX_1=AcX1/16384+0.06;
      AcY_1=AcY1/16384-0.02;
      AcZ_1=(AcZ1/16384)-0.18;

     }

 void GetMpuValue2(const int MPU){ 
   
      Wire.beginTransmission(MPU); 
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) 
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true); // request a total of 14 registers 
      AcX2=Wire.read()<<8| Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
      AcY2=Wire.read()<<8|  Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ2=Wire.read()<<8| Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
     

      AcX_2=AcX2/16384;
      AcY_2=AcY2/16384-0.0;
      AcZ_2=(AcZ2/16384)-0;

         }

 void ClearLCD()
 {
  lcd.setCursor(0,0);   
  lcd.print("                "); 
  lcd.setCursor(0,1);   
  lcd.print("                "); 
  return;

 }
 

     
