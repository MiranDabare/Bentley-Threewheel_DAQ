///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// THREE WHEELER DATA ACQUISITION CODE ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/////// ASSIGNED ARDUINO PIN NUMBERS //////
// THROTTLE = A0
// CLUTCH   = A1
// BRAKE    = A2
// GEAR     = A3
// LEFT ENCODER = D2
// RIGHT ENCODER= D3
// DATA SWITCH  = D5

///// SD CARD MODULE PINS //////
// CS   = D10
// SCK  = D13
// MOSI = D11
// MISO = D12

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>

//// SD CARD FILES ////////
File myFile;
char filename[16] = {'/0'};
int pinCS = 53; // Pin 10 on Arduino Uno

///// RANDOM ////////
unsigned long time_1;
float gear;

//////// DATA BUTTON FILES //////////
int button=5;
int button_val=0;
int counter=0;
int addr = 0;
int EEPROM_val;

///////// ENCODER FILES ////////////
float value=0;
float pulse=0;
float rpm;
int oldtime=0;
int time;
float Velocity1=0;
float Velocity2=0;
float Avg_Vel=0;
float wheel_radius=0.15;
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
int throttle_low = 39;
int throttle_high = 850;
int clutch_low = 292;
int clutch_high = 437;
int brake_low = 80;
int brake_high = 800;

float time_new=0;
float time_prev=0;

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

counter= EEPROM.read(addr);




//////////////SD CARD WRITING CODE////////////////

 Serial.begin(9600);
 pinMode(pinCS, OUTPUT);
 pinMode(button, INPUT);
  
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }







//////////ENCODER INTERUPT//////////////////

pinMode(interruptPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(2),isr,RISING);  //attaching the interrupt
pinMode(interruptPinR, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(3),isrR,RISING);  //attaching the interrupt
sprintf(filename, "log%d.txt", counter); 
Serial.println("Setup Completed");
}


//////////////////////////////// LOOP ////////////////////////////////////////////////


void loop() {

  button_val=digitalRead(button);
  sprintf(filename, "log%d.txt", counter);
  myFile = SD.open(filename, FILE_WRITE);

 Serial.println("Data Headers Writing");
 
 //////////////////// DATA LOG HEADERS ///////////////////////
  if (myFile) {   
    
  myFile.print("Time (ms)");
    myFile.print(",");    
    
    myFile.print("Throttle (%)");
    myFile.print(",");    
    
    myFile.print("Clutch (%)");
    myFile.print(",");    
    
    myFile.print("Brake (%)");
    myFile.print(",");   

    myFile.print("Gear");
    myFile.print(","); 
    
    myFile.print("Left RPM");
    myFile.print(",");   

    myFile.print("Right RPM");
    myFile.print(",");   

    myFile.print("Left Velocity (km/h)");
    myFile.print(","); 
    myFile.println("Right Velocity (km/h)");

   myFile.close(); // close the file
  }



////////////////////////////// DATA ACQUISITION /////////////////////////////

if (button_val == HIGH)
{
    Serial.println("Data Log Started");
    Serial.println(filename);
}

while (button_val == HIGH)
{
  
button_val=digitalRead(button);
time_new = millis()- time_prev;


///////////////////// LEFT WHEEL SPEED CALCULATION ///////////////////

time=millis()-oldtime;        //finds the time 
rpm=((pulse/time)*60000)/100;         //calculates rpm
Velocity1= (rpm*(2*3.14/60))*wheel_radius*3.6;
Avg_Vel = (Velocity1+Velocity2)/2;
oldtime=millis();
Velocity2=Velocity1;
pulse=0;
//attachInterrupt(0,isr,RISING);





//////////////// RIGHT WHEEL SPEED CALCULATION //////////////////////////

time=millis()-oldtimeR;        //finds the time 
rpmR=((pulseR/time)*60000)/100;         //calculates rpm
Velocity1R= (rpmR*(2*3.14/60))*wheel_radius*3.6;
Avg_VelR = (Velocity1R+Velocity2R)/2;
oldtimeR=millis();
Velocity2R=Velocity1R;
pulseR=0;
//attachInterrupt(0,isr,RISING);





////////////////// POTENTIOMETER CALCULATION ////////////////////////////

float throttle_pot_read=analogRead(A0);

//Throttle Percentage convertion
float throttle=((throttle_pot_read - throttle_low)/(throttle_high - throttle_low))*100;

float clutch_pot_read=analogRead(A1);

//Clutch Percentage convertion
float clutch=((clutch_pot_read - clutch_low)/(clutch_high - clutch_low))*100;

float brake_pot_read=analogRead(A2);

//Brake Percentage convertion
float brake=((1023-(brake_pot_read - brake_low))/(brake_high - brake_low))*100;

float gear_pot_read=analogRead(A3);





//////////////////////// GEAR PERCENTAGE CONVERTION /////////////////////////

if (( 0<= gear_pot_read)&&(gear_pot_read < 200))
{
  gear=1;
  }

  else if(( 200 < gear_pot_read)&&(gear_pot_read <= 400))
  {
    gear=2;
    }
  else if(( 400 < gear_pot_read)&&(gear_pot_read <= 500))
  {
    gear=3;
    }
      else if(( 500 < gear_pot_read)&&(gear_pot_read <= 600))
  {
    gear=4;
    }
      else if(601< gear_pot_read)
  {
    gear=0;
    }



////////////////////////// SERIAL PRINTING //////////////////////////////

Serial.print("Time: ");
time_1 = millis();
Serial.print(time_new/1000);
Serial.print(",");

Serial.print(" Throttle:  ");
Serial.print(throttle);
Serial.print(",");

Serial.print(" Clutch:  ");
Serial.print(clutch);
Serial.print(",");

Serial.print(" Brake:  ");
Serial.print(brake);
Serial.print(",");

Serial.print(" Gear: ");
Serial.print(gear);

//ENCODER PRINT

Serial.print("Right RPM: ");
Serial.print(rpm);
Serial.print("Right Velocity : ");
Serial.print(Avg_Vel);

Serial.print("Left RPM: ");
Serial.print(rpmR);
Serial.print("Left Velocity : ");
Serial.println(Avg_VelR);



/////////////////////// SD CARD PRINTNG ///////////////////////////////

   myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {    
    
    myFile.print(time_new);
    myFile.print(",");    
    
    myFile.print(throttle);
    myFile.print(",");    
    
    myFile.print(clutch);
    myFile.print(",");    
    
    myFile.print(brake);
    myFile.print(",");   

    myFile.print(gear);
    myFile.print(","); 
    
    myFile.print(rpm);
    myFile.print(",");   

    myFile.print(rpmR);
    myFile.print(",");  
    
    myFile.print(Avg_Vel);
    myFile.print(",");  

    myFile.println(Avg_VelR);

    myFile.close(); // close the file
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening file");
  }
  
  delay(100);
}

Serial.println("Data Log Terminated");


////////////////////////// EEPROM COUNTER STORAGE /////////////////////////

EEPROM.write(addr, counter);


//////////////////////// IDLE STATE ////////////////////////////

while (button_val == LOW)
{
button_val=digitalRead(button);
time_prev=millis();
}

////////// FILE NAME COUNTER //////////////
counter++;

}
