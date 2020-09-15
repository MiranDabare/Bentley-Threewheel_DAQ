#include <Wire.h>
#include <Adafruit_ADS1015.h>

 
Adafruit_ADS1115 ads;
//const float multiplier = 0.1875;
const float multiplier = 1;

unsigned long timeN=0;
unsigned long timeO=0;
float freq=0;


void setup(void) 
{
  Serial.begin(115200);
 
  ads.begin();
}
 
void loop(void) 
{
  timeN=millis();
  int16_t adc0, adc1, adc2, adc3;
 
  
  adc0 = ads.readADC_SingleEnded(0)* multiplier;
  adc1 = ads.readADC_SingleEnded(1)* multiplier;
  adc2 = ads.readADC_SingleEnded(2)* multiplier;
  adc3 = ads.readADC_SingleEnded(3)* multiplier;
  Serial.print("AIN0: "); Serial.println(adc0);
  Serial.print("AIN1: "); Serial.println(adc1);
  Serial.print("AIN2: "); Serial.println(adc2);
  Serial.print("AIN3: "); Serial.println(adc3);
  //delay(10);


  timeO=millis() - timeN;
  freq=1000/timeO;
  Serial.println(timeO);
  Serial.print(1000/timeO);
    Serial.println(" ");
  delay(1000);
}
