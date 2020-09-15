/*
 * Rui Santos 
 * Complete Project Details http://randomnerdtutorials.com
 *
 * Based on the example TinyGPS++ from arduiniana.org
 *
 */
 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup(){
  Serial.begin(9600);
  ss.begin(GPSBaud);

      Serial.print("Lat");
    Serial.print(",");
      Serial.print("Long");
    Serial.print(",");
  
  Serial.print("Hour");
    Serial.print(",");
      Serial.print("Minute");
    Serial.print(",");
      Serial.print("Second");
    Serial.print(",");
      Serial.print("Centi-Sec");
    Serial.print(",");
      Serial.print("Speed m/s");
    Serial.print(",");
      Serial.print("Speed km/h");
    Serial.print(",");
      Serial.print("COurse deg");
    Serial.print(",");
      Serial.print("Atli m");
    Serial.print(",");
      Serial.print("No. Sats");
    Serial.print(",");
      Serial.println("Precision");


}

void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      // Latitude in degrees (double)
       Serial.print(gps.location.lat(), 6);      
      // Longitude in degrees (double)
      Serial.print(","); 
      Serial.print(gps.location.lng(), 6); 
        Serial.print(","); 


      // Raw date in DDMMYY format (u32)
      Serial.print(gps.date.value()); 
 Serial.print(","); 

     // Hour (0-23) (u8)
        Serial.print(gps.time.hour()); 
      Serial.print(","); 
      Serial.print(gps.time.minute()); 
      Serial.print(","); 
      Serial.print(gps.time.second()); 
      Serial.print(","); 
      Serial.print(gps.time.centisecond()); 


      // Speed in meters per second (double)
      Serial.print(gps.speed.mps()); 
      // Speed in kilometers per hour (double)
 Serial.print(","); 
      Serial.print(gps.speed.kmph()); 
       Serial.print(","); 

      // Raw course in 100ths of a degree (i32)
      Serial.print(gps.course.deg()); 
       Serial.print(","); 

      // Raw altitude in centimeters (i32)
      // Altitude in meters (double)
      Serial.print(gps.altitude.meters()); 
       Serial.print(","); 

      // Number of satellites in use (u32)

      Serial.print(gps.satellites.value()); 
       Serial.print(","); 

      // Horizontal Dim. of Precision (100ths-i32)
      Serial.println(gps.hdop.value()); 
    }
  }
}
