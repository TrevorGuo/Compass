// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code turns on the LOCUS built-in datalogger. The datalogger
// turns off when power is lost, so you MUST turn it on every time
// you want to use it!
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h> //Adafruit GPS Library
#include <SoftwareSerial.h>
#include <Math.h>

//Adafruit LSM9DS1 library
//Adafruit Unified Sensor library


// what's the name of the hardware serial port?

// Connect to the GPS on the hardware port

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

String NMEA1;
String NMEA2;
char c;


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("starting");
  GPS.begin(9600);
  GPS.sendCommand("$PGCMD,33,0*6D");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
  Serial.println("Adafruit GPS logging data dump!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800
}

uint32_t updateTime = 1000;
double latPoint = 0.0;
double longPoint = 0.0;

void loop()                     // run over and over again
{
  readGPS();
  if(GPS.fix==1)
  {
    Serial.print(GPS.latitude);
    Serial.print(GPS.lat);
    Serial.print(GPS.longitude);
    Serial.println(GPS.lon);
    Serial.println("NEXT\n");
  }
}//loop

//https://stackoverflow.com/questions/3932502/calculate-angle-between-two-latitude-longitude-points
//The math/code to find the bearing between two coordinates was found at the above link. 
double getBearingToWaypoint(double lat1, double long1, double lat2, double long2) {
    double dLon = (long2 - long1);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1)
            * cos(lat2) * cos(dLon);

    double brng = atan2(y, x);

    brng = toDegrees(brng);
    brng = (brng + 360) % 360;
    brng = 360 - brng; //This line might not be needed?

    return brng;
}

void savePoint(double lat, double lon) {
    latPoint = lat;
    longPoint = lon;
}

double changeInDegree(double oldBrng, double newBrng) { //Positive rotates CW, negative rotates CCW
    /* Continuous Circuit
    double degreeDelta = newBrng - oldBrng;
    if (degreeDelta < 180 && degreeDelta >= 0) //newBrng > oldBrng, and shortest rotation is CW
        return degreeDelta;
    else if (degreeDelta > 180 && degreeDelta < 360) //newBrng > oldBrng, and shortest rotation is CCW
        return degreeDelta - 360;
    else if (degreeDelta > -180 && degreeDelta < 0) //newBrng < oldBrng, and shortest rotation is CCW
        return degreeDelta;
    else //newBrng < oldBrng, and shortest rotation is CW
        return 360 + degreeDelta;
    */
   //N is 90, S is 0 and 180
   double oldDegree = 90 - (oldBrng / 2);
   double newDegree = 90 - (newBrng / 2);

   return newDegree - oldDegree;
}

void readGPS()
{
  clearGPS();
  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NMEA1=GPS.lastNMEA();
  
  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NMEA2=GPS.lastNMEA();
  Serial.println(NMEA1);
  Serial.println(NMEA2);
  Serial.println("--");
}

void clearGPS() //clear old data from serial port
{
  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
}
