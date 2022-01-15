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
//Adafruit LSM9DS1 library
//Adafruit Unified Sensor library

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(1000);
  Serial.println("Adafruit GPS logging data dump!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);

  while (GPSSerial.available())
     GPSSerial.read();

  delay(1000);
  GPS.sendCommand("$PMTK622,1*29");
  Serial.println("----------------------------------------------------");
}

uint32_t updateTime = 1000;
double latPoint = 0.0;
double longPoint = 0.0;

void loop()                     // run over and over again
{
  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
}//loop

//https://stackoverflow.com/questions/3932502/calculate-angle-between-two-latitude-longitude-points
//The math/code to find the bearing between two coordinates was found at the above link. 
double getBearingToWaypoint(double lat1, double long1, double lat2, double long2) {
    double dLon = (long2 - long1);

    double y = Math.sin(dLon) * Math.cos(lat2);
    double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1)
            * Math.cos(lat2) * Math.cos(dLon);

    double brng = Math.atan2(y, x);

    brng = Math.toDegrees(brng);
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