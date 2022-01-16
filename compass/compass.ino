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
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
//#include <SoftwareSerial.h>
#include <math.h>


// what's the name of the hardware serial port?

// Connect to the GPS on the hardware port

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

String NMEA1;
String NMEA2;
char c;
#define BUTTON1 4
#define BUTTON2 5
#define BUTTON3 6
#define BUTTON4 7
int b1_duration = 0;
int b2_duration = 0;
int b3_duration = 0;
int b4_duration = 0;
double loc1[] = {-1, -1};
double loc2[] = {-1, -1};
double loc3[] = {-1, -1};
double loc4[] = {-1, -1};
int active = -1;


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
//  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("starting");
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  pinMode(BUTTON4, INPUT);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
//    Serial.println("LSM9DS1 data read demo");
//  
//  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
//  Serial.println("Found LSM9DS1 9DOF");
  setupSensor();
  Serial.println("set up");
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
  Serial.print("Deg: ");
  delay(500);
  Serial.println(getYaw());
  if(GPS.fix==1)
  {
    Serial.print(GPS.latitude);
    Serial.print(GPS.lat);
    Serial.print(GPS.longitude);
    Serial.println(GPS.lon);
    Serial.println("NEXT\n");
  }
}//loop

void handleButtons()
{
  if(BUTTON1)
  {
    active = 1;
    if(b1_duration-millis() > 4000)
    {
      loc1[0] = getLon();
      loc1[1] = getLat();
    }
  }else if(BUTTON2)
  {
    active = 2;
    if(b1_duration-millis() > 4000)
    {
      loc2[0] = getLon();
      loc2[1] = getLat();
    }
  }else if(BUTTON3)
  {
    active = 3;
    if(b1_duration-millis() > 4000)
    {
      loc2[0] = getLon();
      loc2[1] = getLat();
    }
  }else if(BUTTON4)
  {
    active = 4;
    if(b1_duration-millis() > 4000)
    {
      loc2[0] = getLon();
      loc2[1] = getLat();
    }
  }else
  {
    
  }
}

double getLon()
{
  return 1;
}

double getLat()
{
  return 1;
}

//https://stackoverflow.com/questions/3932502/calculate-angle-between-two-latitude-longitude-points
//The math/code to find the bearing between two coordinates was found at the above link. 

float toRadians(float degree) {
  return degree * M_PI / 180;
}

float getBearingToWaypoint(float lat1, float long1, float lat2, float long2) {
    lat1 = toRadians(lat1);
    long1 = toRadians(long1);
    lat2 = toRadians(lat2);
    long2 = toRadians(long2);

    float dLon = (long2 - long1);

    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1)
            * cos(lat2) * cos(dLon);

    float brng = atan2(y, x);

    return brng / M_PI * 180;
}

double getTrueNorth() {
    lsm.read();  /* ask it to read in the data */ 
//    Serial.println("read");

    /* Get a new sensor event */ 
    sensors_event_t a, m, g, temp;

    lsm.getEvent(&a, &m, &g, &temp); 

    double magy = m.magnetic.y;
    double magx = m.magnetic.x;

//    double y = 180 / M_PI * acos(fmod(((-(magy) / 50)+1), 2)-1);
//    double x = 180 / M_PI * asin(fmod((((magx - 3) / 50)), 2));

    double deg = 180 / M_PI * atan2(abs(magy), abs(magx));
    
    Serial.println(deg);
    if(magx < 0 && magy > 0)
    {
      deg = 180-deg;
    }else if(magx < 0 && magy < 0)
    {
      deg = 180+deg;
    }else if(magx > 0 && magy < 0)
    {
      deg = 360-deg;
    }
    
    Serial.print("magx: ");
    Serial.println(magx);
    Serial.print("magy: ");
    Serial.println(magy);
    return deg;
//    Serial.print("x: ");
//    Serial.println(x);
//    Serial.print("y: ");
//    Serial.println(y);
//    

//    double degree = y;
//    if (x < 0) {
//      degree = 360 - degree;
//    }
//    return degree;
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
