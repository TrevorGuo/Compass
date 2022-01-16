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
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>

Servo myservo;
float currentYaw = 0;

// what's the name of the hardware serial port?

// Connect to the GPS on the hardware port

Adafruit_GPS GPS(&Serial);

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
double currPos[2];
int active = -1;


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
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  pinMode(BUTTON4, INPUT);
  setup9DOF();
  GPS.begin(9600);
  GPS.sendCommand("$PGCMD,33,0*6D");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  delay(1000);
  clearGPS();
  Serial.println("Adafruit GPS logging data dump!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800


  myservo.attach(9);
}

uint32_t updateTime = 1000;
double latPoint = 0.0;
double longPoint = 0.0;

void loop()                     // run over and over again
{
  currentYaw = getYaw();
  Serial.println(currentYaw);
  myservo.write(90 - (currentYaw / 2));

  clearGPS();
  currPos[0] = getLon();
  currPos[1] = getLat();
  handleButtons();

  Serial.print("Fix: ");
  Serial.println(GPS.fix);

  Serial.print("C: ");
  Serial.print(currPos[0]);
  Serial.println(currPos[1]);
  
  Serial.print("1: ");
  Serial.print(loc1[0]);
  Serial.println(loc1[1]);

  Serial.print("2: ");
  Serial.print(loc2[0]);
  Serial.println(loc2[1]);

  Serial.print("3: ");
  Serial.print(loc3[0]);
  Serial.println(loc3[1]);

  Serial.print("4: ");
  Serial.print(loc4[0]);
  Serial.println(loc4[1]);
  
}//loop

void handleButtons()
{
  if(digitalRead(BUTTON1))
  {
    active = 1;
    if(b1_duration-millis() > 4000)
    {
      clearGPS();
      loc1[0] = getLon();
      loc1[1] = getLat();
    }
  }else if(digitalRead(BUTTON2))
  {
    active = 2;
    if(b2_duration-millis() > 4000)
    {
      clearGPS();
      loc2[0] = getLon();
      loc2[1] = getLat();
    }
  }else if(digitalRead(BUTTON3))
  {
    active = 3;
    if(b3_duration-millis() > 4000)
    {
      clearGPS();
      loc3[0] = getLon();
      loc3[1] = getLat();
    }
  }else if(digitalRead(BUTTON4))
  {
    active = 4;
    if(b4_duration-millis() > 4000)
    {
      clearGPS();
      loc4[0] = getLon();
      loc4[1] = getLat();
    }
  }else
  {
    b1_duration = millis();
    b2_duration = millis();
    b3_duration = millis();
    b4_duration = millis();
  }
}

double getLon()
{
  double retval = -1;
  if(GPS.fix==1)
  {
    retval = GPS.longitude;
    if(GPS.lon == 'W'){retval*=-1;};
  }
  return retval;
}

double getLat()
{
  double retval = -1;
  if(GPS.fix==1)
  {
    retval = GPS.latitude;
    if(GPS.lat == 'S'){retval*=-1;};
  }
  return retval;
}

//https://stackoverflow.com/questions/3932502/calculate-angle-between-two-latitude-longitude-points
//The math/code to find the bearing between two coordinates was found at the above link. 
float getBearingToWaypoint(double lat1, double long1, double lat2, double long2) {
    float dLon = (long2 - long1);

    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1)
            * cos(lat2) * cos(dLon);

    float brng = atan2(y, x);

    brng = brng / M_PI * 180;
    brng = fmod((brng + 360),360);
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
