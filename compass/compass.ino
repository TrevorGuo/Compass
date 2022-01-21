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

// what's the name of the hardware serial port?

// Connect to the GPS on the hardware port

#define GPSSerial Serial2

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);


String NMEA1;
String NMEA2;
char c;
#define BUTTON1 4
#define BUTTON2 5
#define BUTTON3 6
#define BUTTON4 7
unsigned long b1_duration = 0;
unsigned long b2_duration = 0;
unsigned long b3_duration = 0;
unsigned long b4_duration = 0;

double powellLat = 3407.18620578034;
double powellLong =  -11844.217660203313;
double locs[4][2] = {{powellLat, powellLong}, {powellLat, powellLong}, {powellLat, powellLong}, {powellLat, powellLong}};
double currPos[] = {0,0};
int active = 0;

double lat2 = -1;
double long2 = -1; 

float currentYaw = 0;
double north;
double bearing;
double heading;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

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
//  GPS.sendCommand("$PGCMD,33,0*6D");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  delay(1000);
//  clearGPS();
  Serial.println("Adafruit GPS logging data dump!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800


  myservo.attach(12);
}

uint32_t updateTime = 1000;

void loop()                     // run over and over again
{
//  readGPS();
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  
  currentYaw = getYaw();
  Serial.println(currentYaw);

  setPosition(currPos);
//  Serial.println(getBearingToWaypoint(currPos[1],currPos[0],lat2,long2));
//  Serial.println(fmod(90 - (currentYaw + getBearingToWaypoint(currPos[1],currPos[0],lat2,long2))/2.0, 180));
//  Serial.println(currentYaw - fmod(90 - (currentYaw + getBearingToWaypoint(currPos[1],currPos[0],lat2,long2))/2.0, 180));
//  myservo.write(fmod(90 - (currentYaw + getBearingToWaypoint(currPos[1],currPos[0],lat2,long2))/2.0, 180));
  //myservo.write((90 - fmod(currentYaw + getBearingToWaypoint(currPos[1],currPos[0],lat2,long2),180)/2.0));
   north = 90 - (currentYaw / 2);
   bearing = getBearingToWaypoint();
   heading = 90 - (currentYaw / 2 - bearing/2);
   if(heading > 180)
   {
    heading -= 180;
   }else if(heading < 0)
   {
    heading += 180;
   }
//  myservo.write(90 - (currentYaw / 2));
  Serial.println(bearing);
  Serial.println(heading);
  myservo.write(heading);
//
////  clearGPS();

  handleButtons();
//
//  Serial.print("Fix: ");
//  Serial.println((int)GPS.fix);
//
  Serial.print("C: ");
  Serial.print(currPos[0]);
  Serial.println(currPos[1]);
//  
  Serial.print("1: ");
  Serial.print(locs[0][0]);
  Serial.println(locs[1][1]);
//
  Serial.print("2: ");
  Serial.print(locs[1][0]);
  Serial.println(locs[1][1]);
//
  Serial.print("3: ");
  Serial.print(locs[2][0]);
  Serial.println(locs[2][1]);
//
  Serial.print("4: ");
  Serial.print(locs[3][0]);
  Serial.println(locs[4][1]);
  
}//loop

void handleButtons()
{
  if(digitalRead(BUTTON1))
  {
    active = 0;
    if(millis()-b1_duration > 4000)
    {
      setPosition(locs[active]);
    }
  }else if(digitalRead(BUTTON2))
  {
    active = 1;
    if(millis()-b2_duration > 4000)
    {
      setPosition(locs[active]);
    }
  }else if(digitalRead(BUTTON3))
  {
    active = 2;
    if(millis()-b3_duration > 4000)
    {
      setPosition(locs[active]);
    }
  }else if(digitalRead(BUTTON4))
  {
    active = 3;
    if(millis()-b4_duration > 4000)
    {
      setPosition(locs[active]);
    }
  }else
  {
    b1_duration = millis();
    b2_duration = millis();
    b3_duration = millis();
    b4_duration = millis();
  }
}

void setPosition(double* loc) {
  loc[0] = getLat();
  loc[1] = getLon();
}

double getLat()
{
  double retval = currPos[1];
  if(GPS.fix)
  {
    retval = GPS.latitude;
    if(GPS.lat == 'S'){retval*=-1;};
  }
  return retval;
}

double getLon()
{
  double retval = currPos[0];
  if(GPS.fix)
  {
      retval = GPS.longitude;
      if(GPS.lon == 'W'){retval*=-1;};
  }
  return retval;
}

double toRadians(double degree) {
  return degree * M_PI / 180;
}

double getBearingToWaypoint() {
    lat1 = toRadians(currPos[0]);
    long1 = toRadians(currPos[1]);
    lat2 = toRadians(locs[active][0]);
    long2 = toRadians(locs[active][1]);

    double dLon = (long2 - long1);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1)
            * cos(lat2) * cos(dLon);

    double brng = atan2(y, x);

    brng = brng / M_PI * 180;
    if (brng > 180) {
      brng -= 360;
    }
    return brng;
}

// void readGPS()
// {
// //  Serial.println("in REad");
// //  clearGPS();
// //  Serial.println("after clear");
//   while(!GPS.newNMEAreceived())
//   {
//     c=GPS.read();
//   }
//   GPS.parse(GPS.lastNMEA());
//   NMEA1=GPS.lastNMEA();
  
//   while(!GPS.newNMEAreceived())
//   {
//     c=GPS.read();
//   }
//   GPS.parse(GPS.lastNMEA());
//   NMEA2=GPS.lastNMEA();
//   Serial.println(NMEA1);
//   Serial.println(NMEA2);
//   Serial.println("--");
// }

// void clearGPS() //clear old data from serial port
// {
//   while(!GPS.newNMEAreceived())
//   {
// //     Serial.print("recieved: ");
// //     Serial.println(GPS.newNMEAreceived());
//      c=GPS.read();
//   }
//   GPS.parse(GPS.lastNMEA());
//   while(!GPS.newNMEAreceived())
//   {
//     c=GPS.read();
//   }
//   GPS.parse(GPS.lastNMEA());
//   while(!GPS.newNMEAreceived())
//   {
//     c=GPS.read();
//   }
//   GPS.parse(GPS.lastNMEA());
// }
