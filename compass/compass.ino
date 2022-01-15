#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

String NMEA1;
String NMEA2;
char c;

void setup()
{
  Serial.begin(115200);
  Serial.println("starting");
  GPS.begin(9600);
  GPS.sendCommand("$PGCMD,33,0*6D");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
}

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
