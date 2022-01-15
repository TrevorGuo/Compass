// Mahony AHRS for the LSM9DS1  S.J. Remington 2/2021
// Requires the Sparkfun LSM9DS1 library
// Standard sensor orientation X North (yaw=0), Y West, Z up
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// NOTE: Sensor X axis is remapped to the opposite direction of the "X arrow" on the Adafruit sensor breakout!

// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion program LSM9DS1_cal_data
//
/*
  Adafruit 3V or 5V board
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  LSM9DS1 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VIN ------------- 5V
   GND ------------- GND

   CSG, CSXM, SDOG, and SDOXM should all be pulled high.
   pullups on the ADAFRUIT breakout board do this.
*/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// default settings gyro  245 d/s, accel = 2g, mag = 4G
LSM9DS1 imu;

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The filter will not produce sensible results if these are not correct

//Gyro scale 245 dps convert to radians/sec and offsets
float Gscale = (M_PI / 180.0) * 0.00875; //245 dps scale sensitivity = 8.75 mdps/LSB
int G_offset[3] = {-2, 1, 0};//{75, 31, 142};

//Accel scale 16457.0 to normalize
 float A_B[3]
 { -291.60, -234.68,  518.15};

 float A_Ainv[3][3]
  {{  1.01540,  0.00106,  0.00163},
  {  0.00106,  1.01569,  0.00272},
  {  0.00163,  0.00272,  0.99795}};

//Mag scale 3746.0 to normalize
 float M_B[3]
 {  338.78, 1803.65,  655.55};

 float M_Ainv[3][3]
  {{  1.10188,  0.04520,  0.01773},
  {  0.04520,  1.16098, -0.00301},
  {  0.01773, -0.00301,  1.17076}};

// local magnetic declination in degrees
float declination = -11.72; //-14.84;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized. Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output


void setup()
{
  Serial.begin(9600);
  while (!Serial); //wait for connection
  Serial.println("LSM9DS1 AHRS starting");

  Wire.begin();

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }
}

float getYaw()
{
  static char updated = 0; //flags for sensor updates
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data

  // Update the sensor values whenever new data is available
  if ( imu.accelAvailable() ) {
    updated |= 1;  //acc updated
    imu.readAccel();
  }
  if ( imu.magAvailable() ) {
    updated |= 2; //mag updated
    imu.readMag();
  }
  if ( imu.gyroAvailable() ) {
    updated |= 4; //gyro updated
    imu.readGyro();
  }
  if (updated == 7) //all sensors updated?
  {
    updated = 0; //reset update flags
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // correct accel/gyro handedness
    // Note: the illustration in the LSM9DS1 data sheet implies that the magnetometer
    // X and Y axes are rotated with respect to the accel/gyro X and Y, but this is not case.

    Axyz[0] = -Axyz[0]; //fix accel/gyro handedness
    Gxyz[0] = -Gxyz[0]; //must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);
    
// skip ypr calc for speed? (see below)
//     if (millis() - lastPrint > PRINT_SPEED) {

    // Define Tait-Bryan angles.
    // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
    // this code corrects for magnetic declination.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. 
    // Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order.
    //
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // http://www.ngdc.noaa.gov/geomag-web/#declination
    //conventional nav, yaw increases CW from North, corrected for local magnetic declination

    yaw = -(yaw + declination);
    if (yaw < 0) yaw += 360.0;
    if (yaw >= 360.0) yaw -= 360.0;
// skip ypr printing    
    if (millis() - lastPrint > PRINT_SPEED) {
      Serial.print("ypr ");
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }
  }
  return yaw;
  // consider averaging a few headings for better results
}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
  Gxyz[0] = Gscale * (imu.gx - G_offset[0]);
  Gxyz[1] = Gscale * (imu.gy - G_offset[1]);
  Gxyz[2] = Gscale * (imu.gz - G_offset[2]);

  Axyz[0] = imu.ax;
  Axyz[1] = imu.ay;
  Axyz[2] = imu.az;
  Mxyz[0] = imu.mx;
  Mxyz[1] = imu.my;
  Mxyz[2] = imu.mz;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony fusion scheme uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
// [ax,ay,az] and [mx,my,mz] MUST be normalized (unit vectors)
// [gx,gy,gz] MUST be in radians/sec

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;
  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of the reference vectors
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);
  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
