/* Code from Adafruit. Modified for use with ROS by Mastafa Awal for the York University Robotics Team.

*/

#include <ros.h>
#include <messages/imu_data.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

//Interval for how fast to update
const long interval = 500;
unsigned long previousMillis = 0;

messages::imu_data imu_msg;
ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh;
ros::Publisher imu_pub("rover/imu_data", &imu_msg);

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void initSensors()
{

  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    nh.logfatal("Accel Err");
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    nh.logfatal("Mag Err");
  }
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    nh.logfatal("BMP Err");
  }
}


void setup(void)
{
  nh.initNode();
  nh.advertise(imu_pub);
  /* Initialise the sensors */
  initSensors();
}

void loop()
{

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    imu_msg.roll = orientation.roll;
    imu_msg.pitch = orientation.pitch;

  }

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    imu_msg.heading = orientation.heading;
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {

    /* Calculate temp */

    float temperature;
    bmp.getTemperature(&temperature);
    imu_msg.temp = temperature;
    imu_msg.altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);

  }

  //update every x millis instead of constantly
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
     previousMillis = currentMillis;
     imu_pub.publish(&imu_msg);
  }
  
  nh.spinOnce();
}
