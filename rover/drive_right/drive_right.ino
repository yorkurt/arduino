//Arduino Libraries for SyRen/Sabertooth
//Copyright (c) 2012-2013 Dimension Engineering LLC
//http://www.dimensionengineering.com/arduino
//Modified for use for the York University Rover Team by Mastafa Awal. This code is written for the right side of the drive system, though it is identical to the left side.
//The reason why two arduino's were chosen instead of one is due to latency. 

//Import required files
#include <ros.h>
#include <joysticks/drive.h>
#include <std_msgs/Bool.h>
#include <AltSoftSerial.h>
#include <Sabertooth.h>

bool first_run = true; //This is strictly so that EEPROM values can be set without risk of destroying the chip prematurely 
const int DRIVER_ADDRESS= 128;
//const int L_DRIVER_ADDRESS = 129;
const byte FORWARD = 0;
const byte BACKWARD = 1;

const int SERIAL_TIMEOUT = 1500; 
const int MIN_VOLTAGE = 11;

bool timeout = false;


ros::NodeHandle  nh;


//AltSoftSerial uses Pin 8 for Rx and Pin 9 for Tx, and that is unchangeable because hardware interrupts. PWM Pin 10 cannot be used.
//Connect Arduino Rx to driver Tx , arduino Tx to driver Rx. To save on hardware resources, we daisy chain.
AltSoftSerial drive_cmd;
Sabertooth driver(DRIVER_ADDRESS, drive_cmd);

//Reset function
void(* resetFunc) (void) = 0;

//Code for when a drive message is received
void drive_method(const joysticks::drive &drive) {

  //Since timeout callback is always done first, this ensures that if the controller is left disconnected it won't burn the EEPROM on 
  //the driver by constantly setting the values, since on detection of a timeout, the arduino resets.
  
  if(first_run) {
    driver.setTimeout(SERIAL_TIMEOUT);
    driver.setMinVoltage((MIN_VOLTAGE - 6)*5);
    first_run = false;
  }
  
  if(drive.right > 0) {
    setMotorSpeed(FORWARD, drive.right / 2); 
  } else if (drive.right < 0) {
    setMotorSpeed(BACKWARD, - drive.right / 2);
  } else {
    setMotorSpeed(FORWARD, 0);
  }
  
}


void setTimeOut(const std_msgs::Bool &timeout_data) {
  timeout = timeout_data.data;
 
  if(timeout) {
    setMotorSpeed(FORWARD, 0);
  }
  
}

ros::Subscriber<std_msgs::Bool> sub_timeout("timeout", &setTimeOut);
ros::Subscriber<joysticks::drive> sub_drive("drive", &drive_method);


void setup()
{
  
  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_timeout);
  
  drive_cmd.begin(9600);
  drive_cmd.write(170);

} 


void loop()

{
  if (!timeout) {
    nh.spinOnce();
  } else {
    setMotorSpeed(FORWARD, 0);
    resetFunc(); //Commit seppuku
  }

}

void setMotorSpeed(byte dir, byte spd) {
  drive_cmd.write(DRIVER_ADDRESS);
  drive_cmd.write(dir);
  drive_cmd.write(spd);
  drive_cmd.write((DRIVER_ADDRESS + dir + spd) & 0b01111111);
}
