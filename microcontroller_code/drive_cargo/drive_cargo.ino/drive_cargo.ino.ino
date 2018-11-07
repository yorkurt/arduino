//Import required files
#include <ros.h>
#include <std_msgs/Int16.h>
#include <joysticks/drive.h>
//#include <AltSoftSerial.h>

const byte PWM = 3;
const byte DIR = 4;

//Create a ROS node-handler to handle ROS stuff
ros::NodeHandle  nh;

//Code for when a drive message is received
void spinWheel(const joysticks::drive &drive) {
  if (drive.right > 0) {
    digitalWrite(DIR, HIGH);
    analogWrite(PWM, drive.right);

  } else if (drive.right < 0) {

    digitalWrite(DIR, LOW);
    analogWrite(PWM, - drive.right);
  }
}


// Setup subscriber to joystick drive node & turret
ros::Subscriber<joysticks::drive> sub_drive("drive", &spinWheel);


void setup()
{

  nh.initNode();
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  nh.subscribe(sub_drive);
  //nh.subscribe(sub_turret);
  //Configure auto-baudrate
}


void loop()

{
  nh.spinOnce();
}

